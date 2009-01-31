// kegboard3 - v3.0.0
// Arduino implementation of Kegboard firmware.
//
// This firmware is intended for an Arduino Diecimila board (or similar)
// http://www.arduino.cc/en/Main/ArduinoBoardDiecimila
//
// This firmware implements the Kegboard Serial Protocol, version 1
// (KBSP v1). For more information on what that means, see the kegbot
// docs: http://kegbot.org/docs/
//
// The expected digital pin usage is:
//   2 - flowmeter 0 pulse (input)
//   3 - flowmeter 1 pulse (input)
//   4 - relay 0 control (output)
//   5 - relay 1 control (output)
//   6
//   7 - thermo 0 (1-wire, input/output)
//   8 - thermo 1 (1-wire, input/output)
//   9
//   10
//   11
//   12 - selftest jumper (input)
//   13
//
// All connections are optional, but spurious data may be generated by
// unconnected inputs.
//
// You may change the pin configuration by editing kegboard_config.h; you should
// not need to change anything in this file.
//
// TODO:
//  - implement serial reading (relay on/off) commands
//  - get/set boardname with eeprom
//  - implement selftest mode
//  - Thermo:
//    * check CRC
//    * clean up code
//  - leak detect circuit/alarm support

#include "kegboard.h"
#include "kegboard_config.h"
#include "ds1820.h"

#include <util/crc16.h>

#ifdef KB_ENABLE_EEPROM
#include <EEPROM.h>
#endif

#ifdef KB_ENABLE_ONEWIRE
#include "OneWire.h"
#endif


//
// Config Globals -- defaults may be overridden by readEeprom()
//

int gBaudRate = KB_DEFAULT_BAUD_RATE;
char gBoardName[KB_BOARDNAME_MAXLEN+1] = KB_DEFAULT_BOARDNAME;
int gBoardNameLen = KB_DEFAULT_BOARDNAME_LEN;
int gUpdateInterval = KB_DEFAULT_UPDATE_INTERVAL;


//
// Other Globals
//

#ifdef KB_ENABLE_ONEWIRE
OneWire gThermoBusA(KB_PIN_THERMO_A);
DS1820Sensor gThermoSensorA(&gThermoBusA);
OneWire gThermoBusB(KB_PIN_THERMO_B);
DS1820Sensor gThermoSensorB(&gThermoBusB);
#endif

// meter 'a' tick counter
volatile unsigned long gMeterCountA = 0;

// meter 'b' tick counter
volatile unsigned long gMeterCountB = 0;


//
// EEPROM functions
//

#ifdef KB_ENABLE_EEPROM
int eepReadIntTag(byte off, int* val)
{
  int ret;
  ret = EEPROM.read(off++) << 8;
  ret |= EEPROM.read(off++) & 0xff;
  *val = ret;
  return off;
}

int eepWriteIntTag(byte off, byte tag, int val)
{
  EEPROM.write(off++, tag);
  EEPROM.write(off++, 2);
  EEPROM.write(off++, (val >> 8));
  EEPROM.write(off++, (val & 0xff));
  return off;
}

void eepReadBoardname(byte off, byte len)
{
  char tmpname[KB_BOARDNAME_MAXLEN];
  int i;
  int tmplen = 0;

  if (len > KB_BOARDNAME_MAXLEN)
    return;

  for (i=0; i<KB_BOARDNAME_MAXLEN && i < len; i++) {
    byte c = EEPROM.read(off+i);
    if (c == '\0')
      break;
    if (c < 'A' || c > 'z')
      return;
    tmpname[tmplen++] = c;
  }

  if (tmplen < 1)
    return;

  for (i=0; i<tmplen; i++)
    gBoardName[i] = tmpname[i];
  gBoardName[++i] = '\0';
  gBoardNameLen = tmplen;
}

int eepWriteBoardname(byte off)
{
  int i;
  if (gBoardNameLen <= 0)
    return off;

  EEPROM.write(off++, KB_EEP_TAG_BOARDNAME);
  EEPROM.write(off++, gBoardNameLen);
  for (i=0; i <= gBoardNameLen; i++)
    EEPROM.write(off++, gBoardName[i]);
  return off;
}

int eepReadBaudrate(byte off, byte len)
{
  if (len != 2)
    return -1;
  byte hi = EEPROM.read(off);
  byte low = EEPROM.read(off+1);
  int rate = (hi << 8) | (low & 0xff);

  switch (rate) {
    case 9600:
    case 19200:
    case 28800:
    case 57600:
    case 115200:
      gBaudRate = rate;
      break;
    default:
      return -1;
  }
}

int eepWriteBaudrate(byte off)
{
  return eepWriteIntTag(off, KB_EEP_TAG_BAUDRATE, gBaudRate);
}

int eepReadUpdateInterval(byte off, byte len)
{
  if (len != 2)
    return -1;
  byte hi = EEPROM.read(off);
  byte low = EEPROM.read(off+1);
  int rate = (hi << 8) | (low & 0xff);

  if (rate < KB_UPDATE_INTERVAL_MIN || rate > KB_UPDATE_INTERVAL_MAX)
    return -1;

  gUpdateInterval = rate;
}

int eepWriteUpdateInterval(byte off)
{
  return eepWriteIntTag(off, KB_EEP_TAG_UPDATE_INTERVAL, gUpdateInterval);
}

// readEeprom
// Reads a TLV-formatted eeprom, parsing known tags and ignoring others.
int readEeprom()
{
  byte tmp1, tmp2;
  int off;

  // Does this look like my EEPROM? First two bytes must match the magic values.
  tmp1 = EEPROM.read(0);
  tmp2 = EEPROM.read(1);
  if (tmp1 != KB_EEP_MAGIC_0 || tmp2 != KB_EEP_MAGIC_1)
    return -1;

  while (off < 512) {
    byte tag = EEPROM.read(off++);
    byte len = EEPROM.read(off++);

    switch (tag) {
      case KB_EEP_TAG_BOARDNAME:
        eepReadBoardname(off, len);
        break;
      case KB_EEP_TAG_BAUDRATE:
        eepReadBaudrate(off, len);
        break;
      case KB_EEP_TAG_UPDATE_INTERVAL:
        eepReadUpdateInterval(off, len);
        break;
      case KB_EEP_TAG_END:
        return 0;
    }
    off += len;
  }
  return 0;
}

int writeEeprom()
{
  int off=0;
  EEPROM.write(off++, KB_EEP_MAGIC_0);
  EEPROM.write(off++, KB_EEP_MAGIC_1);
  off = eepWriteBoardname(off);
  off = eepWriteBaudrate(off);
  off = eepWriteUpdateInterval(off);
  while (off < 512)
    EEPROM.write(KB_EEP_TAG_END, off++);
}
#endif // KB_ENABLE_EEPROM

//
// ISRs
//

void meterInterruptA()
{
  gMeterCountA += 1;
}

void meterInterruptB()
{
  gMeterCountB += 1;
}


//
// Main
//

void setup()
{
  pinMode(KB_PIN_METER_A, INPUT);
  pinMode(KB_PIN_METER_B, INPUT);
  
  // enable internal pullup to prevent disconnected line from ticking away
  digitalWrite(KB_PIN_METER_A, HIGH);
  digitalWrite(KB_PIN_METER_B, HIGH);
  
  pinMode(KB_PIN_SELFTEST, INPUT);
  digitalWrite(KB_PIN_SELFTEST, HIGH);

  attachInterrupt(0, meterInterruptA, RISING);
  attachInterrupt(1, meterInterruptB, RISING);

  pinMode(KB_PIN_RELAY_A, OUTPUT);
  pinMode(KB_PIN_RELAY_B, OUTPUT);

  Serial.begin(57600);
  
#ifdef KB_ENABLE_EEPROM
  readEeprom();
#endif
}

uint16_t genCrc(unsigned long val)
{
  uint16_t crc=0;
  int i=0;
  for (i=3;i>=0;i--)
    crc = _crc_xmodem_update(crc, (val >> (8*i)) & 0xff);
  return crc;
}

void writeStatusPacket()
{
  const unsigned long meter_a_tmp = gMeterCountA;
  const unsigned long meter_b_tmp = gMeterCountB;
  const unsigned short meter_a_crc = genCrc(meter_a_tmp);
  const unsigned short meter_b_crc = genCrc(meter_b_tmp);

  // prefix: 'kbsp v1 kegboard: update'
  Serial.print(KBSP_PREFIX);
  Serial.print(gBoardName);
  Serial.print(": update\t");

  // flow_0 value && crc
  Serial.print("flow_0=");
  Serial.print(meter_a_tmp, DEC);
  Serial.print("\tflow_0_crc=0x");
  Serial.print(meter_a_crc, HEX);

  // flow_1 value && crc
  Serial.print("\tflow_1=");
  Serial.print(meter_b_tmp, DEC);
  Serial.print("\t");
  Serial.print("\tflow_1_crc=0x");
  Serial.print(meter_b_crc, HEX);

  // relay_0 && relay_1
  Serial.print("\trelay_0=");
  Serial.print(digitalRead(KB_PIN_RELAY_A), DEC);
  Serial.print("\trelay_1=");
  Serial.print(digitalRead(KB_PIN_RELAY_B), DEC);
  
#ifdef KB_ENABLE_ONEWIRE
  Serial.print("\tthermo_0=");
  gThermoSensorA.PrintTemp();
  Serial.print("\tthermo_1=");
  gThermoSensorB.PrintTemp();
#endif

  Serial.print("\r\n");
}

void loop()
{
  unsigned long clock = millis();
#ifdef KB_ENABLE_ONEWIRE
  gThermoSensorA.Update(clock);
  gThermoSensorB.Update(clock);
#endif
  writeStatusPacket();
  delay(gUpdateInterval);
}

// vim: syntax=c
