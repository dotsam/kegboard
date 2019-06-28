/**
 * kegboard.pde - Kegboard v3 Arduino project
 * Copyright 2003-2011 Mike Wakerly <opensource@hoho.com>
 *
 * This file is part of the Kegbot package of the Kegbot project.
 * For more information on Kegbot, see http://kegbot.org/
 *
 * Kegbot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * Kegbot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Kegbot.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * This firmware is intended for an Arduino Diecimila board (or similar)
 * http://www.arduino.cc/en/Main/ArduinoBoardDiecimila
 *
 * This firmware implements the Kegboard Serial Protocol, version 1 (KBSP v1).
 * For more information on what that means, see the kegbot docs:
 * http://kegbot.org/docs/
 *
 * You may change the pin configuration by editing kegboard_config.h; you should
 * not need to change anything in this file.
 *
 * TODO:
 *  - get/set boardname with eeprom
 *  - Thermo:
 *    * check CRC
 *    * clean up code
 *  - leak detect circuit/alarm support
 */

#include "Arduino.h"

#include <avr/pgmspace.h>
#include <string.h>
#include <util/delay.h>
#include <EEPROM.h>

#include "kegboard.h"
#include "kegboard_config.h"
#include "kegboard_eeprom.h"
#include "KegboardPacket.h"
#include "version.h"

//
// Other Globals
//

// Up to 6 meters supported if using Arduino Mega
static unsigned long volatile gMeters[] = {0, 0, 0, 0, 0, 0};
static unsigned long volatile gLastMeters[] = {0, 0, 0, 0, 0, 0};

static KegboardPacket gInputPacket;

// Structure that holds the state of incoming serial bytes.
typedef struct {
  uint8_t header_bytes_read;
  uint8_t payload_bytes_remain;
  bool have_packet;
} RxPacketStat;

static RxPacketStat gPacketStat;

// Structure to keep information about this device's uptime. 
typedef struct {
  unsigned long uptime_ms;
  unsigned long uptime_days;
  unsigned long last_uptime_ms;
  unsigned long last_meter_event;
  unsigned long last_heartbeat;
} UptimeStat;

static UptimeStat gUptimeStat;

#if KB_ENABLE_CHIP_LED
static int gChipLedBrightness = 0x0f;
#endif

static uint8_t gSerialNumber[SERIAL_NUMBER_SIZE_BYTES];

#if KB_ENABLE_SELFTEST
static unsigned long gLastTestPulseMillis = 0;
#endif

//
// ISRs
//

#if KB_ENABLE_SOFT_DEBOUNCE
#define CHECK_METER(pin, meter_index)                   \
  do {                                                  \
    delayMicroseconds(KB_SOFT_DEBOUNCE_MICROS);         \
    if (digitalRead(pin) == 0)                          \
      gMeters[meter_index] += 1;                        \
  } while(0)
#else
#define CHECK_METER(pin, meter_index)  gMeters[meter_index] += 1
#endif

void meterInterruptA()
{
  CHECK_METER(KB_PIN_METER_A, 0);
}

#ifdef KB_PIN_METER_B
void meterInterruptB()
{
  CHECK_METER(KB_PIN_METER_B, 1);
}
#endif

#ifdef KB_PIN_METER_C
void meterInterruptC()
{
  CHECK_METER(KB_PIN_METER_C, 2);
}
#endif

#ifdef KB_PIN_METER_D
void meterInterruptD()
{
  CHECK_METER(KB_PIN_METER_D, 3);
}
#endif

#ifdef KB_PIN_METER_E
void meterInterruptE()
{
  CHECK_METER(KB_PIN_METER_E, 4);
}
#endif

#ifdef KB_PIN_METER_F
void meterInterruptF()
{
  CHECK_METER(KB_PIN_METER_F, 5);
}
#endif

//
// Serial I/O
//

void writeHelloPacket()
{
  int firmware_version = FIRMWARE_VERSION;
  int protocol_version = PROTOCOL_VERSION;
  KegboardPacket packet;
  packet.SetType(KBM_HELLO_ID);
  packet.AddTag(KBM_HELLO_TAG_FIRMWARE_VERSION, sizeof(firmware_version), (char*)&firmware_version);
  packet.AddTag(KBM_HELLO_TAG_PROTOCOL_VERSION, sizeof(protocol_version), (char*)&protocol_version);
  packet.AddTag(KBM_HELLO_TAG_SERIAL_NUMBER, SERIAL_NUMBER_SIZE_BYTES, (char*)gSerialNumber);
  packet.AddTag(KBM_HELLO_TAG_UPTIME_MILLIS, sizeof(gUptimeStat.uptime_ms), (char*)&gUptimeStat.uptime_ms);
  packet.AddTag(KBM_HELLO_TAG_UPTIME_DAYS, sizeof(gUptimeStat.uptime_days), (char*)&gUptimeStat.uptime_days);
  packet.Print();
}

void writeMeterPacket(int channel)
{
  char name[5] = "flow";
  unsigned long status = gMeters[channel];
  if (status == gLastMeters[channel]) {
    return;
  } else {
    gLastMeters[channel] = status;
  }

  // Turn on the other LED when sending a meter update packet
  // Will be turned off again right away in the main loop
  TXLED1;

  name[4] = 0x30 + channel;
  KegboardPacket packet;
  packet.SetType(KBM_METER_STATUS);
  packet.AddTag(KBM_METER_STATUS_TAG_METER_NAME, 5, name);
  packet.AddTag(KBM_METER_STATUS_TAG_METER_READING, sizeof(status), (char*)(&status));
  packet.Print();
}

#if KB_ENABLE_SELFTEST
void doTestPulse()
{
  // Strobes the test pin `KB_SELFTEST_PULSES` times, every
  // `KB_SELFTEST_INTERVAL_MS` milliseconds
  unsigned long now = millis();
  if ((now - gLastTestPulseMillis) >= KB_SELFTEST_INTERVAL_MS) {
    gLastTestPulseMillis = now;
    for (int i=0; i<KB_SELFTEST_PULSES; i++) {
      digitalWrite(KB_PIN_TEST_PULSE, 1);
      digitalWrite(KB_PIN_TEST_PULSE, 0);
    }
  }
}
#endif

#if KB_ENABLE_CHIP_LED
void pulseChipLed() {
  // Pulse slow when serial is connected, fast otherwise.
  int rate = Serial ? 2 : 8;
  if (gChipLedBrightness >= 0) {
    analogWrite(KB_PIN_LED_CHIP, gChipLedBrightness);
  } else {
    analogWrite(KB_PIN_LED_CHIP, 0);
  }
  gChipLedBrightness -= rate;
  if (gChipLedBrightness < -32) {
    gChipLedBrightness = 0xff;
  }
}
#endif

//
// Main
//

void setup()
{
  memset(&gUptimeStat, 0, sizeof(UptimeStat));
  memset(&gPacketStat, 0, sizeof(RxPacketStat));
  memset(gSerialNumber, 0, SERIAL_NUMBER_SIZE_BYTES);

  if (eeprom_is_valid()) {
    eeprom_read_serialno(gSerialNumber);
  }

  // Flow meter steup. Enable internal weak pullup to prevent disconnected line
  // from ticking away.
  pinMode(KB_PIN_METER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KB_PIN_METER_A), meterInterruptA, FALLING);

#ifdef KB_PIN_METER_B
  pinMode(KB_PIN_METER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KB_PIN_METER_B), meterInterruptB, FALLING);
#endif

#ifdef KB_PIN_METER_C
  pinMode(KB_PIN_METER_C, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KB_PIN_METER_C), meterInterruptC, FALLING);
#endif

#ifdef KB_PIN_METER_D
  pinMode(KB_PIN_METER_D, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KB_PIN_METER_D), meterInterruptD, FALLING);
#endif

#ifdef KB_PIN_METER_E
  pinMode(KB_PIN_METER_E, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KB_PIN_METER_E), meterInterruptE, FALLING);
#endif

#ifdef KB_PIN_METER_F
  pinMode(KB_PIN_METER_F, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KB_PIN_METER_F), meterInterruptF, FALLING);
#endif

  Serial.begin(115200);

  writeHelloPacket();
}

void updateTimekeeping() {
  // TODO(mikey): it would be more efficient to take control of timer0
  unsigned long now = millis();
  gUptimeStat.uptime_ms += now - gUptimeStat.last_uptime_ms;
  gUptimeStat.last_uptime_ms = now;

  if (gUptimeStat.uptime_ms >= MS_PER_DAY) {
    gUptimeStat.uptime_days += 1;
    gUptimeStat.uptime_ms -= MS_PER_DAY;
  }

  if ((now - gUptimeStat.last_heartbeat) > KB_HEARTBEAT_INTERVAL_MS) {
    gUptimeStat.last_heartbeat = now;
    writeHelloPacket();
  }
}

static void readSerialBytes(char *dest_buf, int num_bytes, int offset) {
  while (num_bytes-- != 0) {
    dest_buf[offset++] = Serial.read();
  }
}

void resetInputPacket() {
  memset(&gPacketStat, 0, sizeof(RxPacketStat));
  gInputPacket.Reset();
}

void readIncomingSerialData() {
  char serial_buf[KBSP_PAYLOAD_MAXLEN];
  volatile uint8_t bytes_available = Serial.available();

  // Do not read a new packet if we have one awiting processing.  This should
  // never happen.
  if (gPacketStat.have_packet) {
    return;
  }

  // Look for a new packet.
  if (gPacketStat.header_bytes_read < KBSP_HEADER_PREFIX_LEN) {
    while (bytes_available > 0) {
      char next_char = Serial.read();
      bytes_available -= 1;

      if (next_char == KBSP_PREFIX[gPacketStat.header_bytes_read]) {
        gPacketStat.header_bytes_read++;
        if (gPacketStat.header_bytes_read == KBSP_HEADER_PREFIX_LEN) {
          // Found start of packet, break.
          break;
        }
      } else {
        // Wrong character in prefix; reset framing.
        if (next_char == KBSP_PREFIX[0]) {
          gPacketStat.header_bytes_read = 1;
        } else {
          gPacketStat.header_bytes_read = 0;
        }
      }
    }
  }

  // Read the remainder of the header, if not yet found.
  if (gPacketStat.header_bytes_read < KBSP_HEADER_LEN) {
    if (bytes_available < 4) {
      return;
    }
    gInputPacket.SetType(Serial.read() | (Serial.read() << 8));
    gPacketStat.payload_bytes_remain = Serial.read() | (Serial.read() << 8);
    bytes_available -= 4;
    gPacketStat.header_bytes_read += 4;

    // Check that the 'len' field is not bogus. If it is, throw out the packet
    // and reset.
    if (gPacketStat.payload_bytes_remain > KBSP_PAYLOAD_MAXLEN) {
      goto out_reset;
    }
  }

  // If we haven't yet found a frame, or there are no more bytes to read after
  // finding a frame, bail out.
  if (bytes_available == 0 || (gPacketStat.header_bytes_read < KBSP_HEADER_LEN)) {
    return;
  }

  // TODO(mikey): Just read directly into KegboardPacket.
  if (gPacketStat.payload_bytes_remain) {
    int bytes_to_read = (gPacketStat.payload_bytes_remain >= bytes_available) ?
        bytes_available : gPacketStat.payload_bytes_remain;
    readSerialBytes(serial_buf, bytes_to_read, 0);
    gInputPacket.AppendBytes(serial_buf, bytes_to_read);
    gPacketStat.payload_bytes_remain -= bytes_to_read;
    bytes_available -= bytes_to_read;
  }

  // Need more payload bytes than are now available.
  if (gPacketStat.payload_bytes_remain > 0) {
    return;
  }

  // We have a complete payload. Now grab the footer.
  if (!gPacketStat.have_packet) {
    if (bytes_available < KBSP_FOOTER_LEN) {
      return;
    }
    readSerialBytes(serial_buf, KBSP_FOOTER_LEN, 0);

    // Check CRC

    // Check trailer
    if (strncmp((serial_buf + 2), KBSP_TRAILER, KBSP_FOOTER_TRAILER_LEN)) {
      goto out_reset;
    }
    gPacketStat.have_packet = true;
  }

  // Done!
  return;

out_reset:
  resetInputPacket();
}

void handleInputPacket() {
  if (!gPacketStat.have_packet) {
    return;
  }

  // Process the input packet.
  switch (gInputPacket.GetType()) {
    case KBM_PING:
      writeHelloPacket();
      break;

    case KBM_SET_OUTPUT: {
      break;
    }

    case KBM_SET_SERIAL_NUMBER: {
      // Serial number can only be set if not already set.
      if (eeprom_is_valid()) {
        break;
      }

      if (gInputPacket.FindTagLength(KBM_SET_SERIAL_NUMBER_TAG_SERIAL) >= SERIAL_NUMBER_SIZE_BYTES) {
        break;
      }

      memset(gSerialNumber, 0, SERIAL_NUMBER_SIZE_BYTES);
      gInputPacket.CopyTagData(KBM_SET_SERIAL_NUMBER_TAG_SERIAL, gSerialNumber);
      eeprom_write_serialno(gSerialNumber);
      writeHelloPacket();

      break;
    }
  }
  resetInputPacket();
}

void writeMeterPackets() {
  unsigned long now = millis();

  // Forcibly coalesce meter updates; we want to be responsive, but sending
  // meter updates at every opportunity would cause too many messages to be
  // sent.
  if ((now - gUptimeStat.last_meter_event) > KB_METER_UPDATE_INTERVAL_MS) {
    gUptimeStat.last_meter_event = now;
  } else {
    return;
  }

  writeMeterPacket(0);
#ifdef KB_PIN_METER_B
  writeMeterPacket(1);
#endif
#ifdef KB_PIN_METER_C
  writeMeterPacket(2);
#endif
#ifdef KB_PIN_METER_D
  writeMeterPacket(3);
#endif
#ifdef KB_PIN_METER_E
  writeMeterPacket(4);
#endif
#ifdef KB_PIN_METER_F
  writeMeterPacket(5);
#endif
}

void loop()
{
  updateTimekeeping();

#if KB_ENABLE_CHIP_LED
  pulseChipLed();
#endif

  readIncomingSerialData();
  handleInputPacket();

  writeMeterPackets();

#if KB_ENABLE_SELFTEST
  doTestPulse();
#endif

  TXLED0;
}

// vim: syntax=c
