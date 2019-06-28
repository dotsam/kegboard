//
// Feature configuration
//

// You may enable/disable kegboard features here as desired. The deafult are
// safe.

// Enable a selftest pulse?
#define KB_ENABLE_SELFTEST  1

// Enable software debounce? EXPERIMENTAL. Enabling this feature may negatively
// affect pour accuracy.  In particular, a delay is added to each flow meter
// ISR, disabling all other interrupts during this time.
#define KB_ENABLE_SOFT_DEBOUNCE 0

// Approximate minimum pulse width required for incoming external interrupts.
#define KB_SOFT_DEBOUNCE_MICROS 1200

// Enable chip LED?
#define KB_ENABLE_CHIP_LED 1

//
// Pin configuration - KEGBOARD VERSION
//

// You may change values in this section if you know what you are doing --
// though you ordinarily shouldn't need to change these.
//
//  Digital pin allocation:
//    2 - flowmeter 0 pulse (input)
//    3 - flowmeter 1 pulse (input)
//    4 - flow 0 LED (output)
//    5 - flow 1 LED (output)
//    6 - rfid (input from ID-12)
//    7 - thermo onewire bus (1-wire, input/output)
//    8 - presence onewire bus (1-wire, input/output)
//    9 - gpio pin C
//   10 - rfid reset
//   11 - buzzer (output)
//   12 - test pulse train (output)
//   13 - alarm (output)
//

#define KB_PIN_METER_A            0
#define KB_PIN_METER_B            1
#define KB_PIN_METER_C            2
#define KB_PIN_METER_D            3
#define KB_PIN_METER_E            7

#define KB_NUM_METERS             5

#define KB_PIN_LED_CHIP           17

#define KB_PIN_TEST_PULSE         10

//
// Device configuration defaults
//

#define KB_DEFAULT_BOARDNAME          "kegboard"
#define KB_DEFAULT_BOARDNAME_LEN      8  // must match #chars above
#define KB_DEFAULT_BAUD_RATE          115200
