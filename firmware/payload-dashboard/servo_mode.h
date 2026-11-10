// Servo control mode for the CAARL payload: who drives the two parachute servos.
//
// Plain C++ with no Arduino code, so the same file runs in the firmware and in
// tests/servo_mode_test.cpp. Call update() once per loop; write out[] to the servos.
//
// Two toggles:
//   1. allowAuto: a setting saved on the payload, changed from the dashboard before flight
//      ("Manual only" = false, the default; "Auto allowed" = true).
//   2. The transmitter switch on the ER4's mode channel (CH3 by default): high = auto, low = manual.
//      The pilot can always take manual control by flipping it to manual.
//
// Rules (in order):
//   - Not allowed: always MANUAL.
//   - Allowed and the switch says auto: AUTO. Switch says manual: MANUAL.
//   - Radio link lost (no mode-channel pulses): keep the last switch position.
//     Before any switch position has been seen, that's MANUAL.
//   - MANUAL copies the ER4's servo channels through. A channel with no valid pulse gets
//     0 = no pulses, which stops a continuous-rotation servo.
//   - AUTO uses the payload's own command. Steering isn't written yet, so the auto command
//     is "stopped" (0) until it is.
#pragma once
#include <stdint.h>

enum ServoMode : uint8_t { SM_MANUAL, SM_AUTO };
inline const char *servoModeName(ServoMode m) { return m == SM_AUTO ? "AUTO" : "MANUAL"; }

enum SwitchPos : uint8_t { SW_NONE, SW_MANUAL, SW_AUTO };  // NONE = never seen a valid pulse
inline const char *switchName(SwitchPos s) { return s == SW_AUTO ? "auto" : s == SW_MANUAL ? "manual" : "none"; }

struct ServoModeConfig {
  int MODE_CHANNEL = 2;          // index into the receiver channels: 2 = CH3
  uint16_t AUTO_ABOVE_US = 1700;  // switch pulse above this = auto
  uint16_t MANUAL_BELOW_US = 1300;  // below this = manual; in between keeps the last position
  uint16_t MIN_US = 900, MAX_US = 2100;  // a pulse outside this isn't a real servo command
};

class ServoModeControl {
 public:
  ServoModeConfig cfg;
  bool allowAuto = false;     // the saved dashboard setting
  ServoMode mode = SM_MANUAL;
  SwitchPos sw = SW_NONE;     // last valid switch position
  bool link = false;          // mode channel currently has pulses
  uint16_t out[2] = {0, 0};   // what to send each servo, in us; 0 = no pulses (stopped)

  // rc[]: the receiver channels in us (0 = no pulse lately). autoCmd[]: what auto mode wants.
  // Returns true when the mode changed.
  bool update(const uint16_t rc[4], const uint16_t autoCmd[2]) {
    uint16_t m = rc[cfg.MODE_CHANNEL];
    link = valid(m);
    if (link) {
      if (m >= cfg.AUTO_ABOVE_US) sw = SW_AUTO;
      else if (m <= cfg.MANUAL_BELOW_US) sw = SW_MANUAL;
      else if (sw == SW_NONE) sw = SW_MANUAL;  // first sight of a mid position: be safe
    }
    ServoMode before = mode;
    mode = (allowAuto && sw == SW_AUTO) ? SM_AUTO : SM_MANUAL;
    for (int i = 0; i < 2; i++) {
      if (mode == SM_AUTO) out[i] = valid(autoCmd[i]) ? autoCmd[i] : 0;
      else out[i] = valid(rc[i]) ? rc[i] : 0;  // servo 1 follows CH1, servo 2 follows CH2
    }
    return mode != before;
  }

 private:
  bool valid(uint16_t us) const { return us >= cfg.MIN_US && us <= cfg.MAX_US; }
};
