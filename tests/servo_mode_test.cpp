// Checks every rule in firmware/payload-dashboard/servo_mode.h. No hardware needed.
//   g++ -std=c++17 -Wall -o /tmp/servo_mode_test tests/servo_mode_test.cpp && /tmp/servo_mode_test
// Exit code 0 = every check passed.
#include "../firmware/payload-dashboard/servo_mode.h"
#include <stdio.h>

static int failures = 0;
static void check(bool ok, const char *what) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what); if (!ok) failures++; }

int main() {
  const uint16_t STOP[2] = {0, 0};  // today's auto command: steering not written yet
  const uint16_t AUTO_PULL[2] = {1600, 1400};  // a made-up auto command to prove auto output is used

  printf("Manual only (the default setting)\n");
  {
    ServoModeControl s;
    uint16_t rc[4] = {1600, 1400, 2000, 1500};  // sticks moved, switch on AUTO
    s.update(rc, AUTO_PULL);
    check(s.mode == SM_MANUAL, "switch on auto is ignored when auto isn't allowed");
    check(s.out[0] == 1600 && s.out[1] == 1400, "servos copy CH1 and CH2");
  }

  printf("No transmitter / no signal at power-up\n");
  {
    ServoModeControl s; s.allowAuto = true;
    uint16_t rc[4] = {0, 0, 0, 0};
    s.update(rc, AUTO_PULL);
    check(s.mode == SM_MANUAL && s.sw == SW_NONE, "stays MANUAL before any switch position is seen");
    check(s.out[0] == 0 && s.out[1] == 0, "no pulses to the servos (stopped)");
  }

  printf("Auto allowed\n");
  {
    ServoModeControl s; s.allowAuto = true;
    uint16_t rc[4] = {1500, 1500, 1000, 1500};  // switch manual
    s.update(rc, AUTO_PULL);
    check(s.mode == SM_MANUAL, "switch on manual = MANUAL");
    rc[2] = 2000;
    bool changed = s.update(rc, AUTO_PULL);
    check(changed && s.mode == SM_AUTO, "switch on auto = AUTO, and update() reports the change");
    check(s.out[0] == 1600 && s.out[1] == 1400, "AUTO sends the payload's own command, not the sticks");
    s.update(rc, STOP);
    check(s.out[0] == 0 && s.out[1] == 0, "with no steering yet, AUTO means stopped");
    rc[2] = 1000;
    s.update(rc, AUTO_PULL);
    check(s.mode == SM_MANUAL && s.out[0] == 1500, "pilot flips back to manual: sticks take over at once");
  }

  printf("Radio link lost\n");
  {
    ServoModeControl s; s.allowAuto = true;
    uint16_t rc[4] = {1500, 1500, 2000, 1500};
    s.update(rc, AUTO_PULL);
    uint16_t lost[4] = {0, 0, 0, 0};
    s.update(lost, AUTO_PULL);
    check(s.mode == SM_AUTO && !s.link, "in AUTO, losing the link keeps AUTO (out of range under canopy)");
    ServoModeControl m; m.allowAuto = true;
    uint16_t man[4] = {1600, 1400, 1000, 1500};
    m.update(man, AUTO_PULL);
    m.update(lost, AUTO_PULL);
    check(m.mode == SM_MANUAL && m.out[0] == 0 && m.out[1] == 0, "in MANUAL, losing the link stops the servos");
  }

  printf("Switch noise and odd pulses\n");
  {
    ServoModeControl s; s.allowAuto = true;
    uint16_t rc[4] = {1500, 1500, 2000, 1500};
    s.update(rc, AUTO_PULL);
    rc[2] = 1500;  // switch passing through the middle
    s.update(rc, AUTO_PULL);
    check(s.mode == SM_AUTO, "a mid-position pulse keeps the last mode (no flicker)");
    ServoModeControl f; f.allowAuto = true;
    uint16_t mid[4] = {1500, 1500, 1500, 1500};
    f.update(mid, AUTO_PULL);
    check(f.mode == SM_MANUAL && f.sw == SW_MANUAL, "first sight of a mid position counts as manual");
    uint16_t junk[4] = {3000, 500, 2000, 1500};
    ServoModeControl j;
    j.update(junk, AUTO_PULL);
    check(j.out[0] == 0 && j.out[1] == 0, "out-of-range stick pulses become 'stopped', not passed on");
  }

  printf("Turning auto off while in AUTO\n");
  {
    ServoModeControl s; s.allowAuto = true;
    uint16_t rc[4] = {1550, 1450, 2000, 1500};
    s.update(rc, AUTO_PULL);
    s.allowAuto = false;
    s.update(rc, AUTO_PULL);
    check(s.mode == SM_MANUAL && s.out[0] == 1550, "setting 'Manual only' drops straight back to MANUAL");
  }

  printf("\n%s (%d failure%s)\n", failures ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
