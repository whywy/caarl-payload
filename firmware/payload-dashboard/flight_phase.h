// Flight phase detector for the CAARL payload. LOG-ONLY: it decides which part of the flight
// we're in and reports changes, but it never moves a servo.
//
// Plain C++ with no Arduino code, so the same file runs in the firmware and in the desk
// simulation (tests/phase_sim.cpp). Feed it one sample per reading with update().
//
//   PAD      waiting on the ground (keeps learning the ground-level altitude)
//   BOOST    motor burning: acceleration > LAUNCH_G for LAUNCH_SAMPLES readings in a row
//            (or, as a backup, the barometer says we're LAUNCH_ALT_M up and climbing fast)
//   COAST    motor out (acceleration < BURNOUT_G), still climbing
//   DESCENT  past apogee: filtered altitude has dropped APOGEE_DROP_M below its peak
//   CHUTE    chute open: an opening shock (> SHOCK_G), or a steady CHUTE_MIN..MAX_MPS descent
//            for CHUTE_STEADY_MS
//   SETTLED  CHUTE + SETTLE_MS for the canopy to stop swinging (steering would start here)
//   LANDED   altitude hasn't changed for LANDED_MS near the ground
//
// Altitude and vertical speed come from an alpha-beta filter on the barometer, which smooths
// the noise and gives vertical speed without integrating the (drifting) accelerometer.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct PhaseConfig {
  float LAUNCH_G = 3.0f;          int LAUNCH_SAMPLES = 3;      // 3 readings = 0.15 s at 20 Hz
  float LAUNCH_ALT_M = 30.0f;     float LAUNCH_VZ_MPS = 15.0f; // barometer backup for launch
  float BURNOUT_G = 1.5f;         int BURNOUT_SAMPLES = 3;     // drag alone reads ~1 g at high speed
  uint32_t MAX_BOOST_MS = 8000;                                // give up waiting for burnout
  uint32_t MIN_APOGEE_MS = 4000;  float APOGEE_DROP_M = 3.0f;  // no apogee in the first 4 s
  float SHOCK_G = 3.0f;
  float CHUTE_MIN_MPS = 2.0f, CHUTE_MAX_MPS = 15.0f;  uint32_t CHUTE_STEADY_MS = 2000;
  uint32_t SETTLE_MS = 3000;
  float LANDED_MAX_AGL_M = 50.0f, LANDED_BAND_M = 1.5f;  uint32_t LANDED_MS = 5000;
  float FILTER_ALPHA = 0.25f, FILTER_BETA = 0.03f;            // alpha-beta filter gains
};

enum Phase : uint8_t { PH_PAD, PH_BOOST, PH_COAST, PH_DESCENT, PH_CHUTE, PH_SETTLED, PH_LANDED };
inline const char *phaseName(Phase p) {
  static const char *n[] = {"PAD", "BOOST", "COAST", "DESCENT", "CHUTE", "SETTLED", "LANDED"};
  return n[p];
}

class PhaseDetector {
 public:
  PhaseConfig cfg;
  Phase phase = PH_PAD;
  float alt = 0, vz = 0;          // filtered altitude (m, barometer scale) and vertical speed (m/s)
  float groundAlt = 0, maxAlt = -1e9f;
  char note[160] = "";            // text describing the last change; empty if none this sample

  float agl() const { return alt - groundAlt; }

  // t_ms: time of the reading. aMag: total acceleration in g (sqrt(ax²+ay²+az²)).
  // baroOk/baroAlt: barometer altitude in m. Returns true when the phase changed.
  bool update(uint32_t t_ms, bool imuOk, float aMag, bool baroOk, float baroAlt) {
    note[0] = 0;
    if (baroOk) filterBaro(t_ms, baroAlt);
    if (!haveFilter) return false;
    Phase before = phase;

    switch (phase) {
      case PH_PAD:
        if (fabsf(vz) < 3.0f) groundAlt += (alt - groundAlt) * 0.02f;  // follow the weather, not a climb
        highG = (imuOk && aMag > cfg.LAUNCH_G) ? highG + 1 : 0;
        if (highG >= cfg.LAUNCH_SAMPLES) enter(PH_BOOST, t_ms, "launch (%.1f g)", aMag);
        else if (agl() > cfg.LAUNCH_ALT_M && vz > cfg.LAUNCH_VZ_MPS)
          enter(PH_BOOST, t_ms, "launch (barometer: %.0f m up at %.0f m/s)", agl(), vz);
        if (phase == PH_BOOST) launchMs = t_ms;
        break;

      case PH_BOOST:
        lowG = (imuOk && aMag < cfg.BURNOUT_G) ? lowG + 1 : 0;
        if (lowG >= cfg.BURNOUT_SAMPLES) enter(PH_COAST, t_ms, "burnout at %.0f m AGL, %.0f m/s up", agl(), vz);
        else if (t_ms - launchMs > cfg.MAX_BOOST_MS) enter(PH_COAST, t_ms, "burnout assumed (%.0f s)", cfg.MAX_BOOST_MS / 1000.0);
        trackApogee(t_ms);  // a short burn can reach apogee before burnout is recognised
        break;

      case PH_COAST:
        // The ejection charge fires at apogee, so the opening shock can arrive just before the
        // barometer has confirmed apogee. Remember it and count it once apogee is confirmed.
        if (imuOk && aMag > cfg.SHOCK_G && t_ms - launchMs >= cfg.MIN_APOGEE_MS) { shockMs = t_ms; shockG = aMag; }
        trackApogee(t_ms);
        if (phase == PH_DESCENT && shockMs && t_ms - shockMs <= 3000)
          enter(PH_CHUTE, t_ms, "chute open: opening shock %.1f g just before apogee was confirmed", shockG);
        break;

      case PH_DESCENT:
        if (imuOk && aMag > cfg.SHOCK_G) { enter(PH_CHUTE, t_ms, "chute open: opening shock %.1f g at %.0f m AGL", aMag, agl()); break; }
        steadyCheck(t_ms);
        break;

      case PH_CHUTE:
        if (t_ms - phaseMs >= cfg.SETTLE_MS) enter(PH_SETTLED, t_ms, "canopy settled at %.0f m AGL, %.1f m/s down", agl(), -vz);
        break;

      case PH_SETTLED:
        landedCheck(t_ms);
        break;

      case PH_LANDED:
        break;
    }
    // A ballistic or chute-less descent can still land: watch for that from DESCENT too.
    if (phase == PH_DESCENT) landedCheck(t_ms);
    return phase != before;
  }

 private:
  bool haveFilter = false;
  uint32_t lastT = 0, launchMs = 0, phaseMs = 0, steadySince = 0, stillSince = 0, shockMs = 0;
  int highG = 0, lowG = 0;
  float stillRef = 0, shockG = 0;

  void filterBaro(uint32_t t_ms, float z) {
    if (!haveFilter) { alt = groundAlt = z; vz = 0; lastT = t_ms; haveFilter = true; return; }
    float dt = (t_ms - lastT) / 1000.0f;
    lastT = t_ms;
    if (dt <= 0 || dt > 1.0f) { alt = z; return; }  // gap in the data: restart from the reading
    alt += vz * dt;
    float r = z - alt;
    alt += cfg.FILTER_ALPHA * r;
    vz += cfg.FILTER_BETA * r / dt;
  }

  void trackApogee(uint32_t t_ms) {
    if (alt > maxAlt) maxAlt = alt;
    if (t_ms - launchMs >= cfg.MIN_APOGEE_MS && maxAlt - alt >= cfg.APOGEE_DROP_M)
      enter(PH_DESCENT, t_ms, "apogee %.1f m AGL", maxAlt - groundAlt);
  }

  void steadyCheck(uint32_t t_ms) {
    float down = -vz;
    if (down >= cfg.CHUTE_MIN_MPS && down <= cfg.CHUTE_MAX_MPS) {
      if (!steadySince) steadySince = t_ms;
      if (t_ms - steadySince >= cfg.CHUTE_STEADY_MS)
        enter(PH_CHUTE, t_ms, "chute open: steady %.1f m/s descent at %.0f m AGL", down, agl());
    } else {
      steadySince = 0;
    }
  }

  void landedCheck(uint32_t t_ms) {
    if (agl() > cfg.LANDED_MAX_AGL_M) { stillSince = 0; return; }
    if (!stillSince || fabsf(alt - stillRef) > cfg.LANDED_BAND_M) { stillSince = t_ms; stillRef = alt; return; }
    if (t_ms - stillSince >= cfg.LANDED_MS) enter(PH_LANDED, t_ms, "landed (%.0f m AGL by barometer)", agl());
  }

  template <typename... A>
  void enter(Phase p, uint32_t t_ms, const char *fmt, A... args) {
    phase = p;
    phaseMs = t_ms;
    steadySince = 0;
    // Two changes in one reading (apogee confirmed and chute already open) keep both notes.
    int n = (int)strlen(note);
    if (n > 0 && n < (int)sizeof note - 4) { strcpy(note + n, " | "); n += 3; }
    n += snprintf(note + n, sizeof note - n, "PHASE %s: ", phaseName(p));
    if (n < (int)sizeof note) snprintf(note + n, sizeof note - n, fmt, args...);
    for (char *c = note; *c; c++) if (*c == ',' || *c == '"') *c = ' ';  // keep the CSV valid
  }
};
