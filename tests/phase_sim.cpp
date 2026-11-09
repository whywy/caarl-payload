// Desk simulation for firmware/payload-dashboard/flight_phase.h.
// Runs the real detector against simple made-up flights (with sensor noise) and checks
// which phases it reports, and when. No hardware needed.
//
//   g++ -std=c++17 -O1 -Wall -o /tmp/phase_sim tests/phase_sim.cpp && /tmp/phase_sim
//
// Exit code 0 = every scenario passed.
#include "../firmware/payload-dashboard/flight_phase.h"
#include <random>
#include <string>
#include <vector>

struct Event { uint32_t t; Phase p; std::string note; };

struct Flight {
  const char *name;
  float boostG = 8.0f, burnS = 1.5f;       // net upward acceleration during the burn, burn time
  float dragK = 0.0015f;                    // drag deceleration = dragK * v^2 (m/s^2)
  bool fly = true;                          // false = stays on the pad
  float padBumpG = 0;                       // one-sample spike while on the pad (someone bumps it)
  float shockG = 6.0f;                      // opening shock size (0 = soft opening)
  float shockAfterApogeeS = 0.5f;           // when the shock happens after true apogee
  float chuteMps = 6.0f;                    // descent speed under canopy (0 = chute fails)
  float baroNoiseM = 0.4f, accNoiseG = 0.03f;
};

static std::vector<Event> run(const Flight &f, float *trueApogee, uint32_t *trueApogeeMs, uint32_t *landMs) {
  std::mt19937 rng(1234);
  std::normal_distribution<float> nb(0, f.baroNoiseM), na(0, f.accNoiseG);
  PhaseDetector det;
  std::vector<Event> ev;
  const float dt = 0.05f, g = 9.81f, ground = 120.0f;  // 20 Hz; launch site 120 m above sea level
  float h = 0, v = 0, t = 0, apex = 0;
  bool burning = false, pastApogee = false, chuteOpen = false, landed = false;
  float launchT = 10.0f, apogeeT = -1, shockT = -1;
  *trueApogee = 0; *trueApogeeMs = 0; *landMs = 0;

  for (int i = 0; i < 20 * 400; i++, t += dt) {
    float acc = 0, specific = 1.0f;  // net vertical acceleration (m/s^2), accelerometer reading (g)
    if (f.fly && t >= launchT && !landed) {
      burning = t < launchT + f.burnS;
      float drag = f.dragK * v * fabsf(v);
      if (burning) { acc = f.boostG * g - drag; specific = (acc + g) / g; }
      else if (!chuteOpen) { acc = -g - drag; specific = -drag / g; }
      else { acc = (-f.chuteMps - v) * 2.0f; specific = (acc + g) / g; }  // canopy pulls v to -chuteMps
      v += acc * dt; h += v * dt;
      if (h > apex) apex = h;
      if (!pastApogee && t > launchT + f.burnS && v < 0) { pastApogee = true; apogeeT = t; *trueApogee = apex; *trueApogeeMs = (uint32_t)(t * 1000); shockT = t + f.shockAfterApogeeS; }
      if (pastApogee && !chuteOpen && f.chuteMps > 0 && t >= shockT) { chuteOpen = true; if (f.shockG > 0) specific = f.shockG; }
      if (h <= 0 && t > launchT + 1) { h = 0; v = 0; landed = true; *landMs = (uint32_t)(t * 1000); }
    }
    if (landed) specific = 1.0f;
    if (!f.fly && f.padBumpG > 0 && i == 20 * 5) specific = f.padBumpG;
    float aMag = fabsf(specific + na(rng));
    float baro = ground + h + nb(rng);
    uint32_t ms = (uint32_t)(t * 1000);
    if (det.update(ms, true, aMag, true, baro)) ev.push_back({ms, det.phase, det.note});
  }
  (void)apogeeT;
  return ev;
}

static int failures = 0;
static void check(bool ok, const char *what) { printf("    [%s] %s\n", ok ? "PASS" : "FAIL", what); if (!ok) failures++; }
// A phase counts as reported if the detector entered it, even if it moved on in the same reading.
static const Event *find(const std::vector<Event> &e, Phase p) {
  std::string tag = std::string("PHASE ") + phaseName(p) + ":";
  for (auto &x : e) if (x.p == p || x.note.find(tag) != std::string::npos) return &x;
  return nullptr;
}

int main() {
  Flight nominal{"nominal flight, opening shock 0.5 s after apogee"};
  Flight soft{"soft opening (no shock), 6 m/s under canopy"}; soft.shockG = 0;
  Flight lateShock{"shock 1.5 s after apogee"}; lateShock.shockAfterApogeeS = 1.5f;
  Flight noChute{"chute never opens"}; noChute.chuteMps = 0;
  Flight bump{"sitting on the pad, bumped once (5 g spike)"}; bump.fly = false; bump.padBumpG = 5.0f;
  Flight noisy{"nominal, double sensor noise"}; noisy.baroNoiseM = 0.8f; noisy.accNoiseG = 0.06f;

  for (Flight *f : {&nominal, &soft, &lateShock, &noChute, &bump, &noisy}) {
    float apo; uint32_t apoMs, landMs;
    auto ev = run(*f, &apo, &apoMs, &landMs);
    printf("\n== %s  (true apogee %.0f m at %.1f s)\n", f->name, apo, apoMs / 1000.0);
    for (auto &e : ev) printf("  %6.2f s  %s\n", e.t / 1000.0, e.note.c_str());
    const Event *L = find(ev, PH_BOOST), *A = find(ev, PH_DESCENT), *C = find(ev, PH_CHUTE), *S = find(ev, PH_SETTLED), *G = find(ev, PH_LANDED);
    if (!f->fly) { check(ev.empty(), "no phase change on the pad"); continue; }
    check(L && L->t >= 10000 && L->t < 10300, "launch within 0.3 s");
    check(A && A->t >= apoMs && A->t < apoMs + 3000, "apogee detected within 3 s after the true apogee (never before)");
    if (f->chuteMps > 0) {
      check(C != nullptr, "chute-open detected");
      check(S != nullptr, "settled reported");
      check(G != nullptr, "landing detected");
    } else {
      check(C == nullptr, "no false chute-open when the chute fails");
    }
  }
  printf("\n%s (%d failure%s)\n", failures ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
