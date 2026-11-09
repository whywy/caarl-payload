// Replay a recorded flight log through the flight phase detector, with whatever settings you like.
// Uses the exact same firmware/payload-dashboard/flight_phase.h as the payload, so what it prints is
// what the payload would have decided with those settings.
//
// Build once:
//   g++ -std=c++17 -O2 -Wall -Wno-format-security -o /tmp/phase_replay tools/phase_replay.cpp
// Use:
//   /tmp/phase_replay downloads/LOG012.CSV
//   /tmp/phase_replay downloads/LOG012.CSV APOGEE_DROP_M=2 SHOCK_G=2.5      (try other settings)
//   /tmp/phase_replay downloads/LOG012.CSV --out /tmp/replayed.csv         (save phase + vz per row)
//   /tmp/phase_replay --settings                                            (list settings and defaults)
//
// It prints: the settings used, every phase change the replay finds, the phase changes the payload
// itself recorded (if the log has a "phase" column), and a short flight summary.
// Exit code 0 = ran fine, 2 = bad arguments or unreadable file.
#include "../firmware/payload-dashboard/flight_phase.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Setting { const char *name; float *f; int *i; uint32_t *u; const char *what; };

std::vector<Setting> settingsOf(PhaseConfig &c) {
  return {
      {"LAUNCH_G", &c.LAUNCH_G, nullptr, nullptr, "g, launch threshold"},
      {"LAUNCH_SAMPLES", nullptr, &c.LAUNCH_SAMPLES, nullptr, "readings in a row over LAUNCH_G"},
      {"LAUNCH_ALT_M", &c.LAUNCH_ALT_M, nullptr, nullptr, "m AGL, barometer launch backup"},
      {"LAUNCH_VZ_MPS", &c.LAUNCH_VZ_MPS, nullptr, nullptr, "m/s up, barometer launch backup"},
      {"BURNOUT_G", &c.BURNOUT_G, nullptr, nullptr, "g, motor-out threshold"},
      {"BURNOUT_SAMPLES", nullptr, &c.BURNOUT_SAMPLES, nullptr, "readings in a row under BURNOUT_G"},
      {"MAX_BOOST_MS", nullptr, nullptr, &c.MAX_BOOST_MS, "ms, give up waiting for burnout"},
      {"MIN_APOGEE_MS", nullptr, nullptr, &c.MIN_APOGEE_MS, "ms after launch before apogee may be called"},
      {"APOGEE_DROP_M", &c.APOGEE_DROP_M, nullptr, nullptr, "m below the peak to call apogee"},
      {"SHOCK_G", &c.SHOCK_G, nullptr, nullptr, "g, opening shock threshold"},
      {"CHUTE_MIN_MPS", &c.CHUTE_MIN_MPS, nullptr, nullptr, "m/s down, steady-descent window low end"},
      {"CHUTE_MAX_MPS", &c.CHUTE_MAX_MPS, nullptr, nullptr, "m/s down, steady-descent window high end"},
      {"CHUTE_STEADY_MS", nullptr, nullptr, &c.CHUTE_STEADY_MS, "ms of steady descent to call chute open"},
      {"SETTLE_MS", nullptr, nullptr, &c.SETTLE_MS, "ms after chute open before SETTLED"},
      {"LANDED_MAX_AGL_M", &c.LANDED_MAX_AGL_M, nullptr, nullptr, "m AGL, landing only counts below this"},
      {"LANDED_BAND_M", &c.LANDED_BAND_M, nullptr, nullptr, "m, altitude must stay within this"},
      {"LANDED_MS", nullptr, nullptr, &c.LANDED_MS, "ms, for this long, to call landed"},
      {"FILTER_ALPHA", &c.FILTER_ALPHA, nullptr, nullptr, "altitude filter gain (0-1)"},
      {"FILTER_BETA", &c.FILTER_BETA, nullptr, nullptr, "vertical-speed filter gain"},
  };
}

std::string show(const Setting &s) {
  char b[32];
  if (s.f) snprintf(b, sizeof b, "%g", *s.f);
  else if (s.i) snprintf(b, sizeof b, "%d", *s.i);
  else snprintf(b, sizeof b, "%u", *s.u);
  return b;
}

std::vector<std::string> splitCsv(const std::string &line) {
  std::vector<std::string> out;
  std::string cell;
  std::stringstream ss(line);
  while (std::getline(ss, cell, ',')) out.push_back(cell);
  if (!line.empty() && line.back() == ',') out.push_back("");
  return out;
}

bool num(const std::vector<std::string> &row, int col, double *v) {
  if (col < 0 || col >= (int)row.size() || row[col].empty()) return false;
  char *end = nullptr;
  *v = strtod(row[col].c_str(), &end);
  return end && end != row[col].c_str() && std::isfinite(*v);
}

int usage() {
  std::cerr << "usage: phase_replay <log.csv> [SETTING=value ...] [--out replayed.csv]\n"
               "       phase_replay --settings\n";
  return 2;
}

}  // namespace

int main(int argc, char **argv) {
  PhaseDetector det;
  auto settings = settingsOf(det.cfg);

  if (argc >= 2 && std::string(argv[1]) == "--settings") {
    printf("%-18s %-8s %s\n", "SETTING", "DEFAULT", "MEANING");
    for (auto &s : settings) printf("%-18s %-8s %s\n", s.name, show(s).c_str(), s.what);
    return 0;
  }
  if (argc < 2) return usage();

  std::string path = argv[1], outPath;
  std::vector<std::string> changed;
  for (int a = 2; a < argc; a++) {
    std::string arg = argv[a];
    if (arg == "--out" && a + 1 < argc) { outPath = argv[++a]; continue; }
    auto eq = arg.find('=');
    if (eq == std::string::npos) { std::cerr << "not understood: " << arg << "\n"; return usage(); }
    std::string key = arg.substr(0, eq), val = arg.substr(eq + 1);
    bool found = false;
    for (auto &s : settings) {
      if (key != s.name) continue;
      found = true;
      if (s.f) *s.f = strtof(val.c_str(), nullptr);
      else if (s.i) *s.i = atoi(val.c_str());
      else *s.u = (uint32_t)strtoul(val.c_str(), nullptr, 10);
      changed.push_back(key);
    }
    if (!found) { std::cerr << "unknown setting: " << key << " (run with --settings to list them)\n"; return 2; }
  }

  std::ifstream in(path);
  if (!in) { std::cerr << "can't open " << path << "\n"; return 2; }
  std::string line;
  if (!std::getline(in, line)) { std::cerr << path << " is empty\n"; return 2; }
  if (!line.empty() && line.back() == '\r') line.pop_back();
  auto head = splitCsv(line);
  auto col = [&](const char *n) { for (size_t i = 0; i < head.size(); i++) if (head[i] == n) return (int)i; return -1; };
  int cT = col("t_ms"), cX = col("ax_g"), cY = col("ay_g"), cZ = col("az_g"), cAlt = col("baro_alt_m");
  int cPhase = col("phase"), cEvent = col("event");
  if (cT < 0 || cAlt < 0) { std::cerr << path << " has no t_ms or baro_alt_m column: not a payload log?\n"; return 2; }

  std::ofstream out;
  if (!outPath.empty()) {
    out.open(outPath);
    if (!out) { std::cerr << "can't write " << outPath << "\n"; return 2; }
    out << "t_ms,a_g,baro_alt_m,replay_alt_agl_m,replay_vz_mps,replay_phase,recorded_phase\n";
  }

  printf("Log: %s\nSettings:", path.c_str());
  for (auto &s : settings) {
    bool mine = false;
    for (auto &c : changed) mine |= c == s.name;
    if (mine) printf(" %s=%s", s.name, show(s).c_str());
  }
  printf("%s\n\n", changed.empty() ? " all defaults" : "  (everything else default)");

  struct Ev { double t; std::string text; };
  std::vector<Ev> replayed, recorded;
  double t0 = -1, lastT = 0, maxAgl = -1e9, maxA = 0, tMaxA = 0;
  long rows = 0, badRows = 0;
  std::string lastRecorded;
  double chuteSum = 0; long chuteN = 0;

  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    auto row = splitCsv(line);
    double t, ax, ay, az, alt;
    if (!num(row, cT, &t)) { badRows++; continue; }
    if (t0 < 0) t0 = t;
    lastT = t;
    rows++;
    bool imuOk = num(row, cX, &ax) && num(row, cY, &ay) && num(row, cZ, &az);
    double aMag = imuOk ? std::sqrt(ax * ax + ay * ay + az * az) : 0;
    bool baroOk = num(row, cAlt, &alt);

    if (det.update((uint32_t)t, imuOk, (float)aMag, baroOk, (float)alt))
      replayed.push_back({(t - t0) / 1000.0, det.note});

    if (imuOk && aMag > maxA) { maxA = aMag; tMaxA = (t - t0) / 1000.0; }
    if (det.phase != PH_PAD && det.agl() > maxAgl) maxAgl = det.agl();
    if (det.phase == PH_SETTLED && -det.vz > 0.5f) { chuteSum += -det.vz; chuteN++; }  // skip time already on the ground

    std::string rec = (cPhase >= 0 && cPhase < (int)row.size()) ? row[cPhase] : "";
    if (!rec.empty() && rec != lastRecorded) {
      std::string ev = (cEvent >= 0 && cEvent < (int)row.size()) ? row[cEvent] : "";
      auto p = ev.find("PHASE ");
      if (!lastRecorded.empty()) recorded.push_back({(t - t0) / 1000.0, p != std::string::npos ? ev.substr(p) : rec});
      lastRecorded = rec;
    }
    if (out) {
      char b[200];
      snprintf(b, sizeof b, "%.0f,%.4f,%s,%.2f,%.2f,%s,%s\n", t, aMag, baroOk ? std::to_string(alt).c_str() : "",
               det.agl(), det.vz, phaseName(det.phase), rec.c_str());
      out << b;
    }
  }

  printf("Replayed with these settings (seconds from the start of the log):\n");
  if (replayed.empty()) printf("  (no phase changes: stayed in PAD)\n");
  for (auto &e : replayed) printf("  %8.2f s  %s\n", e.t, e.text.c_str());

  if (cPhase >= 0) {
    printf("\nRecorded by the payload during the flight:\n");
    if (recorded.empty()) printf("  (no phase changes: stayed in PAD)\n");
    for (auto &e : recorded) printf("  %8.2f s  %s\n", e.t, e.text.c_str());
  } else {
    printf("\n(This log has no phase column: it was recorded before firmware 1.5.)\n");
  }

  printf("\nSummary: %ld rows over %.1f s", rows, rows ? (lastT - t0) / 1000.0 : 0.0);
  if (badRows) printf(", %ld unreadable rows skipped", badRows);
  if (maxAgl > -1e8) printf("\n  highest point after launch: %.1f m above the pad\n", maxAgl);
  else printf("\n  highest point after launch: n/a (no launch detected)\n");
  printf("  biggest acceleration: %.2f g at %.2f s\n", maxA, tMaxA);
  if (chuteN) printf("  average descent rate while SETTLED: %.1f m/s\n", chuteSum / chuteN);
  if (!outPath.empty()) printf("  per-row replay written to %s\n", outPath.c_str());
  return 0;
}
