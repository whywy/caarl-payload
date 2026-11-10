// CAARL payload v2: dashboard firmware
// Sends every sensor reading to the PC as one line of JSON, and records
// readings to the SD card. Pair it with dashboard.py.
//
// Recording starts by itself at power-up (AUTO_LOG below) into a new /LOGnnn.CSV,
// so it works with no PC attached (rocket, battery or Flipper power). The PC can
// still stop and start recordings with LOG STOP / LOG START. If the card isn't
// ready at power-up it keeps trying for 30 s. An LED on GPIO13 shows the state
// (see LED_PIN below).
//
// Board: "ESP32 Dev Module". Serial: 921600 baud (the Serial Monitor must be set
// to 921600 too, or it shows garbage).
// Pins: same as perfboard/WIRING.md.
//
// Commands it understands (one per line, sent by dashboard.py):
//   INFO            which parts were found, SD card size
//   LOG START       start recording to a new /LOGnnn.CSV
//   LOG STOP        stop recording
//   MARK <text>     write <text> into the "event" column of the next row
//   LS              list the files on the card
//   GET /<file>     send a file to the PC (base64, in chunks)
//   DEL /<file>     delete a file
//   CAL GYRO        measure the gyro's offset (keep the board still for 1 s)
//   SERVO MODE MANUAL | SERVO MODE AUTO
//                   who may drive the parachute servos (saved on the payload, see servo_mode.h).
//                   MANUAL (default): the servos always follow the ER4 sticks (CH1, CH2).
//                   AUTO: the transmitter switch on CH3 chooses manual or auto in flight.
//
// Servo signals come FROM the ESP32 (GPIO25 -> servo 1, GPIO26 -> servo 2), not straight from
// the ER4: see perfboard/WIRING.md "Servo control through the ESP32". In manual mode the ESP32
// copies the ER4's CH1/CH2 through; if it resets, the servos get no pulses and stop.
//
// The gyro offset is also measured automatically at power-up (AUTO_CAL_GYRO), but only if
// the board is still; the first row of each recording says whether it was applied.

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP3XX.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Preferences.h>
#include "flight_phase.h"  // log-only flight phase detection (never moves a servo)
#include "servo_mode.h"    // manual (ER4) / auto servo control

const char *FW_VERSION = "1.6";
const bool AUTO_LOG = true;      // start recording at power-up; set false for PC-only recording
const bool AUTO_CAL_GYRO = true; // measure the gyro offset at power-up if the board is still
const float STILL_GYRO_DPS = 2.0f, STILL_ACC_G = 0.05f;  // max swing during that 1 s to count as "still"
const uint32_t SD_RETRY_MS = 2000, SD_RETRY_FOR_MS = 30000;  // keep looking for the card for 30 s
// Status LED. The Inland ESP32 board has only a power LED, so this is an external LED on the
// perfboard: GPIO13 (hole V31) -> 330 ohm -> LED (red/green/yellow) -> GND (hole U31). ~3-5 mA.
//   short flash every second = recording   fast blink = looking for the SD card
//   solid on = problem (no SD card found, or a write failed)   off = not recording (stopped from the PC)
const int LED_PIN = 13;
const uint32_t BAUD = 921600;
const int SDA_PIN = 21, SCL_PIN = 22, SD_CS = 5;
const int RC_PINS[4] = {36, 39, 34, 35};
const int SERVO_PINS[2] = {25, 26};  // ESP32 -> servo signal pins (perfboard D25, D28)
const int SERVO_HZ = 50, SERVO_BITS = 14;
const uint32_t SAMPLE_MS = 50;   // 20 readings a second, all of them recorded
const int STREAM_EVERY = 2;      // every 2nd reading goes to the PC (10 a second)
const uint32_t FLUSH_MS = 1000;  // save the recording to the card every second

// ---------- IMU: MPU-6050 clone, read through its registers ----------
const uint8_t IMU_ADDR = 0x68;
uint8_t imuId = 0;
// Flight ranges: +-16 g and +-2000 deg/s, so the boost doesn't saturate the readings.
// The scale used for the maths is worked out from what the chip reports back, so the
// numbers stay right even if a clone ignores the setting.
const uint8_t IMU_ACC_FS = 3, IMU_GYRO_FS = 3;  // 0..3 = +-2/4/8/16 g and +-250/500/1000/2000 deg/s
// On-chip low-pass filter (DLPF_CFG 4 = about 20 Hz). We sample 20 times a second, so without it
// motor vibration aliases into random jumps in the log, and the stillness check sees only noise.
const uint8_t IMU_DLPF = 4;
float accLsbPerG = 16384.0f, gyroLsbPerDps = 131.0f;
int accRangeG = 2, gyroRangeDps = 250;
uint8_t imuDlpf = 0;  // filter setting the chip reports back
float gyroBias[3] = {0, 0, 0};

uint8_t imuRead8(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(IMU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}
bool imuBegin() {
  Wire.beginTransmission(IMU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  imuId = imuRead8(0x75);
  Wire.beginTransmission(IMU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();  // wake up
  delay(100);
  Wire.beginTransmission(IMU_ADDR); Wire.write(0x1A); Wire.write(IMU_DLPF); Wire.endTransmission();
  Wire.beginTransmission(IMU_ADDR); Wire.write(0x1B); Wire.write(IMU_GYRO_FS << 3); Wire.endTransmission();
  Wire.beginTransmission(IMU_ADDR); Wire.write(0x1C); Wire.write(IMU_ACC_FS << 3); Wire.endTransmission();
  delay(10);
  uint8_t gfs = (imuRead8(0x1B) >> 3) & 3, afs = (imuRead8(0x1C) >> 3) & 3;  // read back what it accepted
  gyroLsbPerDps = 131.0f / (1 << gfs);  gyroRangeDps = 250 << gfs;
  accLsbPerG = 16384.0f / (1 << afs);   accRangeG = 2 << afs;
  imuDlpf = imuRead8(0x1A) & 7;
  return true;
}
bool imuReadRaw(int16_t v[7]) {  // ax ay az temp gx gy gz
  Wire.beginTransmission(IMU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_ADDR, (uint8_t)14) != 14) return false;
  for (int i = 0; i < 7; i++) { int16_t hi = Wire.read(); v[i] = (hi << 8) | Wire.read(); }
  return true;
}

// ---------- Receiver: measure pulse widths with interrupts ----------
// GPIO34-39 have no internal pull resistors, so with the ER4 unplugged the pins float and
// pick up noise that can look like a pulse. A real receiver sends a steady train: one
// 700-2400 us pulse every 2.5-30 ms (50-400 Hz). A channel is only reported after
// RC_GOOD_NEEDED such pulses in a row; any odd edge starts the count over.
constexpr uint8_t RC_GOOD_NEEDED = 3;
volatile uint32_t rcRise[4], rcPeriod[4], rcWidth[4], rcSeen[4];
volatile uint8_t rcGood[4];
void IRAM_ATTR rcIsr(void *arg) {
  int i = (int)(intptr_t)arg;
  uint32_t now = micros();
  if (digitalRead(RC_PINS[i])) {
    rcPeriod[i] = now - rcRise[i];  // time since the previous pulse started
    rcRise[i] = now;
  } else {
    uint32_t w = now - rcRise[i], p = rcPeriod[i];
    bool valid = w > 700 && w < 2400 && p >= 2500 && p <= 30000 && w < p;
    if (!valid) { rcGood[i] = 0; return; }
    if (rcGood[i] < RC_GOOD_NEEDED) rcGood[i] = rcGood[i] + 1;
    if (rcGood[i] >= RC_GOOD_NEEDED) { rcWidth[i] = w; rcSeen[i] = now; }
  }
}

// Latest pulse width on each receiver channel in us; 0 = no pulse in the last 0.1 s.
void readRc(uint16_t rc[4]) {
  uint32_t now = micros();
  for (int i = 0; i < 4; i++) {
    noInterrupts();
    uint32_t w = rcWidth[i], seen = rcSeen[i];
    interrupts();
    rc[i] = (seen && now - seen < 100000) ? w : 0;
  }
}

// ---------- State ----------
Adafruit_BMP3XX bmp;
SFE_UBLOX_GNSS gps;
bool haveImu = false, haveBaro = false, haveGps = false, haveSd = false;
int baroWarmup = 3;  // the first readings after power-up are wrong; skip them

struct Reading {
  uint32_t t;
  bool imuOk; float a[3], g[3], imuT;
  bool baroOk; float p, bT, alt;
  int fix, sv; double lat, lon; float gAlt, spd, hAcc; char utc[9];
  uint16_t rc[4];
} r = {};

File logFile;
bool logging = false;
char logName[16] = "";
uint32_t logRows = 0, logStartMs = 0, lastFlush = 0;
char pendingMark[256] = "";
PhaseDetector phaseDet;
ServoModeControl servoCtl;
Preferences prefs;  // settings kept in the ESP32's flash between power-ups
char logError[64] = "";
bool autoPending = false;  // auto-record still waiting for the SD card after power-up

// ---------- Helpers ----------
void sendOk(const char *cmd, const char *extra = "") {
  Serial.printf("{\"type\":\"ok\",\"cmd\":\"%s\"%s}\n", cmd, extra);
}
void sendErr(const char *cmd, const char *msg) {
  Serial.printf("{\"type\":\"err\",\"cmd\":\"%s\",\"msg\":\"%s\"}\n", cmd, msg);
}
bool safeName(const char *path) {  // "/NAME.EXT", letters, digits, _ - . only
  if (path[0] != '/' || strlen(path) < 2 || strlen(path) > 40) return false;
  for (const char *c = path + 1; *c; c++)
    if (!isalnum((unsigned char)*c) && *c != '_' && *c != '-' && *c != '.') return false;
  return true;
}

const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
void base64(const uint8_t *in, size_t n, char *out) {
  size_t o = 0;
  for (size_t i = 0; i < n; i += 3) {
    uint32_t v = in[i] << 16;
    if (i + 1 < n) v |= in[i + 1] << 8;
    if (i + 2 < n) v |= in[i + 2];
    out[o++] = B64[(v >> 18) & 63];
    out[o++] = B64[(v >> 12) & 63];
    out[o++] = i + 1 < n ? B64[(v >> 6) & 63] : '=';
    out[o++] = i + 2 < n ? B64[v & 63] : '=';
  }
  out[o] = 0;
}

void sendInfo() {
  if (!haveSd) haveSd = SD.begin(SD_CS);  // lets you insert a card after boot
  Serial.printf("{\"type\":\"info\",\"fw\":\"%s\",\"imu\":%s,\"imuId\":%u,\"baro\":%s,\"gps\":%s,\"sd\":%s,"
                "\"sdTotal\":%llu,\"sdUsed\":%llu,\"rate\":%lu,\"gyroBias\":[%.3f,%.3f,%.3f],"
                "\"accRangeG\":%d,\"gyroRangeDps\":%d,\"dlpf\":%u,\"servoAllow\":\"%s\"}\n",
                FW_VERSION, haveImu ? "true" : "false", imuId, haveBaro ? "true" : "false",
                haveGps ? "true" : "false", haveSd ? "true" : "false",
                haveSd ? SD.totalBytes() : 0ULL, haveSd ? SD.usedBytes() : 0ULL,
                (unsigned long)(1000 / SAMPLE_MS), gyroBias[0], gyroBias[1], gyroBias[2],
                accRangeG, gyroRangeDps, imuDlpf, servoCtl.allowAuto ? "auto" : "manual");
}

// ---------- SD recording ----------
const char *CSV_HEADER =
    "t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,imu_temp_c,pressure_pa,baro_temp_c,baro_alt_m,"
    "gps_fix,gps_sats,lat,lon,gps_alt_m,gps_speed_mps,gps_hacc_m,utc,rc1_us,rc2_us,rc3_us,rc4_us,phase,vz_mps,servo_mode,srv1_us,srv2_us,event";

void logStart() {
  if (logging) { sendErr("LOG START", "already recording"); return; }
  if (!haveSd) haveSd = SD.begin(SD_CS);
  if (!haveSd) { sendErr("LOG START", "no SD card found"); return; }
  for (int n = 1; n < 1000; n++) {
    snprintf(logName, sizeof logName, "/LOG%03d.CSV", n);
    if (!SD.exists(logName)) break;
  }
  logFile = SD.open(logName, FILE_WRITE);
  if (!logFile) { sendErr("LOG START", "could not create the file"); return; }
  logFile.println(CSV_HEADER);
  logging = true;
  logRows = 0;
  logStartMs = lastFlush = millis();
  logError[0] = 0;
  autoPending = false;
  char extra[40];
  snprintf(extra, sizeof extra, ",\"file\":\"%s\"", logName);
  sendOk("LOG START", extra);
}

void logStop(const char *reason = nullptr) {
  if (!reason) autoPending = false;  // a LOG STOP from the PC also cancels a pending auto-start
  if (!logging) { if (!reason) sendErr("LOG STOP", "not recording"); return; }
  logFile.close();
  logging = false;
  if (reason) {
    strncpy(logError, reason, sizeof logError - 1);
    sendErr("LOG", reason);
  } else {
    char extra[60];
    snprintf(extra, sizeof extra, ",\"file\":\"%s\",\"rows\":%lu", logName, (unsigned long)logRows);
    sendOk("LOG STOP", extra);
  }
}

void logRow() {
  char line[560];
  int n = snprintf(line, sizeof line, "%lu,", (unsigned long)r.t);
  if (r.imuOk)
    n += snprintf(line + n, sizeof line - n, "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,",
                  r.a[0], r.a[1], r.a[2], r.g[0], r.g[1], r.g[2], r.imuT);
  else n += snprintf(line + n, sizeof line - n, ",,,,,,,");
  if (r.baroOk) n += snprintf(line + n, sizeof line - n, "%.1f,%.2f,%.2f,", r.p, r.bT, r.alt);
  else n += snprintf(line + n, sizeof line - n, ",,,");
  n += snprintf(line + n, sizeof line - n, "%d,%d,%.7f,%.7f,%.2f,%.2f,%.2f,%s,%u,%u,%u,%u,%s,%.2f,%s,%u,%u,%s",
                r.fix, r.sv, r.lat, r.lon, r.gAlt, r.spd, r.hAcc, r.utc,
                r.rc[0], r.rc[1], r.rc[2], r.rc[3], phaseName(phaseDet.phase), phaseDet.vz,
                servoModeName(servoCtl.mode), servoCtl.out[0], servoCtl.out[1], pendingMark);
  pendingMark[0] = 0;
  if (logFile.println(line) == 0) { logStop("SD write failed: is the card still in?"); return; }
  logRows++;
  if (millis() - lastFlush >= FLUSH_MS) { logFile.flush(); lastFlush = millis(); }
}

// ---------- Commands ----------
void cmdLs() {
  if (!haveSd) haveSd = SD.begin(SD_CS);
  if (!haveSd) { sendErr("LS", "no SD card found"); return; }
  File root = SD.open("/");
  if (!root) { sendErr("LS", "could not read the card"); return; }
  Serial.print("{\"type\":\"ls\",\"files\":[");
  bool first = true;
  File f;
  while ((f = root.openNextFile())) {
    String path = String("/") + f.name();
    if (!f.isDirectory() && safeName(path.c_str())) {
      Serial.printf("%s{\"name\":\"%s\",\"size\":%lu}", first ? "" : ",", path.c_str(), (unsigned long)f.size());
      first = false;
    }
    f.close();
  }
  root.close();
  Serial.printf("],\"total\":%llu,\"used\":%llu}\n", SD.totalBytes(), SD.usedBytes());
}

void cmdGet(const char *path) {
  if (!safeName(path)) { sendErr("GET", "bad file name"); return; }
  if (logging && strcmp(path, logName) == 0) { sendErr("GET", "stop recording before opening this file"); return; }
  File f = SD.open(path);
  if (!f) { sendErr("GET", "file not found"); return; }
  Serial.printf("{\"type\":\"get\",\"name\":\"%s\",\"size\":%lu}\n", path, (unsigned long)f.size());
  uint8_t buf[384];
  char out[520];
  int n;
  while ((n = f.read(buf, sizeof buf)) > 0) {
    base64(buf, n, out);
    Serial.print("{\"type\":\"chunk\",\"d\":\"");
    Serial.print(out);
    Serial.print("\"}\n");
  }
  f.close();
  Serial.printf("{\"type\":\"end\",\"name\":\"%s\"}\n", path);
}

void cmdDel(const char *path) {
  if (!safeName(path)) { sendErr("DEL", "bad file name"); return; }
  if (logging && strcmp(path, logName) == 0) { sendErr("DEL", "that file is being recorded"); return; }
  if (!SD.remove(path)) { sendErr("DEL", "could not delete it"); return; }
  sendOk("DEL");
}

// Average the gyro for about 1 s. Also reports how much the board moved meanwhile (the largest
// max-min swing of any gyro axis, deg/s, and of any accelerometer axis, g) so callers can tell
// whether it was really still. Returns false if the IMU couldn't be read.
bool measureGyroBias(float bias[3], float *gyroSwing, float *accSwing) {
  double sum[3] = {0, 0, 0};
  float gMin[3], gMax[3], aMin[3], aMax[3];
  for (int k = 0; k < 3; k++) { gMin[k] = aMin[k] = 1e9; gMax[k] = aMax[k] = -1e9; }
  int good = 0;
  for (int i = 0; i < 200; i++) {
    int16_t v[7];
    if (imuReadRaw(v)) {
      for (int k = 0; k < 3; k++) {
        float g = v[4 + k] / gyroLsbPerDps, a = v[k] / accLsbPerG;
        sum[k] += g;
        gMin[k] = min(gMin[k], g); gMax[k] = max(gMax[k], g);
        aMin[k] = min(aMin[k], a); aMax[k] = max(aMax[k], a);
      }
      good++;
    }
    delay(5);
  }
  if (good < 100) return false;
  *gyroSwing = *accSwing = 0;
  for (int k = 0; k < 3; k++) {
    bias[k] = sum[k] / good;
    *gyroSwing = max(*gyroSwing, gMax[k] - gMin[k]);
    *accSwing = max(*accSwing, aMax[k] - aMin[k]);
  }
  return true;
}

void cmdCalGyro() {
  if (!haveImu) { sendErr("CAL GYRO", "no IMU"); return; }
  float bias[3], gs, as;
  if (!measureGyroBias(bias, &gs, &as)) { sendErr("CAL GYRO", "IMU read failed"); return; }
  for (int k = 0; k < 3; k++) gyroBias[k] = bias[k];
  char extra[80];
  snprintf(extra, sizeof extra, ",\"bias\":[%.3f,%.3f,%.3f]", gyroBias[0], gyroBias[1], gyroBias[2]);
  sendOk("CAL GYRO", extra);
}

// At power-up, calibrate the gyro only if the board is sitting still (on the pad, on a desk).
// If it's being moved, keep the offset at 0 rather than baking the motion into every reading.
char autoCalNote[64] = "";
void autoCalGyro() {
  if (!AUTO_CAL_GYRO || !haveImu) return;
  float bias[3], gs, as;
  if (!measureGyroBias(bias, &gs, &as)) { strcpy(autoCalNote, "gyro auto-cal failed: IMU read error"); return; }
  if (gs > STILL_GYRO_DPS || as > STILL_ACC_G) {
    snprintf(autoCalNote, sizeof autoCalNote, "gyro NOT calibrated: moving (%.1f dps / %.2f g swing)", gs, as);
    return;
  }
  for (int k = 0; k < 3; k++) gyroBias[k] = bias[k];
  snprintf(autoCalNote, sizeof autoCalNote, "gyro auto-cal bias %.2f %.2f %.2f dps", bias[0], bias[1], bias[2]);
}

void handleCommand(char *line) {
  if (!strcmp(line, "INFO")) sendInfo();
  else if (!strcmp(line, "LOG START")) logStart();
  else if (!strcmp(line, "LOG STOP")) logStop();
  else if (!strncmp(line, "MARK ", 5)) {
    if (!logging) { sendErr("MARK", "start recording first"); return; }
    // keep the CSV valid: no commas or quotes in the note
    int o = 0;
    for (char *c = line + 5; *c && o < (int)sizeof pendingMark - 1; c++)
      pendingMark[o++] = (*c == ',' || *c == '"' || *c == '\\') ? ' ' : *c;
    pendingMark[o] = 0;
    sendOk("MARK");
  }
  else if (!strcmp(line, "LS")) cmdLs();
  else if (!strncmp(line, "GET ", 4)) cmdGet(line + 4);
  else if (!strncmp(line, "DEL ", 4)) cmdDel(line + 4);
  else if (!strcmp(line, "CAL GYRO")) cmdCalGyro();
  else if (!strcmp(line, "SERVO MODE MANUAL") || !strcmp(line, "SERVO MODE AUTO")) {
    servoCtl.allowAuto = !strcmp(line, "SERVO MODE AUTO");
    prefs.putBool("autoOK", servoCtl.allowAuto);
    addNote(servoCtl.allowAuto ? "SERVO setting: auto allowed (CH3 switch chooses)" : "SERVO setting: manual only");
    sendOk("SERVO MODE", servoCtl.allowAuto ? ",\"allow\":\"auto\"" : ",\"allow\":\"manual\"");
  }
  else if (line[0]) sendErr("?", "unknown command");
}

char cmdBuf[96];
int cmdLen = 0;
void readCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdLen] = 0;
      if (cmdLen) handleCommand(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < (int)sizeof cmdBuf - 1) cmdBuf[cmdLen++] = c;
  }
}

// ---------- Reading the sensors ----------
void readSensors() {
  r.t = millis();

  int16_t v[7];
  r.imuOk = haveImu && imuReadRaw(v);
  if (r.imuOk) {  // ranges set in imuBegin()
    for (int k = 0; k < 3; k++) {
      r.a[k] = v[k] / accLsbPerG;
      r.g[k] = v[4 + k] / gyroLsbPerDps - gyroBias[k];
    }
    r.imuT = v[3] / 340.0f + 36.53f;
  }

  r.baroOk = false;
  if (haveBaro && bmp.performReading()) {
    if (baroWarmup > 0) baroWarmup--;
    else {
      r.baroOk = true;
      r.p = bmp.pressure;
      r.bT = bmp.temperature;
      r.alt = 44330.0f * (1.0f - powf(r.p / 101325.0f, 0.1903f));  // standard atmosphere
    }
  }

  if (haveGps && gps.getPVT(0)) {  // true only when a new fix report has arrived (1 per second)
    r.fix = gps.getFixType();
    r.sv = gps.getSIV();
    r.lat = gps.getLatitude() / 1e7;
    r.lon = gps.getLongitude() / 1e7;
    r.gAlt = gps.getAltitudeMSL() / 1000.0f;
    r.spd = gps.getGroundSpeed() / 1000.0f;
    r.hAcc = gps.getHorizontalAccEst() / 1000.0f;
    if (gps.getTimeValid())
      snprintf(r.utc, sizeof r.utc, "%02d:%02d:%02d", gps.getHour(), gps.getMinute(), gps.getSecond());
  }

  readRc(r.rc);
}

void streamReading(uint32_t loopMs) {
  char line[768];
  int n = snprintf(line, sizeof line, "{\"type\":\"d\",\"t\":%lu", (unsigned long)r.t);
  if (r.imuOk)
    n += snprintf(line + n, sizeof line - n, ",\"a\":[%.4f,%.4f,%.4f],\"g\":[%.2f,%.2f,%.2f],\"it\":%.2f",
                  r.a[0], r.a[1], r.a[2], r.g[0], r.g[1], r.g[2], r.imuT);
  if (r.baroOk)
    n += snprintf(line + n, sizeof line - n, ",\"p\":%.1f,\"bt\":%.2f,\"alt\":%.2f", r.p, r.bT, r.alt);
  if (haveGps)
    n += snprintf(line + n, sizeof line - n,
                  ",\"gps\":{\"fix\":%d,\"sv\":%d,\"lat\":%.7f,\"lon\":%.7f,\"h\":%.2f,\"spd\":%.2f,\"hacc\":%.2f,\"utc\":\"%s\"}",
                  r.fix, r.sv, r.lat, r.lon, r.gAlt, r.spd, r.hAcc, r.utc);
  n += snprintf(line + n, sizeof line - n, ",\"rc\":[%u,%u,%u,%u]", r.rc[0], r.rc[1], r.rc[2], r.rc[3]);
  n += snprintf(line + n, sizeof line - n, ",\"srv\":{\"mode\":\"%s\",\"allow\":\"%s\",\"sw\":\"%s\",\"link\":%s,\"out\":[%u,%u]}",
                servoModeName(servoCtl.mode), servoCtl.allowAuto ? "auto" : "manual", switchName(servoCtl.sw),
                servoCtl.link ? "true" : "false", servoCtl.out[0], servoCtl.out[1]);
  n += snprintf(line + n, sizeof line - n, ",\"ph\":\"%s\",\"vz\":%.2f,\"agl\":%.1f",
                phaseName(phaseDet.phase), phaseDet.vz, phaseDet.agl());
  n += snprintf(line + n, sizeof line - n,
                ",\"log\":{\"on\":%s,\"file\":\"%s\",\"rows\":%lu,\"bytes\":%lu,\"secs\":%.1f,\"err\":\"%s\"}",
                logging ? "true" : "false", logName, (unsigned long)logRows,
                logging ? (unsigned long)logFile.size() : 0UL,
                logging ? (millis() - logStartMs) / 1000.0 : 0.0, logError);
  snprintf(line + n, sizeof line - n, ",\"lt\":%lu}\n", (unsigned long)loopMs);
  Serial.print(line);
}

// ---------- Setup and loop ----------
void setup() {
  Serial.setRxBufferSize(512);
  Serial.begin(BAUD);
  delay(300);

  WiFi.mode(WIFI_OFF);  // radios off, same as flight
  btStop();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // the perfboard's short wires are fine at 400 kHz

  haveImu = imuBegin();
  autoCalGyro();  // ~1 s; only applied if the board is still
  haveBaro = bmp.begin_I2C(0x77);
  if (haveBaro) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
  haveGps = gps.begin(Wire, 0x42);
  if (haveGps) {
    gps.setI2COutput(COM_TYPE_UBX);
    gps.setAutoPVT(true);  // the GPS sends a report every second; reading it never waits
  }
  haveSd = SD.begin(SD_CS);

  for (int i = 0; i < 4; i++) {
    pinMode(RC_PINS[i], INPUT);
    attachInterruptArg(RC_PINS[i], rcIsr, (void *)(intptr_t)i, CHANGE);
  }
  sendInfo();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  prefs.begin("caarl", false);
  servoCtl.allowAuto = prefs.getBool("autoOK", false);  // default: manual only
  for (int i = 0; i < 2; i++) {
    ledcAttach(SERVO_PINS[i], SERVO_HZ, SERVO_BITS);
    ledcWrite(SERVO_PINS[i], 0);  // no pulses until there's a real command
  }

  if (AUTO_LOG) {
    if (haveSd) {
      logStart();
      snprintf(pendingMark, sizeof pendingMark, "auto start at power-up | %s", autoCalNote);  // first row of the file
    } else {
      autoPending = true;  // keep trying in loop(): the card may need longer to power up
    }
  }
}

// Power-up on a weak supply can leave the SD card not ready when setup() runs.
// Keep trying for SD_RETRY_FOR_MS, then start the auto recording as soon as it answers.
void retrySd(uint32_t now) {
  static uint32_t lastTry = 0;
  if (!autoPending) return;
  if (now > SD_RETRY_FOR_MS) {
    autoPending = false;
    strcpy(logError, "no SD card found after power-up");
    sendErr("LOG", logError);
    return;
  }
  if (now - lastTry < SD_RETRY_MS) return;
  lastTry = now;
  SD.end();
  haveSd = SD.begin(SD_CS);
  if (haveSd) {
    logStart();
    if (logging) snprintf(pendingMark, sizeof pendingMark, "auto start - SD found %lus after power-up | %s",
                          (unsigned long)(now / 1000), autoCalNote);
  }
}

void updateLed(uint32_t now) {
  bool on;
  if (logging) on = now % 1000 < 100;          // short flash every second
  else if (autoPending) on = now % 250 < 125;  // fast blink: looking for the SD card
  else on = logError[0] != 0;                  // solid on: something went wrong
  digitalWrite(LED_PIN, on);
}

// Append a note to the "event" column of the next recorded row.
void addNote(const char *text) {
  if (!logging) return;
  int n = strlen(pendingMark);
  snprintf(pendingMark + n, sizeof pendingMark - n, "%s%s", n ? " | " : "", text);
}

// Servo outputs. Runs every loop so manual control follows the sticks without delay.
void updateServos() {
  static const uint16_t AUTO_STOP[2] = {0, 0};  // steering isn't written yet: auto = stopped
  static uint16_t written[2] = {0xFFFF, 0xFFFF};
  uint16_t rc[4];
  readRc(rc);
  if (servoCtl.update(rc, AUTO_STOP)) {
    char note[80];
    snprintf(note, sizeof note, "SERVO MODE %s (switch %s)", servoModeName(servoCtl.mode), switchName(servoCtl.sw));
    addNote(note);
  }
  for (int i = 0; i < 2; i++) {
    uint16_t us = servoCtl.out[i];
    if (us == written[i]) continue;
    written[i] = us;
    ledcWrite(SERVO_PINS[i], us ? (uint32_t)us * ((1u << SERVO_BITS) - 1) / 20000u : 0);
  }
}

// Log-only flight phase detection: changes go into the "event" column; the current phase,
// vertical speed and height above the pad are in every row and in the PC stream.
void updatePhase() {
  float aMag = r.imuOk ? sqrtf(r.a[0] * r.a[0] + r.a[1] * r.a[1] + r.a[2] * r.a[2]) : 0;
  if (phaseDet.update(r.t, r.imuOk, aMag, r.baroOk, r.alt)) addNote(phaseDet.note);
}

void loop() {
  static uint32_t next = 0, count = 0;
  readCommands();
  updateServos();
  uint32_t now = millis();
  updateLed(now);
  if ((int32_t)(now - next) < 0) return;
  retrySd(now);
  next = now + SAMPLE_MS;

  uint32_t start = millis();
  readSensors();
  updatePhase();
  if (logging) logRow();
  if (++count % STREAM_EVERY == 0) streamReading(millis() - start);
}
