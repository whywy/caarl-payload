# CAARL Payload Software

2026-11-08 · **Prototype 1**: ground dashboard, SD-card logger and 3D-printed case for the CAARL avionics payload
(Inland ESP32, MPU-6050 clone IMU, BMP388 barometer, NEO-M8P-2 GPS, HW-125 microSD, RadioMaster ER4).

This is the first version of the payload's software. Later versions (flight logic, parachute steering)
build on it.

---

## ⏩ Forgot how to use it? Start here

1. **Plug the payload into the computer** with the micro-USB cable.
2. **Close the Arduino IDE's Serial Monitor** if it's open (only one program can use the USB port at a time).
3. Open a terminal and run:
   ```bash
   cd ~/Projects/caarl-payload && python3 dashboard.py
   ```
   Or double-click **`start-dashboard.sh`** in the Files app (choose **Run in Terminal** if it asks).
4. Your browser opens the dashboard at **http://127.0.0.1:8765**. The top bar turns green when it finds
   the payload and each sensor.
5. When you're done, click the terminal and press **Ctrl+C** to stop it.

**If the page says "Looking for the payload…"**
- Check the cable (some micro-USB cables are charge-only and carry no data).
- Close the Arduino Serial Monitor, or any other program using the port.
- The ESP32 may have different firmware on it (for example, the old breadboard test). Upload this
  project's firmware (see "Uploading the firmware" below).

**If it says "Connected · no data (wrong firmware?)"**: the payload is running other firmware. Upload this one.

**Where things are:** this project is in `~/Projects/caarl-payload`. There's also a shortcut to it at
`~/School/Avionics/payload-dashboard`, and the case files also appear at `~/School/Avionics/case`.

---

## What's in this project

| Path | What it is |
|---|---|
| `firmware/payload-dashboard/payload-dashboard.ino` | The ESP32 program. Reads every sensor 20 times a second, sends readings to the PC 10 times a second, records to the SD card from power-up (or on command) |
| `dashboard.py` | The PC program. Talks to the ESP32 over USB and serves the web page. Standard-library Python only, so nothing to install |
| `web/index.html` | The dashboard page itself |
| `start-dashboard.sh` | Double-click launcher for `dashboard.py` |
| `downloads/` | Copies of files you open or save from the SD card (not uploaded to GitHub) |
| `hardware/case/` | 3D-printable case (sled + cover) that slides into the payload bay tube. Print, assembly and edit steps in its own README |

## What's on the page

| Panel | Shows |
|---|---|
| Orientation | Artificial horizon, pitch, roll and total g, worked out from gravity. **Calibrate gyro** measures the gyro's resting offset (keep the board still for 1 s) |
| Motion | Live X/Y/Z acceleration (g) and rotation rate (°/s), last 30 seconds |
| Altitude | Height above the spot where you pressed **Set zero here**, plus pressure and temperatures |
| GPS | Fix, satellites, position (with a map link once it has a fix), speed, accuracy, UTC time |
| Receiver | CH1–CH4 pulse widths as bars, 1000–2000 µs. 1500 µs = parachute winches stopped |
| SD card | **Start/Stop recording**, notes, and the card's file list with **Open**, **Save** and **Delete** |

**Recording** makes a new `LOGnnn.CSV` on the card each time, with 20 rows a second. **Add note** writes
your text into the `event` column of the next row, so you can find moments later ("lifted board", "dropped").

**Open** copies a file from the card, then shows a summary (duration, highest point, max g) and a graph of any
column. Every file you open or save is also copied into `downloads/`.
You can't open a file while it's being recorded: stop recording first.

GPS needs open sky. Indoors it will say "No fix", which is normal.

## Uploading the firmware

**Stop the dashboard first** (Ctrl+C), because the upload needs the USB port.

Arduino IDE: open `firmware/payload-dashboard/payload-dashboard.ino`, pick board **ESP32 Dev Module**
and port **/dev/ttyUSB0**, then press Upload.

Or from a terminal:
```bash
CLI=/var/lib/flatpak/app/cc.arduino.IDE2/x86_64/stable/active/files/arduino-ide/resources/app/lib/backend/resources/arduino-cli
"$CLI" compile --fqbn esp32:esp32:esp32 --upload -p /dev/ttyUSB0 ~/Projects/caarl-payload/firmware/payload-dashboard
```

Libraries it needs (already installed in the Arduino IDE on this computer): **Adafruit BMP3XX** and
**SparkFun u-blox GNSS Arduino Library** (v2). ESP32 board package 3.x.

This firmware talks at **921600 baud** (the old breadboard test used 115200). If you open the Serial
Monitor with it, set the speed to 921600 and you'll see the raw data lines.

## Pins (same as the perfboard)

| Part | ESP32 pins |
|---|---|
| I²C bus (IMU 0x68, barometer 0x77, GPS 0x42) | SDA = GPIO21, SCL = GPIO22 |
| SD card (SPI) | CS = 5, SCK = 18, MOSI = 23, MISO = 19 |
| ER4 receiver CH1–CH4 | GPIO36, 39, 34, 35 |

Full wiring: `~/School/Avionics/perfboard/WIRING.md` and `PERFBOARD-BUILD-GUIDE.md`.

## The CSV columns

`t_ms` (ms since power-on) · `ax_g ay_g az_g` · `gx_dps gy_dps gz_dps` (gyro offset removed if calibrated) ·
`imu_temp_c` · `pressure_pa` · `baro_temp_c` · `baro_alt_m` (standard-atmosphere estimate) · `gps_fix gps_sats lat lon
gps_alt_m gps_speed_mps gps_hacc_m utc` · `rc1_us … rc4_us` (0 = no signal) · `event`

They open straight in LibreOffice Calc or any spreadsheet.

## Good to know
- **Recording starts by itself at power-up** (firmware 1.1+), into a new `LOGnnn.CSV` each time, so it records
  with no computer attached. The first row's `event` column says `auto start at power-up`. Every power-up makes a
  new file (USB bench sessions too), so delete old ones from the dashboard now and then. To turn this off, set
  `AUTO_LOG = false` in the firmware. If the SD card isn't ready at power-up (weak supply), it keeps trying for
  30 s (firmware 1.2+).
- **Status LED**, firmware 1.2+. The Inland ESP32 has no LED of its own to blink, so it's an added LED on the
  perfboard: **GPIO13 (hole V31) → 330 Ω → LED (red, green or yellow; long leg toward the resistor) → GND
  (hole U31)**. Blue and white LEDs need more than 3.3 V and stay dark or very dim.

  | LED | Meaning |
  |---|---|
  | Short flash every second | Recording |
  | Fast blink | Looking for the SD card (first 30 s after power-up) |
  | Solid on | Problem: no SD card found, or writing to it failed |
  | Off | Not recording (stopped from the dashboard) |
- **Never connect USB while the board is powered from its 5 V input** (battery, Flipper, bench supply), unless
  you've confirmed the ESP32 board has a diode between USB and VIN.
- The recording is saved to the card every second, so pulling the power loses at most about one second.
- The SD card must be **FAT32** (a fresh 64 GB card usually comes as exFAT, which won't work).
- The gyro calibration is forgotten at power-off: calibrate again after each power-up.
- "Set zero here" is remembered by the browser, not the payload.
- The barometer's first readings after power-up are wrong, so the firmware skips them.
- The IMU's X/Y axes are rotated 90° from the rocket's (see WIRING.md); the dashboard shows the board's own axes.
- With WiFi or Bluetooth on, the ESP32's regulator overheats on this board and the radio jams the ER4.
  The firmware keeps both **off**.

## How it works (for later versions)

The ESP32 and the PC talk in lines of text over USB at 921600 baud.
- **ESP32 → PC:** one JSON object per line. `{"type":"d",...}` is a reading. Other types answer commands:
  `info`, `ok`, `err`, `ls`, and `get`/`chunk`/`end` for a file transfer (base64).
- **PC → ESP32:** one command per line: `INFO`, `LOG START`, `LOG STOP`, `MARK <text>`, `LS`,
  `GET /<file>`, `DEL /<file>`, `CAL GYRO`. The comment block at the top of the firmware lists them.

`dashboard.py` only accepts these commands from the page, and only listens on this computer (127.0.0.1).

## Saving changes to GitHub

This folder is a git repository, backed up privately at **github.com/whywy/caarl-payload**.
After you change something:
```bash
cd ~/Projects/caarl-payload
git status
```
`git status` shows what changed.
```bash
git add -A
git commit -m "Describe what you changed"
git push
```
`git add -A` is safe here because it only covers this project folder, and `.gitignore` keeps out
`downloads/` and other junk.
