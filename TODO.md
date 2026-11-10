# CAARL Payload: to do

Updated 2026-11-09. Tick items off (`- [x]`) as they're done. Board details: `~/School/Avionics/perfboard/WIRING.md`.

## Parts to get

- [ ] **ExpressLRS 2.4 GHz transmitter** (any radio with ExpressLRS, e.g. RadioMaster)
  - Needed to control the servos by hand, to switch manual/auto (CH3), and to test the receiver and failsafe.
  - Borrowing a teammate's works: bind it to the ER4 with the same binding phrase.
- [ ] **Servo BEC: 6 V, 5 A or more continuous** (8 A is better), switching type, accepts 2S
  - e.g. Hobbywing UBEC 8A (jumper 5/6/7.4 V) or Hobbywing 5A UBEC (non-HV). Avoid "HV" models (they need 3S+).
  - Solders into **C31 (+) / D31 (−)**. Set it to 6 V and **measure it before connecting the servos**.
- [ ] **ESP system BEC: 5 V, 1 A or more**, switching type, accepts 2S
  - Powers the ESP32, SD card and sensors through the 5 V input **D15 (+) / D16 (−)**.
  - The board draws about 0.33 A, with peaks around 0.5 A.
- [ ] **Flight battery: small 2S LiPo**, plus a switch or arming plug rated 5 A or more, before both BECs
- [ ] *(Optional)* 470–1000 µF capacitor, 10 V or higher, across C31/D31 (smooths servo current surges in flight)

## Wiring

- [ ] **Rewire the servo signals through the ESP32** (firmware 1.6 needs this for manual/auto control)
  1. **Remove first:** wire 35 (**A25 → D25**) and wire 36 (**B25 → D28**). The ER4 and the ESP32 must never
     drive the same pin.
  2. **Add:** ESP32 GPIO25 → servo 1 signal (**P31 → D25**) and GPIO26 → servo 2 signal (**Q31 → D28**),
     clear of the 6 V servo wires.
  3. Beep-check: P31 ↔ D25 and Q31 ↔ D28 beep; A25 ↔ D25 and B25 ↔ D28 do **not**.
- [ ] Wire the two BECs to the battery switch, and each BEC's − straight to the battery −
- [ ] *(Optional)* Update `soldering-diagram.svg` to show the new wires 35/36 (it still shows the old ones)

## Transmitter setup (once you have one)

- [ ] Bind it to the ER4
- [ ] Map a **2-position switch to CH3**: low = manual, high = auto. The dashboard's CH3 bar should jump between
      about 1000 and 2000 µs.
- [ ] Set the ER4 **failsafe to 1500 µs (stop) on CH1 and CH2**

## Tests still to do

- [ ] **Receiver:** CH1–CH4 bars on the dashboard follow the sticks
- [ ] **Manual servo control** (after the rewire, setting on "Manual only"): each servo follows its stick
- [ ] **Manual/auto switch:** set "Auto allowed", flip CH3 high → dashboard shows AUTO and the servos stop;
      flip CH3 low → the sticks take over again
- [ ] **Failsafe:** turn the transmitter off with the servos running → they stop and stay stopped
- [ ] **Full power test on BECs** (no USB): LED flashes once a second, recording starts, servos work
- [ ] **30 s SD retry:** power up without the SD card (LED fast blink), insert it within 30 s (LED → slow flash)
- [ ] **Outdoor GPS test:** power from a USB power bank (not the Flipper: it can't power the board), get a fix
- [ ] **Power-up while moving:** first log row says `gyro NOT calibrated: moving (...)`

## After the first flight

- [ ] Download the log, check the `PHASE` events against what happened, and tune the detector with
      **Simulation & replay** on the dashboard (or `tools/phase_replay`)

## Before designing steering

- [ ] Decide the canopy: round chute (barely steers) or parafoil
- [ ] Decide how the servos pull the lines: continuous-rotation servos have no position, so add end stops or
      switch to position-controlled winch servos
- [ ] Single deploy at apogee, or dual deploy?
- [ ] Check that guided recovery is allowed by the club / competition and the launch's safety officer
