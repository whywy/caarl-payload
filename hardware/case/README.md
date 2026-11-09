# CAARL Payload Case

2026-11-09 · A two-part 3D-printed case that holds the 70 × 90 mm payload perfboard and slides into the
rocket's payload bay tube.

## The parts

| File | What it is |
|---|---|
| `caarl-case-sled.stl` | **Sled:** the floor the board screws onto (4 standoffs), plus an end plate at each end |
| `caarl-case-cover.stl` | **Cover:** the curved top. It slides down over the end plates and screws to them |
| `caarl-case.step` | Both parts, for any CAD program (Fusion, Onshape, SolidWorks) |
| `caarl-case.FCStd` | Both parts plus a stand-in board, for FreeCAD |
| `make_case.py` | The script that makes all of the above. **Change the sizes here, not in the files** |

Assembled, it's **120 mm long and 97.4 mm across**, D-shaped: flat underneath, round on top.
It fits a tube with a **98 mm inside diameter** (a typical 4" body tube).

## ⚠ Measure two things before printing

1. **Your tube's inside diameter.** Measure it with calipers in two directions and use the smaller number.
   If it isn't 98.0 mm, change `TUBE_ID` in `make_case.py` and rebuild (see below).
   The case comes out 0.6 mm smaller than the tube, so it slides in.
2. **Your board's corner holes.** Measure from the edge of the board to the **centre** of a corner hole.
   I assumed **3.0 mm**. If yours is different, change `HOLE_INSET`.

Also check your screw size. The standoff holes are 2.5 mm, for **M3 self-tapping screws**. For M2.5
screws, set `SCREW_PILOT = 2.1`.

## Printing (Bambu Studio)

1. **Open both STL files.** They load already placed the right way up: the sled sitting on its floor,
   and the cover standing on its end.
2. **Settings:**
   - PETG if you have it (handles the heat of a car or a launch pad); PLA is fine for testing.
   - 0.2 mm layers, 3 walls, 15–20 % infill.
3. **No supports needed.** The end-plate openings have pointed tops so they print without supports.
   The cover's USB hole bridges 12 mm, which is fine.
4. The sled is about 68 cm³ of shape and the cover about 46 cm³. At 15 % infill they come out much lighter.

## Assembly

1. **Screw the board onto the 4 standoffs**, components up. Turn it so the **ESP32's USB end points
   sideways out of the case**, then put the cover on loosely to check the USB lines up with its side
   hole. If the hole is at the wrong end, turn the cover around: it fits either way.
   - Use 4 × M3 × 8 mm self-tapping screws.
   - The 6 mm standoffs leave room for the wiring underneath.
2. **Run the wires out through the end-plate openings:** servo leads, the BEC lead and the antennas.
   **Zip-tie them** through the slots in the floor near each end, so a tug on a wire can't pull on a
   solder joint.
3. **Slide the cover down over both end plates.** Fix it with 4 × M3 × 8 screws through its holes,
   2 at each end, into the end plates.
   - The same holes can take longer screws **through the rocket tube** to hold the case in the bay.
   - Drill the tube to match, with the case in place.

**Access with the case closed:**
- **USB (programming and the dashboard):** through the side hole in the cover.
- **SD card:** through the opening in the SD-card end plate.

## What it was designed around

| Thing | Value |
|---|---|
| Board | 70 × 90 × 1.6 mm, corner holes 3 mm in from the edges (assumed) |
| Module sockets | 8.3 mm tall (measured) |
| ESP32 USB socket | Its centre is 11.4 mm above the board. The ESP32 sticks 2 mm past the right edge (measured) |
| Tallest part allowed | 25 mm above the board. There's actually 45 mm of room at the board's edges and 61 mm in the middle |
| Space under the board | 6 mm, for wiring |

The script checks all of this every time it runs, and prints `CHECKS: all passed` (or what's wrong).

## Changing the design

1. Open `make_case.py` in a text editor and change any number in the **SETTINGS** block at the top.
2. Rebuild everything:
   ```bash
   flatpak run --command=FreeCADCmd org.freecad.FreeCAD -c "exec(open('$HOME/Projects/caarl-payload/hardware/case/make_case.py').read())"
   ```
   Or in FreeCAD: **Macro → Macros…**, pick `make_case.py`, then **Execute**.
3. Read the last lines it prints: the case size, the room above the board, and `CHECKS`.

## Things to think about (not in this version)

- **GPS antenna:** the patch antenna has to face the sky (toward the nose cone). GPS can see through a
  cardboard or fiberglass tube, but **not through carbon fiber or metal**. There's plenty of room on
  top of the cover, or on an end plate, to stick it with foam tape.
- **Barometer:** the bay tube needs small static vent holes (usually 3 × 2–3 mm, spaced evenly around the
  tube), or the barometer can't sense altitude. The case's open ends let the air reach the board.
- **Battery and 5 V converter:** the empty space under the floor (inside the tube) or a second sled.
- **A central threaded rod:** if your bay has one running through the middle, this design won't fit as
  it is. Tell Claude and the board can move off-centre.
