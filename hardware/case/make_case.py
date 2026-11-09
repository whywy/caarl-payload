# CAARL payload case: a sled + cover that slides into the rocket's payload bay tube.
#
# Run it from a terminal (makes every file in this folder):
#   flatpak run --command=FreeCADCmd org.freecad.FreeCAD -c "exec(open('$HOME/Projects/caarl-payload/hardware/case/make_case.py').read())"
# Or in FreeCAD: Macro -> Macros... -> pick this file -> Execute.
#
# Change the numbers in SETTINGS, run it again, and every file is rebuilt.
# All sizes in millimetres.
#
# Axes: X = across the board (left edge = column A, right edge = column Z),
#       Y = along the rocket (board row 1 / SD-card end = +Y),
#       Z = up (components side).

import os
import FreeCAD as App
import Part
import Mesh
from FreeCAD import Vector as V

# ------------------------------- SETTINGS ---------------------------------
TUBE_ID = 98.0          # payload bay tube inside diameter. MEASURE YOURS and change this.
TUBE_CLEAR = 0.6        # case is this much smaller than the tube (diameter), so it slides in
WALL = 2.0              # cover thickness
FIT = 0.3               # gap between the cover and the bulkheads it slides over

BOARD_W = 70.0          # perfboard across (letters A-Z)
BOARD_L = 90.0          # perfboard along (numbers 1-31)
BOARD_T = 1.6
HOLE_INSET = 3.0        # board's corner holes: centre distance from each edge. MEASURE.
SCREW_PILOT = 2.5       # pilot hole for M3 self-tapping screws (use 2.1 for M2.5)
STANDOFF_D = 6.5
STANDOFF_H = 6.0        # gap under the board for the 24/30 AWG wiring and solder joints

DECK_T = 3.0            # floor thickness
DECK_TOP = -22.0        # floor height relative to the tube centre (lower = more room above)
END_GAP = 9.0           # space between the board's ends and the bulkheads (SD card sticks out 2.6 mm)
BULKHEAD_T = 6.0        # end plate thickness

WINDOW_R = 36.0         # end-plate opening: radius of its round sides
WINDOW_APEX = 30.0      # end-plate opening: height of its pointed top (keeps it printable)

COVER_SCREW_CLEAR = 3.4 # M3 clearance hole in the cover

# Measured from the board (see ~/School/Avionics/perfboard): ESP32 micro-USB and microSD positions
USB_ROW_MID = 26.5      # midway between ESP32 pin rows 22 and 31
SOCKET_H = 8.3          # female headers the modules plug into (measured by Wyatt)
USB_HEIGHT = SOCKET_H + 1.6 + 1.5   # USB socket centre above the board's top face
ESP32_OVERHANG = 2.0    # ESP32 + USB stick out past the board's right edge (measured by Wyatt)
USB_HOLE = (16.0, 12.0) # cover opening for the plug's moulded body (along Y, up)
# ---------------------------------------------------------------------------

try:
    OUT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    OUT_DIR = os.path.expanduser("~/Projects/caarl-payload/hardware/case")

RO = TUBE_ID / 2 - TUBE_CLEAR / 2          # outside of the case
RI = RO - WALL                             # inside of the cover
RB = RI - FIT                              # bulkhead radius (the cover slides over it)
LENGTH = BOARD_L + 2 * END_GAP + 2 * BULKHEAD_T
DECK_BOT = DECK_TOP - DECK_T
BOARD_BOT = DECK_TOP + STANDOFF_H
BOARD_TOP = BOARD_BOT + BOARD_T
BIG = 400.0


def cyl_y(r, y0, length):
    """Cylinder along the tube axis."""
    return Part.makeCylinder(r, length, V(0, y0, 0), V(0, 1, 0))


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def above(z):
    return box(-BIG, BIG, -BIG, BIG, z, BIG)


def hole_x(col):   # column A=0 ... Z=25
    return -BOARD_W / 2 + (BOARD_W - 25 * 2.54) / 2 + col * 2.54


def hole_y(row):   # row 1 ... 31
    return BOARD_L / 2 - (BOARD_L - 30 * 2.54) / 2 - (row - 1) * 2.54


def radial_hole(d, depth, angle_deg, y):
    """Hole pointing at the tube axis from the outside, angle 0 = +X, 90 = straight up."""
    import math
    a = math.radians(angle_deg)
    direction = V(-math.cos(a), 0, -math.sin(a))
    start = V((RO + 1) * math.cos(a), y, (RO + 1) * math.sin(a))
    return Part.makeCylinder(d / 2, depth + 1, start, direction)


# ---- sled: floor + standoffs + two end plates ----
deck = box(-BIG, BIG, -LENGTH / 2, LENGTH / 2, DECK_BOT, DECK_TOP).common(cyl_y(RO, -LENGTH / 2, LENGTH))

window = cyl_y(WINDOW_R, -BIG, 2 * BIG).common(above(DECK_TOP + 2))
roof = box(-BIG, BIG, -BIG, BIG, -BIG, WINDOW_APEX)
roof = roof.cut(Part.makeBox(BIG, 2 * BIG, BIG, V(0, -BIG, WINDOW_APEX)).rotate(V(0, 0, WINDOW_APEX), V(0, 1, 0), 45))
roof = roof.cut(Part.makeBox(BIG, 2 * BIG, BIG, V(-BIG, -BIG, WINDOW_APEX)).rotate(V(0, 0, WINDOW_APEX), V(0, 1, 0), -45))
window = window.common(roof)

sled = deck
for y0 in (-LENGTH / 2, LENGTH / 2 - BULKHEAD_T):
    plate = cyl_y(RB, y0, BULKHEAD_T).common(above(DECK_BOT))
    sled = sled.fuse(plate.cut(window))

holes = []
for sx in (-1, 1):
    for sy in (-1, 1):
        x = sx * (BOARD_W / 2 - HOLE_INSET)
        y = sy * (BOARD_L / 2 - HOLE_INSET)
        sled = sled.fuse(Part.makeCylinder(STANDOFF_D / 2, STANDOFF_H + 0.01, V(x, y, DECK_TOP - 0.01)))
        holes.append(Part.makeCylinder(SCREW_PILOT / 2, STANDOFF_H + DECK_T + 2, V(x, y, DECK_BOT - 1)))

# zip-tie slots at each end, for strain-relieving the servo/BEC/antenna wires
for sy in (-1, 1):
    yc = sy * (BOARD_L / 2 + END_GAP / 2)
    for x in (-24, -14, 14, 24):
        holes.append(box(x - 1.5, x + 1.5, yc - 2.0, yc + 2.0, DECK_BOT - 1, DECK_TOP + 1))

# cover screw holes in the end plates (pilot), 45 degrees either side of the top
for y in (LENGTH / 2 - BULKHEAD_T / 2, -LENGTH / 2 + BULKHEAD_T / 2):
    for ang in (45, 135):
        holes.append(radial_hole(SCREW_PILOT, 12, ang, y))

for h in holes:
    sled = sled.cut(h)
sled = sled.removeSplitter()

# ---- cover: an arch from floor edge to floor edge, over the end plates ----
cover = cyl_y(RO, -LENGTH / 2, LENGTH).cut(cyl_y(RI, -BIG, 2 * BIG)).common(above(DECK_TOP))
usb_y = hole_y(USB_ROW_MID)
usb_z = BOARD_TOP + USB_HEIGHT
cover = cover.cut(box(0, BIG, usb_y - USB_HOLE[0] / 2, usb_y + USB_HOLE[0] / 2,
                      usb_z - USB_HOLE[1] / 2, usb_z + USB_HOLE[1] / 2))
for y in (LENGTH / 2 - BULKHEAD_T / 2, -LENGTH / 2 + BULKHEAD_T / 2):
    for ang in (45, 135):
        cover = cover.cut(radial_hole(COVER_SCREW_CLEAR, WALL + 2, ang, y))
cover = cover.removeSplitter()

# ---- stand-in board, for checking fit only (not printed) ----
board = box(-BOARD_W / 2, BOARD_W / 2, -BOARD_L / 2, BOARD_L / 2, BOARD_BOT, BOARD_TOP)
tallest = box(-BOARD_W / 2 + 2, BOARD_W / 2 - 2, -BOARD_L / 2 + 2, BOARD_L / 2 - 2, BOARD_TOP, BOARD_TOP + 25)
esp32 = box(BOARD_W / 2 - 28, BOARD_W / 2 + ESP32_OVERHANG, hole_y(USB_ROW_MID) - 14, hole_y(USB_ROW_MID) + 14,
            BOARD_TOP, BOARD_TOP + SOCKET_H + 6)   # ESP32 on its sockets, incl. the USB overhang

# ---- checks ----
problems = []
for name, s in (("sled", sled), ("cover", cover)):
    if not s.isValid():
        problems.append(f"{name} solid is not valid")
if tallest.common(cover).Volume > 0.01:
    problems.append("parts 25 mm tall would hit the cover")
if esp32.common(cover).Volume > 0.01:
    problems.append("the ESP32 / USB socket hits the cover")
if board.common(sled).Volume > 0.01 or board.common(cover).Volume > 0.01:
    problems.append("the board collides with the case")
if sled.common(cover).Volume > 0.01:
    problems.append("sled and cover overlap")

# ---- save ----
doc = App.newDocument("CAARL_Case")
for name, shape in (("Sled", sled), ("Cover", cover), ("Board_stand_in", board)):
    doc.addObject("Part::Feature", name).Shape = shape
doc.recompute()
doc.saveAs(os.path.join(OUT_DIR, "caarl-case.FCStd"))
Part.export([doc.getObject("Sled"), doc.getObject("Cover")], os.path.join(OUT_DIR, "caarl-case.step"))

# STL files placed the way they print best: sled on its floor, cover standing on its end
sled_print = sled.copy()
sled_print.translate(V(0, 0, -DECK_BOT))
cover_print = cover.copy()
cover_print.rotate(V(0, 0, 0), V(1, 0, 0), 90)   # tube axis now points up
bb = cover_print.BoundBox
cover_print.translate(V(-bb.Center.x, -bb.Center.y, -bb.ZMin))
for name, shape in (("caarl-case-sled.stl", sled_print), ("caarl-case-cover.stl", cover_print)):
    Mesh.Mesh(shape.tessellate(0.05)).write(os.path.join(OUT_DIR, name))

print("CASE length %.1f mm, outside diameter %.1f mm (tube %.1f)" % (LENGTH, 2 * RO, TUBE_ID))
print("Room above the board: %.1f mm at the centre, %.1f mm at the board's edges" % (
    RI - BOARD_TOP, (RI ** 2 - (BOARD_W / 2) ** 2) ** 0.5 - BOARD_TOP))
print("Sled %.0f cm3, cover %.0f cm3 (solid; printed with 15%% infill they weigh less)" % (
    sled.Volume / 1000, cover.Volume / 1000))
print("USB opening centre: Y %.1f, Z %.1f" % (usb_y, usb_z))
print("CHECKS:", "all passed" if not problems else "; ".join(problems))
