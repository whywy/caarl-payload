# Instructions for AI assistants working in this repo

(Claude Code reads this through `CLAUDE.md`; Codex and other tools read it directly.)

## Keep TODO.md up to date

[`TODO.md`](TODO.md) is the team's list of parts to get, wiring, transmitter setup, tests and decisions.
Keep it current as part of every task, not as an afterthought:

- **When something on it gets done**, tick it (`- [ ]` → `- [x]`) in the same commit as the work, and add a
  short note of what was found if it matters (e.g. `- [x] Failsafe test: servos stop within 0.5 s`).
- **When something new comes up** that has to be done later (a part to buy, a wire to change, a test that
  couldn't be run, a decision for the team), add it under the right heading.
- **When a test can't be finished** (missing hardware, no transmitter), leave it unticked and say why.
- **Don't delete finished items**; they show what's been done. Update the "Updated" date at the top.
- If the work changes the wiring, also update `~/School/Avionics/perfboard/WIRING.md` (outside this repo) and
  add the matching TODO item if something still has to be soldered.

## Before committing

- Run the checks for anything you touched; all must pass:
  ```bash
  g++ -std=c++17 -O1 -Wall -Wno-format-security -o /tmp/phase_sim tests/phase_sim.cpp && /tmp/phase_sim
  g++ -std=c++17 -Wall -o /tmp/servo_mode_test tests/servo_mode_test.cpp && /tmp/servo_mode_test
  ```
- Firmware changes: compile (and flash, if the board is plugged in) with the command in the README, and bump
  `FW_VERSION` in `firmware/payload-dashboard/payload-dashboard.ino`.
- Never commit flight data (`downloads/`) or build output (`build/`); `.gitignore` covers both.

## Safety rules for the servos

- The servos drive the parachute lines. Never make a change that could move them without the pilot's
  control: "Manual only" stays the default, and anything automatic must go through `servo_mode.h` and its tests.
