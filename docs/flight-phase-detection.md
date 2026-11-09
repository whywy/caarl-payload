# Flight phase detection: the maths and how to iterate on it

How the payload decides which part of the flight it is in, why each rule is built the way it is,
what the numbers mean, and how to change them safely.

- **Code:** [`firmware/payload-dashboard/flight_phase.h`](../firmware/payload-dashboard/flight_phase.h)
  (plain C++, all thresholds in `PhaseConfig`)
- **Desk simulation:** [`tests/phase_sim.cpp`](../tests/phase_sim.cpp)
- **Status (firmware 1.5):** **log-only.** It writes its decisions to the `phase`, `vz_mps` and `event`
  columns. Nothing reads them to move a servo yet. The thresholds have been checked against simulated
  flights only, not a real one.

Every number marked *(computed)* below was calculated from the current settings (α = 0.25, β = 0.03,
20 Hz). If you change a setting, recompute it (section 9 shows how).

---

## 1. Why this exists

Steering under canopy has to start **after the chute is open and stable**, and never during boost,
coast, or a failed deployment. A wrong guess at the wrong moment can tangle lines or steer a
falling rocket. So the first job is a reliable answer to "where are we in the flight?", recorded on
real flights and compared with what actually happened, **before** anything acts on it.

---

## 2. The two signals it uses

### 2.1 Barometer → altitude

The BMP388 measures pressure $p$. The firmware converts it with the International Standard Atmosphere
(troposphere) formula:

$$
h = \frac{T_0}{L}\left[1 - \left(\frac{p}{p_0}\right)^{\frac{R L}{g M}}\right]
  = 44330.8\left[1 - \left(\frac{p}{101325}\right)^{0.19027}\right]\ \text{m}
$$

with $T_0 = 288.15$ K (sea-level temperature), $L = 0.0065$ K/m (temperature lapse rate),
$R = 8.31446$ J/(mol·K), $g = 9.80665$ m/s², $M = 0.0289644$ kg/mol. The code uses the rounded form
`44330 * (1 - pow(p / 101325, 0.1903))`.

What that means in practice:

- **Absolute altitude is wrong, differences are right.** $p_0$ is fixed at standard sea-level
  pressure, so the absolute number is off by whatever the weather is (tens of metres). The detector
  only uses **height above the pad** ($h - h_\text{ground}$), so the weather offset cancels.
- **Sensitivity** near sea level: $\left|\frac{dh}{dp}\right| \approx 8.3$ m per hPa *(computed)*. A
  sensor noise of a few hundredths of a hPa is a few tens of centimetres.
- **Temperature scale error.** The formula assumes standard temperatures. On a 30 °C day the true
  height difference is about **5 % larger** than reported *(computed: 303.15/288.15)*. That's fine for
  phase detection, but matters if you compare apogee with a commercial altimeter.
- **Pressure disturbances the formula can't know about:**
  - **Ejection charge:** the gas pulse inside the bay looks like a sudden altitude jump. The
    avionics bay needs vent holes, and the `MIN_APOGEE_MS` guard exists partly for this.
  - **Transonic flight:** shock waves near Mach 1 cause false pressure readings. Not expected for
    this rocket, but it's the other reason for the guard.
  - **Port placement:** a vent hole in a high-pressure spot on the airframe biases the reading
    while moving fast.
- **The firmware skips the first 3 barometer readings** after power-up (they're wrong).

### 2.2 Accelerometer → specific force

An accelerometer does **not** measure acceleration. It measures **specific force**: acceleration
minus gravity,

$$
\vec f = \vec a - \vec g
$$

so it reads **1 g sitting still**, **0 g in free fall**, and only the non-gravity forces (thrust, drag,
parachute pull) divided by mass in flight. The detector uses the magnitude
$|\vec f| = \sqrt{a_x^2 + a_y^2 + a_z^2}$ (in g), so **it doesn't matter how the board is mounted**.

| Situation | What $|\vec f|$ reads |
|---|---|
| On the pad | ≈ 1 g (this clone chip reads about **1.03 g**: a 3 % scale error) |
| Boost | (thrust − drag) / (m·g), several g |
| Coast | drag / (m·g): **can exceed 1 g at high speed** (see section 5.2) |
| Free fall near apogee | ≈ 0 g |
| Chute opening | a spike: (parachute force) / (m·g) |
| Steady descent under canopy | ≈ 1 g (drag balances weight) |

The IMU is set to **±16 g** and **±2000 °/s**, with the on-chip low-pass filter at about **20 Hz**
(`IMU_DLPF = 4`). The firmware samples **20 times a second**, so frequencies above 10 Hz can't be
represented (the Nyquist limit). Without the filter, motor and airflow vibration would fold down
("alias") into random-looking jumps in the log.

---

## 3. The alpha-beta filter (altitude and vertical speed)

### 3.1 Why a filter at all

Raw barometer altitude jumps around by tens of centimetres every reading. Taking the difference of
two readings to get speed makes it far worse: 0.4 m of noise between readings 0.05 s apart is up to
8 m/s of fake speed. Integrating the accelerometer gives speed without that noise, but any small
offset in it grows without limit. The alpha-beta filter gets a smooth altitude **and** a usable
vertical speed from the barometer alone.

### 3.2 The equations

Each reading $z_k$ (barometer altitude) at time step $\Delta t$:

$$
\begin{aligned}
\hat h^-_k &= \hat h_{k-1} + \hat v_{k-1}\,\Delta t &&\text{(predict: keep moving at the last speed)}\\
r_k &= z_k - \hat h^-_k &&\text{(residual: how wrong the prediction was)}\\
\hat h_k &= \hat h^-_k + \alpha\, r_k &&\text{(correct the altitude by a fraction of it)}\\
\hat v_k &= \hat v_{k-1} + \frac{\beta}{\Delta t}\, r_k &&\text{(correct the speed by a smaller fraction)}
\end{aligned}
$$

- $\alpha$ (0 to 1) = how much to trust each new altitude reading. Higher reacts faster but is noisier.
- $\beta$ = how quickly the speed estimate adapts. Higher reacts faster but is noisier.

In the code: `FILTER_ALPHA = 0.25`, `FILTER_BETA = 0.03`, $\Delta t$ = time between readings
(normally 0.05 s). If the gap is over 1 s, the filter restarts from the reading.

### 3.3 What these settings give *(computed)*

| Property | Formula | Value |
|---|---|---|
| Altitude noise after filtering | $\sqrt{\dfrac{2\alpha^2 + 2\beta - 3\alpha\beta}{\alpha(4 - 2\alpha - \beta)}}$ × sensor noise | **0.43 ×** sensor noise |
| Speed noise after filtering | $\dfrac{\beta}{\Delta t}\sqrt{\dfrac{2}{\alpha(4 - 2\alpha - \beta)}}$ × sensor noise | **0.91 m/s per 1 m** of sensor noise |
| Response time constant | from the filter's poles, $z^2 - (2-\alpha-\beta)z + (1-\alpha) = 0$, $\lvert z\rvert = 0.866$ | **0.35 s** |
| Speed reaches 90 % of a sudden change | (simulated step) | **0.75 s** |
| Altitude lag under constant acceleration $A$ | $\dfrac{(1-\alpha)\,A\,\Delta t^2}{\beta}$ | **0.61 m per g** |
| **Speed lag** under constant acceleration $A$ | $A\,\Delta t\left(\dfrac{\alpha}{\beta} - 1\right)$ | **3.6 m/s per g** |

On the bench (sensor noise around 0.3–0.4 m), the filtered speed stayed within **±0.4 m/s**, which
matches the 0.91 × noise prediction.

**The important consequence:** while the rocket is accelerating or decelerating hard (boost, coast),
`vz` lags the truth by several m/s per g. The rules are built around that:

- **Apogee** uses the *altitude* estimate (0.6 m lag per g), not `vz`.
- **The steady-descent rule** waits 2 s for `vz` to settle (about 3 time constants) before trusting it.

### 3.4 Relation to a Kalman filter

An alpha-beta filter is what a Kalman filter for "position and constant velocity" settles into
once it has run for a while. The link is the **tracking index**

$$
\Lambda = \frac{\sigma_a\,\Delta t^2}{\sigma_z}, \qquad \frac{\beta^2}{1-\alpha} = \Lambda^2
$$

where $\sigma_z$ is the barometer noise and $\sigma_a$ is how much real acceleration we expect.
For α = 0.25, β = 0.03: $\Lambda^2 = 0.0012$, so with $\sigma_z \approx 0.4$ m the filter behaves as if it
expects about **5.5 m/s²** of unpredictable acceleration *(computed)*. That's a reasonable middle
ground: it follows a parachute descent well and smooths the pad noise.

**Next step if we need better speed during boost and coast:** a 3-state Kalman filter (altitude,
speed, acceleration) that also uses the accelerometer. It would remove most of the 3.6 m/s-per-g lag.
It needs the accelerometer's axis along the rocket and its offset, which is why it isn't done yet.

---

## 4. Ground reference

On the pad the detector keeps learning ground level:

$$
h_\text{ground} \leftarrow h_\text{ground} + 0.02\,(\hat h - h_\text{ground}) \quad\text{each reading, only while } |\hat v| < 3 \text{ m/s}
$$

That's an exponential average with a time constant of $\Delta t / 0.02 = 2.5$ s, so slow weather
changes (sitting on the pad for an hour) don't build up into a false height. It **stops learning**
whenever the filter sees vertical movement, so a slow launch can't drag the ground level up with it.
**AGL** (above ground level) everywhere below means $\hat h - h_\text{ground}$.

---

## 5. The phases, one rule at a time

```
PAD ──launch──▶ BOOST ──burnout──▶ COAST ──apogee──▶ DESCENT ──chute open──▶ CHUTE ──3 s──▶ SETTLED ──▶ LANDED
                  └──────(short burn: apogee straight from BOOST)───┘          ▲
                                     (shock just before apogee confirmed)──────┘
                                                        DESCENT ──(no chute, lands)──▶ LANDED
```

### 5.1 PAD → BOOST (launch)

**Rule:** $|\vec f| >$ `LAUNCH_G` (3 g) for `LAUNCH_SAMPLES` (3) readings in a row, i.e. 0.15 s.
**Backup:** AGL > `LAUNCH_ALT_M` (30 m) **and** $\hat v >$ `LAUNCH_VZ_MPS` (15 m/s).

- **Why 3 g:** a high-power motor gives several g. Carrying, setting down or bumping the rocket can
  spike past 3 g, but only for one or two readings. The simulated 5 g single-reading bump is correctly
  ignored.
- **Why the backup:** if the IMU fails, a real climb still gets noticed. Both height **and** speed are
  required, so barometer drift on the pad can't trigger it.
- **Tuning:** a low-thrust motor might need `LAUNCH_G` lowered (check the motor's average thrust
  ÷ weight). Rough handling on the pad might need `LAUNCH_SAMPLES` raised.

### 5.2 BOOST → COAST (burnout)

**Rule:** $|\vec f| <$ `BURNOUT_G` (1.5 g) for 3 readings, or `MAX_BOOST_MS` (8 s) after launch.

After burnout the accelerometer reads drag ÷ weight. At high speed that's **more than 1 g**: in the
simulation, drag at 100 m/s is about 15 m/s² ≈ 1.5 g. So a 1 g threshold reported burnout late,
and 1.5 g is a compromise. Burnout is still reported about **0.5 s late** in the simulation.

**Nothing depends on burnout yet.** It's informational. A precise version would look at the
**sign** of the acceleration along the rocket's axis (thrust pushes forward, drag pushes backward),
which needs the IMU axis that points along the rocket written into the firmware.

### 5.3 COAST → DESCENT (apogee)

**Rule:** filtered altitude has dropped `APOGEE_DROP_M` (3 m) below its highest value, and at least
`MIN_APOGEE_MS` (4 s) have passed since launch. Also checked during BOOST, in case a short burn
reaches apogee before burnout is recognised.

- **Expected delay:** near apogee the rocket is almost stopped, so it falls about like a dropped
  stone: $\Delta h = \tfrac12 g t^2$, so a 3 m drop takes $t = \sqrt{2 \cdot 3 / 9.81} = $ **0.78 s**
  *(computed)*, plus a little filter lag. The simulation measured about **0.75 s** after the true apogee.
- **Why a drop and not "speed = 0":** right at the top the speed estimate passes through zero while
  it still lags the truth (section 3.3), and noise makes it cross zero several times. A height drop
  bigger than the filtered noise (0.43 × 0.4 m ≈ 0.17 m) is far harder to fake.
- **Why the 4 s guard:** protects against ejection-charge and transonic pressure glitches early in the
  flight. If the real time to apogee is shorter (small motor), lower it.
- **Tuning `APOGEE_DROP_M`:** smaller = earlier detection but more risk from noise and pressure
  glitches. With 0.17 m of filtered noise, going below about 1.5 m isn't wise.

### 5.4 DESCENT → CHUTE (chute open)

**Two independent ways in**, because either alone can miss:

1. **Opening shock:** $|\vec f| >$ `SHOCK_G` (3 g) while in DESCENT, **or** within 3 s before apogee
   was confirmed (the ejection charge fires right at apogee, so the shock can arrive before the 3 m drop).
2. **Steady descent:** downward speed between `CHUTE_MIN_MPS` and `CHUTE_MAX_MPS` (2–15 m/s)
   continuously for `CHUTE_STEADY_MS` (2 s).

**The physics behind both.** Parachute drag is

$$
F_D = \tfrac12\,\rho\,v^2\,C_d\,A
$$

and under a fully open chute it balances the weight, giving the steady **descent rate**

$$
v_t = \sqrt{\frac{2\,m\,g}{\rho\,C_d\,A}}
$$

Worked example *(computed)*: a 2 kg payload under a 0.9 m round chute ($C_d \approx 1.5$,
$A = 0.636$ m², $\rho = 1.2$ kg/m³) comes down at **5.85 m/s**. The drag that chute makes at different
speeds, as g on 2 kg:

| Speed when the chute opens | Drag force | In g |
|---|---|---|
| 5 m/s | 14 N | 0.73 g |
| 10 m/s | 57 N | **2.9 g** |
| 15 m/s | 129 N | 6.6 g |
| 20 m/s | 229 N | 11.7 g |

So **a chute that opens right at apogee, when the rocket is barely moving, may never produce 3 g.**
That's why the steady-descent rule exists. Real openings also overshoot these numbers for an instant
(the canopy inflates past its steady size), so a fast opening usually spikes higher.

**Tuning:**
- Set the 2–15 m/s window around **your** canopy's $v_t$ (put in your mass, $C_d$ and area). A
  dual-deploy drogue might come down at 20–30 m/s, which the current window treats as "not open yet":
  that's intended, since you'd steer under the main.
- **Never** let `CHUTE_MAX_MPS` cover free fall. The "chute never opens" simulation must keep
  passing (no false CHUTE).

### 5.5 CHUTE → SETTLED

**Rule:** `SETTLE_MS` (3 s) after CHUTE.

After opening, the payload swings under the canopy like a pendulum. The period is

$$
T = 2\pi\sqrt{\frac{\ell}{g}}
$$

For 2 m of lines, $T \approx 2.8$ s *(computed)*. Waiting 3 s lets about one full swing die down before
steering would start. Longer lines need a longer wait.

### 5.6 → LANDED

**Rule:** AGL < `LANDED_MAX_AGL_M` (50 m) and filtered altitude stays within `LANDED_BAND_M` (1.5 m)
for `LANDED_MS` (5 s). Checked from SETTLED, and also from DESCENT (a chute-less descent still lands).

- **Why the 50 m ceiling:** a payload hanging still in a thermal at 300 m must not count as landed.
- **Why 1.5 m:** the filtered noise is about 0.17 m and weather drift over 5 s is tiny. Under canopy
  at ~6 m/s the altitude changes 30 m in 5 s, far outside the band.

---

## 6. Parameter reference

All in `PhaseConfig` at the top of `flight_phase.h`.

| Setting | Default | Unit | Raise it to… | Lower it to… |
|---|---|---|---|---|
| `LAUNCH_G` | 3.0 | g | ignore harder handling bumps | detect weak motors |
| `LAUNCH_SAMPLES` | 3 | readings | ignore longer bumps | detect launch sooner |
| `LAUNCH_ALT_M`, `LAUNCH_VZ_MPS` | 30, 15 | m, m/s | make the backup stricter | make the backup quicker |
| `BURNOUT_G` | 1.5 | g | report burnout sooner | avoid early burnout on low-thrust tails |
| `BURNOUT_SAMPLES` | 3 | readings | steadier | sooner |
| `MAX_BOOST_MS` | 8000 | ms | allow long burns | — |
| `MIN_APOGEE_MS` | 4000 | ms | ignore later glitches | allow small, fast flights |
| `APOGEE_DROP_M` | 3.0 | m | resist pressure glitches | detect apogee sooner (keep ≥ 1.5) |
| `SHOCK_G` | 3.0 | g | ignore ejection jolts | catch gentle openings |
| `CHUTE_MIN_MPS`, `CHUTE_MAX_MPS` | 2, 15 | m/s | match your canopy's $v_t$ | match your canopy's $v_t$ |
| `CHUTE_STEADY_MS` | 2000 | ms | more certainty | sooner (keep ≥ 3 time constants ≈ 1 s) |
| `SETTLE_MS` | 3000 | ms | longer lines / more swing | — |
| `LANDED_MAX_AGL_M`, `LANDED_BAND_M`, `LANDED_MS` | 50, 1.5, 5000 | m, m, ms | — | — |
| `FILTER_ALPHA`, `FILTER_BETA` | 0.25, 0.03 | — | faster, noisier | smoother, more lag |

**If you change the filter gains,** recompute section 3.3 (section 9) and re-check the rules that
depend on it: `CHUTE_STEADY_MS` should stay at least 3 time constants, and `APOGEE_DROP_M` well above
the filtered noise.

---

## 7. The desk simulation

[`tests/phase_sim.cpp`](../tests/phase_sim.cpp) compiles the **same** `flight_phase.h` on the computer
and feeds it made-up flights at 20 Hz.

**The model (one dimension, straight up and down):**

$$
\dot v = \begin{cases}
a_\text{boost}\,g - k v|v| & \text{burning}\\
-g - k v|v| & \text{coasting or falling, no chute}\\
2\,(-v_\text{chute} - v) & \text{under canopy (pulls speed towards } -v_\text{chute}\text{)}
\end{cases}
\qquad \dot h = v
$$

with the accelerometer reading $(\dot v + g)/g$ during boost and under canopy, and drag ÷ g while
coasting. It adds Gaussian noise (0.4 m barometer, 0.03 g accelerometer by default) and an
opening-shock spike when the chute opens. Defaults: 8 g for 1.5 s, $k = 0.0015$ /m, apogee
≈ 427 m, 6 m/s under canopy.

**Scenarios and what must happen:**

| Scenario | Must… |
|---|---|
| Nominal (shock 0.5 s after apogee) | launch < 0.3 s; apogee 0–3 s after the true one; chute, settled, landed all reported |
| Soft opening (no shock) | same, chute found by steady descent |
| Shock 1.5 s after apogee | same |
| Chute never opens | **no** CHUTE |
| 5 g bump on the pad | **no** phase change |
| Double sensor noise | same as nominal |

**What it does *not* model** (so passing it isn't proof):
- wind and horizontal motion, tilt, spin
- ejection-charge pressure spikes, transonic effects, barometer lag
- the real shape of thrust curves and opening shocks
- the accelerometer's 3 % scale error, offsets, or clipping at 16 g
- dual deploy (drogue then main)

Adding any of these is a good way to harden the detector: add a scenario, watch it fail, fix the rule.

---

## 8. How to iterate

1. **Change** a setting in `PhaseConfig` (or a rule in `update()`).
2. **Run the simulation.** Every check must pass:
   ```bash
   cd ~/Projects/caarl-payload && g++ -std=c++17 -O1 -Wall -Wno-format-security -o /tmp/phase_sim tests/phase_sim.cpp && /tmp/phase_sim
   ```
3. **Add a scenario** for whatever you're changing, if one doesn't exist (e.g. dual deploy).
4. **Flash and bench check:** on a desk it must sit in `PAD` with `vz` near 0 (dashboard stream:
   `ph`, `vz`, `agl`).
5. **Fly it.** Afterwards, open the log and compare:
   - the `PHASE …` entries in `event` and the `phase` column, against what you saw and what the
     rocket's own altimeter recorded (apogee height and time)
   - `vz_mps` against the altitude curve: is the descent rate under canopy what section 5.4 predicts?
   - the biggest `|a|` around apogee: how big was the opening shock really?
6. **Tune** from the real numbers, add the flight as a new simulation scenario if something surprised
   you, and repeat.

A useful next tool would be a **replay program** that runs `flight_phase.h` over a real flight's CSV
with different settings, so thresholds can be tuned against real data without flying again.

---

## 9. Recomputing the filter numbers

```bash
python3 - <<'PY'
import math
a, b, dt = 0.25, 0.03, 0.05          # FILTER_ALPHA, FILTER_BETA, seconds between readings
den = a*(4 - 2*a - b)
print("altitude noise x %.3f" % math.sqrt((2*a*a + 2*b - 3*a*b)/den))
print("speed noise %.3f m/s per m" % ((b/dt)*math.sqrt(2/den)))
z = abs(complex(*( (2-a-b)/2, math.sqrt(max(0, (1-a) - ((2-a-b)/2)**2)) )))
print("time constant %.2f s" % (-dt/math.log(z)))
print("altitude lag %.2f m per g, speed lag %.2f m/s per g" % ((1-a)*9.81*dt*dt/b, 9.81*dt*(a/b - 1)))
PY
```

---

## 10. Towards steering (not built yet)

Once the phase is `SETTLED`, a steering loop would run each GPS update. The likely maths:

**Where is the target?** Over a few km, treat the ground as flat. With latitudes and longitudes in
radians and $R = 6\,371\,000$ m:

$$
\Delta N = R\,(\phi_t - \phi), \qquad \Delta E = R\,(\lambda_t - \lambda)\cos\phi, \qquad
\text{bearing} = \operatorname{atan2}(\Delta E, \Delta N)
$$

**Which way are we going?** The GPS course over ground (`headMot`), which is only meaningful above
roughly 1–2 m/s of horizontal speed.

**Heading error**, wrapped to ±180°:
$e = \operatorname{wrap}(\text{bearing} - \text{course})$

**Control:** a proportional command with a dead band (don't hunt over small errors), a limit (don't
pull the lines all the way), and a turn-rate limit from the gyro's yaw rate (don't spiral):

$$
u = \begin{cases} 0 & |e| < e_\text{dead}\\ \operatorname{clamp}(K_p\,e,\ -u_\text{max},\ u_\text{max}) & \text{otherwise}\end{cases}
\qquad\text{and reduce } |u| \text{ if } |\omega_\text{yaw}| > \omega_\text{max}
$$

**Safety gates** (all must be true, or the servos stop): phase is `SETTLED`; AGL above a floor
(e.g. 20 m); 3D GPS fix with good accuracy (`gps_hacc_m` small); the manual/auto switch on the
transmitter set to auto; the ER4 link not in failsafe.

Open hardware questions before any of this: canopy type (a round chute barely steers; a parafoil
does), what the continuous-rotation servos physically do (they control speed, not position, so line
length needs end-stops or position-controlled winch servos), and rewiring the servo signals to ESP32
output pins.

---

## References

- Bosch BMP388 datasheet (pressure noise, relative accuracy, output data rates)
- InvenSense MPU-6000/6050 register map (0x1A `CONFIG` / DLPF, 0x1B gyro range, 0x1C accel range)
- U.S. Standard Atmosphere, 1976 (the barometric formula and its constants)
- P. R. Kalata, "The Tracking Index: A Generalized Parameter for α-β and α-β-γ Target Trackers",
  *IEEE Transactions on Aerospace and Electronic Systems*, 1984 (alpha-beta filter and tracking index)
- T. W. Knacke, *Parachute Recovery Systems Design Manual*, 1992 (drag, opening shock, descent rate)
