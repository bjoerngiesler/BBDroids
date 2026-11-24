# BB-8 Drive Simulation and Controller Tuning

Status: 2026-11-24. Covers the firmware in `Arduino/BB8Droid` with `Arduino/LibBB`.

This folder holds theoretical work that was done entirely in simulation. **None of it has been tested on the hardware.** The firmware builds with `pio run`, but it has not run on the droid with these changes.

## Contents

| File | Purpose |
|---|---|
| `README.md` | This document |
| `BB8-DriveSimulation.html` | The simulation as a single file that runs offline (three.js embedded). Open it in a browser. |
| `sim.js` | Simulation core (physics + replicated firmware). Runs in Node and in the browser. |
| `page.html` | UI template. `build.py` replaces `/*SIMJS*/` with `sim.js`. |
| `build.py` | Builds the HTML file from the template and the core |
| `check.js` | Reproduces the result tables in this document (`node check.js`, about 0.5 s) |
| `robustness.js` | Roll stability sweep over speeds, mass variants and disturbances (section 3.7, about 25 s) |
| `check-jsc.sh` | Runs `check.js` (or `./check-jsc.sh robustness.js`) without Node, using the JavaScriptCore shell that ships with macOS |

---

## 1. Simulation model

### 1.1 Mechanics

There are two decoupled planar models with three degrees of freedom each, integrated with RK4 at 1 kHz:

- **Pitch plane (x–z):** sphere position x (rolling without slip), absolute pendulum angle θ (forward positive), absolute head angle α (forward positive).
- **Roll plane (y–z):** sphere position y, absolute pendulum angle χ, absolute head angle ζ.

Masses:

- Sphere as a thin shell, I = 2/3·m_s·R².
- Pendulum as a point mass m_p at distance l below the sphere center.
- Head as a point mass m_h at distance L above the sphere center.

Both planes share the same mass matrix (arrow shaped, solvable directly):

```
M = | m_s + I_s/R² + m_p + m_h    m_p·l·cos a1    m_h·L·cos a2 |
    | m_p·l·cos a1                m_p·l²          0            |
    | m_h·L·cos a2                0               m_h·L²       |

right-hand side:  Q_s + m_p·l·sin a1·ȧ1² + m_h·L·sin a2·ȧ2²
                  Q_1 − m_p·g·l·sin a1
                  Q_2 + m_h·g·L·sin a2
```

How the actuators couple in (generalized forces):

| | Pitch plane | Roll plane |
|---|---|---|
| Frame / IMU | frame = pendulum; IMU pitch = θ | the axle tilts with the sphere; IMU roll ρ = y/R |
| Actuator | motor between sphere and frame: Q_x = τ/R, Q_θ = τ | roll servo between axle and pendulum, relative angle γ = χ + y/R: Q_y = τ/R, Q_χ = τ |
| Head | mast on the pendulum frame, relative angle β = α + θ | mast on the axle, relative angle δ = ζ − y/R |
| Encoder | measures the sphere relative to the pendulum: v_enc = ẋ + R·θ̇ | – |

An important consequence: the head mast sits on the pendulum frame. When the pendulum swings forward under acceleration, the head tips back with it (see section 6).

### 1.2 Drive, servos, sensors

- **Drive motor:** linear characteristic τ = τ_stall·(PWM/255 − ω_rel/ω_0) − b·ω_rel, plus viscous rolling resistance.
- **Dynamixel servos:** profile generator with the velocity and acceleration limits from the config (units of 0.229 rpm and 214.577 rpm²; 0 = unlimited), followed by PD position control (4 Hz, ζ = 0.8). That is limited by the torque-speed curve of the XM430-W350 at 12 V (4.1 Nm, 46 rpm).
- **Servo load:** reported as a fraction of the stall torque. The firmware shuts down at `roll_torque_percent` = 90.
- **Encoder:** 4776.384 ticks per revolution, quantized.
- **IMU:** Madgwick is assumed ideal, with no noise and no delay. This is a known weakness of the model (see section 6.2).

### 1.3 Firmware replica (100 Hz)

These are replicated 1:1:

- `bb::PIDController`: I bound, D filter 5 Hz, and scaling of the control term by the input's `controlGain()`. That gain is 1 for IMU inputs and 1/mmPerTick ≈ 3.0 for the drive encoder in mm.
- `bb::LowPassFilter`: 2nd-order Butterworth, bilinear without prewarping
- the `IMUControlInput` low-pass
- the encoder low-pass at 25 Hz
- `BB8RollController`: feedforward, P/I, gyro damping with a 15 Hz low-pass at 104 Hz and speed scheduling, head compensation, I bleed, input shaper
- the balance → speed → PWM cascade in DRIVE_VEL mode, with the setpoint ramp
- `stepDome()` with the head feedforward

The position modes are not replicated.

> **Correction (2026-11-24):** the first version of the simulation left out `controlGain()`. That made the speed loop three times softer than the real one. All tables in this document were recalculated with the corrected model. Section 2, item 1 and section 4 changed the most.

### 1.4 Cornering

With the axle leaning, the frame yaws at ψ̇ = (v/R)·sin ρ. This assumes the sphere rolls without slip and without spinning around the vertical at the contact point. The curve radius is R/sin ρ, about 1.5 m at 10°.

The centrifugal force acts on all three masses in the roll plane, in the rotating frame: Q_y = −m_tot·a, Q_χ = −m_p·a·l·cos χ, Q_ζ = −m_h·a·L·cos ζ with a = v·ψ̇.

### 1.5 Parameters and where they come from

| Quantity | Value | Source |
|---|---|---|
| Sphere radius R | 253 mm | `BB8Config.h` |
| Pendulum m_p, l | 4.5 kg, 155 mm | Björn: 24 mono cells (≈ 3.5 kg) + motor + servo, distance 120–155 mm |
| Roll pivot | sphere center | Björn |
| Roll servo | XM430-W350, 1:1 | Björn (`ROLL_REDUCTION` = 80/22 was left over from a cable drive) |
| Head m_h, L | 1.0 kg, 320 mm | **estimated**, to be weighed |
| Sphere shell m_s | 3.0 kg | **estimated** |
| Drive τ_stall, v_0 | 30 Nm, 2600 mm/s | **estimated** |
| Rolling resistance, bearing friction | 3 N/(m/s), 0.05 Nm/(rad/s) | **estimated** |
| Dome pitch servo reduction | 2 servo degrees per degree | **assumption**, by analogy with `DOME_ROLL_REDUCTION` |
| Servo position control | 4 Hz, ζ = 0.8 | **assumption** |

The results depend most on the pendulum moment m_p·l, the head moment m_h·L, and their ratio. The drive data only affect driving performance.

---

## 2. Findings in the original firmware

1. **`setIBounds(±255)` in the speed controller bounds the I error, not the I share.** Because of `controlGain()` ≈ 3.0, the I share was capped at about 3.0 × `drive_speed_ki` × 255 ≈ 153 PWM. The firmware now bounds the I share itself to `drive_speed_imax` PWM (default 255) by bounding the I error to ±`drive_speed_imax`/(`drive_speed_ki` × `controlGain`). The effect on top speed is moderate, about +10% (section 4). The bound stays as a safeguard against windup.
2. **The balance controller has no I term:** `bal_kp`·pitch pulls the speed setpoint down in steady state. While driving, the pendulum stays forward because of rolling resistance (section 4).
3. **`balanceController_.setRamp(100)`:**
   - The ramp only acts on `goal_`, which was always 0. The joystick speed passes through `controlOffset` without any ramp.
   - The more dangerous part: with a ramp ≠ 0, `setGoal()` resets the internal setpoint to `present()` every time, which also steps the `IMUControlInput` filter a second time. If the setpoint is set every cycle (head compensation), the controller effectively sees no error at all.
   - The ramp is now 0. `drive_accel_max` takes over the speed ramp.
4. **`IMUControlInput` low-pass:** a 100 Hz cutoff at 100 Hz sampling. Because the bilinear transform is applied without prewarping, it effectively acts as a low-pass of about 40 Hz rather than going above Nyquist. Not critical, but not what the parameter suggests.
5. **`fa_dome_pitch_to_weight` was declared (default 0.5) but never used.** With the simulation's sign, +0.5 would even have made the problem worse. It is now used, with default −0.85 (section 5).
6. **`fa_speed_sp_to_dome` = −0.01** pulled the head back further under acceleration in the simulation. The default is now +0.01, the same magnitude. It is possible that the old sign only reflects a convention difference between simulation and hardware. **Check on hardware** (section 8).
7. **`ROLL_REDUCTION` = 80/22** was a leftover. The servo is coupled 1:1. At 1:1 the pendulum only gets 4.1 Nm instead of 14.9 Nm, which explains the earlier emergency shutdowns with `roll_direct` (section 3.4).
8. **`body_roll_vel` = 500** corresponds to about 687°/s at the servo. The XM430-W350 only manages about 276°/s at 12 V, so the profile limit never kicks in.
9. **Roll defaults (0.3 / 0 / 0.1):** without an I term and with `roll_kff` = 0, the droid only reaches about 2–4° of a 10° setpoint.
10. **The position controllers control speed (confirmed by reading the code).** `posController_` and `autoPosController_` use `driveEncoder_` as their input, `Encoder::present()` returns whatever the encoder's mode is, and `setControlParameters()` forces `INPUT_SPEED`. The console command `drive position` has the reverse problem: it switches the encoder to position mode while the speed loop keeps running on it.
11. **The roll loop swings up when cornering at speed** (found after the first version of this document). At full speed, moving the head to one side and then to the other makes the lean oscillate with growing amplitude until the roll servo saturates, and the droid falls over on the subsequent stop. Fixed with speed-scheduled gyro damping and `body_roll_accel` 60, see section 3.7.

---

## 3. Roll controller

### 3.1 Approach

Classic tuning in simulation:

1. Raise K_p until it oscillates, then back off.
2. Raise K_i.
3. Settle it with K_d (gyro damping).

This was followed by a grid optimization with a robustness check over eight mass variants (m_p 4/5 kg, l 145/165 mm, m_h 0.7/1.3 kg) plus the nominal case, each for steps to 10° and 20°. Constraints: at most 10–20% overshoot, servo load at most 80% and never above 90%.

Findings from the first round (still with 80/22 and an estimated 6 kg / 120 mm, exploratory and not part of `check.js`):

- Without damping, the critical gain is K_u ≈ 0.5 with T_u ≈ 2.2 s. That is the sideways rocking mode of the sphere.
- Ziegler-Nichols (K_p 0.25, K_i 0.1) is stable but extremely slow: only 8.5° of 10° after 25 s.
- Only with gyro damping (K_d ≈ 0.4) does the loop tolerate considerably more K_p and K_i.
- After that, the upper limit is set by the roll servo, not by the control scheme. Without a head, the lean reacts faster, and an acceleration-limited servo produces a limit cycle.

### 3.2 Result (1:1, `body_roll_accel` 60, final masses)

Computed with the current defaults, i.e. with the ideal head compensation (section 5) and `body_roll_accel` 60 (section 3.7). The head compensation also cancels the dome roll that `fa_roll_sp_to_dome` commands opposite to the lean, which would otherwise work against the lean. That is why the responses are faster than with the old head settings (recommended set with the old head settings: t90 3.6 s, t ±5% 4.9 s, overshoot over the variants ≤ 8%).

| Set | t90 | t ±5% | Overshoot | Load | over all variants |
|---|---|---|---|---|---|
| Firmware default 0.3 / 0 / 0.1 | – (only reaches 3.7°) | – | 0% | 31% | – |
| **Recommended 0.5 / 0.4 / 0.4** | 2.8 s | 4.2 s | 0% | 36% | t ±5% ≤ 6.1 s, overshoot ≤ **16%**, load ≤ 60% |
| ZVD shaper T = 2.3 s + 0.85 / 1.3 / 0.3 | 2.8 s | 3.0 s | 3% | 25% | t ±5% ≤ 3.1 s, overshoot ≤ 5%, load ≤ 52% |

The worst-case overshoot of the recommended set over the mass variants is in the upper half of the 10–20% criterion (heavy head on a short pendulum). After weighing the head, recompute this, and lower `roll_ki` slightly if needed.

On `body_roll_accel`: at 1:1, 30 corresponds to about 640°/s² at the pendulum and 60 to about 1290°/s². At standstill, higher values bring no speed, only more load (30 → 60: nominal 22% → 36%, worst variant 37% → 60%). The default is nevertheless 60, because at speed the roll loop needs the faster servo response (section 3.7). From about 100 the load exceeds 80% in some variants; unlimited (0) overloads the servo.

Variants that don't help:

- **Pure gain tuning:** no set is faster than about 5 s.
- **`roll_kff` as a step:** excites the rocking mode.
- **Setpoint ramp:** overshoots too much with a heavy head on a short pendulum.

### 3.3 ZVD input shaper (optional)

The roll setpoint is split into three steps (¼, ½, ¼) spaced T/2 apart. A setpoint step then barely excites the rocking mode, and the controller tolerates considerably more gain.

Downside: the lean starts later (t50 about 1.7 s instead of 0.95 s), and there is a small setback after just over 1 s. The period T depends on the masses and has to be retuned after weighing the head. The shaper is in the firmware but off by default (`roll_shaper` = 0).

### 3.4 Servo load and reduction

Exploratory runs with the earlier model and the old head settings, not part of `check.js`. The roll loop does not depend on `controlGain`.

| Case (20° step) | Max load | > 90% | Result |
|---|---|---|---|
| 80/22, recommended 0.5 / 0.4 / 0.4 | 34% | – | unstable: through the reduction only a quarter of the profile acceleration reaches the pendulum |
| 1:1, `roll_direct`, accel 30 | 100% | about 0.7 s | rocks up and tips over; matches the observed emergency shutdowns |
| 1:1, recommended 0.5 / 0.4 / 0.4 | 21–33% | – | sufficient reserve |
| 100:70, accel 300, 1.0 / 0.7 / 0.4 | about 71% | – | about 3.4 s, but tight |

### 3.5 I bleed when steering back

**Observation:** after releasing the stick, the droid keeps cornering for a long time. Actively countersteering fixes this quickly, without oscillation.

**Cause:** in a steady curve the control error is zero, and the pendulum deflection is held almost entirely by the I state. On release, the P term reacts at once, but the I state only drains in proportion to the error. More K_p on the way back makes it worse: the error shrinks faster, so the I state drains even more slowly. Countersteering works because it deliberately makes the error large.

**Solution:**
- When the stick moves towards center, a time window opens (`roll_return_window`, 1.5 s).
- While the setpoint is smaller in magnitude than the lean, the I state is bled off with `roll_ibleed_tau` (0.4 s).
- It is only bled proportionally, towards I_start·goal_new/goal_start. Partial steering back (10° → 5°) therefore doesn't sag through.
- Disturbances at setpoint 0, such as a head turned sideways, are still fully corrected, because the window only opens after a stick movement.

| From a 10° curve at 0.75 m/s setpoint (current defaults) | upright (< 1°) | Undershoot | Keeps turning | Load |
|---|---|---|---|---|
| without I bleed | 6.4 s | 0° | 60° | 8% |
| K_p 1.5 on the way back | 7.2 s | 0° | 60° | 25% |
| **I bleed 0.4 s** | 1.0 s | −0.3° | 18° | 10% |

Further results (exploratory, earlier model with the old head settings):
- 10° → 5° settles to ±0.5° after 1.0 s instead of 4.5 s.
- Over ten mass variants, the worst case is 2.2 s, 1° undershoot and 32% load.

### 3.6 Additional lead

Exploratory, earlier model. Lead only helps if the head is heavy. What matters is the ratio m_p·l / (m_h·L).

- **Head 1 kg (ratio 2.2):** lead brings no speed, only more servo load.
- **Head 2 kg without lead (ratio 1.1):** close to the static tipping limit, 40% overshoot or 10 s settling, and 66–68% load.
- **Head 2 kg with 1 kg of lead about 200 mm below the center:** good again, about 2–2.5 s.

### 3.7 Roll stability at speed

**Observation (Björn):** at full speed with no roll input, move the head fully to one side, hold it, then move it straight to the other side. The roll loop swings up, and when the droid is stopped it falls over on its side.

**Reproduced in simulation:** with `body_roll_accel` 30 and constant `roll_kd` 0.4, the lean starts a growing oscillation at about 2.4 m/s as soon as the head moves, with ±25–35° of lean, yaw rates of ±300°/s and the roll servo at 100% load. After the stop the lean reaches −55° and the roll I-term winds up to its limit. On the droid, 100% load means the servo's overload shutdown, after which it tips over.

**Cause.** Isolated by switching parts of the model off:

| Experiment (full speed, head swing) | Result |
|---|---|
| cornering model off (no yaw from lean, no centrifugal force) | stable |
| head compensation off, or head mass 0 | still unstable (the head swing is only the trigger) |
| `body_roll_accel` unlimited | stable |
| `roll_kd` 0.1 instead of 0.4 | stable |
| kp = ki = 0, kd 0.4 | unstable |
| no roll control at all | stable |

When the droid leans at speed, it yaws (ψ̇ = v/R·sin ρ), and the centrifugal force pushes it back upright. That restoring effect grows with v², so the roll plane gets much stiffer and its natural motion much faster than at standstill. The gyro damping then asks for tilt changes faster than the servo's acceleration-limited profile can deliver, and the lag turns damping into excitation. Small disturbances stay below the acceleration limit, so they stay stable. That's why it depends on amplitude: a ±2 servo degree head swing is harmless, ±5 is enough. A heavier or lower pendulum raises the roll natural frequency too, so the variant with 5 kg @ 165 mm already swings up at about 0.8 m/s.

**Fix:** two changes, both of which are needed:

1. **Speed-scheduled gyro damping**, `roll_kd_speed` (1000 mm/s): above this measured speed, `roll_kd` is scaled by `roll_kd_speed`/|v|. The tolerated damping falls off roughly as 1/v (largest kd stable over all variants: about 0.4 up to 1 m/s, 0.25 at 2.6 m/s with `body_roll_accel` 60). The **measured** encoder speed is used, not the setpoint: when braking, the setpoint drops before the droid slows down, and full damping would come back while it is still fast.
2. **`body_roll_accel` 60** instead of 30 roughly doubles the damping the loop tolerates at speed.

| `robustness.js`: 11 speeds × 9 mass variants × 6 disturbances (594 cases) | failed (load > 80% or lean > 25°) | worst lean | worst load |
|---|---|---|---|
| `body_roll_accel` 30, no kd scheduling (before) | 193 | 174° | 100% |
| `body_roll_accel` 30 + `roll_kd_speed` 1000 | 14 | 65° | 100% |
| `body_roll_accel` 60, no kd scheduling | 74 | 158° | 100% |
| **`body_roll_accel` 60 + `roll_kd_speed` 1000** | **0** | 16° (includes the commanded ±10° slalom) | 72% |

The disturbances are full and half head swings, ±5° and ±10° roll-stick slaloms, and head swing plus slalom combined, each ending with a stop. A head swing combined with a ±10° slalom *at standstill* (up to 30° lean reversal) exceeds the roll servo's torque even with the fix. That's a capacity limit, not an instability, and is not part of the sweep.

| Björn's scenario (`check.js`): full speed, head right 3 s, head left 3 s, stop | max lean driving | after stop | roll servo load |
|---|---|---|---|
| before | 36° | 56° | 100% |
| kd scheduling only | 3.6° | 2.1° | 24% |
| `body_roll_accel` 60 only | 54° | 76° | 100% |
| **new defaults** | 3.3° | 1.9° | 20% |

**Limits of the model**, important here:
- The cornering model assumes no slip. At large lean and high speed the real lateral acceleration is lower (the sphere slides), so the simulation probably overstates the effect at large amplitudes. The onset is the part that matters.
- The IMU sits on the pendulum frame, which pitches forward at speed (about 16° steady, much more while accelerating). The roll gyro then also picks up yaw rate × sin(pitch). That is not modelled, and it adds a further lean → yaw → measured roll rate coupling. If the hardware still oscillates at speed, this is the first suspect; lower `roll_kd_speed`.

---

## 4. Speed and top speed

At full throttle with `drive_speed_max` = 3000 mm/s and a ramp of 2000 mm/s², three limits act one after the other:

1. **Balance deduction:** while driving, the pendulum stays forward because it has to push against rolling resistance and friction. The balance controller subtracts `bal_kp` × pitch from the setpoint (25 (mm/s)/° × pendulum angle).
2. **I bound of the speed controller:** the old firmware capped the I share at about 153 PWM (section 2, item 1). The new bound is 255 PWM.
3. **Motor:** beyond that, the drive is the limit (estimated 2.6 m/s at no load).

| Full throttle | Actual | Pendulum steady | Pendulum max | PWM | Head at speed |
|---|---|---|---|---|---|
| old head settings, old I bound (≤ 153 PWM) | 1852 mm/s | 34° | 74° | 198 | 52° back |
| old head settings, new I bound (≤ 255 PWM) | 2074 mm/s | 37° | 74° | 221 | 54° back |
| new defaults, old I bound | 2167 mm/s | 13° | 68° | 231 | 2° fwd |
| **new defaults, new I bound** | 2401 mm/s | 16° | 69° | 255 | 1° back |

Conclusions:

- **The head feedforward is the bigger lever, not the I bound.** With the old settings the head lies far back at speed, so the pendulum has to swing far forward to compensate, and the balance controller takes a lot off the setpoint. With the head held upright, the steady pendulum angle drops from about 35° to about 15°.
- **The I bound** adds about 10% and makes `drive_speed_imax` mean what its name says.
- **Don't simply remove the balance deduction.** An I term in the balance controller (`bal_ki`) tries to regulate the physically required pendulum angle to 0 and brakes the droid. A cleaner fix would be a speed-dependent feedforward on the balance setpoint, i.e. the expected pendulum angle for the rolling resistance at speed v. That requires measuring the rolling resistance first. (A high-pass in front of the balance controller let the pendulum swing past 90° in the earlier model, so it is not a solution either.)
- **The pendulum swings up to about 70° while accelerating at full throttle**, in every configuration. That is a reason not to set `drive_speed_max` above about 2000 mm/s until the motor data and rolling resistance are known.

All figures depend strongly on the estimated rolling resistance (3 N/(m/s)) and the estimated motor data.

---

## 5. Head compensation at standstill

**Goal:** head motion should cause neither acceleration nor lean. Actual acceleration and actual roll should come only from the body stick.

**Static ideal values** from the balance of the center of mass over the contact point:

- **Roll** (head on the axle, pendulum relative to the axle):
  `fa_dome_roll_to_weight = −m_h·L / (m_p·l)` = **−0.46**
- **Pitch** (head on the pendulum frame, swings with it, hence the smaller denominator):
  `fa_dome_pitch_to_weight = −m_h·L / (m_p·l − m_h·L)` = **−0.85**

Both are now the defaults in the firmware and in the simulation.

**Implementation:**
- **Roll:** feedforward on the pendulum tilt, as before.
- **Pitch:** a shift of the balance setpoint by `fa_dome_pitch_to_weight` × the measured dome pitch position. The controller then expects the pendulum where it balances the head, and doesn't turn that into a speed command.

| Head 30 servo deg forward / 20 servo deg sideways, at standstill | without | with |
|---|---|---|
| Drift after 8 s | 1.80 m (rolls away at 252 mm/s) | 0.06 m |
| Max sideways lean | 6.3° | 2.1° |

Signs are in the simulation's convention (pitch positive = pendulum forward, head positive = forward or in the lean direction). Check on hardware, see section 8.

---

## 6. Head attitude when pulling away and braking

### 6.1 Problem

When accelerating, the pendulum swings forward. Because the mast sits on the pendulum frame, the head tips back, by up to 56° on a stick step. The old `fa_speed_sp_to_dome` = −0.01 made that worse. At constant speed the pendulum stays forward because of rolling resistance, and the head lies back accordingly.

### 6.2 Rejected: head follows IMU pitch

A factor `fa_pitch_to_dome` with dome pitch = k·IMU pitch was stable in simulation, but only because of the ideal IMU. On the hardware, servo, frame reaction and IMU close an oscillating loop. Björn tried this earlier without success. On top of that, the tracking is positive feedback (head forward → CoM forward → more acceleration). It is not in the firmware.

### 6.3 Implemented: pure feedforward from the joystick setpoint

1. **`drive_accel_max`** (2000 mm/s²): a ramp on the speed setpoint from the stick. This makes the commanded acceleration a_cmd a clean signal, even for stick steps.
2. **`fa_accel_to_dome`** (15° per m/s²): head offset ∝ a_cmd, low-passed with `dome_accel_tau` (0.5 s) so the head moves roughly with the pendulum. For reference: quasi-statically the pendulum deflects by θ ≈ a·(R·M_eff − m_p·l)/(m_p·g·l), here about 16° per m/s².
3. **`fa_speed_sp_to_dome`** (+0.01): compensates the pendulum deflection at constant speed.
4. **`dome_comp_remote_only`** (true): the dome pitch compensation only acts on the remote share of the head. If it acts on the automatic feedforward too, it fights acceleration and braking.

No signal goes through the IMU or the encoder, so there is no feedback.

| Stick step to half speed (1.5 m/s) | old firmware head settings | IMU tracking (rejected) | **Feedforward (new defaults)** |
|---|---|---|---|
| Pendulum deflection | 46° | 35° | 33° |
| Head when pulling away | 56° back | 8° fwd / 23° back | 3° fwd / 12° back |
| Head at constant speed | 28° back | 2° fwd | 1° back |
| Head when braking | 28° back, then 28° fwd | 12° back / 25° fwd | 4° back / 2° fwd |
| Stopped after release | 2.3 s | 2.4 s | 1.9 s |

With the corrected (stiffer) speed loop, the pendulum swings forward sharply when pulling away, and a feedforward from the ramped setpoint can't completely keep up: the head briefly drops back by about 12°. Braking is almost neutral. A grid over `fa_accel_to_dome` 10–35, `dome_accel_tau` 0.1–0.5 s and `fa_speed_sp_to_dome` 0.005–0.015, checked over all mass variants, confirmed 15 / 0.5 s / 0.01 as the best combination (worst case over the variants 16.5° of head deflection). Ways to improve further: a separate factor for pulling away, or a gentler `drive_accel_max`.

---

## 7. Firmware changes

### 7.1 Changes

| File | Change |
|---|---|
| `BB8Config.h` | `ROLL_REDUCTION` = 1.0; new constant `DOME_PITCH_REDUCTION` = 2.0 (**assumption**); roll defaults 0.5 / 0.4 / 0.4; `body_roll_accel` 60; ideal head set as defaults; new parameters (see below) |
| `BB8Controllers.h/.cpp` | `BB8RollController`: speed-scheduled gyro damping (`setKdSpeedScheduling()`, `setSpeed()`), I bleed when steering back, optional ZV/ZVD shaper with a ring buffer (401 values, 4 s at 100 Hz), `setRollGoal()`/`rawGoal()`. Shaping happens in `update()`, the ramp stays 0. No second call to `input_.present()`. |
| `BB8Droid.h` | members `velGoal_`, `velRamped_`, `accelCmdFilt_`; method `domePitchAngle()` |
| `BB8Droid.cpp` | setpoint ramp and commanded acceleration in `stepDriveMotor()`; balance setpoint = `fa_dome_pitch_to_weight` × `domePitchAngle()`; head feedforward in `stepDome()`; balance ramp 0; I bound of the speed controller on the I share in PWM, including `controlGain()` (`drive_speed_imax`); roll setpoint via `setRollGoal()`; measured encoder speed passed to the roll controller every cycle; `roll_direct` and `fa_roll_sp_to_dome` use the unshaped setpoint |
| `BB8Droid.cpp` (other fixes) | remote loss (> 500 ms) also returns the roll setpoint to 0 and releases the dome; `stop()` switches the drive and servo torque off; console `mode` goes through `setMode()` (`off` switches the drive and torque off, leaving `off` switches torque back on, no drive toggle in `MODE_OFF`); running status shows the last primary packet |

### 7.2 New and changed parameters

The head parameters are the simulation values for the default masses. Their **signs depend on the hardware convention and are not verified** (section 8).

| Parameter | Default | Previously | Sign depends on hardware |
|---|---|---|---|
| `roll_kp` / `roll_ki` / `roll_kd` | 0.5 / 0.4 / 0.4 | 0.3 / 0 / 0.1 | no |
| `roll_kd_speed` | 1000 mm/s | – | no |
| `body_roll_accel` | 60 | 30 | no |
| `roll_ibleed_tau` | 0.4 s | – | no |
| `roll_return_window` | 1.5 s | – | no |
| `roll_shaper` / `roll_shaper_t` | 0 (off) / 2.3 s; optionally 2 / 2.3 s with 0.85 / 1.3 / 0.3 | – | no |
| `drive_accel_max` | 2000 mm/s² | – | no |
| `drive_speed_imax` | 255 PWM (bounds the I share) | errI ≤ 255 (≈ 153 PWM) | no |
| `fa_dome_roll_to_weight` | **−0.46** | 0 | **yes** |
| `fa_dome_pitch_to_weight` | **−0.85** | 0.5 (unused) | **yes** |
| `fa_accel_to_dome` | **15** | – | **yes** |
| `dome_accel_tau` | 0.5 s | – | no |
| `fa_speed_sp_to_dome` | **+0.01** | −0.01 | **yes** |
| `dome_comp_remote_only` | true | – | no |
| `ROLL_REDUCTION` (constant) | 1.0 (or 100/70 with changed lever arms) | 80/22 | no |

The compensation factors depend directly on m_h·L. Recompute them after weighing the head:
- Roll: −m_h·L/(m_p·l)
- Pitch: −m_h·L/(m_p·l − m_h·L)

The button "Ideal head compensation for these masses" in the simulation does this for the masses entered there.

---

## 8. Commissioning on the hardware (proposal)

The defaults now contain compensation values whose signs come from the simulation. **If a sign is wrong, the effect doubles instead of cancelling.** So check each one before driving freely, and flip the sign if it's wrong.

1. **Build and flash:** `pio run` in `Arduino/BB8Droid`.
2. **Check the roll axis:** with `ROLL_REDUCTION` = 1 and `body_roll_range` 25, the servo range is ±25 servo degrees. Before anything else, check at standstill with the drive off that the pendulum has its full travel free.
3. **Roll controller:** first at standstill or on the test stand, with small steps (5°) and `roll_debug` on. The gyro rate must match the angle rate in sign (comment in `setControlOutput()`). Watch the servo load (`maxRollLoad_`).
4. **Roll at speed (section 3.7):** raise the speed step by step, at each step make a small head swing from side to side, and watch for oscillation of the lean. `roll_debug` prints the effective kd. If it oscillates, lower `roll_kd_speed` (e.g. 700). Only then try the full head swing at full speed, and stop from there.
5. **Sign of `fa_dome_roll_to_weight`** (acts even with the drive off): at standstill, move the head sideways and watch which way the droid leans. Compare with `fa_dome_roll_to_weight 0`. If the lean gets worse with −0.46, use +0.46.
6. **Sign of `fa_dome_pitch_to_weight`** (only acts with the drive on): at standstill with the drive on, move the head forward/back. With the right sign, the droid rolls away less than with 0.
7. **Signs of `fa_accel_to_dome` and `fa_speed_sp_to_dome`:** pull away gently. The head should lean forward when accelerating and stay upright at constant speed. If it leans back at constant speed with +0.01, the old −0.01 was right for the hardware, and `fa_accel_to_dome` very likely needs flipping too. To be careful, start with `fa_accel_to_dome` 5.
8. **Shaper (optional):** measure the rocking period (lean step with `roll_direct`, time between two maxima), set `roll_shaper_t` to it and `roll_shaper` = 2, then raise the gains.

---

## 9. What has been checked and what hasn't

- **Simulation core:** step responses, robustness over mass variants and servo load by script. `check.js` reproduces the tables in sections 3.2, 3.5, 3.7 (Björn's scenario), 4, 5 and 6.3; `robustness.js` the sweep in section 3.7. Figures marked as exploratory come from earlier runs.
- **UI:** tested in headless Chrome during development (desktop and phone, light and dark; keyboard, sticks, cornering). After the translation into English, only the script syntax was checked, not the page in a browser.
- **Firmware:**
  - The complete firmware builds with `pio run` (MKR WiFi 1010) without warnings in the changed code.
  - `BB8Controllers.cpp` was additionally logic-tested against a simple plant: the I bleed shortens straightening up from 4.2 s to 1.0 s, and ZVD produces the steps ¼, ½, ¼.
  - Not run on the droid.
- **Not modelled:**
  - Madgwick delay and IMU noise
  - servo dead time
  - coupling between the pitch and roll planes (except cornering)
  - slip and spin friction
  - position modes
  - dynamics of the dome heading and the head magnets
  - yaw rate leaking into the roll gyro when the frame pitches (section 3.7)
  - range limits of the dome pitch servo interacting with the feedforward (at high acceleration plus high speed, the commanded dome pitch can hit `dome_pitch_range`, after which `dome_comp_remote_only` subtracts more than the servo actually does)

## 10. Open items

1. Weigh the head and measure its CoM height. Then recompute the compensation factors and the shaper period (`check.js` and the simulation), and recheck the roll overshoot (section 3.2).
2. Measure the dome pitch servo reduction (`DOME_PITCH_REDUCTION`).
3. Motor data of the drive (stall torque, no-load speed at the output).
4. Measure rolling resistance (coast-down test). Afterwards, optionally add a speed-dependent feedforward on the balance setpoint (section 4).
5. Fix the position controllers (section 2, item 10).
6. Set `body_roll_vel` to a realistic value (XM430-W350: about 200 units ≈ 275°/s).
7. Head drops back about 12° when pulling away: a separate factor for pulling away, or a gentler acceleration ramp (section 6.3).
8. Verify roll stability at speed on the hardware (section 3.7, commissioning step 4). If needed, model the yaw-into-roll-gyro coupling and retune `roll_kd_speed`.

## 11. Using and extending the simulation

- **Open:** `BB8-DriveSimulation.html` in a browser.
- **Keyboard:** W/S accelerate and brake, A/D steer, Space pause. Right stick for the head (mouse or trackpad).
- **Buttons:**
  - "Roll set + I bleed (recommended)"
  - "Roll set ZVD + I bleed"
  - "Ideal head compensation for these masses": recomputes the compensation for the masses entered and sets the feedforward
  - "Defaults": the controller parameters as in the firmware
- **Reproduce the figures:** `node check.js` and `node robustness.js`, or `./check-jsc.sh` and `./check-jsc.sh robustness.js` on a Mac without Node.
- **Rebuild the HTML** after changes to `sim.js` or `page.html`: `python3 build.py` (CDN variant). For the offline variant, put `three.min.js` r128 next to the script and run `python3 build.py --offline` (see the comment in `build.py`).
