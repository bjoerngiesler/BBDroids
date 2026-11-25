# D-O Drive Simulation

A browser simulation of the D-O drive: planar pitch model (wheels, chassis with counterweight below the axle, neck and head above it), yaw from the left/right wheel torque difference, and the DODroid / LibBB firmware control replicated at 100 Hz (head and drive alternate, 50 Hz each). Open `DO-Drive-Simulation.html` in a browser; it runs offline.

| File | Purpose |
|---|---|
| `DO-Drive-Simulation.html` | The simulation as a single file (three.js and the 3D model embedded) |
| `dosim.js` | Simulation core (physics + firmware replica) |
| `do-model.js` | 3D model data (`window.DO_MODEL`) |
| `page.html` | UI template; `build.py` fills in `/*DOMODEL*/` and `/*DOSIMJS*/` |
| `build.py` | Builds the HTML (`python3 build.py --offline` with `three.min.js` r128 next to it, or without `--offline` for the CDN variant) |

## Drive modes

Selector in the remote panel, as in the firmware (`DODroid::switchDrive()`):

- **Velocity** (`DRIVE_VEL`): stick = speed and rotation setpoint.
- **Position** (`DRIVE_POS`, secondary remote in the firmware): stick = position offset of ±`posStickRange` (firmware: axis × 1000 mm) around where the mode was entered; rotation is ignored. Gains `pos_kp` / `pos_ki` / `pos_kd` (1.2 / 0.2 / 0).
- **Auto position** (`auto_pos_control`, firmware default): velocity while the sticks are moved, `DRIVE_AUTO_POS` 500 ms after they are released. Gains `auto_pos_kp` / `auto_pos_ki` / `auto_pos_kd` (6 / 0.2 / 0).

Both position controllers use the average encoder position in mm (`DOPosControlInput`) and set the goal velocity (`DOPosControlOutput`).

## Changes against the first version (2026-11-25)

- **Wheel speed controller gain:** LibBB multiplies the PID control term by the input's `controlGain()`. The wheel speed controllers take the encoders in mm, so that is 1/mmPerTick = `WHEEL_TICKS_PER_TURN` / `WHEEL_CIRCUMFERENCE` ≈ 7.3. The first version left it out, so its wheel loops were 7.3× softer than on the droid (same issue as found in the BB-8 simulation).
- **Firmware encoder scale:** the firmware converts ticks with `WHEEL_CIRCUMFERENCE` = 722.566 mm (r = 0.115 m), the physics uses the CAD radius 0.126 m. The simulated encoders now report what the firmware sees, so firmware speeds and positions are about 9% smaller than the real ones.
- Effect, full stick and release: top speed 961 → 1023 mm/s, PWM max 171 → 213, standstill sway in auto position 2.1° → 1.8°; stable.
- Drive mode selector and the manual position mode `DRIVE_POS` added; the `auto_pos_control` checkbox is replaced by the selector (saved settings with it switched off load as velocity mode).
