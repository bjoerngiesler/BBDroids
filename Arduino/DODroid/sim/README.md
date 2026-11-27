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

## Obstacle course and zoom

- **Obstacle course** (button "Load obstacle course"): the same course as in the BB-8 simulations, from `../../BB8Droid/sim/course.js` (one source, inserted by `build.py`): 5-cone slalom, ramps at 5 / 10 / 12.5 / 15 %, 10 % seesaw that tips under the droid's weight, straight ahead from the start. Cone hits are counted.
- The physics uses the slope at the wheel contact along the heading. When it changes (ramp edges, seesaw, turning on a slope), chassis and neck keep their world attitude (the slope-frame state is shifted). D-O has no roll plane in this model, so the sideways component of a slope is ignored.
- In velocity mode D-O stalls on the steeper ramps at low speed (stick 0.3: start of the 10 % ramp; stick 0.6: rising half of the seesaw): the balance controller subtracts the chassis angle needed on the slope from the wheel speed setpoint, as on the BB-8.
- **Zoom** slider (and mouse wheel) for the 3D view, 0.5 ... 10x.

## Changes against the first version (2026-11-25)

- **Wheel speed controller gain:** LibBB multiplies the PID control term by the input's `controlGain()`. The wheel speed controllers take the encoders in mm, so that is 1/mmPerTick = `WHEEL_TICKS_PER_TURN` / `WHEEL_CIRCUMFERENCE` ≈ 7.3. The first version left it out, so its wheel loops were 7.3× softer than on the droid (same issue as found in the BB-8 simulation).
- **Firmware encoder scale:** the firmware converts ticks with `WHEEL_CIRCUMFERENCE` = 722.566 mm (r = 0.115 m), the physics uses the CAD radius 0.126 m. The simulated encoders now report what the firmware sees, so firmware speeds and positions are about 9% smaller than the real ones.
- Effect, full stick and release: top speed 961 → 1023 mm/s, PWM max 171 → 213, standstill sway in auto position 2.1° → 1.8°; stable.
- Drive mode selector and the manual position mode `DRIVE_POS` added; the `auto_pos_control` checkbox is replaced by the selector (saved settings with it switched off load as velocity mode).

## Physical remote

The sim can be driven by a real remote through a **SimBridge** board (an ESP32-S3 running
`BBRemotes/examples/SimBridge`): pair the remote with "SimBridge" like with a droid (ESP-NOW), plug the
board into the PC via USB, and click **Connect remote** (Chrome or Edge -- it uses the Web Serial API).
The bridge uses D-O's input names and default mixes: speed and turn from the joystick, head pitch and
heading from the remote's orientation, neck lean from the secondary remote. While data arrives, the remote
overrides the on-screen sticks and keys; if data stops for 0.5 s, drive stops, like the firmware's watchdog.
