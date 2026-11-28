# NewRemote handoff notes (2026-11-28)

Context for whoever works on NewRemote next (e.g. a Claude session in another window). Written at the end of a BBRemotes session; delete this file once it has been absorbed.

## What changed recently

- **Per-block protocol storage** (`d7c9472`, `8f6e786`): NVS keys `proto_dir` and `proto_<n>` instead of the single `proto_s` blob. The old blob is erased on boot (`removeLegacyStorage()` in `RemoteSubsys.cpp`). The 1 s delay after each storage write is gone.
- **Battery bar crash fixed** (`c5a2dbd`): `RemoteVisWidget` indexed `batteryState_[-1]` when the battery reading was below 1/6, e.g. with a dead battery. That caused a reset loop. Both `visualizeFromInput()` and `visualizeFromControlPacket()` now use `showBattery()`.
- **Monaco/UDP** (`4d1177e`, merged): config menu "New... → Monaco/UDP". Wi-Fi is set up from `include/Secrets.h`: either the remote's own access point or joining a network. `Secrets.h` is in `.gitignore`; the pre-build script creates it from the committed `Secrets.example.h` if it's missing. Wi-Fi only comes up when a UDP protocol starts, via `MUDPProtocol::setNetworkSetupCallback()` in `RemoteSubsys::initialize()`. Tested by the user against `udp_rx` and `udp_simbridge` on a Mac (BBRemotes `examples/HostUDP`).

## Open problem: no serial input from display and XBee (left remote)

Symptoms since flashing on 2026-11-27:
- Sending works on both UARTs: the display shows what it should.
- Nothing is ever received:
  - The XBee gives "no response entering AT mode" at every baud rate.
  - The display gives "Timeout waiting for display reply" for every command.
- It happens with and without battery power. Before that flash, everything worked.

Findings so far:
- **Not the firmware code:** a build of NewRemote, LibBB and BBRemotes exactly as they were before the session (Code `195c13b~1`, BBRemotes `dcb74df`) fails the same way. The receive code paths and the pin setup (`Serial1`/`Serial2.setPins` in `main.cpp`) are unchanged. The only console stream is USB `Serial`, so nothing else consumes UART input.
- **Two independent UARTs failing the same direction** points to something they share: receive lines, a level shifter or buffer, supply, or ground. The user will check with a multimeter.
- **State that survives reflashing,** so flashing an old build doesn't restore the old state:
  - **NVS** (protocol keys, LibBB `ConfigStorage`). `pio run -t erase` before flashing would rule it out, but it also erases stored protocols and calibration.
  - **The bootloader and partition table,** rewritten by every PlatformIO upload.
  - **The XBee's own settings:** `MXBProtocol::init()` writes baud 230400 and other settings with `ATWR`. Testing the XBee directly with a USB-serial adapter would rule it out. The XBee wouldn't explain the display, though.
- **Boot is slow while this persists:** XBee auto-detection tries 8 baud rates of about 2 s each, and it runs twice (interremote load fails, then create). Expect 20–40 s before the console is ready.

## Smaller issues seen in the boot log

- `Loading protocol "l��?"`: the interremote protocol's name is printed with `%s` without `.c_str()`, probably near the top of `RemoteSubsys::initialize()`. The build uses `-Wno-format`, so the compiler doesn't warn.
- `Wire.endTransmission() returns error 2 while detecting IMU at 0x6a`: the IMU is then found and initialized, apparently at its other address. Probably harmless.
- Post-build `esptool merge-bin failed` appeared once; the next upload was fine.

## Known design issues, fix with care

- **IMU pitch/roll swap:** LibBB `IMU::getFilteredPRH()` swaps pitch and roll on purpose. NewRemote sends roll on axis 2 and pitch on axis 3, and NewDODroid's default mixes swap them back. The droids use the same IMU function in their balance and drive controllers, so fixing it means auditing every caller and changing the remote, the droid mixes and stored mixes together.
  - Head heading and head roll are also reversed relative to the D-O head; this is fixed only in SimBridge's mixes so far.
  - Gimbal lock comes from sending Euler angles. The user's preferred fix: compute with quaternions on the remote, keep Euler angles on the wire, send singularity-compensated angles.
- **Wi-Fi and ESP-NOW:** once Monaco/UDP has brought Wi-Fi up, the radio channel is fixed until reboot, so an ESP-NOW protocol in the same session may not reach droids on another channel. NewRemote's UDP setup uses `WIFI_AP_STA` in access-point mode to keep the station interface for ESP-NOW.
- **BB-8 isn't on BBRemotes yet:** it uses the old XBee `ControlPacket` code. Its axis signs differ from D-O's (head pitch, roll and heading reversed), see `BB8Droid/sim/README.md`.

## Where things are

- BBRemotes (library): `../../../BBRemotes`, symlinked via `platformio.ini`. Host build and UDP examples are in `examples/HostUDP`; SimBridge (ESP-NOW to USB) is in `examples/SimBridge`.
- Sims: `../DODroid/sim`, `../BB8Droid/sim`. Both have "Connect remote" (Web Serial to SimBridge) and "Network remote" (WebSocket to `udp_simbridge`).
- Goal behind all this: a sim2real loop. Drive the sim from a network remote, then the sim and the real droid together, then fit sim parameters and tune controllers in sim.
