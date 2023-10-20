# Droids

Code for the Bavarian R2 Builders' Droid Control System, consisting of dual miniature multi-axis remote controls and a droid control board, currently for BB-8 and D-O. Please refer to [the Wiki](https://github.com/bjoerngiesler/BB8/wiki/00-Home) for reference.

Hardware concept, layout and realization by Felix Beyer, software concept and realization by Björn Giesler.

*Please note that this is work in progress. We had a LOT to do coming up to Celebration Europe 2023, so a number of cleanups are still necessary in hardware, software and documentation. Watch this space for official releases!*

## About

The project started with the plan to build a RC BB-8 replica, and has since evolved to a general-purpose remote control, a slightly less general-purpose droid control board, and a very much general-purpose software system and architecture. It can be used to realize and control almost arbitrary droids, e.g. BB-8, R2-D2, B2EMO, and many others.

From an end-user perspective, the most important aspect are the two remotes used to control the droid, one for the right and one for the left hand. Both have a 2-axis clickable joystick, a 6-axis gyroscope and accelerometer (of which currently only the gyroscope is used), four chordable buttons for index finger, ring finger and pinky, and three additional buttons. The left remote contains a display displaying parameters, droid status, and other telemetry information, and has an encoder to navigate the display. The right remote has two potentiometers that can be operated with the thumb.

Electronically, the system is built on the basis of Arduinos. Currently, it is using Arduino RP2040 nano connect boards in the remotes and a MKR Wifi 1010 in the droid body. Communication between remotes and droid is done via XBee or alternatively via UDP (although this is not recommended for conventions or similar due to the limited range of the small integrated antennas).

All three components (two remotes and mainboard) offer command line consoles for deep introspection into system health, setting parameters, stopping and restarting subsystems, etc. The consoles can be accessed via serial port (directly accessible on the remotes) or alternatively via a TCP connection. For this purpose all three components can join infrastructure networks or (default) open their own individual access points, which are identifiable via MAC addresses so they will not clash even in crowded convention situations.

## Hardware

### Remotes

Both left and right remote use an Arduino RP2040 nano connect as their computing platforms, and require the Arduino-pico core to compile. Connected peripherals common to both remotes are:
- a two-axis analog PS/2 joystick
- a button integrated into the joystick
- four buttons along front and sides of the remote housing, nicknamed "left", "right", "index",  and "pinky"
- three buttons on top, nicknamed "top left", "top right", and "confirm"
- two RGB status LEDs (Adafruit Neopixels)
- a 6-axis accelerometer and gyro integrated on the RP2040 nano connect. The accelerometer is connected via i2c
- an XBee module
- a Wifi module integrated on the RP2040 nano connect.

#### Left

The left remote additionally sports a small graphic display and a digital encoder.

#### Right

The right remote additionally sports two analog potentiometers.

### Droid Control Board

The droid control bord uses an Arduino MKR Wifi 1010 plus a Dynamixel shied for servo control. Connected peripherals are:

- an XBee module (slot on the board)
- a DFPlayer Mini (slot on the board)
- two connectors for H-bridge motor drivers
- one connector for a differential motor encoder
- two connectors for i2c modules
- one connector for a Spektrum RC range extender (to be used as alternative to the XBee and remotes, not currently supported but will return in the future)
- two RGB status LEDs (Adafruit Neopixels)
- a Wifi module integrated on the MKR Wifi 1010.

The BB8 Arduino sketch expects the following external hardware:

- four Dynamixel servos with the following IDs: 1 (dome rotation), 2 (dome roll), 3 (dome pitch), 4 (body roll)
- one motor driver for the main drive, fitted with an encoder and connected to motor driver connector A
- one encoder fitted to the main motor drivier
- one motor driver for the spot-turn drive, connected to motor driver connector B (no encoder on this one)
- one 6DOF IMU (Adafruit ISM330DHCX) connected to i2c and mounted in the body center rigid to the droid body.
