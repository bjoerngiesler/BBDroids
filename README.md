# BB-8

Code for the Bavarian R2 Builders' Droid Control System, consisting of dual miniature multi-axis remote controls and a droid control board, currently for BB-8.

Hardware concept, layout and realization by Felix Beyer, software concept and realization by Björn Giesler.

## About

The project started with the plan to build a RC BB-8 replica, and has since evolved to a general-purpose remote control, a slightly less general-purpose droid control board, and a very much general-purpose software system and architecture. It can be used to realize and control almost arbitrary droids, e.g. BB-8, R2-D2, B2EMO, and many others.

From an end-user perspective, the most important aspect are the two remotes used to control the droid, one for the right and one for the left hand. Both have a 2-axis clickable joystick, a 6-axis gyroscope and accelerometer (of which currently only the gyroscope is used), four chordable buttons for index finger, ring finger and pinky, and three additional buttons. The left remote contains a display displaying parameters, droid status, and other telemetry information, and has an encoder to navigate the display. The right remote has two potentiometers that can be operated with the thumb.

Electronically, the system is built on the basis of Arduinos. Currently, it is using Arduino RP2040 nano connect boards in the remotes and a MKR Wifi 1010 in the droid body. Communication between remotes and droid is done via XBee or alternatively via UDP (although this is not recommended for conventions or similar due to the limited range of the small integrated antennas).

All three components (two remotes and mainboard) offer command line consoles for deep introspection into system health, setting parameters, stopping and restarting subsystems, etc. The consoles can be accessed via serial port (directly accessible on the remotes) or alternatively via a TCP connection. For this purpose all three components can join infrastructure networks or (default) open their own individual access points, which are identifiable via MAC addresses so they will not clash even in crowded convention situations.

## Concepts

Contents:

## Arduino
Code for BB-8 and the Remote Control, all of these run on Arduino.

### BB8
Arduino code for running BB-8. Assumes
* an Arduino MKR Wifi 1010 as basic platform
* DC motors for drive and spot-turn
* Dynamixel servos for leaning and head movement
* Wifi/UDP connection with a remote control, although using a Spektrum transmitter is also supported

Edit Config.h to adapt.

### Remote
Arduino code for running the remote control. Assumes
* an Arduino MKR Wifi 1010 as basic platform
* Felix's remote control setup
* Wifi/UDP connection with BB-8.

Edit Config.h to adapt.
