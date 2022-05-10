# BB-8

Code for the Bavarian R2 Builders' BB-8.

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
