# DODroid: Arduino Software for D-O

This software is intended to drive a self-balancing robot of the "D-O" type from the LucasFilm Star Wars movie Rise of Skywalker. It is intended to run on an Arduino MKR Wifi 1010, using an ISM330DHCX gyroscope, encoder motors and Dynamixel servos.

This software was written in 2023 and later by Björn Giesler <bjoern@giesler.de>. It is licensed under the Apache 2.0 license.

## TODO
- add mode switch
- implement speed control mode
- implement position control mode
- implement auto mode (position control if no remote input, speed control otherwise)
- check for max current draw
- move IMU code into LibBB
- move Servo code into LibBB
- move sound code into LibBB
- refactor Drive Controller and move into LibBB