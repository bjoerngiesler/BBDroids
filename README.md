# BB-8

Code for the Bavarian R2 Builders' Droid Control System, consisting of dual miniature multi-axis remote controls and a droid control board, currently for BB-8.

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

## Software

### Getting started

TODO

### Structure

The remote and droid codes share quite a bit of code (console, runloop, XBee and Wifi comm, etc.). This joint code resides in the LibBB folder. This should go into your Arduino libraries folder. Where that is depends on your operating system; on Mac, it's ~/Documents/Arduino/libraries.

LibBB contains all the code required to setup and run a system you can communicate with, but it won't actually do anything because it does not contain subsystems for motion, sound, lights, or displays. Sketches using LibBB should provide these as their own subsystems.

### Subsystem Concept

Subsystems are runnable entities that can be initialized, started, stopped, and periodically called by the runloop to perform work. Subsystems have names, descriptions and help texts. Since subsystems are identified by name, the names must be unique. Subsystems can expose parameters that can be read and modified from the command line, and that can be loaded from and stored into flash memory. Subsystems can also offer console commands.

Standard subsystems offered by LibBB are
- console
- runloop
- wifi
- xbee.
A sketch based on LibBB is expected to 

In code, subsystems inherit from the virtual class <Subsystem>, overwriting the start(), stop(), and step() methods, and optionally the initialize(), getParameterValue(), setParameterValue(), and handleConsoleCommand() methods. They are required to maintain the started_ (bool) and operationStatus_ (Result) members of their base class. 

As a convention, subsystems are typically singletons objects (there can be only one wifi subsystem, for example). This is achieved by setting the constructor protected or private, and adding one static instance of the class as a class member variable, like so:

class MySubsystem: virtual Subsystem {
public:
	static MySubsystem mysub;
	virtual Result initialize();
	virtual Result start();
	virtual Result stop();
	virtual Result step();
protected:
	MySubsystem();
};

### Runloop

The runloop is a subsystem that cyclically calls all other subsystems' step() methods in an infinite loop. It tries to maintain a cycle time (currently set at compile time) by measuring the sum of time all step() methods consume, and then sleeping for the rest. Please note that this is not guaranteed - if the sum of all step() methods takes too much time the cycle time will not be held. There is currently no mechanism to penalize individual subsystems for taking too much time. 

The unloop's whose start() method contains an infinite loop. It is not stoppable by design. A way to stop it programmatically may be added in the future, but this cannot be possible from the console because the console itself depends on the runloop running, so stopping it would disable the console and remove any way of restarting it beyond a reset.

### Console

LibBB provides code for using a command line to interact with the system. Some standard commands are provided to inquire system status, get help and status on individual subsystems, start / stop / restart individual subsystems, get and set parameter values and store them to flash. Several consoles can be opened at the same time, each getting its individual interaction stream, and broadcast messages can be sent to all of them.

The console expects line feeds as end of line characters (set your terminal to send LF or CRLF, CR alone will not work), and uses space to separate arguments. If string arguments contain spaces, use double quotes ("") to enclose them. Double quotes within strings can be escaped with a backslash. So if your Wifi key is 'My"Great Droid', specify it on the command line as '"My\"Great Droid"' (remove single quotes for both).

In code, the console subsystem uses subclasses of the <ConsoleStream> class for interaction over different communication channels. One serial console stream is always active and listening on <Serial>, so it can be accessed via the Arduino's USB port. The wifi subsystem provides another console stream.

### Wifi

The wifi subsystem uses the WiFiNINA library to interact with the Wifi module installed on the Arduino. It can be configured to connect to existing infrastructure networks or to create access points. For this purpose it offers parameters for SSID, WPA key, and a flag that states whether to connect to infrastructure or open an access point. The wifi subsystem offers a TCP server on a configurable port (3000 by default, can be set via parameter) that exposes a console.

The wifi subsystem has also been used in the past to send commands and state from the droid to the remote and back. Due to the very limited Wifi range of the Arduinos this is not currently supported, but it may come back in the future.

### XBee

The xbee subsystem uses one of the Arduino's serial ports to communicate with an installed XBee module. Configuration is entirely done with XBee AT commands, no pre-configuration using XCTU or other tools is necessary (but can be done of course). It will auto-detect the connected XBee's bps rate and set channel, PAN, station and partner ID according to parameters.

The xbee subsystem is the main communication backbone. It exposes a delegate class <PacketReceiver> that should be used as base class for all that want to receive packets in the system. At the moment, it operates in transparent mode, which is probably not optimal; it may be changed to API mode in the future. In its current implementation, all bytes in a packet are required to have their high bit set to 0, because a byte with a high bit is used to detect packet boundaries. The packet format is protected using a 7-bit CRC and has a well-defined length, both of which serve as protection against packet corruption.

