# BBDroids

Code for BBDroids, the Bavarian Droid Builders' Droid Control System, currently for BB-8 and D-O. You can call this an "operating system" if you like. Please refer to [the Wiki](https://github.com/bjoerngiesler/BB8/wiki/00-Home) for reference and documentation.

Hardware concept, layout and realization by Felix Beyer, software concept and realization by Björn Giesler.

*Please note that this is work in progress.*

*Recent work in Q1/2024 has been prioritizing D-O, and the results look very promising, as the videos in this playlist show. https://www.youtube.com/playlist?list=PLQXuN0SWfVLk_Lird5GfVPj6_yB7YLnPW*

*Work on BB-8 is on hold until D-O software is finished, at which point the improvements will be ported over. This is expected to happen still in April 2024.*

*Watch this space for official releases!*

## About

The project started with the plan to build a RC BB-8 replica, and has since evolved to a general-purpose remote control, a slightly less general-purpose droid control board, and a very much general-purpose software system and architecture. It can be used to realize and control almost arbitrary droids, e.g. BB-8, R2-D2, B2EMO, and many others, although this repository currently contains complete applications for only D-O and BB-8.

From an end-user perspective, the most important aspect are the two remotes used to control the droid, one for the right and one for the left hand. Both have a 2-axis clickable joystick, a 6-axis gyroscope and accelerometer (of which currently only the gyroscope is used), four chordable buttons for index finger, ring finger and pinky, and three additional buttons. The left remote contains a display displaying parameters, droid status, and other telemetry information, and has an encoder to navigate the display. The right remote has two potentiometers that can be operated with the thumb.

Electronically, the system is built on the basis of Arduinos. Currently, it is using Arduino RP2040 nano connect boards in the remotes and a MKR Wifi 1010 in the droid body. Communication between remotes and droid is done via XBee or alternatively via UDP (although this is not recommended for conventions or similar due to the limited range of the small integrated antennas).

All three components (two remotes and mainboard) offer command line consoles for deep introspection into system health, setting parameters, stopping and restarting subsystems, etc. The consoles can be accessed via serial port (directly accessible on the remotes) or alternatively via a TCP connection. For this purpose all three components can join infrastructure networks or (default) open their own individual access points, which are identifiable via MAC addresses so they will not clash even in crowded convention situations.

## Hardware

### Remotes

Both left and right remote use an Arduino RP2040 nano connect as their computing platforms, and require the Arduino-pico core to compile. We are in the process of converting to ESP32 to remove this requirement. Connected peripherals common to both remotes are:
- a two-axis analog PS/2 joystick
- a button integrated into the joystick
- four buttons along front and sides of the remote housing, nicknamed "left", "right", "index",  and "pinky"
- three buttons on top, nicknamed "top left", "top right", and "confirm"
- two RGB status LEDs (Adafruit Neopixels)
- a 6-axis accelerometer and gyro integrated on the RP2040 nano connect, or an external one for ESPnow remotes. The accelerometer is connected via i2c.
- an XBee module (conversion to ESP32 will make this optional, and ESPnow control will be offered as an alternative).
- a Wifi module integrated on the RP2040 nano connect or ESP32.

#### Left

The left remote additionally sports a small graphic display and a digital encoder.

#### Right

The right remote additionally sports two analog potentiometers.

### Droid Control Board

The droid control bord uses an Arduino MKR Wifi 1010 plus a Dynamixel shield for servo control. Connected peripherals are:

- an XBee module (slot on the board)
- a DFPlayer Mini (slot on the board)
- two connectors for H-bridge motor drivers and differential motor encoders
- two STEMMA/QWIIC standard connectors for i2c modules
- one connector for a Spektrum RC range extender (to be used as alternative to the XBee and remotes, not currently supported but will return in the future)
- two RGB status LEDs (Adafruit Neopixels)
- a Wifi module integrated on the MKR Wifi 1010.

The BB8 Arduino sketch expects the following external hardware:

- four Dynamixel servos with the following IDs: 1 (dome rotation), 2 (dome roll), 3 (dome pitch), 4 (body roll)
- one motor driver for the main drive, fitted with an encoder and connected to motor driver connector A
- one encoder fitted to the main motor drivier
- one motor driver for the spot-turn drive, connected to motor driver connector B (no encoder on this one)
- two voltage/current meters (Adafruit INA219) connected to i2c and hooked up to each of the two battery half-rings
- one 6DOF IMU (Adafruit ISM330DHCX) connected to i2c and mounted in the body center rigid to the droid body.

The D-O Arduino sketch expects the following external hardware:

- four Dynamixel servos with the following IDs: 1 (neck tilt), 2 (head pitch), 3 (head yaw), 4 (head roll)
- two motor drivers for the left and right main drives, fitted with encoders
- one voltage/current meter (Adafruit INA219) connected to i2c and hooked up to the batteries
- one 6DOF IMU (Adafruit ISM330DHCX) connected to i2c and mounted in the body center rigid to the droid body.

## Software

### LibBB

This library (LibBB - Bavarian Builders Lib, *not* LibBB8) encapsulates some aspects of a middleware supporting very different droids. It forms the base of the BB8 and D-O code included in the repository, and can be used for your droid project as well. It provides the following concepts:

* *Subsystems* are what a droid is built out of. They can be initialized, started, stopped, and run within the main runloop. The Subsystem base class offers scheduling, parameter handling and console input/output.
* The *Runloop* (a subsystem) runs and schedules all other subsystems, with configurable update rate.
* The *Console* (a subsystem) provides command line input and output via the Arduino's Serial port or telnet via Wifi.
* The *Wifi Server* (a subsystem) lets the Arduino connect to an infrastructure Wifi network or create its own.
* The *Config Storage* lets you store subsystem parameters in flash or EEPROM.
* The *Controller Framework* provides controllers that can be used for motor position, speed, or servo control, together with input / output base classes that can be used to connect a controller to arbitrary system inputs and outputs. (Currently only PID control is supported, but other controller types may follow.)
* The *DC Motor* class communicates with different types of DC motor hardware.
* The *XBee* class provides communication and remote control via XBee / Zigbee modules.

### Example

Here is a simple example on how to construct a droid sketch in Arduino using LibBB:

```
#include <LibBB.h>
using namespace bb;

static const String WIFI_SSID = "MySSID";
static const String WIFI_WPA_KEY = "MyWifiKey";
static const bool WIFI_AP_MODE = true;

class MyDroidClass: public Subsystem, public PacketReceiver {
public:
	static MyDroidClass droid;

	virtual Result initialize() { 
		// Add your initialization code here
		return RES_OK;
	}
	virtual Result step() {
		// Add your droid control code here
		return RES_OK;
	}
};

void initializeSubsystems() {
  Runloop::runloop.initialize();
  Console::console.initialize();
  WifiServer::server.initialize(WIFI_SSID, WIFI_WPA_KEY, WIFI_AP_MODE);
  MyDroid::droid.initialize();
}

void startSubsystems() {
  WifiServer::server.start();
  Console::console.start();
  MyDroid::droid.start();
  Runloop::runloop.start(); // never returns; start last!
}

void setup() {
  Serial.begin(2000000);
  Serial.println();
  Serial.println("My Droid starting up");

  initializeSubsystems();
  startSubsystems();
}

void loop() {}
```
