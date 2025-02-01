# BBDroids

Code for BBDroids, the Bavarian Droid Builders' Droid Control System, currently for BB-8 and D-O. You can call this an "operating system" if you like. Please refer to [the Wiki](https://github.com/bjoerngiesler/BB8/wiki/00-Home) for reference and documentation.

Hardware concept, layout and realization by Felix Beyer, software concept and realization by Björn Giesler.

*Please note that this is work in progress.*

*2024 work has been prioritizing D-O, and the results look very promising, as the videos in this playlist show. https://www.youtube.com/playlist?list=PLQXuN0SWfVLk_Lird5GfVPj6_yB7YLnPW*

*Work on BB-8 is on hold until D-O software is finished, at which point the improvements will be ported over.*

*If you want to base your own build off of BBDroids, I would suggest to hold off until work on D-O and BB-8 is (mostly) finished and some cleanup work has been done.*

*Watch this space for official releases!*

## Download and Build the Software

### Installing Git, PlatformIO and VS Code

Although the BBDroids software package is using the Arduino framework, it is not built using the Arduino build environment but instead is using PlatformIO and Visual Studio Code. The reason for this is that PlatformIO will automatically install all required code and libraries on a per-project basis into a sandbox environment, without interfering with other projects which might use other libraries. This is really the only manageable way of distributing larger Arduino packages.

Although like any new tool, this may look like a daunting task, it is really quite simple. To install, there are really only two easy steps:
1. Install Visual Studio Code itself
2. Install the PlatformIO extension for Visual Studio Code.
How to do this is described on the [PlatformIO installation page for Virtual Studio Code](https://platformio.org/install/ide?install=vscode) - please follow that guide!

You also want to install a Git client. Git is a very popular source code management system. We use it to manage our code, and PlatformIO uses it to pull the various libraries it needs from Github. [Please download a Git client for your operating system here.](https://git-scm.com/downloads)

### Download the Remote and Droid Software

You are already on the right page! Please click on the green "Code" button above, then use the "Download ZIP" option on the bottom of the menu that pops up. (If you're familiar with Git you can of course also do a proper checkout, but if you aren't, a plain download is fine.

<center>
	<img src="https://github.com/bjoerngiesler/BBDroids/blob/main/Documentation/Common/Download-the-Software.png" width="500" />
</center>

Unzip the ZIP file somewhere on your hard drive where you are going to build and keep the software.

### Build and Upload the Remote software

Open Visual Studio Code. It presents you with a window that contains a start menu. 

<center>
	<img src="https://github.com/bjoerngiesler/BBDroids/blob/main/Documentation/Common/Open-In-VS-Code.png" width="300" />
</center>

Use that menu's "Open" command to navigate to the `Arduino/Remote` folder in the Zip contents you just unpacked. Yes, you will open a folder here, not a file.

PlatformIO will now present you with a couple of windows in the bottom right corner doing its thing initializing the environment and getting everything ready. Give it a bit. Once it is done, click on the drop-down arrow on the build menu in the upper right corner and choose the "Build" option.

<center>
	<img src="https://github.com/bjoerngiesler/BBDroids/blob/main/Documentation/Common/Build-In-VSCode.png" width="300" />
</center>

This will build the software for the *left* remote. So get your left remote ready (the one with the display) and connect it to your computer using USB.

Before you upload the software, you need to get the ESP32 microcontroller on the remote into bootloader mode. You do this by
1. pushing and hold down the tiny button to the *right* of the USB connector on the ESP32 microcontroller (this is the *Bootloader* button),
2. pushing and releasing the tiny button to the *left* of the USB connector (this is the *Reset* button),
3. releasing the Bootloader button to the right of the USB connector.

Now the microcontroller is in bootloader mode, select "Upload" from the same menu you just selected "Build" from. This should upload the software to the right remote. If you're getting an error message saying "no reply on serial port" or somesuch, your microcontroller is not in bootloader mode, so do the dance with the buttons once more. They're tiny but clicky, you can feel when you're pressing them down correctly.

To build the software for the *right* remote, we need to go into the PlatformIO target list (because the left and right remote build from exactly the same source, but are built slightly differently). You can find it on the left hand side of the window, at the very bottom, the little alien head.

<center>
	<img src="https://github.com/bjoerngiesler/BBDroids/blob/main/Documentation/Common/Upload-Right-Remote.png" width="400" />
</center>

This will give you a list of targets, including `left_remote` and `right_remote`, in the window pane one over to the right. Confusingly, if you click on the arrow next to `right_remote`, PlatformIO will first go and think for a while, and likely close the `right_remote` menu again. Fear not, this is normal, just go ahead and click on that arrow again. At some point there will be a "Build" and "Upload" option there. Disconnect the left remote from your computer, connect the right one, get it into bootloader mode, and click on "Upload". 

Congratulations, you've built and flashed the software for the remotes!

### Build and Upload the D-O software (or any droid software)

Visual Studio Code's menu bar has the option "Open Folder" in the File menu. Use that option to open the `Arduino/DODroid` folder. Once it's done initializing everything, connect the droid's MKR Wifi 1010 controller to the computer via USB cable, and use the "Upload" option in the drop-down menu on the top right to build and upload the software to the microcontroller.

### Build and Upload the Aerial software 

Open the `Arduino/Utilities/AntennaI2CServer` folder in the File menu. Once it's done initializing everything, connect the QtPy microcontroller to the computer via USB cable, and use the "Upload" option in the drop-down menu on the top right to build and upload the software to the microcontroller.

## Build the hardware for the D-Ov2Evo droid

Please refer to the documentation collection on the Wiki page: https://github.com/bjoerngiesler/BBDroids/wiki/91-Individual-Droid:-D%E2%80%90Ov2Evo

## About

The project started with the plan to build a RC BB-8 replica, and has since evolved to a general-purpose remote control, a slightly less general-purpose droid control board, and a very much general-purpose software system and architecture. It can be used to realize and control almost arbitrary droids, e.g. BB-8, R2-D2, B2EMO, and many others, although this repository currently contains complete applications for only D-O and BB-8.

From an end-user perspective, the most important aspect are the two remotes used to control the droid, now called the Monaco Control System and described here: https://github.com/bjoerngiesler/BBDroids/wiki/11-Monaco-Control-System

Electronically, the system is built on the basis of Arduinos. Currently, it is using ESP32s in the remotes and a MKR Wifi 1010 in the droid body. Communication between remotes and droid is done via XBee, although the remote hardware also supports ESPnow and the software will soon follow suit.

All three components (two remotes and mainboard) offer command line consoles for deep introspection into system health, setting parameters, stopping and restarting subsystems, etc. The consoles can be accessed via serial port (directly accessible on the remotes) or alternatively via a TCP connection. For this purpose all three components can join infrastructure networks or (default) open their own individual access points, which are identifiable via MAC addresses so they will not clash even in crowded convention situations.

## Droid Control Board

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

### How to Build

Most of the software does not use the Arduino IDE anymore (the exception being everything under the `Utilities` folder). We use VSCode and PlatformIO (https://platformio.org) instead, as it allows explicit library management, hierarchical build targets, incremental builds, and much more. If you cannot install PlatformIO please be in touch, and we will supply you with binaries.

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
