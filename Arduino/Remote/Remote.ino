#include <LibBB.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiNINA.h>

#include "Config.h"
#include "RemoteInput.h"
#include "RemoteDisplay.h"
#include "IMUFilter.h"

uint8_t seqnum = 0;

using namespace bb;

int getAnalogReadResolution() { return 12; } // whatever

class Remote: public Subsystem, public PacketReceiver {
public:
  Remote(): statusPixels_(2, P_NEOPIXEL, NEO_GRB+NEO_KHZ800) {
    name_ = "remote";
    description_ = "Main subsystem for the BB8 remote";
    help_ = "Main subsystem for the BB8 remote"\
"Available commands:\r\n"\
"\tstatus                 Prints current status (buttons, axes, etc.)\r\n"\
"\trunning_status on|off  Continuously prints status";

    started_ = false;
    onInitScreen_ = true;
    operationStatus_ = RES_SUBSYS_NOT_STARTED;
    deltaR_ = 0; deltaP_ = 0; deltaH_ = 0;
    memset(&lastPacketSent_, 0, sizeof(Packet));
  }

  Result initialize() { 
    if(!RemoteInput::input.begin()) return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
    if(!IMUFilter::imu.begin()) return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;

    return Subsystem::initialize();
  }

	Result start(ConsoleStream *stream = NULL) {
    (void)stream;
    runningStatus_ = false;
    operationStatus_ = RES_OK;
    statusPixels_.begin();
    statusPixels_.clear();
    statusPixels_.setPixelColor(0, statusPixels_.Color(150, 150, 0));
    statusPixels_.show();
    started_ = true;
    return RES_OK;
  }
	
  Result stop(ConsoleStream *stream = NULL) {
    (void)stream;
    statusPixels_.clear();
    statusPixels_.show();
    operationStatus_ = RES_OK;
    started_ = false;
    
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);


    return RES_OK;
  }
	
  Result step() {
    bb::Runloop::runloop.excuseOverrun();
    unsigned long m = micros();

    bool allOK = true;
    std::vector<Subsystem*> subsys = SubsystemManager::manager.subsystems();
    for(size_t i=0; i<subsys.size(); i++) {
      if(subsys[i]->isStarted() == false || subsys[i]->operationStatus() != RES_OK) {
        allOK = false;
      }
    }

    int i=0;
    //Serial.print(String(i++) + (micros()-m) + " ");
    //m = micros();

    if(allOK) {
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, LOW);
    } else {
      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDG, HIGH);
      digitalWrite(LEDB, LOW);
    }

    if(!started_) return RES_SUBSYS_NOT_STARTED;
    //if(!bb::XBee::xbee.isStarted()) return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;

    //Serial.print(String(i++) + (micros()-m) + " ");
    //m = micros();
    RemoteInput::input.update();
    //Serial.print(String(i++) + (micros()-m) + " ");
    //m = micros();
    
    Packet packet;

    memset((uint8_t*)&packet, 0, sizeof(packet));
    packet.type = PACKET_TYPE_COMMAND;

#if defined(LEFT_REMOTE)
    packet.source = PACKET_SOURCE_LEFT_REMOTE;
#else
    packet.source = PACKET_SOURCE_RIGHT_REMOTE;
#endif

    packet.payload.cmd.button0 = RemoteInput::input.btnPinky;
    packet.payload.cmd.button1 = RemoteInput::input.btnIndex;
    packet.payload.cmd.button2 = RemoteInput::input.btnL;
    packet.payload.cmd.button3 = RemoteInput::input.btnR;
    packet.payload.cmd.button4 = RemoteInput::input.btnJoy;

    packet.payload.cmd.setAxis(0, RemoteInput::input.joyH * AXIS_MAX);
    packet.payload.cmd.setAxis(1, RemoteInput::input.joyV * AXIS_MAX);

    //Serial.print(String(i++) + (micros()-m) + " ");
    //m = micros();

    if (IMUFilter::imu.available()) {
      float roll, pitch, heading;
      IMUFilter::imu.update();
      IMUFilter::imu.getFilteredEulerAngles(roll, pitch, heading);

      roll = -roll;

      if(lastPacketSent_.payload.cmd.button2 == false && packet.payload.cmd.button2 == true) {
        deltaR_ = roll; deltaP_ = pitch; deltaH_ = heading;
      }

      float d = AXIS_MAX / 180.0;
      packet.payload.cmd.setAxis(2, d * (roll-deltaR_)); 
      packet.payload.cmd.setAxis(3, d * (pitch-deltaP_));
      packet.payload.cmd.setAxis(4, d * (heading-deltaH_));
    }
    //Serial.print(String(i++) + (micros()-m) + " ");
    //m = micros();

    if(runningStatus_) {
      printStatus();
      Console::console.printBroadcast("\r");
    }

#if defined(LEFT_REMOTE)
    if(RemoteDisplay::display.currentScreen() == RemoteDisplay::LOGO_SCREEN && RemoteInput::input.anyButtonPressed()) {
      RemoteDisplay::display.showScreen(RemoteDisplay::LEFT_REMOTE_SCREEN);
    }
    
    if(onInitScreen_ == false) RemoteDisplay::display.update();
#endif // LEFT_REMOTE
    //Serial.print(String(i++) + (micros()-m) + " ");
    //m = micros();

    lastPacketSent_ = packet;
    Result retval = bb::XBee::xbee.send(packet);
    //Serial.print(String(i++) + (micros()-m) + " ");
    //m = micros();
    //Serial.println();

    //if(retval != RES_OK) Console::console.printlnBroadcast(String("Error sending packet! ") + errorMessage(retval));
    return retval;
  }

  Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
    if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;
	  if(words[0] == "status") {
		  if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
      printStatus(stream);
      stream->println();
		  return RES_OK;
    } else if(words[0] == "running_status") {
      if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
      if(words[1] == "on" || words[1] == "true") {
        runningStatus_ = true;
        return RES_OK;
      } else if(words[1] == "off" || words[1] == "false") {
        runningStatus_ = false;
        return RES_OK;
      }
      return RES_CMD_INVALID_ARGUMENT;
    }
    return RES_CMD_UNKNOWN_COMMAND;
  } 

  Result incomingPacket(const Packet& packet) {
#if !defined(LEFT_REMOTE)
    Console::console.printlnBroadcast("Huh? Right remote received a packet but shouldn't");
    return RES_SUBSYS_COMM_ERROR;
#endif

    if(packet.source == PACKET_SOURCE_DROID) {
      //Console::console.printlnBroadcast("Packet from droid!");
#if defined(LEFT_REMOTE)
      RemoteDisplay::display.setLastPacketFromDroid(packet);
#endif
      return RES_OK;
    }

    else if(packet.source == PACKET_SOURCE_RIGHT_REMOTE) {
      //Console::console.printlnBroadcast("Packet from right remote!");
#if defined(LEFT_REMOTE)
      RemoteDisplay::display.setLastPacketFromRightRemote(packet);
#endif
      return RES_OK;
    }

    else if(packet.source == PACKET_SOURCE_LEFT_REMOTE) {
      Console::console.printlnBroadcast("Packet from left remote - huh? That's ourselves! Shouldn't happen.");
      return RES_SUBSYS_COMM_ERROR;
    }

    return RES_OK;
  }

  void printStatus(ConsoleStream *stream = NULL) {
    char buf[255];

    float roll, pitch, yaw;
    IMUFilter::imu.getRawEulerAngles(roll, pitch, yaw);

    sprintf(buf, "Buttons: P%cI%cJ%cL%cR%cC%cTL%cTR%c Axes: JH%5.2f JV%5.2f R%7.2fd P%7.2fd Y%7.2f Batt%7.2f P1%7.2f P2%7.2f    ",
      RemoteInput::input.btnPinky ? 'X' : '_',
      RemoteInput::input.btnIndex ? 'X' : '_',
      RemoteInput::input.btnJoy ? 'X' : '_',
      RemoteInput::input.btnL ? 'X' : '_',
      RemoteInput::input.btnR ? 'X' : '_',
      RemoteInput::input.btnConfirm ? 'X' : '_',
      RemoteInput::input.btnTopL ? 'X' : '_',
      RemoteInput::input.btnTopR ? 'X' : '_',
      RemoteInput::input.joyH, RemoteInput::input.joyV,
      roll, pitch, yaw,
      RemoteInput::input.battery,
      RemoteInput::input.pot1, 
      RemoteInput::input.pot2);

    if(stream != NULL) stream->print(buf);
    else Console::console.printBroadcast(buf);
  }

protected:
  bool runningStatus_;
  Adafruit_NeoPixel statusPixels_;
  bool onInitScreen_;
  Packet lastPacketSent_;

#if defined(LEFT_REMOTE)
  Packet lastPacketFromDroid_, lastPacketFromRightRemote_;
#endif

  float deltaR_, deltaP_, deltaH_;
};

void setup() {
  rp2040.enableDoubleResetBootloader();

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);

  Remote remote;

  Console::console.initialize();
  Console::console.start();
  
  Runloop::runloop.initialize();
#if defined(LEFT_REMOTE)
  RemoteDisplay::display.initialize();
  XBee::xbee.initialize(DEFAULT_CHAN, 0x3333, DEFAULT_STATION_LEFT_REMOTE, DEFAULT_STATION_DROID, DEFAULT_BPS);
#else 
  XBee::xbee.initialize(DEFAULT_CHAN, 0x3333, DEFAULT_STATION_RIGHT_REMOTE, DEFAULT_STATION_DROID, DEFAULT_BPS);
#endif
  XBee::xbee.setPacketMode(true);
  WifiServer::server.initialize("Hogwarts", "1s(h1pu+Bj0rn", false, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT);
  remote.initialize();

#if defined(LEFT_REMOTE)
  RemoteDisplay::display.start();
#endif
  WifiServer::server.start();
  XBee::xbee.start();
  remote.start();
  XBee::xbee.stop();
  XBee::xbee.start();

#if defined(LEFT_REMOTE)
  XBee::xbee.addPacketReceiver(&remote);
#endif
  Runloop::runloop.start(); // never returns
}


void loop() {
}
