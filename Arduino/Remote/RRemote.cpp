#include <cstdio>
#include "RRemote.h"

RRemote RRemote::remote;

RRemote::RRemote(): statusPixels_(2, P_NEOPIXEL, NEO_GRB+NEO_KHZ800) {
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

Result RRemote::initialize() { 
  if(!RemoteInput::input.begin()) return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;
  if(!IMUFilter::imu.begin()) return RES_SUBSYS_RESOURCE_NOT_AVAILABLE;

  mainMenu_ = new RMenu("Main Menu");
  settingsMenu_ = new RMenu("Settings");
  droidsMenu_ = new RMenu("Droids");
  remotesMenu_ = new RMenu("Remotes");
  waitMessage_ = new RMessage("Please wait");

  mainMenu_->addEntry("Status", []() { RRemote::remote.showStatus(); });
  mainMenu_->addEntry("Settings...", []() { RRemote::remote.showSettingsMenu(); });
  
  settingsMenu_->addEntry("Right Remote...", []() { RRemote::remote.showRemotesMenu(); });
  settingsMenu_->addEntry("Droid...", []() { RRemote::remote.showDroidsMenu(); });
  settingsMenu_->addEntry("Back", []() { RRemote::remote.showMainMenu(); });
  showMenu(mainMenu_);

  return Subsystem::initialize();
}

void RRemote::showMenu(RMenu *menu) {
  currentDrawable_ = menu;
  RemoteInput::input.setDelegate(menu);
  menu->setNeedsCls(true);
  needsDraw_ = true;
}

void RRemote::showDroidsMenu() {
  currentDrawable_ = waitMessage_;
  RemoteInput::input.setDelegate(NULL);
  waitMessage_->draw();

  droidsMenu_->clear();

  Result res = XBee::xbee.discoverNodes(discoveredNodes_);
  if(res != RES_OK) {
    Console::console.printfBroadcast("%s\n", errorMessage(res));
    return;
  }

  droidsMenu_->clear();
  for(auto& n: discoveredNodes_) {
    if(XBee::stationTypeFromId(n.stationId) != XBee::STATION_DROID) continue;
    droidsMenu_->addEntry(n.name, [=]() { RRemote::remote.selectDroid(n.stationId); });
  }
  droidsMenu_->addEntry("Back", []() { RRemote::remote.showSettingsMenu(); });

  showMenu(droidsMenu_);
}

void RRemote::showRemotesMenu() {
  currentDrawable_ = waitMessage_;
  RemoteInput::input.setDelegate(NULL);
  waitMessage_->draw();

  remotesMenu_->clear();
  Result res = XBee::xbee.discoverNodes(discoveredNodes_);
  if(res != RES_OK) {
    Console::console.printfBroadcast("%s\n", errorMessage(res));
    return;
  }

  remotesMenu_->clear();
  for(auto& n: discoveredNodes_) {
    if(XBee::stationTypeFromId(n.stationId) != XBee::STATION_REMOTE) continue;
    remotesMenu_->addEntry(n.name, [=]() { RRemote::remote.selectRightRemote(n.stationId); });
  }
  remotesMenu_->addEntry("Back", []() { RRemote::remote.showSettingsMenu(); });

  showMenu(remotesMenu_);
}

void RRemote::selectDroid(uint16_t droid) {
  Console::console.printfBroadcast("Selecting droid 0x%x\n", droid);
  
  showMenu(mainMenu_);
}

void RRemote::selectRightRemote(uint16_t remote) {
  Console::console.printfBroadcast("Selecting right remote 0x%x\n", remote);
  
  waitMessage_->draw();
  
  uint8_t chan; uint16_t pan, station, partner;

  bb::XBee::xbee.getConnectionInfo(chan, pan, station, partner, true);
  Console::console.printfBroadcast("Before: chan 0x%x pan 0x%x station 0x%x partner 0x%x\n", chan, pan, station, partner);
  bb::XBee::xbee.setConnectionInfo(chan, pan, station, remote, true);
  bb::XBee::xbee.getConnectionInfo(chan, pan, station, partner);
  Console::console.printfBroadcast("After: chan 0x%x pan 0x%x station 0x%x partner 0x%x\n", chan, pan, station, partner);

  showMenu(mainMenu_);
}

Result RRemote::start(ConsoleStream *stream) {
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

Result RRemote::stop(ConsoleStream *stream) {
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

Result RRemote::step() {
  bb::Runloop::runloop.excuseOverrun();

  if(needsDraw_) {
    currentDrawable_->draw();
    needsDraw_ = false;
  }

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
    Console::console.printfBroadcast("\r");
  }

  //Serial.print(String(i++) + (micros()-m) + " ");
  //m = micros();

  lastPacketSent_ = packet;
  Result retval = bb::XBee::xbee.send(packet);
  //Serial.print(String(i++) + (micros()-m) + " ");
  //m = micros();
  //Serial.println();

  //if(retval != RES_OK) Console::console.printfBroadcast("Error sending packet! %s\n", errorMessage(retval));
  return retval;
}

Result RRemote::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
  if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;
  if(words[0] == "status") {
    if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
    printStatus(stream);
    stream->printf("\n");
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

Result RRemote::incomingPacket(const Packet& packet) {
#if !defined(LEFT_REMOTE)
  Console::console.printfBroadcast("Huh? Right remote received a packet but shouldn't\n");
  return RES_SUBSYS_COMM_ERROR;
#endif

  if(packet.source == PACKET_SOURCE_DROID) {
    //Console::console.printfBroadcast("Packet from droid!\n");
#if defined(LEFT_REMOTE)
    RDisplay::display.setLastPacketFromDroid(packet);
#endif
    return RES_OK;
  }

  else if(packet.source == PACKET_SOURCE_RIGHT_REMOTE) {
    //Console::console.printfBroadcast("Packet from right remote!\n");
#if defined(LEFT_REMOTE)
    RDisplay::display.setLastPacketFromRightRemote(packet);
#endif
    return RES_OK;
  }

  else if(packet.source == PACKET_SOURCE_LEFT_REMOTE) {
    Console::console.printfBroadcast("Packet from left remote - huh? That's ourselves! Shouldn't happen.\n");
    return RES_SUBSYS_COMM_ERROR;
  }

  return RES_OK;
}

void RRemote::printStatus(ConsoleStream *stream) {
  char buf[255];

  float roll, pitch, yaw;
  IMUFilter::imu.getRawEulerAngles(roll, pitch, yaw);

  sprintf(buf, "Buttons: P%cI%cJ%cL%cR%cC%cTL%cTR%c Axes: JH%5.2f JV%5.2f R%7.2fd P%7.2fd Y%7.2f Batt%7.2f P1%7.2f P2%7.2f    \n",
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

  if(stream != NULL) stream->printf(buf);
  else Console::console.printfBroadcast(buf);
}

