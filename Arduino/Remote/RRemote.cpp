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

  Console::console.printfBroadcast("Discovered %d nodes\n", discoveredNodes_.size());

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

  waitMessage_->draw();

  bb::Packet packet;
  packet.type = bb::PACKET_TYPE_CONFIG;
  packet.seqnum = 0;
  packet.source = bb::PACKET_SOURCE_LEFT_REMOTE; 
  bb::ConfigPacket& c = packet.payload.config;
  c.type = bb::CONFIG_SET_DESTINATION_ID;
  c.bits0to6 = droid & 0x7f;
  droid >> 7;
  c.bits7to13 = droid & 0x7f;
  droid >> 7;
  c.bits14to20 = droid & 0x7f;
  c.bits21to27 = c.bits28to34 = c.bits35to41 = c.bits42to48 = 0;
  
  XBee::xbee.send(packet);

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

  selectDroid(0xbabe);

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
  //Runloop::runloop.excuseOverrun();
  std::vector<unsigned long> timing;
  timing.push_back(micros());
  
  if(needsDraw_) {
    currentDrawable_->draw();
    needsDraw_ = false;
  }

  timing.push_back(micros()); // 1

  bool allOK = true;
  std::vector<Subsystem*> subsys = SubsystemManager::manager.subsystems();
  for(size_t i=0; i<subsys.size(); i++) {
    if(subsys[i]->isStarted() == false || subsys[i]->operationStatus() != RES_OK) {
      allOK = false;
    }
  }

  timing.push_back(micros()); // 2

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
  RemoteInput::input.update();
  
  timing.push_back(micros()); // 3

  Packet packet;

  memset((uint8_t*)&packet, 0, sizeof(packet));
  packet.type = PACKET_TYPE_CONTROL;

#if defined(LEFT_REMOTE)
  packet.source = PACKET_SOURCE_LEFT_REMOTE;
#else
  packet.source = PACKET_SOURCE_RIGHT_REMOTE;
#endif

  packet.payload.control.button0 = RemoteInput::input.btnPinky;
  packet.payload.control.button1 = RemoteInput::input.btnIndex;
  packet.payload.control.button2 = RemoteInput::input.btnL;
  packet.payload.control.button3 = RemoteInput::input.btnR;
  packet.payload.control.button4 = RemoteInput::input.btnJoy;

  packet.payload.control.setAxis(0, RemoteInput::input.joyH * AXIS_MAX);
  packet.payload.control.setAxis(1, RemoteInput::input.joyV * AXIS_MAX);

  if (IMUFilter::imu.available()) {
    float roll, pitch, heading;
    IMUFilter::imu.update();
    IMUFilter::imu.getFilteredEulerAngles(roll, pitch, heading);

    roll = -roll;

    if(lastPacketSent_.payload.control.button2 == false && packet.payload.control.button2 == true) {
      deltaR_ = roll; deltaP_ = pitch; deltaH_ = heading;
    }

    float d = AXIS_MAX / 180.0;
    packet.payload.control.setAxis(2, d * (roll-deltaR_)); 
    packet.payload.control.setAxis(3, d * (pitch-deltaP_));
    packet.payload.control.setAxis(4, d * (heading-deltaH_));
  }

  timing.push_back(micros()); // 4

  if(runningStatus_) {
    printStatus();
    Console::console.printfBroadcast("\r");
  }

  static long counter = 0;
  Result retval = RES_OK;
  if(counter%4 == 0) {
    lastPacketSent_ = packet;
    retval = bb::XBee::xbee.send(packet);
    if(retval != RES_OK) Console::console.printfBroadcast("%s\n", errorMessage(retval));
  }
  counter++;
  timing.push_back(micros()); // 5

#if 0
  for(int i=1; i<timing.size(); i++) {
    Console::console.printfBroadcast("%d: %d ", i, timing[i]-timing[i-1]);
  }
  Console::console.printfBroadcast("\n");
#endif

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
#if defined(LEFT_REMOTE)
  Console::console.printfBroadcast("Huh? Left remote received a packet but shouldn't\n");
  return RES_SUBSYS_COMM_ERROR;
#endif

  if(packet.source == PACKET_SOURCE_DROID) {
    Console::console.printfBroadcast("Packet from droid!\n");
#if defined(LEFT_REMOTE)
    RDisplay::display.setLastPacketFromDroid(packet);
#endif
    return RES_OK;
  }

  else if(packet.source == PACKET_SOURCE_LEFT_REMOTE) {
    if(packet.type == PACKET_TYPE_CONFIG) {
      if(packet.payload.config.type == CONFIG_SET_DESTINATION_ID) {
        uint16_t destId = packet.payload.config.bits0to6;
        destId |= (packet.payload.config.bits7to13 < 7);
        destId |= (packet.payload.config.bits14to20 < 14);
        
        Console::console.printfBroadcast("Setting destination ID to 0x%x, please wait...\n", destId);
        uint8_t chan; uint16_t pan, station, partner;
        XBee::xbee.getConnectionInfo(chan, pan, station, partner, true);  // get current values, and stay in AT mode
        XBee::xbee.setConnectionInfo(chan, pan, station, destId, true);   // set new destionation id, and stay in AT mode
        XBee::xbee.getConnectionInfo(chan, pan, station, partner, false); // check whether this worked, and leave AT mode
        if(partner == destId) Console::console.printfBroadcast("Destination ID is now 0x%x.\n", destId);
        else Console::console.printfBroadcast("Setting destination ID failed.\n", destId);
      } else {
        Console::console.printfBroadcast("Unknown config packet type 0x%x.\n", packet.payload.config.type);
      }
    } else if(packet.type == PACKET_TYPE_CONTROL) {
      Console::console.printfBroadcast("Control packet received, forwarding.\n");
      XBee::xbee.send(packet);
    }
    return RES_OK;
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

