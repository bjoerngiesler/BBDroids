#include <cstdio>
#include "RRemote.h"

RRemote RRemote::remote;
RRemote::RemoteParams RRemote::params_;
bb::ConfigStorage::HANDLE RRemote::paramsHandle_;

RRemote::RRemote(): 
  statusPixels_(2, P_D_NEOPIXEL, NEO_GRB+NEO_KHZ800),
  imu_(IMU_ADDR) {
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

#if defined(LEFT_REMOTE)
  params_.leftID = XBee::makeStationID(XBee::REMOTE_BAVARIAN_L, BUILDER_ID, REMOTE_ID);
  params_.rightID = 0;
  params_.droidID = 0;
#else
  params_.leftID = 0;
  params_.rightID = XBee::makeStationID(XBee::REMOTE_BAVARIAN_R, BUILDER_ID, REMOTE_ID);
  params_.droidID = 0;
#endif
}

Result RRemote::initialize() { 
  RInput::input.begin();
  imu_.begin();

  paramsHandle_ = ConfigStorage::storage.reserveBlock("remote", sizeof(params_));
	if(ConfigStorage::storage.blockIsValid(paramsHandle_)) {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is valid.\n", paramsHandle_);
    ConfigStorage::storage.readBlock(paramsHandle_, (uint8_t*)&params_);
  } else {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is invalid, using initialized parameters.\n", paramsHandle_);
  }

  mainMenu_ = new RMenu("Main Menu");
  settingsMenu_ = new RMenu("Settings");
  droidsMenu_ = new RMenu("Droids");
  remotesMenu_ = new RMenu("Remotes");
  waitMessage_ = new RMessage("Please wait");
  graphs_ = new RGraphs();

  mainMenu_->addEntry("Settings...", []() { RRemote::remote.showSettingsMenu(); });
  mainMenu_->addEntry("Back", []() { RRemote::remote.showGraphs(); });
  
  settingsMenu_->addEntry("Right Remote...", []() { RRemote::remote.showRemotesMenu(); });
  settingsMenu_->addEntry("Droid...", []() { RRemote::remote.showDroidsMenu(); });
  settingsMenu_->addEntry("Back", []() { RRemote::remote.showMainMenu(); });

  showGraphs();

  return Subsystem::initialize();
}

void RRemote::showMenu(RMenu *menu) {
  currentDrawable_ = menu;
  RInput::input.setDelegate(menu);
  menu->setNeedsCls(true);
  menu->resetCursor();
  needsDraw_ = true;
}

void RRemote::showGraphs() {
  currentDrawable_ = graphs_;
  graphs_->setNeedsCls(true);
  RInput::input.setDelegate(graphs_);
  needsDraw_ = true;
}

void RRemote::showDroidsMenu() {
  currentDrawable_ = waitMessage_;
  RInput::input.setDelegate(NULL);
  waitMessage_->draw();

  droidsMenu_->clear();

  Result res = XBee::xbee.discoverNodes(discoveredNodes_);
  if(res != RES_OK) {
    Console::console.printfBroadcast("%s\n", errorMessage(res));
    return;
  }

  droidsMenu_->clear();
  for(auto& n: discoveredNodes_) {
    Console::console.printfBroadcast("Station \"%s\", ID 0x%x\n", n.name, n.stationId);
    if(XBee::stationTypeFromId(n.stationId) != XBee::STATION_DROID) continue;
    droidsMenu_->addEntry(n.name, [=]() { RRemote::remote.selectDroid(n.stationId); });
  }
  droidsMenu_->addEntry("Back", []() { RRemote::remote.showSettingsMenu(); });

  showMenu(droidsMenu_);
}

void RRemote::showRemotesMenu() {
  currentDrawable_ = waitMessage_;
  RInput::input.setDelegate(NULL);
  waitMessage_->draw();

  remotesMenu_->clear();

  Result res = XBee::xbee.discoverNodes(discoveredNodes_);
  if(res != RES_OK) {
    Console::console.printfBroadcast("%s\n", errorMessage(res));
    showMenu(settingsMenu_);
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
#if defined(LEFT_REMOTE)
  Console::console.printfBroadcast("Selecting droid 0x%x\n", droid);

  params_.droidID = droid;

  if(params_.rightID != 0) {
    bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE);
    packet.payload.config.type = bb::ConfigPacket::CONFIG_SET_DROID_ID;

    packet.payload.config.parameter.id = params_.droidID;
    XBee::xbee.sendTo(params_.rightID, packet, true);
  }

  bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
  bb::ConfigStorage::storage.store();

  showMenu(mainMenu_);
#endif
}

void RRemote::selectRightRemote(uint16_t remote) {
#if defined(LEFT_REMOTE)
  Console::console.printfBroadcast("Selecting right remote 0x%x\n", remote);
  
  params_.rightID = remote;

  bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE);
  packet.payload.config.type = bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID;
  packet.payload.config.parameter.id = params_.leftID;
  XBee::xbee.sendTo(params_.rightID, packet, true);  

  if(params_.droidID != 0) {
    packet.payload.config.type = bb::ConfigPacket::CONFIG_SET_DROID_ID;
    packet.payload.config.parameter.id = params_.droidID;
    XBee::xbee.sendTo(params_.rightID, packet, true);   
  }

  bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
  bb::ConfigStorage::storage.store();

  showMenu(mainMenu_);
#endif
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
  
  return RES_OK;
}

Result RRemote::step() {
  //Runloop::runloop.excuseOverrun();
  if(needsDraw_) {
    currentDrawable_->draw();
    needsDraw_ = false;
  }

  bool allOK = true;
  std::vector<Subsystem*> subsys = SubsystemManager::manager.subsystems();
  for(size_t i=0; i<subsys.size(); i++) {
    if(subsys[i]->isStarted() == false || subsys[i]->operationStatus() != RES_OK) {
      allOK = false;
    }
  }

  if(!started_) return RES_SUBSYS_NOT_STARTED;
  RInput::input.update();

  if((bb::Runloop::runloop.getSequenceNumber() % 4) == 0) {
    fillAndSend();
    if(runningStatus_) {
      printStatus();
    }
  }

  return RES_OK;
}

bb::Result RRemote::fillAndSend() {
  Packet packet;

  memset((uint8_t*)&packet, 0, sizeof(packet));
  packet.type = PACKET_TYPE_CONTROL;

#if defined(LEFT_REMOTE)
  packet.source = PACKET_SOURCE_LEFT_REMOTE;
#else
  packet.source = PACKET_SOURCE_RIGHT_REMOTE;
#endif

  packet.payload.control.button0 = RInput::input.buttons[RInput::BUTTON_PINKY];
  packet.payload.control.button1 = RInput::input.buttons[RInput::BUTTON_INDEX];
  packet.payload.control.button2 = RInput::input.buttons[RInput::BUTTON_LEFT];
  packet.payload.control.button3 = RInput::input.buttons[RInput::BUTTON_RIGHT];
  packet.payload.control.button4 = RInput::input.buttons[RInput::BUTTON_JOY];

  packet.payload.control.setAxis(0, RInput::input.joyH);
  packet.payload.control.setAxis(1, RInput::input.joyV);

  if (imu_.available()) {
    float roll, pitch, heading;
    imu_.update();
    imu_.getFilteredRPH(roll, pitch, heading);

    roll = -roll;

    if(lastPacketSent_.payload.control.button2 == false && packet.payload.control.button2 == true) {
      deltaR_ = roll; deltaP_ = pitch; deltaH_ = heading;
    }

    packet.payload.control.setAxis(2, (roll-deltaR_)/180.0); 
    packet.payload.control.setAxis(3, (pitch-deltaP_)/180.0);
    packet.payload.control.setAxis(4, (heading-deltaH_)/180.0);
  }

#if defined(LEFT_REMOTE)
  if(currentDrawable_ == graphs_) {
    graphs_->plotControlPacket(RGraphs::TOP, packet.payload.control);
    graphs_->advanceCursor(RGraphs::TOP);
  }
#endif

  Result res = RES_OK;
#if !defined(LEFT_REMOTE) // right remote sends to left remote
  if(params_.leftID != 0) {
    res = bb::XBee::xbee.sendTo(params_.leftID, packet, false);
    if(res != RES_OK) Console::console.printfBroadcast("%s\n", errorMessage(res));
  }
#endif

  // both remotes send to droid
  if(params_.droidID != 0) {
    res = bb::XBee::xbee.sendTo(params_.droidID, packet, false);
    if(res != RES_OK) Console::console.printfBroadcast("%s\n", errorMessage(res));
  }

  return res;
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

Result RRemote::incomingPacket(uint16_t source, uint8_t rssi, const Packet& packet) {
#if defined(LEFT_REMOTE)
  if(source == params_.droidID && params_.droidID != 0) {
    Console::console.printfBroadcast("Packet from droid!\n");
    return RES_OK;
  } else if(source == params_.rightID && params_.rightID != 0) {
    if(packet.type == PACKET_TYPE_CONTROL) {
      if(currentDrawable_ == graphs_) {
        graphs_->plotControlPacket(RGraphs::MIDDLE, packet.payload.control);
        graphs_->advanceCursor(RGraphs::MIDDLE);
      }
      return RES_OK;
    } else {
      Console::console.printfBroadcast("Unknown packet type %d from right remote!\n", packet.type);
      return RES_SUBSYS_COMM_ERROR;
    }
  } else {
    Console::console.printfBroadcast("Unknown packet source %d!\n", source);
    return RES_SUBSYS_COMM_ERROR;
  }

#else // Right remote

  if(source == params_.leftID || params_.leftID == 0) { // if we're not bound, we accept config packets from anyone
    if(packet.type == PACKET_TYPE_CONFIG) {
      if(packet.payload.config.type == bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID) {
        Console::console.printfBroadcast("Setting Left Remote ID to 0x%x.\n", packet.payload.config.parameter.id);
        params_.leftID = packet.payload.config.parameter.id;
        bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
        bb::ConfigStorage::storage.store();
        Console::console.printfBroadcast("Stored config parameters.\n");
        return RES_OK;
      } else if(packet.payload.config.type == bb::ConfigPacket::CONFIG_SET_DROID_ID) {
        Console::console.printfBroadcast("Setting Droid ID to 0x%x.\n", packet.payload.config.parameter.id);
        params_.droidID = packet.payload.config.parameter.id;
        bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
        bb::ConfigStorage::storage.store();
        Console::console.printfBroadcast("Stored config parameters.\n");
        return RES_OK;
      } else {
        Console::console.printfBroadcast("Unknown config packet type 0x%x.\n", packet.payload.config.type);
        return RES_SUBSYS_COMM_ERROR;
      }
    } else {
      Console::console.printfBroadcast("Unknown packet type %d from right remote!\n", packet.type);
      return RES_SUBSYS_COMM_ERROR;
    }
  } else {
    Console::console.printfBroadcast("Unknown packet source %d!\n", source);
    return RES_SUBSYS_COMM_ERROR;
  }

#endif

  Console::console.printfBroadcast("Should never get here.\n");
  return RES_SUBSYS_COMM_ERROR;
}

void RRemote::printStatus(ConsoleStream *stream) {
  char buf[255];

  float roll, pitch, yaw;
  imu_.getGyroMeasurement(roll, pitch, yaw);

  sprintf(buf, "Buttons: P%cI%cJ%cL%cR%cC%cTL%cTR%c Axes: JH%5.2f JV%5.2f R%7.2fd P%7.2fd Y%7.2f Batt%7.2f P1%7.2f P2%7.2f    \n",
    RInput::input.buttons[RInput::BUTTON_PINKY] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_INDEX] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_JOY] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_LEFT] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_RIGHT] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_CONFIRM] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_TOP_LEFT] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_TOP_RIGHT] ? 'X' : '_',
    RInput::input.joyH, RInput::input.joyV,
    roll, pitch, yaw,
    RInput::input.battery,
    RInput::input.pot1, 
    RInput::input.pot2);

  if(stream != NULL) stream->printf(buf);
  else Console::console.printfBroadcast(buf);
}

