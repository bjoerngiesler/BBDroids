#include <cstdio>
#include "RRemote.h"

RRemote RRemote::remote;
RRemote::RemoteParams RRemote::params_;
bb::ConfigStorage::HANDLE RRemote::paramsHandle_;

RRemote::RRemote(): 
  mode_(MODE_REGULAR) {
  name_ = "remote";
  description_ = "Main subsystem for the BB8 remote";
  help_ = "Main subsystem for the BB8 remote"\
"Available commands:\r\n"\
"\tstatus                 Prints current status (buttons, axes, etc.)\r\n"\
"\trunning_status on|off  Continuously prints status\r\n"\
"\ttestsuite              Run test suite\r\n"\
"\tcalibrate_imu          Run IMU calibration\r\n"\
"\treset                  Factory reset";

  started_ = false;
  onInitScreen_ = true;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  deltaR_ = 0; deltaP_ = 0; deltaH_ = 0;
  memset((void*)&lastPacketSent_, 0, sizeof(Packet));

  params_.droidAddress = 0x0;
  params_.otherRemoteAddress = 0x0;
#if defined(LEFT_REMOTE)
  params_.isPrimary = true;
#else
  params_.isPrimary = false;
#endif

  waitMessage_.setTitle("Please wait");
  restartMessage_.setTitle("Restart now");

  remoteVisL_.setRepresentsLeftRemote(true);
  remoteVisL_.setName("Left Remote");
  remoteVisR_.setRepresentsLeftRemote(false);
  remoteVisR_.setName("Right Remote");
  mainVis_.addWidget(&remoteVisL_);
  mainVis_.addWidget(&remoteVisR_);

  topLabel_.setSize(RDisplay::DISPLAY_WIDTH, RDisplay::CHAR_HEIGHT);
  topLabel_.setPosition(0, 0);
  topLabel_.setFrameType(RLabelWidget::FRAME_BOTTOM);
  topLabel_.setTitle("Top Label");

  bottomLabel_.setSize(RDisplay::DISPLAY_WIDTH, RDisplay::CHAR_HEIGHT+4);
  bottomLabel_.setPosition(0, RDisplay::DISPLAY_HEIGHT-RDisplay::CHAR_HEIGHT-3);
  bottomLabel_.setJustification(RLabelWidget::HOR_CENTERED, RLabelWidget::BOTTOM_JUSTIFIED);
  bottomLabel_.setFrameType(RLabelWidget::FRAME_TOP);
  bottomLabel_.setTitle(VERSION_STRING);
}

Result RRemote::initialize() { 
  RInput::input.begin();

  paramsHandle_ = ConfigStorage::storage.reserveBlock("remote", sizeof(params_));
	if(ConfigStorage::storage.blockIsValid(paramsHandle_)) {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is valid.\n", paramsHandle_);
    ConfigStorage::storage.readBlock(paramsHandle_, (uint8_t*)&params_);
    Console::console.printfBroadcast("Other Remote Address: 0x%llx\n", params_.otherRemoteAddress);
    Console::console.printfBroadcast("Droid address: 0x%x\n", params_.droidAddress);
    RInput::input.setCalibration(params_.hCalib, params_.vCalib);
  } else {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is invalid, using initialized parameters.\n", paramsHandle_);
    ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
  }

  populateMenus();
  showMain();

  return Subsystem::initialize();
}

void RRemote::setMainWidget(RWidget* widget) {
  widget->setPosition(RDisplay::MAIN_X, RDisplay::MAIN_Y);
  widget->setSize(RDisplay::MAIN_WIDTH, RDisplay::MAIN_HEIGHT);
  widget->takeInputFocus();
  widget->setNeedsFullRedraw();
  widget->setNeedsContentsRedraw();
  setTopTitle(widget->name());
  mainWidget_ = widget;

  needsDraw_ = true;
}

void RRemote::showMain() {
  setMainWidget(&mainVis_);
  RInput::input.setTopRightLongPressCallback([=]{RRemote::remote.showMenu(&mainMenu_);});
  RInput::input.setConfirmShortPressCallback([=]{RRemote::remote.showMenu(&mainMenu_);});
}

void RRemote::showMenu(RMenuWidget* menu) {
  setMainWidget(menu);
  menu->resetCursor();
}

void RRemote::showPairDroidMenu() {
  waitMessage_.setNeedsFullRedraw();
  waitMessage_.draw();
  
  pairDroidMenu_.clear();

  Console::console.printfBroadcast("Discovering nodes...\n");
  Result res = XBee::xbee.discoverNodes(discoveredNodes_);
  if(res != RES_OK) {
    Console::console.printfBroadcast("%s\n", errorMessage(res));
    return;
  }
  Console::console.printfBroadcast("%d nodes discovered.\n", discoveredNodes_.size());

  pairDroidMenu_.clear();
  pairRemoteMenu_.setName(String(discoveredNodes_.size()) + " Droids");
  for(auto& n: discoveredNodes_) {
    if(XBee::stationTypeFromId(n.stationId) != XBee::STATION_DROID) continue;
    Console::console.printfBroadcast("Station \"%s\", ID 0x%x\n", n.name, n.stationId);
    pairDroidMenu_.addEntry(n.name, [=]() { RRemote::remote.selectDroid(n.address); });
  }
  pairDroidMenu_.addEntry("<--", [=]() { RRemote::remote.showMenu(&pairMenu_); });

  Console::console.printfBroadcast("Showing droidsMenu\n");
  showMenu(&pairDroidMenu_);
}

void RRemote::showPairRemoteMenu() {
  waitMessage_.setNeedsFullRedraw();
  waitMessage_.draw();

  pairRemoteMenu_.clear();

  Result res = XBee::xbee.discoverNodes(discoveredNodes_);
  if(res != RES_OK) {
    Console::console.printfBroadcast("%s\n", errorMessage(res));
    showMenu(&pairMenu_);
    return;
  }

  pairRemoteMenu_.setName(String(discoveredNodes_.size()) + " Remotes");
  for(auto& n: discoveredNodes_) {
    if(XBee::stationTypeFromId(n.stationId) != XBee::STATION_REMOTE) continue;
    Console::console.printfBroadcast("Adding remote entry 0x%llx\n", n.address);
    pairRemoteMenu_.addEntry(n.name, [=]() { RRemote::remote.selectRightRemote(n.address); });
  }
  pairRemoteMenu_.addEntry("<--", [=]() { RRemote::remote.showMenu(&pairMenu_); });

  showMenu(&pairRemoteMenu_);
}

void RRemote::populateMenus() {
  mainMenu_.setName("Main Menu");
  pairMenu_.setName("Pair");
  leftRemoteMenu_.setName("Left Remote");
  rightRemoteMenu_.setName("Right Remote");
  droidMenu_.setName("Droid");
  
  mainMenu_.clear();
  if(params_.droidAddress != 0) mainMenu_.addEntry("Droid...", [=]() { showMenu(&droidMenu_); });
  mainMenu_.addEntry("Left Remote...", [=]() { showMenu(&leftRemoteMenu_); });
  if(params_.otherRemoteAddress != 0) mainMenu_.addEntry("Right Remote...", [=]() { showMenu(&rightRemoteMenu_); });
  mainMenu_.addEntry("Pair...", [=]() { RRemote::remote.showMenu(&pairMenu_); });
  mainMenu_.addEntry("<--", []() { RRemote::remote.showMain(); });
  
  pairMenu_.clear();
  pairMenu_.addEntry("Right Remote...", []() { RRemote::remote.showPairRemoteMenu(); });
  pairMenu_.addEntry("Droid...", []() { RRemote::remote.showPairDroidMenu(); });
  pairMenu_.addEntry("<--", [=]() { RRemote::remote.showMenu(&mainMenu_); });

  droidMenu_.clear();
  droidMenu_.addEntry("<--", [=]() {showMenu(&mainMenu_);});

  lRIncrRotMenu_.clear();
  lRIncrRotMenu_.setName("Incr Rotation");
  lRIncrRotMenu_.addEntry("Disable", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_NONE);showMenu(&leftRemoteMenu_);});
  lRIncrRotMenu_.addEntry("Left Button", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_LEFT);showMenu(&leftRemoteMenu_);});
  lRIncrRotMenu_.addEntry("Right Button", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_RIGHT);showMenu(&leftRemoteMenu_);});
  lRIncrRotMenu_.addEntry("Pinky Button", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_PINKY);showMenu(&leftRemoteMenu_);});
  lRIncrRotMenu_.addEntry("Index Button", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_INDEX);showMenu(&leftRemoteMenu_);});
  lRIncrRotMenu_.addEntry("<--", [=]{showMenu(&leftRemoteMenu_);});

  rRIncrRotMenu_.clear();
  rRIncrRotMenu_.setName("Incr Rotation");
  rRIncrRotMenu_.addEntry("Disable", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_NONE);showMenu(&rightRemoteMenu_);});
  rRIncrRotMenu_.addEntry("Left Button", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_LEFT);showMenu(&rightRemoteMenu_);});
  rRIncrRotMenu_.addEntry("Right Button", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_RIGHT);showMenu(&rightRemoteMenu_);});
  rRIncrRotMenu_.addEntry("Pinky Button", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_PINKY);showMenu(&rightRemoteMenu_);});
  rRIncrRotMenu_.addEntry("Index Button", [=]{RInput::input.setIncrementalRot(RInput::BUTTON_INDEX);showMenu(&rightRemoteMenu_);});
  rRIncrRotMenu_.addEntry("<--", [=]{showMenu(&leftRemoteMenu_);});

  leftRemoteMenu_.clear();
  leftRemoteMenu_.addEntry("Incr Rot...", [=]{showMenu(&lRIncrRotMenu_);});
  leftRemoteMenu_.addEntry("Calib Joystick", [=]{startCalibration(true);});
  leftRemoteMenu_.addEntry("Set Primary", [=]{setPrimary(true);});
  leftRemoteMenu_.addEntry("Factory Reset", [=]{factoryReset(true);});
  leftRemoteMenu_.addEntry("<--", [=]() {showMenu(&mainMenu_);});

  rightRemoteMenu_.clear();
  rightRemoteMenu_.addEntry("Incr Rot...", [=]{showMenu(&rRIncrRotMenu_);});
  rightRemoteMenu_.addEntry("Calib Joystick", [=]{startCalibration(false);});
  rightRemoteMenu_.addEntry("Set Primary", [=]{setPrimary(false);});
  rightRemoteMenu_.addEntry("Factory Reset", [=]() {factoryReset(false);});
  rightRemoteMenu_.addEntry("<--", [=]() {showMenu(&mainMenu_);});
}

void RRemote::setTopTitle(const String& title) {
  topLabel_.setTitle(title);
}

void RRemote::setBottomTitle(const String& title) {
  bottomLabel_.setTitle(title);
}

void RRemote::selectDroid(uint64_t droid) {
#if defined(LEFT_REMOTE)
  Console::console.printfBroadcast("Selecting droid 0x%x\n", droid);

  params_.droidAddress = droid;

  if(params_.otherRemoteAddress != 0) {
    bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE, sequenceNumber());
    packet.payload.config.type = bb::ConfigPacket::CONFIG_SET_DROID_ID;

    packet.payload.config.parameter = params_.droidAddress;
    XBee::xbee.sendTo(params_.otherRemoteAddress, packet, true);
  }

  bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
  bb::ConfigStorage::storage.store();

  populateMenus();
  showMenu(&mainMenu_);
#endif
}

void RRemote::selectRightRemote(uint64_t address) {
#if defined(LEFT_REMOTE)
  Console::console.printfBroadcast("Selecting right remote 0x%llx\n", address);
  
  params_.otherRemoteAddress = address;

  bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE, sequenceNumber());
  packet.payload.config.type = bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID;
  packet.payload.config.parameter = XBee::xbee.hwAddress();
  XBee::xbee.sendTo(params_.otherRemoteAddress, packet, true);  

  if(params_.droidAddress != 0) {
    packet.payload.config.type = bb::ConfigPacket::CONFIG_SET_DROID_ID;
    packet.payload.config.parameter = params_.droidAddress;
    XBee::xbee.sendTo(params_.otherRemoteAddress, packet, true);   
  }

  bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
  bb::ConfigStorage::storage.store();

  populateMenus();
  showMenu(&mainMenu_);
#endif
}

Result RRemote::start(ConsoleStream *stream) {
  (void)stream;
  runningStatus_ = false;
  operationStatus_ = RES_OK;

  started_ = true;

  return RES_OK;
}

Result RRemote::stop(ConsoleStream *stream) {
  (void)stream;
  RDisplay::display.setLED(RDisplay::LED_BOTH, 150, 150, 0);
  operationStatus_ = RES_OK;
  started_ = false;
  
  return RES_OK;
}

void RRemote::updateStatusLED() {
  if(mode_ == MODE_CALIBRATION) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 0, 0, 150);
  } else if(Console::console.isStarted() && XBee::xbee.isStarted() && isStarted() && RInput::input.isOK()) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 0, 150, 0);
  } else if(!XBee::xbee.isStarted() || !RInput::input.isOK()) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 150, 0, 0);
  } else {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 150, 150, 0);
  }
}

Result RRemote::step() {
  if(!started_) return RES_SUBSYS_NOT_STARTED;

  RInput::input.update();

  if((bb::Runloop::runloop.getSequenceNumber() % 4) == 0) {
    updateStatusLED();
    topLabel_.draw();
    bottomLabel_.draw();
    if(mainWidget_ != NULL) mainWidget_->draw();
  } else {
    RDisplay::display.setLED(RDisplay::LED_COMM, 0, 0, 0);
  }

  if((bb::Runloop::runloop.getSequenceNumber() % 4) == 0) {
    if(runningStatus_) {
      printStatus();
    }

    fillAndSend();
  }

  return RES_OK;
}

Result RRemote::stepCalib() {
  return RES_OK;
}

void RRemote::setIncrRotButton(bool thisremote, RInput::ButtonIndex button) {
  if(thisremote) {
    RInput::input.setIncrementalRot(button);
    params_.incrRotButton = button;
    bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
    bb::ConfigStorage::storage.store();
  } else {
#if defined(LEFT_REMOTE)
    Console::console.printfBroadcast("Incr Rot other remote not yet implemented\n");
#else
    Console::console.printfBroadcast("Incr Rot other remote only defined for left remote\n");
#endif
  }
}


void RRemote::factoryReset(bool thisremote) {
  if(thisremote) {
    Console::console.printfBroadcast("Factory reset this remote!\n");
    bb::ConfigStorage::storage.factoryReset();
    restartMessage_.draw();
    int i=0, dir=1;
    while(true) {
      RDisplay::display.setLED(RDisplay::LED_BOTH, 0, 0, i);
      i = i+dir;
      if(dir > 0 && i>=150) dir = -1;
      if(dir < 0 && i<=0) dir = 1;
      delay(10);
    }
  } else {
#if defined(LEFT_REMOTE)
    Console::console.printfBroadcast("Factory reset other remote!\n");
    bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE, sequenceNumber());
    packet.payload.config.type = bb::ConfigPacket::CONFIG_FACTORY_RESET;
    packet.payload.config.parameter = bb::ConfigPacket::MAGIC;
    XBee::xbee.sendTo(params_.otherRemoteAddress, packet, true);

    params_.otherRemoteAddress = 0;
    bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
    bb::ConfigStorage::storage.store();

    populateMenus();
#else
    Console::console.printfBroadcast("Factory reset other remote only defined for left remote\m");
#endif
  }
}

void RRemote::startCalibration(bool thisremote) {
  if(thisremote == true) {
#if defined(LEFT_REMOTE)
    mainVis_.showIndex(0);
    setMainWidget(&mainVis_);

    setTopTitle("Calibrating");
    setBottomTitle("Button=End");
    remoteVisL_.crosshair().setMinMax(RInput::input.minJoyRawH, RInput::input.maxJoyRawH,
                                      RInput::input.minJoyRawV, RInput::input.maxJoyRawV);
    remoteVisL_.crosshair().showMinMaxRect();
    remoteVisL_.crosshair().setMinMaxRectColor(RDisplay::RED);
#endif
    RInput::input.setCalibration(RInput::AxisCalib(), RInput::AxisCalib());
    RInput::input.setAllCallbacks([=]{finishCalibration(true);});
    mode_ = MODE_CALIBRATION;
  } else {
#if defined(LEFT_REMOTE)
    bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE, sequenceNumber());
    packet.payload.config.type = bb::ConfigPacket::CONFIG_CALIBRATE;
    packet.payload.config.parameter = bb::ConfigPacket::MAGIC;
    XBee::xbee.sendTo(params_.otherRemoteAddress, packet, true);

    mainVis_.showIndex(1);
    remoteVisR_.crosshair().setMinMax(1024, 4096-1024, 1024, 4096-1024);
    remoteVisR_.crosshair().showMinMaxRect();
    remoteVisR_.crosshair().setMinMaxRectColor(RDisplay::RED);
    showMain();
#else
    Console::console.printfBroadcast("Factory reset other remote only defined for right remote");
#endif
  }
}

void RRemote::finishCalibration(bool thisremote) {
  Runloop::runloop.excuseOverrun();
  if(thisremote == true) {
#if defined(LEFT_REMOTE)
    setBottomTitle(VERSION_STRING);
    showMain();
    RInput::input.setTopRightLongPressCallback([=]{RRemote::remote.showMenu(&mainMenu_);});
    RInput::input.setConfirmShortPressCallback([=]{RRemote::remote.showMenu(&mainMenu_);});
    remoteVisL_.crosshair().showMinMaxRect(false);
#endif

    uint16_t hMin = RInput::input.minJoyRawH;
    uint16_t hMax = RInput::input.maxJoyRawH;
    uint16_t vMin = RInput::input.minJoyRawV;
    uint16_t vMax = RInput::input.maxJoyRawV;
    uint16_t hCur = RInput::input.joyRawH;
    uint16_t vCur = RInput::input.joyRawV;

    Console::console.printfBroadcast("Hor: [%d..%d..%d] Ver: [%d..%d..%d] \n", hMin, hCur, hMax, vMin, vCur, vMax);
    if(hMin < 800 && hMax > 4096-800 && vMin < 800 && vMax > 4096-800) { 
      // accept calibration
      params_.hCalib.min = hMin;
      params_.hCalib.max = hMax;
      params_.hCalib.center = hCur;
      params_.vCalib.min = vMin;
      params_.vCalib.max = vMax;
      params_.vCalib.center = vCur;
      RInput::input.setCalibration(params_.hCalib, params_.vCalib);
      bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
      bb::ConfigStorage::storage.store();

      RDisplay::display.flashLED(RDisplay::LED_BOTH, 2, 250, 250, 0, 150, 0);
    } else {
      // Values out of acceptable bounds - reject calibration.
      RDisplay::display.flashLED(RDisplay::LED_BOTH, 2, 250, 250, 150, 0, 0);
    }
    mode_ = MODE_REGULAR;
  } else {
#if defined(LEFT_REMOTE)
    remoteVisR_.crosshair().showMinMaxRect(false);
#else
    Console::console.printfBroadcast("Finish calib other remote only defined for right remote");
#endif
  }
}

void RRemote::setPrimary(bool thisremote) {
  params_.isPrimary = thisremote;

#if defined(LEFT_REMOTE)
  bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE, sequenceNumber());
  packet.payload.config.type = bb::ConfigPacket::CONFIG_SET_PRIMARY_REMOTE;
  if(thisremote) packet.payload.config.parameter = 0;
  else packet.payload.config.parameter = 1;
  XBee::xbee.sendTo(params_.otherRemoteAddress, packet, true);
#endif
    bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
    bb::ConfigStorage::storage.store();
}


bb::Result RRemote::fillAndSend() {
#if defined(LEFT_REMOTE)
  Packet packet(PACKET_TYPE_CONTROL, PACKET_SOURCE_LEFT_REMOTE, sequenceNumber());
#else
  Packet packet(PACKET_TYPE_CONTROL, PACKET_SOURCE_RIGHT_REMOTE, sequenceNumber());
#endif
  if(params_.isPrimary) {
    packet.source = PACKET_SOURCE_PRIMARY_REMOTE;
    packet.payload.control.primary = true;
  }

  RInput::input.fillControlPacket(packet.payload.control);

  remoteVisL_.visualizeFromPacket(packet.payload.control);

  Result res = RES_OK;
#if !defined(LEFT_REMOTE) // right remote sends to left remote
  if(params_.otherRemoteAddress != 0) {
    //Console::console.printfBroadcast("Sending control to right remote 0x%llx\n", params_.otherRemoteAddress);
    res = bb::XBee::xbee.sendTo(params_.otherRemoteAddress, packet, false);
    if(res != RES_OK) Console::console.printfBroadcast("%s\n", errorMessage(res));
  }
#endif

  // both remotes send to droid
  if(params_.droidAddress != 0 && mode_ == MODE_REGULAR) {
    //Console::console.printfBroadcast("Sending control to droid 0x%llx\n", params_.droidAddress);
    res = bb::XBee::xbee.sendTo(params_.droidAddress, packet, false);
    res = bb::XBee::xbee.sendTo(params_.droidAddress, packet, false);
    res = bb::XBee::xbee.sendTo(params_.droidAddress, packet, false);
    if(res != RES_OK) {
      RDisplay::display.setLED(RDisplay::LED_COMM, 0, 150, 0);
      Console::console.printfBroadcast("%s\n", errorMessage(res)); 
    } else {
      RDisplay::display.setLED(RDisplay::LED_COMM, 150, 0, 0);
    }
  } else {
      RDisplay::display.setLED(RDisplay::LED_COMM, 0, 0, 150);
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
  } 
  
  else if(words[0] == "running_status") {
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

  else if(words[0] == "calibrate") {
    startCalibration(true);

    return RES_OK;
  }

  else if(words[0] == "calibrate_imu") {
    RInput::input.imu().calibrate();
    return RES_OK;
  }

  else if(words[0] == "reset") {
    factoryReset(true);
    return RES_OK;
  }

  else if(words[0] == "testsuite") {
    runTestsuite();
    return RES_OK;
  }

  return Subsystem::handleConsoleCommand(words, stream);
} 

Result RRemote::incomingControlPacket(uint64_t srcAddr, PacketSource source, uint8_t rssi, const ControlPacket& packet) {
#if defined(LEFT_REMOTE)
  //if(srcAddr == params_.otherRemoteAddress && params_.otherRemoteAddress != 0) {
  if(source == PACKET_SOURCE_RIGHT_REMOTE) {
      remoteVisR_.visualizeFromPacket(packet);
      if(packet.button5 == true ||
         packet.button6 == true ||
         packet.button7 == true) {
          remoteVisR_.crosshair().showMinMaxRect(false);
      }
      return RES_OK;    
  }
  Console::console.printfBroadcast("Unknown address 0x%llx sent Control packet to left remote\n", srcAddr);
  return RES_SUBSYS_COMM_ERROR;
#else
  Console::console.printfBroadcast("Address 0x%llx sent Control packet to right remote - should never happen\n", srcAddr);
  return RES_SUBSYS_COMM_ERROR;
#endif
}

Result RRemote::incomingStatePacket(uint64_t srcAddr, PacketSource source, uint8_t rssi, const StatePacket& packet) {
#if defined(LEFT_REMOTE)
  return RES_OK; // Ignore for now
#else
  Console::console.printfBroadcast("Address 0x%llx sent State packet to right remote - should never happen\n", srcAddr);
  return RES_SUBSYS_COMM_ERROR;
#endif
}

Result RRemote::incomingConfigPacket(uint64_t srcAddr, PacketSource source, uint8_t rssi, const ConfigPacket& packet) {
#if defined(LEFT_REMOTE)
  Console::console.printfBroadcast("Address 0x%llx sent Config packet to left remote - should never happen\n", srcAddr);
  return RES_OK; // Ignore for now
  return RES_SUBSYS_COMM_ERROR;
#else
  Console::console.printfBroadcast("Got config packet from 0x%llx, type %d, payload 0x%llx\n", srcAddr, packet.type, packet.parameter);
  if(srcAddr == params_.otherRemoteAddress || params_.otherRemoteAddress == 0) { // if we're not bound, we accept config packets from anyone
    switch(packet.type) {
    case bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID:
      if(packet.parameter != srcAddr) {
        Console::console.printfBroadcast("Error: Pairing packet source 0x%llx and payload 0x%llx don't match\n", srcAddr, packet.parameter);
        return RES_SUBSYS_COMM_ERROR;
      }

      Console::console.printfBroadcast("Setting Left Remote ID to 0x%llx.\n", packet.parameter);
      params_.otherRemoteAddress = packet.parameter;
      bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
      bb::ConfigStorage::storage.store();
      return RES_OK; 

    case bb::ConfigPacket::CONFIG_SET_DROID_ID:
      Console::console.printfBroadcast("Setting Droid ID to 0x%llx.\n", packet.parameter);
      params_.droidAddress = packet.parameter;
      bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
      bb::ConfigStorage::storage.store();
      return RES_OK; 

    case bb::ConfigPacket::CONFIG_FACTORY_RESET:
      if(packet.parameter == bb::ConfigPacket::MAGIC) { // checks out
        factoryReset(true);
        return RES_OK; // HA! This never returns! NEVER! Hahahahahahaaaaa!
      }
      Console::console.printfBroadcast("Got factory reset packet but Magic 0x%llx doesn't check out!\n", packet.parameter);
      return RES_SUBSYS_COMM_ERROR;

    case bb::ConfigPacket::CONFIG_CALIBRATE:
      if(packet.parameter == bb::ConfigPacket::MAGIC) { // checks out
        startCalibration(true);
        return RES_OK;
      }
      Console::console.printfBroadcast("Got factory reset packet but ID 0x%x doesn't check out!\n", packet.parameter);
      return RES_SUBSYS_COMM_ERROR;

    case bb::ConfigPacket::CONFIG_SET_PRIMARY_REMOTE:
      if(packet.parameter != 0) {
        params_.isPrimary = true;
      } else {
        params_.isPrimary = false;
      }
      bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
      bb::ConfigStorage::storage.store();
      return RES_OK; 
    default:
      Console::console.printfBroadcast("Unknown config packet type 0x%x.\n", packet.type);
      return RES_SUBSYS_COMM_ERROR;
    }
  } else {
    Console::console.printfBroadcast("Config packet from unknown source\n");
  }
  Console::console.printfBroadcast("Should never get here.\n");
  return RES_SUBSYS_COMM_ERROR;
#endif
}

void RRemote::printStatus(ConsoleStream *stream) {
  const unsigned int bufsize = 255;
  char buf[bufsize];

  float pitch, roll, heading, ax, ay, az;
  RInput::input.imu().getFilteredPRH(pitch, roll, heading);
  //RInput::input.imu().getAccelMeasurement(ax, ay, az);
  RInput::input.imu().getGravCorrectedAccel(ax, ay, az);

  snprintf(buf, bufsize, "S%ld AX%f AY%f AZ%f\n", seqnum_, ax, ay, az);


#if 0
  snprintf(buf, bufsize, "S%d H%d [%d..%d..%d] %.2f V%d [%d..%d..%d] %.2f P%.1f R%.1f H%.1f AX%f AY%f AZ%f P1%.1f P2%.1f Batt%.1f B%c%c%c%c%c%c%c%c\n",
    seqnum_,
    RInput::input.joyRawH, RInput::input.hCalib.min, RInput::input.hCalib.center, RInput::input.hCalib.max, RInput::input.joyH,
    RInput::input.joyRawV, RInput::input.vCalib.min, RInput::input.vCalib.center, RInput::input.vCalib.max, RInput::input.joyV,
    pitch, roll, heading,
    ax, ay, az,
    RInput::input.pot1, RInput::input.pot2,
    RInput::input.battery,
    RInput::input.buttons[RInput::BUTTON_PINKY] ? '0' : '1',
    RInput::input.buttons[RInput::BUTTON_INDEX] ? '0' : '1',
    RInput::input.buttons[RInput::BUTTON_JOY] ? '0' : '1',
    RInput::input.buttons[RInput::BUTTON_LEFT] ? '0' : '1',
    RInput::input.buttons[RInput::BUTTON_RIGHT] ? '0' : '1',
    RInput::input.buttons[RInput::BUTTON_CONFIRM] ? '0' : '1',
    RInput::input.buttons[RInput::BUTTON_TOP_LEFT] ? '0' : '1',
    RInput::input.buttons[RInput::BUTTON_TOP_RIGHT] ? '0' : '1');
#endif

  if(stream != NULL) stream->printf(buf);
  else Console::console.printfBroadcast(buf);
}

void RRemote::runTestsuite() {
  RInput::input.testMatrix();
}
