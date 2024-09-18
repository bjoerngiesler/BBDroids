#include <cstdio>
#include "RRemote.h"

RRemote RRemote::remote;
RRemote::RemoteParams RRemote::params_;
bb::ConfigStorage::HANDLE RRemote::paramsHandle_;

RRemote::RRemote(): 
  imu_(IMU_ADDR),
  mode_(MODE_REGULAR) {
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
  bottomLabel_.setJustification(RLabelWidget::LEFT_JUSTIFIED, RLabelWidget::BOTTOM_JUSTIFIED);
  bottomLabel_.setFrameType(RLabelWidget::FRAME_TOP);
  bottomLabel_.setTitle(VERSION_STRING);
}

Result RRemote::initialize() { 
  if(imu_.begin() == false) {
    Console::console.printfBroadcast("IMU not available\n");
  }

  RInput::input.begin();
  if(RInput::input.buttons[RInput::BUTTON_LEFT] == true &&
     RInput::input.buttons[RInput::BUTTON_RIGHT] == true) {
    mode_ = MODE_CALIBRATION;
    showCalib();
  } else {
    mode_ = MODE_REGULAR;
    showMain();
  }

  paramsHandle_ = ConfigStorage::storage.reserveBlock("remote", sizeof(params_));
	if(ConfigStorage::storage.blockIsValid(paramsHandle_)) {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is valid.\n", paramsHandle_);
    ConfigStorage::storage.readBlock(paramsHandle_, (uint8_t*)&params_);
    Console::console.printfBroadcast("Remote: leftID 0x%x, rightID 0x%x\n", params_.leftID, params_.rightID);
  } else {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is invalid, using initialized parameters.\n", paramsHandle_);
    ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
  }

  populateMenus();

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

void RRemote::showCalib() {
  topLabel_.setTitle("Left Remote >");
  setMainWidget(&remoteVisL_);
}

void RRemote::showMain() {
  setMainWidget(&mainVis_);
  RInput::input.setTopRightLongPressCallback([=]{RRemote::remote.showMenu(&mainMenu_);});
}

void RRemote::showMenu(RMenuWidget* menu) {
  setMainWidget(menu);
  menu->resetCursor();
}

void RRemote::showGraphs() {
  setMainWidget(&graphs_);
}

void RRemote::showPairDroidMenu() {
  waitMessage_.setNeedsFullRedraw();
  waitMessage_.draw();
  
  pairDroidMenu_.clear();

  Result res = XBee::xbee.discoverNodes(discoveredNodes_);
  if(res != RES_OK) {
    Console::console.printfBroadcast("%s\n", errorMessage(res));
    return;
  }

  pairDroidMenu_.clear();
  for(auto& n: discoveredNodes_) {
    Console::console.printfBroadcast("Station \"%s\", ID 0x%x\n", n.name, n.stationId);
    if(XBee::stationTypeFromId(n.stationId) != XBee::STATION_DROID) continue;
    Console::console.printfBroadcast("Adding entry for \"%s\"\n", n.name);
    pairDroidMenu_.addEntry(n.name, [=]() { RRemote::remote.selectDroid(n.stationId); });
  }
  Console::console.printfBroadcast("Adding entry for \"Back\"\n");
  pairDroidMenu_.addEntry("Back", [=]() { RRemote::remote.showMenu(&pairMenu_); });

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
    pairRemoteMenu_.addEntry(n.name, [=]() { RRemote::remote.selectRightRemote(n.stationId); });
  }
  pairRemoteMenu_.addEntry("Back", [=]() { RRemote::remote.showMenu(&pairMenu_); });

  showMenu(&pairRemoteMenu_);
}

void RRemote::populateMenus() {
  mainMenu_.setName("Main Menu");
  pairMenu_.setName("Pair");
  leftRemoteMenu_.setName("Left Remote");
  rightRemoteMenu_.setName("Right Remote");
  droidMenu_.setName("Droid");
  
  mainMenu_.clear();
  mainMenu_.addEntry("Pair...", [=]() { RRemote::remote.showMenu(&pairMenu_); });
  if(params_.droidID != 0) mainMenu_.addEntry("Droid...", [=]() { showMenu(&droidMenu_); });
  mainMenu_.addEntry("Left Remote...", [=]() { showMenu(&leftRemoteMenu_); });
  if(params_.rightID != 0) mainMenu_.addEntry("Right Remote...", [=]() { showMenu(&rightRemoteMenu_); });
  mainMenu_.addEntry("Back", []() { RRemote::remote.showMain(); });
  
  pairMenu_.clear();
  pairMenu_.addEntry("Right Remote...", []() { RRemote::remote.showPairRemoteMenu(); });
  pairMenu_.addEntry("Droid...", []() { RRemote::remote.showPairDroidMenu(); });
  pairMenu_.addEntry("Back", [=]() { RRemote::remote.showMenu(&mainMenu_); });

  droidMenu_.clear();
  droidMenu_.addEntry("Back", [=]() {showMenu(&mainMenu_);});

  leftRemoteMenu_.clear();
  leftRemoteMenu_.addEntry("Factory Reset", [=]() {factoryReset(true);});
  leftRemoteMenu_.addEntry("Back", [=]() {showMenu(&mainMenu_);});

  rightRemoteMenu_.clear();
  rightRemoteMenu_.addEntry("Factory Reset", [=]() {factoryReset(false);});
  rightRemoteMenu_.addEntry("Back", [=]() {showMenu(&mainMenu_);});
}

void RRemote::setTopTitle(const String& title) {
  topLabel_.setTitle(title);
}

void RRemote::setBottomTitle(const String& title) {
  bottomLabel_.setTitle(title);
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

  showMenu(&mainMenu_);
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

  showMenu(&mainMenu_);
#endif
}

void RRemote::factoryReset(bool leftremote) {
  if(leftremote) {
    Console::console.printfBroadcast("Factory reset left remote!\n");
    bb::ConfigStorage::storage.factoryReset();
    RDisplay::display.setLED(RDisplay::LED_BOTH, 0, 0, 150);
    restartMessage_.draw();
    while(true);
  } else {
    Console::console.printfBroadcast("Factory reset right remote!\n");
  }
}

Result RRemote::start(ConsoleStream *stream) {
  (void)stream;
  runningStatus_ = false;
  operationStatus_ = RES_OK;

  Runloop::runloop.setCycleTimeMicros(1e6/imu_.dataRate());
  started_ = true;

  return RES_OK;
}

Result RRemote::stop(ConsoleStream *stream) {
  (void)stream;
  RDisplay::display.setLED(RDisplay::LED_BOTH, 150, 150, 0);
  RDisplay::display.showLEDs();
  operationStatus_ = RES_OK;
  started_ = false;
  
  return RES_OK;
}

Result RRemote::step() {
  if(!started_) return RES_SUBSYS_NOT_STARTED;

  imu_.update();
  RInput::input.update();

  if(mode_ & MODE_CALIBRATION) return stepCalib();

  if((bb::Runloop::runloop.getSequenceNumber() % 4) == 0) {
    calibLabel_.setTitle(String("H") + RInput::input.joyRawH + " V" + RInput::input.joyRawV);

    float r, p, h;
    float ax, ay, az;

    imu_.getFilteredRPH(r, p, h);
    imu_.getAccelMeasurement(ax, ay, az);
    imuViz_.setRPH(r, p, h);
    imuViz_.setAccel(ax, ay, az);

    if(1) { // needsDraw_) {
      topLabel_.draw();
      bottomLabel_.draw();
      if(mainWidget_ != NULL) mainWidget_->draw();
      needsDraw_ = false;
    }

    fillAndSend();
    if(runningStatus_) {
      printStatus();
    }
  } else {
    RDisplay::display.setLED(RDisplay::LED_BOTH, 0, 0, 0);
    RDisplay::display.showLEDs();
  }

  return RES_OK;
}

Result RRemote::stepCalib() {
  if(mode_ == MODE_CALIBRATION) {
    if(RInput::input.buttons[RInput::BUTTON_LEFT] == false &&
       RInput::input.buttons[RInput::BUTTON_RIGHT] == false) {
      setCalibrationMode(MODE_CALIB_CENTER);
      rawHBuf.clear();
      rawVBuf.clear();
      hCalib = RInput::AxisCalib();
      vCalib = RInput::AxisCalib();
      calibRounds = MAX_CALIB_ROUNDS_BEFORE_ABORT;
    } else Console::console.printfBroadcast("Calibration - Please release front buttons\n");
    return RES_OK;
  }

  if(calibRounds-- < 0) {
    abortCalibration();
    return RES_OK;
  }

  while(rawHBuf.size() > MAX_CALIB_BUFSIZE) rawHBuf.pop_front();
  while(rawVBuf.size() > MAX_CALIB_BUFSIZE) rawVBuf.pop_front();
  rawHBuf.push_back(RInput::input.joyRawH);
  rawVBuf.push_back(RInput::input.joyRawV);
  if(rawHBuf.size() < MAX_CALIB_BUFSIZE || rawVBuf.size() < MAX_CALIB_BUFSIZE) {
    return RES_OK;
  }

  uint16_t maxH = *std::max_element(rawHBuf.begin(), rawHBuf.end());
  uint16_t minH = *std::min_element(rawHBuf.begin(), rawHBuf.end());
  uint16_t maxV = *std::max_element(rawVBuf.begin(), rawVBuf.end());
  uint16_t minV = *std::min_element(rawVBuf.begin(), rawVBuf.end());

  // Console::console.printfBroadcast("Calibration: Buf size %d/%d H %d..%d (%d), V %d..%d (%d)\n", rawHBuf.size(), rawVBuf.size(), minH, maxH, maxH-minH, minV, maxV, maxV-minV);
  
  switch(mode_) {
  case MODE_CALIB_CENTER:
    if(maxH - minH < MAX_CALIB_DIFF && maxV - minV < MAX_CALIB_DIFF && 
       minH > 1024 && maxH < 3072 && minV > 1024 && maxV < 3072) {
      hCalib.center = (maxH + minH)/2;
      vCalib.center = (maxV + minV)/2;
      rawHBuf.clear();
      rawVBuf.clear();
      calibRounds = MAX_CALIB_ROUNDS_BEFORE_ABORT;
      setCalibrationMode(MODE_CALIB_X_NEG);
    }
    break;

  case MODE_CALIB_X_NEG:
    if(maxH - minH < MAX_CALIB_DIFF && maxH < 1024) {
      hCalib.min = minH;
      rawHBuf.clear();
      rawVBuf.clear();
      calibRounds = MAX_CALIB_ROUNDS_BEFORE_ABORT;
      setCalibrationMode(MODE_CALIB_X_POS);
    }
    break;

  case MODE_CALIB_X_POS:
    if(maxH - minH < MAX_CALIB_DIFF && minH > 3072) {
      hCalib.max = maxH;
      rawHBuf.clear();
      rawVBuf.clear();
      calibRounds = MAX_CALIB_ROUNDS_BEFORE_ABORT;
      setCalibrationMode(MODE_CALIB_Y_NEG);
    }
    break;

  case MODE_CALIB_Y_NEG:
    if(maxV - minV < MAX_CALIB_DIFF && maxV < 1024) {
      vCalib.min = minV;
      rawHBuf.clear();
      rawVBuf.clear();
      calibRounds = MAX_CALIB_ROUNDS_BEFORE_ABORT;
      setCalibrationMode(MODE_CALIB_Y_POS);
    }
    break;
  case MODE_CALIB_Y_POS:
    if(maxV - minV < MAX_CALIB_DIFF && maxV > 3072) {
      vCalib.max = maxV;
      finishCalibration();
    }
    break;
  }

  return RES_OK;
}

void RRemote::setCalibrationMode(Mode mode) {
  bb::Runloop::runloop.excuseOverrun();

  switch(mode_) {
  case MODE_CALIB_CENTER:
    RDisplay::display.flashLED(RDisplay::LED_BOTH, 2, 100, 100, 0, 0, 150);
    break;
  case MODE_CALIB_X_NEG:
    RDisplay::display.flashLED(RDisplay::LED_RIGHT, 2, 100, 100, 150, 0, 150);
    break;
  case MODE_CALIB_X_POS:
    RDisplay::display.flashLED(RDisplay::LED_LEFT, 2, 100, 100, 150, 0, 150);
    break;
  case MODE_CALIB_Y_NEG:
    RDisplay::display.flashLED(RDisplay::LED_BOTH, 2, 100, 100, 150, 0, 150);
    break;
  case MODE_CALIB_Y_POS:
    RDisplay::display.flashLED(RDisplay::LED_BOTH, 2, 100, 100, 0, 0, 150);
    break;
  }

  mode_ = mode;
  Console::console.printfBroadcast("Switching to calibration mode 0x%x\n", mode_);

  switch(mode_) {
  case MODE_CALIB_CENTER:
    Console::console.printfBroadcast("Both blue\n");
    RDisplay::display.setLED(RDisplay::LED_BOTH, 0, 0, 150);
    RDisplay::display.showLEDs();
    break;
  case MODE_CALIB_X_NEG:
    Console::console.printfBroadcast("Left violet, right off\n");
    RDisplay::display.setLED(RDisplay::LED_RIGHT, 150, 0, 150);
    RDisplay::display.setLED(RDisplay::LED_LEFT, 0, 0, 0);
    RDisplay::display.showLEDs();
    break;
  case MODE_CALIB_X_POS:
    Console::console.printfBroadcast("Right violet, left off\n");
    RDisplay::display.setLED(RDisplay::LED_LEFT, 150, 0, 150);
    RDisplay::display.setLED(RDisplay::LED_RIGHT, 0, 0, 0);
    RDisplay::display.showLEDs();
    break;
  case MODE_CALIB_Y_NEG:
    Console::console.printfBroadcast("Both violet\n");
    RDisplay::display.setLED(RDisplay::LED_BOTH, 150, 0, 150);
    RDisplay::display.showLEDs();
    break;
  case MODE_CALIB_Y_POS:
    Console::console.printfBroadcast("Both blue\n");
    RDisplay::display.setLED(RDisplay::LED_BOTH, 0, 0, 150);
    RDisplay::display.showLEDs();
    break;
  }
}

void RRemote::abortCalibration() {
  Console::console.printfBroadcast("Aborting calibration\n");
  mode_ = MODE_REGULAR;
  RDisplay::display.flashLED(RDisplay::LED_BOTH, 3, 100, 100, 150, 0, 0);
}

void RRemote::finishCalibration() {
  Console::console.printfBroadcast("Finishing calibration - H %d..%d..%d, V %d..%d..%d\n", hCalib.min, hCalib.center, hCalib.max, vCalib.min, vCalib.center, vCalib.max);
  mode_ = MODE_REGULAR;
  RDisplay::display.flashLED(RDisplay::LED_BOTH, 3, 100, 100, 0, 150, 0);
  params_.hCalib = hCalib; params_.vCalib = vCalib;
  RInput::input.setCalibration(hCalib, vCalib);
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

#if defined(LEFT_REMOTE)
  packet.payload.control.button0 = RInput::input.buttons[RInput::BUTTON_PINKY];
  packet.payload.control.button1 = RInput::input.buttons[RInput::BUTTON_INDEX];
  packet.payload.control.button2 = RInput::input.buttons[RInput::BUTTON_RIGHT];
  packet.payload.control.button3 = RInput::input.buttons[RInput::BUTTON_LEFT];
  packet.payload.control.button4 = RInput::input.buttons[RInput::BUTTON_JOY];
  packet.payload.control.button5 = false;
  packet.payload.control.button6 = false;
  packet.payload.control.button7 = false;
#else
  packet.payload.control.button0 = RInput::input.buttons[RInput::BUTTON_PINKY];
  packet.payload.control.button1 = RInput::input.buttons[RInput::BUTTON_INDEX];
  packet.payload.control.button2 = RInput::input.buttons[RInput::BUTTON_LEFT];
  packet.payload.control.button3 = RInput::input.buttons[RInput::BUTTON_RIGHT];
  packet.payload.control.button4 = RInput::input.buttons[RInput::BUTTON_JOY];
  packet.payload.control.button5 = RInput::input.buttons[RInput::BUTTON_TOP_LEFT];
  packet.payload.control.button6 = RInput::input.buttons[RInput::BUTTON_TOP_RIGHT];
  packet.payload.control.button7 = RInput::input.buttons[RInput::BUTTON_CONFIRM];
#endif

  packet.payload.control.setAxis(0, RInput::input.joyH, bb::ControlPacket::UNIT_UNITY_CENTERED);
  packet.payload.control.setAxis(1, RInput::input.joyV, bb::ControlPacket::UNIT_UNITY_CENTERED);

  if (imu_.available()) {
    float roll, pitch, heading;
    float ax, ay, az;
    imu_.getFilteredRPH(roll, pitch, heading);
    imu_.getAccelMeasurement(ax, ay, az);

    packet.payload.control.setAxis(2, roll, bb::ControlPacket::UNIT_DEGREES_CENTERED); 
    packet.payload.control.setAxis(3, pitch, bb::ControlPacket::UNIT_DEGREES_CENTERED);
    packet.payload.control.setAxis(4, heading, bb::ControlPacket::UNIT_DEGREES);
    packet.payload.control.setAxis(5, ax, bb::ControlPacket::UNIT_UNITY_CENTERED);
    packet.payload.control.setAxis(6, bb::ControlPacket::UNIT_UNITY_CENTERED);
    packet.payload.control.setAxis(7, bb::ControlPacket::UNIT_UNITY_CENTERED);
  }

  packet.payload.control.setAxis(8, RInput::input.pot1, bb::ControlPacket::UNIT_UNITY);

#if defined(LEFT_REMOTE)
  packet.payload.control.setAxis(9, 0);
#else
  // FIXME replace by values set in menu system
  packet.payload.control.setAxis(9, RInput::input.pot2, bb::ControlPacket::UNIT_UNITY);
#endif

  packet.payload.control.battery = RInput::input.battery * 63;

  remoteVisL_.visualizeFromPacket(packet.payload.control);


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
    if(res != RES_OK) {
      RDisplay::display.setLED(RDisplay::LED_LEFT, 0, 150, 0);
      RDisplay::display.showLEDs();
      Console::console.printfBroadcast("%s\n", errorMessage(res)); 
    } else {
      RDisplay::display.setLED(RDisplay::LED_LEFT, 150, 0, 0);
      RDisplay::display.showLEDs();
    }
  } else {
      RDisplay::display.setLED(RDisplay::LED_LEFT, 0, 0, 150);
      RDisplay::display.showLEDs();
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
    setCalibrationMode(MODE_CALIB_CENTER);
    rawHBuf.clear();
    rawVBuf.clear();
    hCalib = RInput::AxisCalib();
    vCalib = RInput::AxisCalib();
    calibRounds = MAX_CALIB_ROUNDS_BEFORE_ABORT;

    return RES_OK;
  }

  return Subsystem::handleConsoleCommand(words, stream);
} 

Result RRemote::incomingPacket(uint16_t source, uint8_t rssi, const Packet& packet) {
#if defined(LEFT_REMOTE)
  if(source == params_.droidID && params_.droidID != 0) {
    Console::console.printfBroadcast("Packet from droid!\n");
    return RES_OK;
  } else if(source == params_.rightID && params_.rightID != 0) {
    if(packet.type == PACKET_TYPE_CONTROL) {
      remoteVisR_.visualizeFromPacket(packet.payload.control);
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
        memset(&params_, 0, sizeof(params_));
        bb::ConfigStorage::storage.readBlock(paramsHandle_, (uint8_t*)&params_);
        Console::console.printfBroadcast("Re-read left remote as 0x%x\n", params_.leftID);
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

  float roll, pitch, heading;
  imu_.getFilteredRPH(roll, pitch, heading);

#if defined(ESP32_REMOTE)
  float temp = roll;
  roll = pitch;
  pitch = temp;
#endif

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
    roll, pitch, heading,
    RInput::input.battery,
    RInput::input.pot1, 
    RInput::input.pot2);

  if(stream != NULL) stream->printf(buf);
  else Console::console.printfBroadcast(buf);
}

