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
"\trunning_status on|off  Continuously prints status\r\n"\
"\treset                  Factory reset";

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
  bottomLabel_.setJustification(RLabelWidget::HOR_CENTERED, RLabelWidget::BOTTOM_JUSTIFIED);
  bottomLabel_.setFrameType(RLabelWidget::FRAME_TOP);
  bottomLabel_.setTitle(VERSION_STRING);
}

Result RRemote::initialize() { 
  if(imu_.begin() == false) {
    Console::console.printfBroadcast("IMU not available\n");
  }

  RInput::input.begin();

  paramsHandle_ = ConfigStorage::storage.reserveBlock("remote", sizeof(params_));
	if(ConfigStorage::storage.blockIsValid(paramsHandle_)) {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is valid.\n", paramsHandle_);
    ConfigStorage::storage.readBlock(paramsHandle_, (uint8_t*)&params_);
    Console::console.printfBroadcast("Remote: leftID 0x%x, rightID 0x%x\n", params_.leftID, params_.rightID);
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
}

void RRemote::showMenu(RMenuWidget* menu) {
  setMainWidget(menu);
  menu->resetCursor();
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
  if(params_.droidID != 0) mainMenu_.addEntry("Droid...", [=]() { showMenu(&droidMenu_); });
  mainMenu_.addEntry("Left Remote...", [=]() { showMenu(&leftRemoteMenu_); });
  if(params_.rightID != 0) mainMenu_.addEntry("Right Remote...", [=]() { showMenu(&rightRemoteMenu_); });
  mainMenu_.addEntry("Pair...", [=]() { RRemote::remote.showMenu(&pairMenu_); });
  mainMenu_.addEntry("Back", []() { RRemote::remote.showMain(); });
  
  pairMenu_.clear();
  pairMenu_.addEntry("Right Remote...", []() { RRemote::remote.showPairRemoteMenu(); });
  pairMenu_.addEntry("Droid...", []() { RRemote::remote.showPairDroidMenu(); });
  pairMenu_.addEntry("Back", [=]() { RRemote::remote.showMenu(&mainMenu_); });

  droidMenu_.clear();
  droidMenu_.addEntry("Back", [=]() {showMenu(&mainMenu_);});

  leftRemoteMenu_.clear();
  leftRemoteMenu_.addEntry("Calib Joystick", [=]{startCalibration(true);});
  leftRemoteMenu_.addEntry("Factory Reset", [=]{factoryReset(true);});
  leftRemoteMenu_.addEntry("Back", [=]() {showMenu(&mainMenu_);});

  rightRemoteMenu_.clear();
  rightRemoteMenu_.addEntry("Calib Joystick", [=]{startCalibration(false);});
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

    packet.payload.config.parameter = params_.droidID;
    XBee::xbee.sendTo(params_.rightID, packet, true);
  }

  bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
  bb::ConfigStorage::storage.store();

  populateMenus();
  showMenu(&mainMenu_);
#endif
}

void RRemote::selectRightRemote(uint16_t remote) {
#if defined(LEFT_REMOTE)
  Console::console.printfBroadcast("Selecting right remote 0x%x\n", remote);
  
  params_.rightID = remote;

  bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE);
  packet.payload.config.type = bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID;
  packet.payload.config.parameter = params_.leftID;
  XBee::xbee.sendTo(params_.rightID, packet, true);  

  if(params_.droidID != 0) {
    packet.payload.config.type = bb::ConfigPacket::CONFIG_SET_DROID_ID;
    packet.payload.config.parameter = params_.droidID;
    XBee::xbee.sendTo(params_.rightID, packet, true);   
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

  Runloop::runloop.setCycleTimeMicros(1e6/imu_.dataRate());

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
  } else if(Console::console.isStarted() && XBee::xbee.isStarted() && isStarted() && RInput::input.isOK() && imu_.available()) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 0, 150, 0);
  } else if(!XBee::xbee.isStarted() || !RInput::input.isOK() || !imu_.available()) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 150, 0, 0);
  } else {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 150, 150, 0);
  }
}

Result RRemote::step() {
  if(!started_) return RES_SUBSYS_NOT_STARTED;

  imu_.update();
  RInput::input.update();

  if((bb::Runloop::runloop.getSequenceNumber() % 4) == 0) {
    updateStatusLED();
    topLabel_.draw();
    bottomLabel_.draw();
    if(mainWidget_ != NULL) mainWidget_->draw();

    fillAndSend();
    if(runningStatus_) {
      printStatus();
    }
  } else {
    RDisplay::display.setLED(RDisplay::LED_COMM, 0, 0, 0);
  }

  return RES_OK;
}

Result RRemote::stepCalib() {
  return RES_OK;
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
    bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE);
    packet.payload.config.type = bb::ConfigPacket::CONFIG_FACTORY_RESET;
    packet.payload.config.parameter = bb::ConfigPacket::MAGIC;
    XBee::xbee.sendTo(params_.rightID, packet, true);

    params_.rightID = 0;
    bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
    bb::ConfigStorage::storage.store();

    populateMenus();
#else
    Console::console.printfBroadcast("Factory reset other remote only defined for right remote");
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
    bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE);
    packet.payload.config.type = bb::ConfigPacket::CONFIG_CALIBRATE;
    packet.payload.config.parameter = bb::ConfigPacket::MAGIC;
    XBee::xbee.sendTo(params_.rightID, packet, true);

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
    remoteVisL_.crosshair().showMinMaxRect(false);
#endif

    uint16_t hMin = RInput::input.minJoyRawH;
    uint16_t hMax = RInput::input.maxJoyRawH;
    uint16_t vMin = RInput::input.minJoyRawV;
    uint16_t vMax = RInput::input.maxJoyRawV;

    Console::console.printfBroadcast("hMin: %d hMax: %d vMin: %d vMax: %d\n", hMin, hMax, vMin, vMax);
    if(hMin < 800 && hMax > 4096-800 && vMin < 800 && vMax > 4096-800) { 
      // accept calibration
      params_.hCalib.min = hMin;
      params_.hCalib.max = hMax;
      params_.vCalib.min = vMin;
      params_.vCalib.max = vMax;
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
    packet.payload.control.setAxis(6, ay, bb::ControlPacket::UNIT_UNITY_CENTERED);
    packet.payload.control.setAxis(7, az, bb::ControlPacket::UNIT_UNITY_CENTERED);
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
  if(params_.droidID != 0 && mode_ == MODE_REGULAR) {
    res = bb::XBee::xbee.sendTo(params_.droidID, packet, false);
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

  else if(words[0] == "reset") {
    factoryReset(true);
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
      if(packet.payload.control.button5 == true ||
         packet.payload.control.button6 == true ||
         packet.payload.control.button7 == true) {
          remoteVisR_.crosshair().showMinMaxRect(false);
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
      switch(packet.payload.config.type) {
      case bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID:
        Console::console.printfBroadcast("Setting Left Remote ID to 0x%x.\n", packet.payload.config.parameter);
        params_.leftID = packet.payload.config.parameter;
        bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
        bb::ConfigStorage::storage.store();
        return RES_OK; 

      case bb::ConfigPacket::CONFIG_SET_DROID_ID:
        Console::console.printfBroadcast("Setting Droid ID to 0x%x.\n", packet.payload.config.parameter);
        params_.droidID = packet.payload.config.parameter;
        bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
        bb::ConfigStorage::storage.store();
        return RES_OK; 

      case bb::ConfigPacket::CONFIG_FACTORY_RESET:
        if(packet.payload.config.parameter == bb::ConfigPacket::MAGIC) { // checks out
          factoryReset(true);
          return RES_OK; // HA! This never returns! NEVER! Hahahahahahaaaaa!
        }
        Console::console.printfBroadcast("Got factory reset packet but ID 0x%x doesn't check out!\n", packet.payload.config.parameter);
        return RES_SUBSYS_COMM_ERROR;

      case bb::ConfigPacket::CONFIG_CALIBRATE:
        if(packet.payload.config.parameter == bb::ConfigPacket::MAGIC) { // checks out
          startCalibration(true);
          return RES_OK;
        }
        Console::console.printfBroadcast("Got factory reset packet but ID 0x%x doesn't check out!\n", packet.payload.config.parameter);
        return RES_SUBSYS_COMM_ERROR;

      default:
        Console::console.printfBroadcast("Unknown config packet type 0x%x.\n", packet.payload.config.type);
        return RES_SUBSYS_COMM_ERROR;
      }
    } else {
      Console::console.printfBroadcast("Unknown packet type %d from left remote!\n", packet.type);
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

#if defined(ARDUINO_ARCH_ESP32)
  float temp = roll;
  roll = pitch;
  pitch = temp;
#endif

  sprintf(buf, "Buttons: P%cI%cJ%cL%cR%cC%cTL%cTR%c Axes: JH%5.2f %d JV%5.2f %d R%7.2fd P%7.2fd Y%7.2f Batt%7.2f P1%7.2f P2%7.2f    \n",
    RInput::input.buttons[RInput::BUTTON_PINKY] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_INDEX] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_JOY] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_LEFT] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_RIGHT] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_CONFIRM] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_TOP_LEFT] ? 'X' : '_',
    RInput::input.buttons[RInput::BUTTON_TOP_RIGHT] ? 'X' : '_',
    RInput::input.joyH, RInput::input.joyRawH,
    RInput::input.joyV, RInput::input.joyRawV,
    roll, pitch, heading,
    RInput::input.battery,
    RInput::input.pot1, 
    RInput::input.pot2);

  if(stream != NULL) stream->printf(buf);
  else Console::console.printfBroadcast(buf);
}

