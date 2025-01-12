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
"\treset                  Factory reset\n"\
"\tset_droid ADDR         Set droid address to ADDR (64bit hex - max 16 digits, omit the 0x)\n"\
"\tset_other_remote ADDR  Set other remote address to ADDR (64bit hex - max 16 digits, omit the 0x)\n";

  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  deltaR_ = 0; deltaP_ = 0; deltaH_ = 0;
  memset((void*)&lastPacketSent_, 0, sizeof(Packet));

  params_.droidAddress = 0x0;
  params_.otherRemoteAddress = 0x0;
  params_.config.leftIsPrimary = true;
  params_.config.ledBrightness = 7;
  params_.config.sendRepeats = 2;
  params_.config.lIncrRotBtn = RInput::BUTTON_NONE;
  params_.config.rIncrRotBtn = RInput::BUTTON_NONE;
  params_.config.lIncrTransBtn = RInput::BUTTON_NONE;
  params_.config.rIncrTransBtn = RInput::BUTTON_NONE;

  message_.setTitle("?");

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
    Console::console.printfBroadcast("Droid address: 0x%llx\n", params_.droidAddress);
    RInput::input.setCalibration(params_.hCalib, params_.vCalib);
#if defined(LEFT_REMOTE)
    RInput::input.setIncrementalRot(RInput::Button(params_.config.lIncrRotBtn));
#else
    RInput::input.setIncrementalRot(RInput::Button(params_.config.rIncrRotBtn));
#endif
  } else {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is invalid, using initialized parameters.\n", paramsHandle_);
    ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
  }

  return Subsystem::initialize();
}

Result RRemote::start(ConsoleStream *stream) {
  (void)stream;
  runningStatus_ = false;
  operationStatus_ = RES_OK;

#if defined(LEFT_REMOTE)
  populateMenus();
  showMain();
  drawGUI();
  showMessage("Welcome!\n\nMonaco\nCtrl System\nSW " VERSION_STRING, 3000, RDisplay::LIGHTBLUE2);
  mainWidget_->setNeedsFullRedraw();
#endif

  started_ = true;

  return RES_OK;
}

Result RRemote::stop(ConsoleStream *stream) {
  (void)stream;
  RDisplay::display.setLED(RDisplay::LED_BOTH, 255, 255, 0);
  operationStatus_ = RES_OK;
  started_ = false;
  
  return RES_OK;
}

Result RRemote::step() {
  if(!started_) return RES_SUBSYS_NOT_STARTED;

  RInput::input.update();

  if((bb::Runloop::runloop.getSequenceNumber() % 4) == 0) {
    updateStatusLED();
    drawGUI();
  } else {
    RDisplay::display.setLED(RDisplay::LED_COMM, 0, 0, 0);
  }

  if((bb::Runloop::runloop.getSequenceNumber() % 4) == 0) {
    if(runningStatus_) {
      printExtendedStatusLine();
    }

    fillAndSend();
  }

  return RES_OK;
}

void RRemote::setMainWidget(RWidget* widget) {
  widget->setPosition(RDisplay::MAIN_X, RDisplay::MAIN_Y);
  widget->setSize(RDisplay::MAIN_WIDTH, RDisplay::MAIN_HEIGHT);
  widget->takeInputFocus();
  widget->setNeedsFullRedraw();
  widget->setNeedsContentsRedraw();
  setTopTitle(widget->name());
  mainWidget_ = widget;
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
  showMessage("Please wait");
  
  pairDroidMenu_.clear();

  Console::console.printfBroadcast("Discovering nodes...\n");
  Result res = XBee::xbee.discoverNodes(discoveredNodes_);
  if(res != RES_OK) {
    Console::console.printfBroadcast("%s\n", errorMessage(res));
    return;
  }
  Console::console.printfBroadcast("%d nodes discovered.\n", discoveredNodes_.size());

  pairDroidMenu_.clear();
  pairDroidMenu_.setName(String(discoveredNodes_.size()) + " Droids");
  for(auto& n: discoveredNodes_) {
    if(XBee::stationTypeFromId(n.stationId) != XBee::STATION_DROID) continue;
    Console::console.printfBroadcast("Station \"%s\", ID 0x%x\n", n.name, n.stationId);
    pairDroidMenu_.addEntry(n.name, [=]() { RRemote::remote.selectDroid(n.address); showMain(); });
  }
  pairDroidMenu_.addEntry("<--", [=]() { RRemote::remote.showMenu(&pairMenu_); });

  Console::console.printfBroadcast("Showing droidsMenu\n");
  showMenu(&pairDroidMenu_);
}

void RRemote::showPairRemoteMenu() {
  showMessage("Please wait");

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
    pairRemoteMenu_.addEntry(n.name, [=]() { RRemote::remote.selectRightRemote(n.address); showMain(); });
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
  lRIncrRotMenu_.addEntry("Disable", [=]{setIncrRotButtonCB(RInput::BUTTON_NONE, true);showMain();}, RInput::BUTTON_NONE);
  lRIncrRotMenu_.addEntry("Left Button", [=]{setIncrRotButtonCB(RInput::BUTTON_LEFT, true);showMain();}, RInput::BUTTON_LEFT);
  lRIncrRotMenu_.addEntry("Right Button", [=]{setIncrRotButtonCB(RInput::BUTTON_RIGHT, true);showMain();}, RInput::BUTTON_RIGHT);
  lRIncrRotMenu_.addEntry("Pinky Button", [=]{setIncrRotButtonCB(RInput::BUTTON_PINKY, true);showMain();}, RInput::BUTTON_PINKY);
  lRIncrRotMenu_.addEntry("Index Button", [=]{setIncrRotButtonCB(RInput::BUTTON_INDEX, true);showMain();}, RInput::BUTTON_INDEX);
  lRIncrRotMenu_.addEntry("<--", [=]{showMenu(&leftRemoteMenu_);});
  lRIncrRotMenu_.highlightWidgetsWithTag(params_.config.lIncrRotBtn);

  rRIncrRotMenu_.clear();
  rRIncrRotMenu_.setName("Incr Rotation");
  rRIncrRotMenu_.addEntry("Disable", [=]{setIncrRotButtonCB(RInput::BUTTON_NONE, false);showMain();}, RInput::BUTTON_NONE);
  rRIncrRotMenu_.addEntry("Left Button", [=]{setIncrRotButtonCB(RInput::BUTTON_LEFT, false);showMain();}, RInput::BUTTON_LEFT);
  rRIncrRotMenu_.addEntry("Right Button", [=]{setIncrRotButtonCB(RInput::BUTTON_RIGHT, false);showMain();}, RInput::BUTTON_RIGHT);
  rRIncrRotMenu_.addEntry("Pinky Button", [=]{setIncrRotButtonCB(RInput::BUTTON_PINKY, false);showMain();}, RInput::BUTTON_PINKY);
  rRIncrRotMenu_.addEntry("Index Button", [=]{setIncrRotButtonCB(RInput::BUTTON_INDEX, false);showMain();}, RInput::BUTTON_INDEX);
  rRIncrRotMenu_.addEntry("<--", [=]{showMenu(&rightRemoteMenu_);});
  rRIncrRotMenu_.highlightWidgetsWithTag(params_.config.rIncrRotBtn);

  leftRemoteMenu_.clear();
  leftRemoteMenu_.addEntry("Incr Rot...", [=]{showMenu(&lRIncrRotMenu_);});
  leftRemoteMenu_.addEntry("Calib Joystick", [=]{startCalibrationCB(true);});
  leftRemoteMenu_.addEntry("Set Primary", [=]{setLeftIsPrimaryCB(true);showMain();}, params_.config.leftIsPrimary?1:0);
  leftRemoteMenu_.addEntry("Factory Reset", [=]{factoryResetCB(true);});
  leftRemoteMenu_.addEntry("<--", [=]() {showMenu(&mainMenu_);});
  leftRemoteMenu_.highlightWidgetsWithTag(1);

  rightRemoteMenu_.clear();
  rightRemoteMenu_.addEntry("Incr Rot...", [=]{showMenu(&rRIncrRotMenu_);});
  rightRemoteMenu_.addEntry("Calib Joystick", [=]{startCalibrationCB(false);});
  rightRemoteMenu_.addEntry("Set Primary", [=]{setLeftIsPrimaryCB(false);showMain();}, params_.config.leftIsPrimary?0:1);
  rightRemoteMenu_.addEntry("Factory Reset", [=]() {factoryResetCB(false);});
  rightRemoteMenu_.addEntry("<--", [=]() {showMenu(&mainMenu_);});
  rightRemoteMenu_.highlightWidgetsWithTag(1);
}

void RRemote::drawGUI() {
  topLabel_.draw();
  bottomLabel_.draw();
  if(mainWidget_ != NULL) mainWidget_->draw();
  if(needsMenuRebuild_) {
    populateMenus();
    needsMenuRebuild_ = false;
  }
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
  Runloop::runloop.excuseOverrun();
  
  params_.droidAddress = droid;

  if(params_.otherRemoteAddress != 0) {
    ConfigPacket packet;
    ConfigPacket::ConfigReplyType reply;
    packet.type = bb::ConfigPacket::CONFIG_SET_DROID_ID;
    packet.cfgPayload.address = params_.droidAddress;
    Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
    if(res != RES_OK) {
      showMessage(String("D ID -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
    } 
    if(reply != ConfigPacket::CONFIG_REPLY_OK) {
      showMessage(String("D ID -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
    }
  } 
        
  showMessage("Success", MSGDELAY, RDisplay::LIGHTGREEN2);
  storeParams();
  needsMenuRebuild_ = true;
#endif
}

void RRemote::selectRightRemote(uint64_t address) {
#if defined(LEFT_REMOTE)
  Console::console.printfBroadcast("Selecting right remote 0x%llx\n", address);
  Runloop::runloop.excuseOverrun();

  params_.otherRemoteAddress = address;

  ConfigPacket packet;
  ConfigPacket::ConfigReplyType reply;

  // Set left remote address to right remote
  packet.type = bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID;
  packet.cfgPayload.address = XBee::xbee.hwAddress();
  Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
  if(res != RES_OK) {
    showMessage(String("L ID -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
    return;
  }
  if(reply != ConfigPacket::CONFIG_REPLY_OK) {
    showMessage(String("L ID -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
    return;
  }

  // Set droid address to right remote
  if(params_.droidAddress != 0) {
    packet.type = bb::ConfigPacket::CONFIG_SET_DROID_ID;
    packet.cfgPayload.address = params_.droidAddress;
    Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
    if(res != RES_OK) {
      showMessage(String("D ID -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
      return;
    }
    if(reply != ConfigPacket::CONFIG_REPLY_OK) {
      showMessage(String("D ID -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
      return;
    }
  }

  showMessage("Success", MSGDELAY, RDisplay::LIGHTGREEN2);
  storeParams();
  needsMenuRebuild_ = true;
#endif
}

void RRemote::updateStatusLED() {
  if(mode_ == MODE_CALIBRATION) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 0, 0, 255);
  } else if(Console::console.isStarted() && XBee::xbee.isStarted() && isStarted() && RInput::input.imuOK() && RInput::input.buttonsOK()) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 0, 255, 0);
  } else if(!XBee::xbee.isStarted() || !RInput::input.imuOK() || !RInput::input.buttonsOK() ) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 255, 0, 0);
  } else {
    RDisplay::display.setLED(RDisplay::LED_STATUS, 255, 255, 0);
  }
}

void RRemote::setIncrRotButtonCB(RInput::Button button, bool left) {
  RInput::Button tempBtn;
#if defined(LEFT_REMOTE)
  if(left) {
    if(params_.config.lIncrRotBtn == button) return;
    RInput::input.setIncrementalRot(button);
    params_.config.lIncrRotBtn = button;
    lRIncrRotMenu_.highlightWidgetsWithTag(button);
    storeParams();
  } else {
    if(params_.config.rIncrRotBtn == button) return;
    tempBtn = RInput::Button(params_.config.rIncrRotBtn);
    params_.config.rIncrRotBtn = button;
    rRIncrRotMenu_.highlightWidgetsWithTag(button);
    if(sendConfigToRightRemote() != RES_OK) {
      params_.config.rIncrRotBtn = tempBtn;
    } else storeParams();
  }
  needsMenuRebuild_ = true;
#else
  Console::console.printfBroadcast("setIncrRotButtonCB() is left remote only!\n");
#endif
}


void RRemote::factoryResetCB(bool left) {
#if defined(LEFT_REMOTE)
  if(left) {
    factoryReset();
  } else {
    Console::console.printfBroadcast("Factory reset right remote!\n");
    bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE, sequenceNumber());
    packet.payload.config.type = bb::ConfigPacket::CONFIG_FACTORY_RESET;
    packet.payload.config.cfgPayload.magic = bb::ConfigPacket::MAGIC;
    XBee::xbee.sendTo(params_.otherRemoteAddress, packet, true);

    params_.otherRemoteAddress = 0;
    storeParams();
    needsMenuRebuild_ = true;
  }
#else
  Console::console.printfBroadcast("setIncrRotButtonCB() is left remote only!\n");
#endif
}

void RRemote::startCalibrationCB(bool left) {
#if defined(LEFT_REMOTE)
  if(left) {
    startCalibration();
  } else {
    ConfigPacket packet;
    ConfigPacket::ConfigReplyType reply;
    packet.type = bb::ConfigPacket::CONFIG_CALIBRATE;
    packet.cfgPayload.magic = bb::ConfigPacket::MAGIC;

    Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
    if(res != RES_OK) {
      showMessage(String("Calib -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
      showMain();
      return;
    } 
    if(reply != ConfigPacket::CONFIG_REPLY_OK) {
      showMessage(String("Calib -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
      showMain();
      return;
    }

    mainVis_.showIndex(1);
    remoteVisR_.crosshair().setMinMax(1024, 4096-1024, 1024, 4096-1024);
    remoteVisR_.crosshair().showMinMaxRect();
    remoteVisR_.crosshair().setMinMaxRectColor(RDisplay::RED);
    showMain();
  }
#else
  Console::console.printfBroadcast("startCalibrationCB() is left remote only!\n");
#endif
}

void RRemote::finishCalibrationCB(bool left) {
#if defined(LEFT_REMOTE)
  Runloop::runloop.excuseOverrun();
  if(left) {
    setBottomTitle(VERSION_STRING);
    showMain();
    RInput::input.setTopRightLongPressCallback([=]{RRemote::remote.showMenu(&mainMenu_);});
    RInput::input.setConfirmShortPressCallback([=]{RRemote::remote.showMenu(&mainMenu_);});
    remoteVisL_.crosshair().showMinMaxRect(false);

    finishCalibration();
  } else {
    remoteVisR_.crosshair().showMinMaxRect(false);
  }
#else
  if(!left) {
    RInput::input.clearCallbacks();
    finishCalibration();
  }
#endif
}

void RRemote::setLeftIsPrimaryCB(bool yesno) {
  bool oldLIP = params_.config.leftIsPrimary;
  if(oldLIP == yesno) return;

  params_.config.leftIsPrimary = yesno;

#if defined(LEFT_REMOTE)
  Result res = sendConfigToRightRemote();
  if(res != RES_OK) {
    params_.config.leftIsPrimary = oldLIP;

    return;
  }
#endif
  
  storeParams();
  needsMenuRebuild_ = true;
}

void RRemote::factoryReset() {
  bb::ConfigStorage::storage.factoryReset();
  showMessage("Please restart");
  int i=0, dir=1;
  while(true) {
    RDisplay::display.setLED(RDisplay::LED_BOTH, 0, 0, i);
    i = i+dir;
    if(dir > 0 && i>=255) dir = -1;
    if(dir < 0 && i<=0) dir = 1;
    delay(10);
  }
}

void RRemote::startCalibration() {
#if defined(LEFT_REMOTE)
  mainVis_.showIndex(0);
  setMainWidget(&mainVis_);

  setTopTitle("Calibrating");
  setBottomTitle("Button=End");
  remoteVisL_.crosshair().setMinMax(RInput::input.minJoyRawH, RInput::input.maxJoyRawH,
                                    RInput::input.minJoyRawV, RInput::input.maxJoyRawV);
  remoteVisL_.crosshair().showMinMaxRect();
  remoteVisL_.crosshair().setMinMaxRectColor(RDisplay::RED);
  RInput::input.setAllCallbacks([=]{finishCalibrationCB(true);});
#else // LEFT_REMOTE
  RInput::input.setAllCallbacks([=]{finishCalibration();});
#endif
  RInput::input.setCalibration(RInput::AxisCalib(), RInput::AxisCalib());
  mode_ = MODE_CALIBRATION;
}

void RRemote::finishCalibration() {
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
      storeParams();

      RDisplay::display.flashLED(RDisplay::LED_BOTH, 2, 250, 250, 0, 150, 0);
    } else {
      // Values out of acceptable bounds - reject calibration.
      RDisplay::display.flashLED(RDisplay::LED_BOTH, 2, 250, 250, 150, 0, 0);
    }
    mode_ = MODE_REGULAR;
}

void RRemote::storeParams() {
  bb::ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t*)&params_);
  bb::ConfigStorage::storage.store();
}

void RRemote::showMessage(const String& msg, unsigned int delayms, uint8_t color) {
  message_.setTitle(msg);
  message_.setForegroundColor(color);
  message_.setFrameColor(color);
  message_.draw();
  if(delayms != 0) delay(delayms);
}

#if defined(LEFT_REMOTE)
Result RRemote::sendConfigToRightRemote() {
  ConfigPacket packet;
  ConfigPacket::ConfigReplyType reply;
  packet.type = bb::ConfigPacket::CONFIG_SET_REMOTE_PARAMS;
  packet.cfgPayload.remoteConfig = params_.config;

  Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
  if(res != RES_OK) {
    showMessage(String("Config -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
    return res;
  } 
  if(reply != ConfigPacket::CONFIG_REPLY_OK) {
    showMessage(String("Config -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
    return RES_SUBSYS_COMM_ERROR;
  }

  showMessage("Success", MSGDELAY, RDisplay::LIGHTGREEN2);
  storeParams();
  return RES_OK;
}
#endif

bb::Result RRemote::fillAndSend() {
#if defined(LEFT_REMOTE)
  Packet packet(PACKET_TYPE_CONTROL, PACKET_SOURCE_LEFT_REMOTE, sequenceNumber());
#else
  Packet packet(PACKET_TYPE_CONTROL, PACKET_SOURCE_RIGHT_REMOTE, sequenceNumber());
#endif
  if(params_.config.leftIsPrimary == true) {
#if defined(LEFT_REMOTE)
    packet.payload.control.primary = true;
#else
    packet.payload.control.primary = false;
#endif
  } else {
#if defined(LEFT_REMOTE)
    packet.payload.control.primary = false;
#else
    packet.payload.control.primary = true;
#endif
  }

  RInput::input.fillControlPacket(packet.payload.control);
  remoteVisL_.visualizeFromPacket(packet.payload.control);

  Result res = RES_OK;
  int r=0, g=0, b=0;

  if(params_.droidAddress != 0 && params_.otherRemoteAddress != 0) {        // both set -- white
    r = 255; g = 255; b = 255;
  } else if(params_.droidAddress != 0) {                                    // only droid set -- blue
    r = 0; g = 0; b = 255;
  } else if(params_.otherRemoteAddress != 0) {
    r = 255; g = 0; b = 255;
  }

#if !defined(LEFT_REMOTE) // right remote sends to left remote
  if(params_.otherRemoteAddress != 0) {
    res = bb::XBee::xbee.sendTo(params_.otherRemoteAddress, packet, false);
    if(res != RES_OK) Console::console.printfBroadcast("%s\n", errorMessage(res));
  }
#endif

  // both remotes send to droid
  if(params_.droidAddress != 0 && mode_ == MODE_REGULAR) {
    //Console::console.printfBroadcast("Sending %d times to 0x%llx\n", params_.config.sendRepeats+1, params_.droidAddress);
    for(int i=0; i<params_.config.sendRepeats+1; i++) {
      res = bb::XBee::xbee.sendTo(params_.droidAddress, packet, false);
    }
    if(res != RES_OK) {
      r = 255; g = 0; b = 0;
    }
  }

  RDisplay::display.setLED(RDisplay::LED_COMM, r, g, b);

  return res;
}

Result RRemote::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
  if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;
  if(words[0] == "running_status") {
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
    startCalibration();

    return RES_OK;
  }

  else if(words[0] == "calibrate_imu") {
    RInput::input.imu().calibrate();
    return RES_OK;
  }

  else if(words[0] == "reset") {
    factoryReset();
    return RES_OK;
  }

  else if(words[0] == "set_droid") {
    if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words[1].length() > 16) return RES_CMD_INVALID_ARGUMENT_COUNT;
    uint64_t addr = 0;
    String addrstr = words[1];
    for(int i=0; i<addrstr.length(); i++) {
      if(i!=0) addr <<= 4;
      if(addrstr[i] >= '0' && addrstr[i] <= '9') addr = addr + (addrstr[i]-'0');
      else if(addrstr[i] >= 'a' && addrstr[i] <= 'f') addr = addr + (addrstr[i]-'a') + 0xa;
      else if(addrstr[i] >= 'A' && addrstr[i] <= 'F') addr = addr + (addrstr[i]-'a') + 0xa;
      else {
        stream->printf("Invalid character '%c' at position %d - must be 0-9a-fA-F.\n");
        return RES_CMD_INVALID_ARGUMENT;
      }
    }
    stream->printf("Setting droid address to 0x%llx.\n", addr);
    params_.droidAddress = addr;
    return RES_OK;
  }

  else if(words[0] == "set_other_remote") {
    if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words[1].length() > 16) return RES_CMD_INVALID_ARGUMENT_COUNT;
    uint64_t addr = 0;
    String addrstr = words[1];
    for(int i=0; i<addrstr.length(); i++) {
      if(i!=0) addr <<= 4;
      if(addrstr[i] >= '0' && addrstr[i] <= '9') addr = addr + (addrstr[i]-'0');
      else if(addrstr[i] >= 'a' && addrstr[i] <= 'f') addr = addr + (addrstr[i]-'a');
      else if(addrstr[i] >= 'A' && addrstr[i] <= 'F') addr = addr + (addrstr[i]-'a');
      else {
        stream->printf("Invalid character '%c' at position %d - must be 0-9a-fA-F.\n");
        return RES_CMD_INVALID_ARGUMENT;
      }
    }
    stream->printf("Setting other remote address to 0x%llx.\n", addr);
    params_.otherRemoteAddress = addr;
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
  return RES_SUBSYS_COMM_ERROR;
#else
  Console::console.printfBroadcast("Got config packet from 0x%llx, type %d\n", srcAddr, packet.type);
  if(srcAddr == params_.otherRemoteAddress || params_.otherRemoteAddress == 0) { // if we're not bound, we accept config packets from anyone
    switch(packet.type) {
    case bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID:
      if(packet.cfgPayload.address != srcAddr) {
        Console::console.printfBroadcast("Error: Pairing packet source 0x%llx and payload 0x%llx don't match\n", srcAddr, packet.cfgPayload.address);
        return RES_SUBSYS_COMM_ERROR;
      }

      Console::console.printfBroadcast("Setting Left Remote ID to 0x%llx.\n", packet.cfgPayload.address);
      params_.otherRemoteAddress = packet.cfgPayload.address;
      storeParams();
      return RES_OK; 

    case bb::ConfigPacket::CONFIG_SET_DROID_ID:
      Console::console.printfBroadcast("Setting Droid ID to 0x%llx.\n", packet.cfgPayload.address);
      params_.droidAddress = packet.cfgPayload.address;
      storeParams();
      return RES_OK; 

    case bb::ConfigPacket::CONFIG_SET_REMOTE_PARAMS:
      Console::console.printfBroadcast("Setting remote params: LPrimary %d LED %d Repeats %d LIncrR %d RIncrR %d LIncrT %d RIncrT %d\n", 
                                       packet.cfgPayload.remoteConfig.leftIsPrimary, packet.cfgPayload.remoteConfig.ledBrightness, 
                                       packet.cfgPayload.remoteConfig.sendRepeats,
                                       packet.cfgPayload.remoteConfig.lIncrRotBtn, packet.cfgPayload.remoteConfig.rIncrRotBtn, 
                                       packet.cfgPayload.remoteConfig.lIncrTransBtn, packet.cfgPayload.remoteConfig.rIncrTransBtn);
      params_.config = packet.cfgPayload.remoteConfig;
      RInput::input.setIncrementalRot(RInput::Button(params_.config.rIncrRotBtn));
      storeParams();
      return RES_OK; 

    case bb::ConfigPacket::CONFIG_FACTORY_RESET:
      if(packet.cfgPayload.magic == bb::ConfigPacket::MAGIC) { // checks out
        factoryReset();
        return RES_OK; // HA! This never returns! NEVER! Hahahahahahaaaaa!
      }
      Console::console.printfBroadcast("Got factory reset packet but Magic 0x%llx doesn't check out!\n", packet.cfgPayload.magic);
      return RES_SUBSYS_COMM_ERROR;

    case bb::ConfigPacket::CONFIG_CALIBRATE:
      if(packet.cfgPayload.magic == bb::ConfigPacket::MAGIC) { // checks out
        startCalibration();
        return RES_OK;
      }
      Console::console.printfBroadcast("Got factory reset packet but Magic 0x%llx doesn't check out!\n", packet.cfgPayload.magic);
      return RES_SUBSYS_COMM_ERROR;

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

String RRemote::statusLine() {
  String str = bb::Subsystem::statusLine() + ", ";
  if(RInput::input.imuOK()) str += "IMU OK, ";
  else str += "IMU error, ";
  if(RInput::input.buttonsOK()) str += "Buttons OK.";
  else str += "Buttons error.";

  return str;
}

void RRemote::printExtendedStatus(ConsoleStream* stream) {
  Runloop::runloop.excuseOverrun();

  float pitch, roll, heading, rax, ray, raz, ax, ay, az;
  RInput::input.imu().getFilteredPRH(pitch, roll, heading);
  RInput::input.imu().getAccelMeasurement(rax, ray, raz);
  RInput::input.imu().getGravCorrectedAccel(ax, ay, az);

  stream->printf("Sequence number: %ld\n", seqnum_);
  stream->printf("Addressing:\n");
  stream->printf("\tOther remote: 0x%llx\n", params_.otherRemoteAddress);
  stream->printf("\tDroid:        0x%llx\n", params_.droidAddress);
  stream->printf("Joystick:\n");
  stream->printf("\tHor: Raw %d\tnormalized %.2f\tcalib [%4d..%4d..%4d]\n", RInput::input.joyRawH, RInput::input.joyH, RInput::input.hCalib.min, RInput::input.hCalib.center, RInput::input.hCalib.max);
  stream->printf("\tVer: Raw %d\tnormalized %.2f\tcalib [%4d..%4d..%4d]\n", RInput::input.joyRawV, RInput::input.joyV, RInput::input.vCalib.min, RInput::input.vCalib.center, RInput::input.vCalib.max);
  stream->printf("IMU:\n");
  stream->printf("\tRotation             Pitch: %.2f Roll: %.2f Heading: %.2f\n", pitch, roll, heading);
  stream->printf("\tRaw Acceleration     X:%f Y:%f Z:%f\n", rax, ray, raz);
  stream->printf("\tGrav-corrected accel X:%f Y:%f Z:%f\n", ax, ay, az);
  stream->printf("Buttons: Pinky(%d):%c Index(%d):%c Joy(%d):%c Left(%d):%c Right(%d):%c Confirm(%d):%c TopLeft(%d):%c TopRight(%d):%c\n",
                 RInput::BUTTON_PINKY, RInput::input.buttons[RInput::BUTTON_PINKY] ? 'X' : '_',
                 RInput::BUTTON_INDEX, RInput::input.buttons[RInput::BUTTON_INDEX] ? 'X' : '_',
                 RInput::BUTTON_JOY, RInput::input.buttons[RInput::BUTTON_JOY] ? 'X' : '_',
                 RInput::BUTTON_LEFT, RInput::input.buttons[RInput::BUTTON_LEFT] ? 'X' : '_',
                 RInput::BUTTON_RIGHT, RInput::input.buttons[RInput::BUTTON_RIGHT] ? 'X' : '_',
                 RInput::BUTTON_CONFIRM, RInput::input.buttons[RInput::BUTTON_CONFIRM] ? 'X' : '_',
                 RInput::BUTTON_TOP_LEFT, RInput::input.buttons[RInput::BUTTON_TOP_LEFT] ? 'X' : '_',
                 RInput::BUTTON_TOP_RIGHT, RInput::input.buttons[RInput::BUTTON_TOP_RIGHT] ? 'X' : '_');
  stream->printf("Potentiometer 1: %.1f Potentiometer 2: %.1f\n", RInput::input.pot1, RInput::input.pot2);
  stream->printf("Battery: %.1f\n", RInput::input.battery);
}

void RRemote::printExtendedStatusLine(ConsoleStream *stream) {
  const unsigned int bufsize = 255;
  char buf[bufsize];
  memset(buf, 0, bufsize);

  float pitch, roll, heading, rax, ray, raz, ax, ay, az;
  RInput::input.imu().getFilteredPRH(pitch, roll, heading);
  RInput::input.imu().getAccelMeasurement(rax, ray, raz);
  RInput::input.imu().getGravCorrectedAccel(ax, ay, az);

#if 0
  snprintf(buf, bufsize, "S%ld AX%f AY%f AZ%f\n", seqnum_, ax, ay, az);
#else
  snprintf(buf, bufsize, "S%ld H%d [%d..%d..%d] %.2f V%d [%d..%d..%d] %.2f P%.1f R%.1f H%.1f AX%.2f AY%.2f AZ%.2f P1%.1f P2%.1f Batt%.1f B%c%c%c%c%c%c%c%c",
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
  if(stream)
    stream->printf("%s\n", buf);
  else 
    Console::console.printfBroadcast("%s\n", buf);
}

void RRemote::runTestsuite() {
  RInput::input.testMatrix();
}
