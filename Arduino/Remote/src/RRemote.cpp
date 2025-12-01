#include <cstdio>
#include "RRemote.h"
#include "RUI.h"

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

  params_.droidAddress = {0,0};
  params_.otherRemoteAddress = {0,0};
  params_.config.leftIsPrimary = true;
  params_.config.ledBrightness = 7;
  params_.config.sendRepeats = 1;
  params_.config.lIncrRotBtn = RInput::BUTTON_4;
  params_.config.rIncrRotBtn = RInput::BUTTON_4;
  params_.config.lIncrTransBtn = RInput::BUTTON_NONE;
  params_.config.rIncrTransBtn = RInput::BUTTON_NONE;
  params_.config.deadbandPercent = 8;
}

Result RRemote::initialize() { 
  addParameter("led_brightness", "LED Brightness", ledBrightness_, 8);
  addParameter("deadband", "Joystick deadband in percent", deadbandPercent_, 15);
  addParameter("send_repeats", "Send repeats for control packets (0 = send only once)", sendRepeats_, 15);

  paramsHandle_ = ConfigStorage::storage.reserveBlock("remote", sizeof(params_), (uint8_t*)&params_);
	if(ConfigStorage::storage.blockIsValid(paramsHandle_)) {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is valid.\n", paramsHandle_);
    ConfigStorage::storage.readBlock(paramsHandle_);
    Console::console.printfBroadcast("Other Remote Address: 0x%lx:%lx\n", params_.otherRemoteAddress.addrHi, params_.otherRemoteAddress.addrLo);
    Console::console.printfBroadcast("Droid address: 0x%lx:%lx\n", params_.droidAddress.addrHi, params_.droidAddress.addrLo);
  } else {
    Console::console.printfBroadcast("Remote: Storage block 0x%x is invalid, using initialized parameters.\n", paramsHandle_);
    ConfigStorage::storage.writeBlock(paramsHandle_);
  }

  RInput::input.setCalibration(params_.hCalib, params_.vCalib);
  if(isLeftRemote) RInput::input.setIncrementalRot(RInput::Button(params_.config.lIncrRotBtn));
  else RInput::input.setIncrementalRot(RInput::Button(params_.config.rIncrRotBtn));
  deadbandPercent_ = params_.config.deadbandPercent;
  ledBrightness_ = params_.config.ledBrightness;
  sendRepeats_ = params_.config.sendRepeats;
  RInput::input.setDeadbandPercent(params_.config.deadbandPercent);
  RDisplay::display.setLEDBrightness(ledBrightness_<<2);

  if(isLeftRemote) {
    RUI::ui.setSeqnumState(PACKET_SOURCE_RIGHT_REMOTE, params_.otherRemoteAddress.isZero() != false);
    RUI::ui.setSeqnumState(PACKET_SOURCE_DROID, params_.droidAddress.isZero() != false);
    RUI::ui.setSeqnumState(PACKET_SOURCE_LEFT_REMOTE, true);
    setPacketSource(PACKET_SOURCE_LEFT_REMOTE);
  } else {
    setPacketSource(PACKET_SOURCE_RIGHT_REMOTE);
  }

  return Subsystem::initialize();
}

Result RRemote::start(ConsoleStream *stream) {
  (void)stream;
  runningStatus_ = false;
  operationStatus_ = RES_OK;

  bb::printf("Starting RInput\n");
  if(RInput::input.begin() == false) {
    LOG(LOG_FATAL, "Error initializing RInput\n");
  }

  if(isLeftRemote) RUI::ui.start();

  started_ = true;

  return RES_OK;
}

Result RRemote::stop(ConsoleStream *stream) {
  (void)stream;
  RDisplay::display.setLED(RDisplay::LED_BOTH, RDisplay::LED_YELLOW);
  operationStatus_ = RES_OK;
  started_ = false;
  
  return RES_OK;
}

Result RRemote::step() {
  if(!started_) return RES_SUBSYS_NOT_STARTED;

  RInput::input.update();
  if(isLeftRemote) {
    if(millis()-lastDroidMs_ > 500) RUI::ui.setNoComm(PACKET_SOURCE_DROID, true);
    if(millis()-lastRightMs_ > 500) RUI::ui.setNoComm(PACKET_SOURCE_RIGHT_REMOTE, true);
  }

  if((bb::Runloop::runloop.getSequenceNumber() % 4) == 0) {
    if(runningStatus_) {
      printExtendedStatusLine();
    }
    fillAndSend();
  } else if((bb::Runloop::runloop.getSequenceNumber() % 4) == 1) {
    updateStatusLED();
    if(isLeftRemote) {
      if(RInput::input.secondsSinceLastMotion() > 10.0) RUI::ui.drawScreensaver();
      else RUI::ui.drawGUI();
    }
  } else {
    RDisplay::display.setLED(RDisplay::LED_COMM, RDisplay::LED_OFF);
  }

  if((bb::Runloop::runloop.getSequenceNumber() % 10) == 0) {
    if(RInput::input.secondsSinceLastMotion() > 10 || params_.config.ledBrightness <= 2) {
      RDisplay::display.setLEDBrightness(1);
    } else {
      RDisplay::display.setLEDBrightness(params_.config.ledBrightness << 2);
    }
  }

  return RES_OK;
}

void RRemote::parameterChangedCallback(const String& name) {
  if(name == "deadband") {
    params_.config.deadbandPercent = deadbandPercent_;
    RInput::input.setDeadbandPercent(deadbandPercent_);
    Console::console.printfBroadcast("Set deadband percent to %d\n", deadbandPercent_);
  } else if (name == "led_brightness") {
    params_.config.ledBrightness = ledBrightness_;
    RDisplay::display.setLEDBrightness(ledBrightness_<<2);
    Console::console.printfBroadcast("Set LED Brightness to %d\n", ledBrightness_);
  } else if(name == "send_repeats") {
    params_.config.sendRepeats = sendRepeats_;
    Console::console.printfBroadcast("Set send repeats to %d\n", sendRepeats_);
  }
}

void RRemote::selectDroid(const HWAddress& droid) {
  if(!isLeftRemote) {
    LOG(LOG_ERROR, "BUG: selectDroid() called in right remote, only valid in left remote\n");
    return;
  }

  Console::console.printfBroadcast("Selecting droid 0x%lx:%lx\n", droid.addrHi, droid.addrLo);
  Runloop::runloop.excuseOverrun();
  
  // Select droid in left remote
  params_.droidAddress = droid;
  
  // Tell the droid that we're its left remote
  ConfigPacket packet;
  ConfigPacket::ConfigReplyType reply;
  bool showSuccess = true;

  packet.type = bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID;
  packet.cfgPayload.address = XBee::xbee.hwAddress();
  Result res = XBee::xbee.sendConfigPacket(params_.droidAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
  if(res != RES_OK) {
    RUI::ui.showMessage(String("L ID -> D:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
    return;
  } else if(reply != ConfigPacket::CONFIG_REPLY_OK) {
    RUI::ui.showMessage(String("L ID -> D:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
    return;
  }

  // Let the other remote know about the droid
  if(params_.otherRemoteAddress.addrHi != 0 || params_.otherRemoteAddress.addrLo != 0) {
    packet.type = bb::ConfigPacket::CONFIG_SET_DROID_ID;
    packet.cfgPayload.address = params_.droidAddress;
    Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
    if(res != RES_OK) {
      RUI::ui.showMessage(String("D ID -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
      showSuccess = false;
    } else if(reply != ConfigPacket::CONFIG_REPLY_OK) {
      RUI::ui.showMessage(String("D ID -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
      showSuccess = false;
    }
  } 
        
  if(showSuccess) RUI::ui.showMessage("Success", MSGDELAY, RDisplay::LIGHTGREEN2);
  RUI::ui.setSeqnumState(PACKET_SOURCE_DROID, true);
  ConfigStorage::storage.writeBlock(paramsHandle_);
  RUI::ui.setNeedsMenuRebuild();
}

void RRemote::selectRightRemote(const HWAddress& address) {
  if(!isLeftRemote) {
    LOG(LOG_ERROR, "BUG: selectRightRemote() called in right remote, only valid in left remote\n");
    return;
  }

  Console::console.printfBroadcast("Selecting right remote 0x%lx:%lx\n", address.addrHi, address.addrLo);
  Runloop::runloop.excuseOverrun();

  // Select droid in left remote
  params_.otherRemoteAddress = address;

  ConfigPacket packet;
  ConfigPacket::ConfigReplyType reply;
  bool showSuccess = true;

  // Set left remote address to right remote
  packet.type = bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID;
  packet.cfgPayload.address = XBee::xbee.hwAddress();
  Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
  if(res != RES_OK) {
    RUI::ui.showMessage(String("L ID -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
    return;
  }
  if(reply != ConfigPacket::CONFIG_REPLY_OK) {
    RUI::ui.showMessage(String("L ID -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
    return;
  }

  // Set droid address to right remote
  if(!params_.droidAddress.isZero()) {
    packet.type = bb::ConfigPacket::CONFIG_SET_DROID_ID;
    packet.cfgPayload.address = params_.droidAddress;
    Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
    if(res != RES_OK) {
      RUI::ui.showMessage(String("D ID -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
      showSuccess = false;
    }
    if(reply != ConfigPacket::CONFIG_REPLY_OK) {
      RUI::ui.showMessage(String("D ID -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
      showSuccess = false;
    }
  }

  if(showSuccess) RUI::ui.showMessage("Success", MSGDELAY, RDisplay::LIGHTGREEN2);
  ConfigStorage::storage.writeBlock(paramsHandle_);

  RUI::ui.setSeqnumState(PACKET_SOURCE_RIGHT_REMOTE, true);
  RUI::ui.setNeedsMenuRebuild();
}

void RRemote::updateStatusLED() {
  if(mode_ == MODE_CALIBRATION) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, RDisplay::LED_BLUE);
  } else if(Console::console.isStarted() && XBee::xbee.isStarted() && isStarted() && RInput::input.imuOK() && RInput::input.mcpOK()) {
    if(RInput::input.joyAtZero())
      RDisplay::display.setLED(RDisplay::LED_STATUS, RDisplay::LED_WHITE);
    else
      RDisplay::display.setLED(RDisplay::LED_STATUS, RDisplay::LED_GREEN);
  } else if(!XBee::xbee.isStarted() || !RInput::input.imuOK() || !RInput::input.mcpOK() ) {
    RDisplay::display.setLED(RDisplay::LED_STATUS, RDisplay::LED_RED);
  } else {
    RDisplay::display.setLED(RDisplay::LED_STATUS, RDisplay::LED_YELLOW);
  }
}

void RRemote::factoryReset() {
  bb::ConfigStorage::storage.factoryReset();
  if(isLeftRemote) RUI::ui.showMessage("Please restart");
  int i=0, dir=1;
  while(true) {
    RDisplay::display.setLED(RDisplay::LED_BOTH, 0, 0, i);
    i = i+dir;
    if(dir > 0 && i>=255) dir = -1;
    if(dir < 0 && i<=0) dir = 1;
    delay(10);
  }
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
    
    ConfigStorage::storage.writeBlock(paramsHandle_);

    RDisplay::display.flashLED(RDisplay::LED_BOTH, 2, 250, 250, 0, 150, 0);
  } else {
    // Values out of acceptable bounds - reject calibration.
    RDisplay::display.flashLED(RDisplay::LED_BOTH, 2, 250, 250, 150, 0, 0);
  }
  mode_ = MODE_REGULAR;

  RInput::input.clearCallbacks();
  if(isLeftRemote) {
    Runloop::runloop.excuseOverrun();
    RUI::ui.hideCalibration(PACKET_SOURCE_LEFT_REMOTE);
    RUI::ui.showMain();
  }
}

void RRemote::setLEDBrightness(uint8_t brt) {
  if(brt > 7) brt = 7;
  if(brt == params_.config.ledBrightness) return;
  params_.config.ledBrightness = ledBrightness_ = brt;
  RDisplay::display.setLEDBrightness(brt << 2);
  if(isLeftRemote) sendConfigToRightRemote();
  storeParams();
}

void RRemote::setJoyDeadband(uint8_t db) {
  if(db > 15) db = 15;
  if(db == params_.config.deadbandPercent) return;
  params_.config.deadbandPercent = deadbandPercent_ = db;
  RInput::input.setDeadbandPercent(db);
  if(isLeftRemote) sendConfigToRightRemote();
  storeParams();
}

void RRemote::setSendRepeats(uint8_t sr) {
  if(sr > 7) sr = 7;
  if(sr == params_.config.sendRepeats) return;
  params_.config.sendRepeats = sendRepeats_ = sr;
  if(isLeftRemote) sendConfigToRightRemote();
  storeParams();
}

Result RRemote::sendConfigToRightRemote() {
  if(!isLeftRemote) {
    LOG(LOG_ERROR, "BUG: sendConfigToRightRemote() called in right remote, only valid in left remote\n");
    return RES_SUBSYS_COMM_ERROR;
  }
  
  if(params_.otherRemoteAddress.isZero()) {
    LOG(LOG_WARN, "Trying to send to right remote but address is 0\n");
    return RES_CONFIG_INVALID_HANDLE;
  }

  ConfigPacket packet;
  ConfigPacket::ConfigReplyType reply;
  packet.type = bb::ConfigPacket::CONFIG_SET_REMOTE_PARAMS;
  packet.cfgPayload.remoteConfig = params_.config;

  Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
  if(res != RES_OK) {
    RUI::ui.showMessage(String("Config -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
    return res;
  } 
  if(reply != ConfigPacket::CONFIG_REPLY_OK) {
    RUI::ui.showMessage(String("Config -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
    return RES_SUBSYS_COMM_ERROR;
  }

  RUI::ui.showMessage("Success", MSGDELAY, RDisplay::LIGHTGREEN2);
  ConfigStorage::storage.writeBlock(paramsHandle_);
  return RES_OK;
}

bb::Result RRemote::fillAndSend() {
  Packet packet(PACKET_TYPE_CONTROL, isLeftRemote ? PACKET_SOURCE_LEFT_REMOTE : PACKET_SOURCE_RIGHT_REMOTE, sequenceNumber());

  packet.payload.control.primary = isPrimary();

  RInput::input.fillControlPacket(packet.payload.control);
  packet.seqnum = (seqnum_++)%8;
  if(isLeftRemote) RUI::ui.visualizeFromControlPacket(PACKET_SOURCE_LEFT_REMOTE, packet.seqnum, packet.payload.control);

  Result res = RES_OK;
  int r=0, g=0, b=0;

  if((params_.droidAddress.addrLo != 0 || params_.droidAddress.addrHi != 0) && 
      (params_.otherRemoteAddress.addrHi != 0 || params_.otherRemoteAddress.addrLo != 0)) {        // both set -- white
    r = 255; g = 255; b = 255;
  } else if(params_.droidAddress.addrHi != 0 || params_.droidAddress.addrLo != 0) {                                    // only droid set -- blue
    r = 0; g = 0; b = 255;
  } else if(params_.otherRemoteAddress.addrHi != 0 || params_.otherRemoteAddress.addrLo != 0) {
    r = 255; g = 0; b = 255;
  }

  if(!isLeftRemote && !params_.otherRemoteAddress.isZero()) { // right remote sends to left remote
    res = bb::XBee::xbee.sendTo(params_.otherRemoteAddress, packet, false);

    if(res != RES_OK) Console::console.printfBroadcast("%s\n", errorMessage(res));
  }

  // both remotes send to droid (unless we're calibrating)
  if(!params_.droidAddress.isZero() && mode_ == MODE_REGULAR) {
    for(int i=0; i<params_.config.sendRepeats+1; i++) {
      delayMicroseconds(random(100));
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
    params_.droidAddress = {uint32_t(addr>>32), uint32_t(addr&0xffffffff)};
    stream->printf("Setting droid address to 0x%lx:%lx.\n", params_.droidAddress.addrHi, params_.droidAddress.addrLo);
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
    params_.otherRemoteAddress = {uint32_t(addr>>32), uint32_t(addr&0xffffffff)};
    stream->printf("Setting other remote address to 0x%lx:%lx.\n", params_.otherRemoteAddress.addrHi, params_.otherRemoteAddress.addrLo);
    return RES_OK;
  }

  else if(words[0] == "testsuite") {
    runTestsuite();
    return RES_OK;
  }

  return Subsystem::handleConsoleCommand(words, stream);
} 

Result RRemote::incomingControlPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const ControlPacket& packet) {
  if(!isLeftRemote) {
    LOG(LOG_ERROR, "incomingControlPacket() called in right remote, only defined in left remote\n");
    return RES_SUBSYS_COMM_ERROR;
  }

  if(source != PACKET_SOURCE_RIGHT_REMOTE) {
    LOG(LOG_ERROR, "Unknown address 0x%lx:%lx (type %d) sent Control packet to left remote\n", srcAddr.addrHi, srcAddr.addrLo, source);
    return RES_SUBSYS_COMM_ERROR;
  }
 
  // FIXME Must check for address too but we're getting 16bit addressed packets here?!
  if(packet.button5 == true ||
      packet.button6 == true ||
      packet.button7 == true) {
      RUI::ui.hideCalibration(PACKET_SOURCE_RIGHT_REMOTE);
  }
  
  RUI::ui.visualizeFromControlPacket(source, seqnum, packet);
  
  lastRightMs_ = millis();

  return RES_OK;    
}

Result RRemote::incomingStatePacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const StatePacket& packet) {
  if(!isLeftRemote) {
    LOG(LOG_ERROR, "incomingStatePacket() called in right remote, only defined in left remote\n");
    return RES_SUBSYS_COMM_ERROR;
  }

  if(source != PACKET_SOURCE_DROID) {
    LOG(LOG_ERROR, "Unknown address 0x%lx:%lx (type %d) sent State packet to left remote\n", srcAddr.addrHi, srcAddr.addrLo, source);
    return RES_SUBSYS_COMM_ERROR;
  }

  RUI::ui.visualizeFromStatePacket(source, seqnum, packet);

  lastDroidMs_ = millis();

  return RES_OK;    
}

Result RRemote::incomingPairingPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, PairingPacket& packet) {
  if(isLeftRemote) {
    LOG(LOG_ERROR, "Address 0x%lx:%lx sent Config packet to left remote - should never happen\n", srcAddr.addrHi, srcAddr.addrLo);
    return RES_SUBSYS_COMM_ERROR;
  }

  Console::console.printfBroadcast("Got pairing packet from 0x%lx:%lx, type %d\n", srcAddr, packet.type);

  switch(packet.type) {
  default:
    LOG(LOG_ERROR, "Unknown config packet type 0x%x.\n", packet.type);
    return RES_SUBSYS_COMM_ERROR;
  }
}

Result RRemote::incomingConfigPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, ConfigPacket& packet) {
  if(isLeftRemote) {
    LOG(LOG_ERROR, "Address 0x%lx:%lx sent Config packet to left remote - should never happen\n", srcAddr.addrHi, srcAddr.addrLo);
    return RES_SUBSYS_COMM_ERROR;
  }

  Console::console.printfBroadcast("Got config packet from 0x%llx, type %d\n", srcAddr, packet.type);
  // if we're not bound, we accept config packets from anyone; if we are, raise an error
  if(!params_.otherRemoteAddress.isZero() && srcAddr != params_.otherRemoteAddress) {
    LOG(LOG_ERROR, "Config packet from unknown source 0x%lx:%lx\n", srcAddr.addrHi, srcAddr.addrLo);
    return RES_SUBSYS_COMM_ERROR;
  }

  switch(packet.type) {
  case bb::ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID:
    if(packet.cfgPayload.address != srcAddr) {
      Console::console.printfBroadcast("Error: Pairing packet source 0x%lx:%lx and payload 0x%lx:%lx don't match\n", 
        srcAddr.addrHi, srcAddr.addrLo, packet.cfgPayload.address.addrHi, packet.cfgPayload.address.addrLo);
      return RES_SUBSYS_COMM_ERROR;
    }

    Console::console.printfBroadcast("Setting Left Remote ID to 0x%lx:%lx.\n", packet.cfgPayload.address.addrHi, packet.cfgPayload.address.addrLo);
    params_.otherRemoteAddress = packet.cfgPayload.address;
    ConfigStorage::storage.writeBlock(paramsHandle_);
    return RES_OK; 

  case bb::ConfigPacket::CONFIG_SET_DROID_ID:
    Console::console.printfBroadcast("Setting Droid ID to 0x%lx:%lx.\n", packet.cfgPayload.address.addrHi, packet.cfgPayload.address.addrLo);
    params_.droidAddress = packet.cfgPayload.address;
    ConfigStorage::storage.writeBlock(paramsHandle_);
    return RES_OK; 

  case bb::ConfigPacket::CONFIG_SET_REMOTE_PARAMS:
    Console::console.printfBroadcast("Setting remote params: LPrimary %d LED %d Repeats %d LIncrR %d RIncrR %d LIncrT %d RIncrT %d\n", 
                                      packet.cfgPayload.remoteConfig.leftIsPrimary, packet.cfgPayload.remoteConfig.ledBrightness, 
                                      packet.cfgPayload.remoteConfig.sendRepeats,
                                      packet.cfgPayload.remoteConfig.lIncrRotBtn, packet.cfgPayload.remoteConfig.rIncrRotBtn, 
                                      packet.cfgPayload.remoteConfig.lIncrTransBtn, packet.cfgPayload.remoteConfig.rIncrTransBtn);
    params_.config = packet.cfgPayload.remoteConfig;
    RInput::input.setIncrementalRot(RInput::Button(params_.config.rIncrRotBtn));
    ConfigStorage::storage.writeBlock(paramsHandle_);
    return RES_OK; 

  case bb::ConfigPacket::CONFIG_FACTORY_RESET:
    if(packet.cfgPayload.magic == bb::ConfigPacket::MAGIC) { // checks out
      factoryReset();
      return RES_OK; // HA! This never returns! NEVER! Hahahahahahaaaaa!
    }
    LOG(LOG_ERROR, "Got factory reset packet but Magic 0x%llx doesn't check out!\n", packet.cfgPayload.magic);
    return RES_SUBSYS_COMM_ERROR;

  case bb::ConfigPacket::CONFIG_CALIBRATE:
    if(packet.cfgPayload.magic == bb::ConfigPacket::MAGIC) { // checks out
      startCalibration();
      return RES_OK;
    }
    LOG(LOG_ERROR, "Got factory reset packet but Magic 0x%llx doesn't check out!\n", packet.cfgPayload.magic);
    return RES_SUBSYS_COMM_ERROR;

  default:
    LOG(LOG_ERROR, "Unknown config packet type 0x%x.\n", packet.type);
    return RES_SUBSYS_COMM_ERROR;
  }
}

String RRemote::statusLine() {
  String str = bb::Subsystem::statusLine() + ", ";
  if(RInput::input.imuOK()) str += "IMU OK, ";
  else str += "IMU error, ";
  if(RInput::input.mcpOK()) str += "Buttons OK.";
  else str += "Buttons error.";

  return str;
}

void RRemote::printExtendedStatus(ConsoleStream* stream) {
  Runloop::runloop.excuseOverrun();

  stream->printf(isLeftRemote ? "Left remote status\n" : "Right remote status\n");
  stream->printf("Software version: " VERSION_STRING "\n");
  stream->printf("Sequence number: %ld\n", seqnum_);
  stream->printf("Primary remote: %s\n", isPrimary() ? "Yes" : "No");
  stream->printf("Addressing:\n");
  stream->printf("\tThis remote:  0x%lx:%lx\n", XBee::xbee.hwAddress().addrHi, XBee::xbee.hwAddress().addrLo);
  stream->printf("\tOther remote: 0x%lx:%lx\n", params_.otherRemoteAddress.addrHi, params_.otherRemoteAddress.addrLo);
  stream->printf("\tDroid:        0x%lx:%lx\n", params_.droidAddress.addrHi, params_.droidAddress.addrLo);
  stream->printf("Joystick:\n");
  stream->printf("\tHor: Raw %d\tnormalized %.2f\tcalib [%4d..%4d..%4d]\n", RInput::input.joyRawH, RInput::input.joyH, RInput::input.hCalib.min, RInput::input.hCalib.center, RInput::input.hCalib.max);
  stream->printf("\tVer: Raw %d\tnormalized %.2f\tcalib [%4d..%4d..%4d]\n", RInput::input.joyRawV, RInput::input.joyV, RInput::input.vCalib.min, RInput::input.vCalib.center, RInput::input.vCalib.max);

  if(RInput::input.imuOK()) {
    float pitch, roll, heading, rax, ray, raz, ax, ay, az;
    RInput::input.imu().getFilteredPRH(pitch, roll, heading);
    RInput::input.imu().getAccelMeasurement(rax, ray, raz);
    RInput::input.imu().getGravCorrectedAccel(ax, ay, az);
    stream->printf("IMU: OK\n");
    stream->printf("\tRotation             Pitch: %.2f Roll: %.2f Heading: %.2f\n", pitch, roll, heading);
    stream->printf("\tRaw Acceleration     X:%f Y:%f Z:%f\n", rax, ray, raz);
    stream->printf("\tGrav-corrected accel X:%f Y:%f Z:%f\n", ax, ay, az);
  } else {
    stream->printf("IMU: Error\n");
  }

  if(RInput::input.mcpOK()) {      
    stream->printf("Buttons: 1:%c 2:%c 3:%c 4:%c Joy:%c Confirm:%c Left:%c Right:%c\n",
                  RInput::input.buttons[RInput::BUTTON_1] ? 'X' : '_',
                  RInput::input.buttons[RInput::BUTTON_2] ? 'X' : '_',
                  RInput::input.buttons[RInput::BUTTON_3] ? 'X' : '_',
                  RInput::input.buttons[RInput::BUTTON_4] ? 'X' : '_',
                  RInput::input.buttons[RInput::BUTTON_JOY] ? 'X' : '_',
                  RInput::input.buttons[RInput::BUTTON_CONFIRM] ? 'X' : '_',
                  RInput::input.buttons[RInput::BUTTON_LEFT] ? 'X' : '_',
                  RInput::input.buttons[RInput::BUTTON_RIGHT] ? 'X' : '_');
  } else {
    stream->printf("Buttons: Error\n");
  }

  stream->printf("Potentiometer 1: %d %.2f\nPotentiometer 2: %d %.2f\n", RInput::input.pot1Raw, RInput::input.pot1, RInput::input.pot2Raw, RInput::input.pot2);
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

  snprintf(buf, bufsize, "S%ld H%d [%d..%d..%d] %f V%d [%d..%d..%d] %f P%.1f R%.1f H%.1f AX%.2f AY%.2f AZ%.2f P1%.1f P2%.1f Batt%.1f B%c%c%c%c%c%c%c%c",
    seqnum_,
    RInput::input.joyRawH, RInput::input.hCalib.min, RInput::input.hCalib.center, RInput::input.hCalib.max, RInput::input.joyH,
    RInput::input.joyRawV, RInput::input.vCalib.min, RInput::input.vCalib.center, RInput::input.vCalib.max, RInput::input.joyV,
    pitch, roll, heading,
    ax, ay, az,
    RInput::input.pot1, RInput::input.pot2,
    RInput::input.battery,
    RInput::input.buttons[RInput::BUTTON_1] ? '_' : 'X',
    RInput::input.buttons[RInput::BUTTON_2] ? '_' : 'X',
    RInput::input.buttons[RInput::BUTTON_3] ? '_' : 'X',
    RInput::input.buttons[RInput::BUTTON_4] ? '_' : 'X',
    RInput::input.buttons[RInput::BUTTON_JOY] ? '_' : 'X',
    RInput::input.buttons[RInput::BUTTON_CONFIRM] ? '_' : 'X',
    RInput::input.buttons[RInput::BUTTON_LEFT] ? '_' : 'X',
    RInput::input.buttons[RInput::BUTTON_RIGHT] ? '_' : 'X');
  if(stream)
    stream->printf("%s\n", buf);
  else 
     bb::printf("%s\n", buf);
}

void RRemote::runTestsuite() {
  RInput::input.testMatrix();
}

RInput::Button RRemote::incrRotButton(PacketSource source) {
  if(source == PACKET_SOURCE_LEFT_REMOTE)
    return RInput::Button(params_.config.lIncrRotBtn);
  else if(source == PACKET_SOURCE_RIGHT_REMOTE)
    return RInput::Button(params_.config.rIncrRotBtn);
  LOG(LOG_ERROR, "Querying incrRotButton for source type %d is not defined\n", source);
  return RInput::BUTTON_NONE;
}

void RRemote::setIncrRotButton(PacketSource source, RInput::Button btn) {
  if(source == PACKET_SOURCE_LEFT_REMOTE)
    params_.config.lIncrRotBtn = btn;
  else if(source == PACKET_SOURCE_RIGHT_REMOTE)
    params_.config.rIncrRotBtn = btn;
  else
    LOG(LOG_ERROR, "Setting incrRotButton for source type %d is not defined\n", source);
}

void RRemote::sendFactoryReset() {
  if(!isLeftRemote) {
    LOG(LOG_ERROR, "sendFactoryReset() called in right remote, only defined for left remote!\n");
    return;
  }

  Console::console.printfBroadcast("Factory reset right remote!\n");
  bb::Packet packet(bb::PACKET_TYPE_CONFIG, bb::PACKET_SOURCE_LEFT_REMOTE, sequenceNumber());
  packet.payload.config.type = bb::ConfigPacket::CONFIG_FACTORY_RESET;
  packet.payload.config.cfgPayload.magic = bb::ConfigPacket::MAGIC;
  XBee::xbee.sendTo(params_.otherRemoteAddress, packet, true);

  params_.otherRemoteAddress = {0, 0};
  ConfigStorage::storage.writeBlock(paramsHandle_);
  RUI::ui.setNoComm(PACKET_SOURCE_RIGHT_REMOTE, true);
  RUI::ui.setNeedsMenuRebuild();
}

void RRemote::startCalibration() {
  if(isLeftRemote) {
    RUI::ui.showCalibration(PACKET_SOURCE_LEFT_REMOTE);
  }
  RInput::input.setAllCallbacks([=]{finishCalibration();});
  RInput::input.setCalibration(RInput::AxisCalib(), RInput::AxisCalib());
  mode_ = MODE_CALIBRATION;
}

void RRemote::sendStartCalibration() {
  if(!isLeftRemote) {
    LOG(LOG_ERROR, "sendStartCalibration() called in right remote, only defined in left remote\n");
    return;
  }

  ConfigPacket packet;
  ConfigPacket::ConfigReplyType reply;
  packet.type = bb::ConfigPacket::CONFIG_CALIBRATE;
  packet.cfgPayload.magic = bb::ConfigPacket::MAGIC;

  Result res = XBee::xbee.sendConfigPacket(params_.otherRemoteAddress, PACKET_SOURCE_LEFT_REMOTE, packet, reply, sequenceNumber(), true);
  if(res != RES_OK) {
    RUI::ui.showMessage(String("Calib -> R:\n") + bb::errorMessage(res), MSGDELAY, RDisplay::LIGHTRED2);
    RUI::ui.showMain();
    return;
  } 
  if(reply != ConfigPacket::CONFIG_REPLY_OK) {
    RUI::ui.showMessage(String("Calib -> R:\nError ") + int(reply), MSGDELAY, RDisplay::LIGHTRED2);
    RUI::ui.showMain();
    return;
  }

  RUI::ui.showCalibration(PACKET_SOURCE_RIGHT_REMOTE);
}

void RRemote::setLeftIsPrimary(bool yesno) {
  bool oldLIP = params_.config.leftIsPrimary;
  if(oldLIP == yesno) return;

  params_.config.leftIsPrimary = yesno;

  if(isLeftRemote) {
    Result res = sendConfigToRightRemote();
    if(res != RES_OK) {
      params_.config.leftIsPrimary = oldLIP;
  
      return;
    }
  }
  
  storeParams();
  if(isLeftRemote) RUI::ui.setNeedsMenuRebuild();
}
