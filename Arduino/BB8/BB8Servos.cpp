#include <Wire.h>
#include "actuator.h"
#include "BB8Servos.h"
#include "BB8Config.h"

static const uint8_t LED_BIT = 0x01;
static const uint8_t SWITCH_BIT = 0x08;
static const uint8_t ADDR = 0x38;
static const uint8_t INPUT_REG = 0x00, OUTPUT_REG = 0x01, INVERT_REG = 0x02, CONFIG_REG = 0x03;

BB8ServoPower BB8ServoPower::power;

Result BB8ServoPower::initialize() {
  Wire.begin();

  uint8_t regval;

  // clear LED and switch bit in output register
  if (!requestFrom(ADDR, OUTPUT_REG, regval)) return RES_SUBSYS_COMM_ERROR;
  regval &= ~(LED_BIT | SWITCH_BIT);
  writeTo(ADDR, OUTPUT_REG, regval);

  // set LED and switch as output
  if (!requestFrom(ADDR, CONFIG_REG, regval)) return RES_SUBSYS_COMM_ERROR;
  regval &= ~(LED_BIT | SWITCH_BIT);
  writeTo(ADDR, CONFIG_REG, regval);

  return switchOnOff(false);
}

bool BB8ServoPower::isOn() {
  uint8_t regval;
  if (!requestFrom(ADDR, OUTPUT_REG, regval)) return false;
  return (regval & SWITCH_BIT) != 0;
}

Result BB8ServoPower::switchOnOff(bool onoff) {
  uint8_t regval;
  if (!requestFrom(ADDR, OUTPUT_REG, regval)) return RES_SUBSYS_COMM_ERROR;
  if (onoff) {
    regval |= SWITCH_BIT;
    regval &= ~LED_BIT;
  } else {
    regval &= ~SWITCH_BIT;
    regval |= LED_BIT;
  }
  writeTo(ADDR, OUTPUT_REG, regval);
  return RES_OK;
}

bool BB8ServoPower::requestFrom(uint8_t addr, uint8_t reg, uint8_t &byte) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 1, true);
  if (!Wire.available()) return false;
  byte = Wire.read();
  return true;
}

void BB8ServoPower::writeTo(uint8_t addr, uint8_t reg, uint8_t byte) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(byte);
  Wire.endTransmission();
}


BB8Servos BB8Servos::servos;
static const unsigned int bpsList[] = { 57600, 115200, 1000000 };
static const unsigned int numBps = 3;
static const unsigned int goalBps = 1000000;

BB8Servos::BB8Servos() {
  infoXelsSr = NULL;
  infoXelsSw = NULL;
}

Result BB8Servos::initialize() {
  name_ = "servos";
  description_ = "Dynamixel subsystem";
  help_ = "Dynamixel Subsystem\r\n"
          "Available commands:\r\n"
          "\ttorque <servo> on|off           Switch torque of <servo> on or off.\r\n"
          "\tmove <servo> <angle>            <servo> must be a valid servo number. <angle> is in degrees between -180.0 and 180.0.\r\n"
          "\tinfo <servo>                    <servo> must be a valid servo number.\r\n"
          "\t<ctrltableitem> <servo>         Return <ctrltableitem>, currently understood: velocity_limit, current_limit, profile_acceleration.\r\n"
          "\t<ctrltableitem> <servo> <value> Set <ctrltableitem> to <value>, currently understood: velocity_limit, current_limit, profile_acceleration.\r\n"
          "\ttest <servo>                    Test <servo>, going from 180 to +x°/-x° in 5° steps, until it fails\r\n"
          "\tpower on|off                    Switch power on or off";

  ctrlPresentPos_.addr = 0;
  ctrlGoalPos_.addr = 0;
  ctrlGoalVel_.addr = 0;
  BB8ServoPower::power.initialize();
  return Subsystem::initialize();
}

Result BB8Servos::start(ConsoleStream *stream) {
  if(isStarted()) return RES_SUBSYS_ALREADY_STARTED;

  BB8ServoPower::power.switchOnOff(true);
  delay(500);

  dxl_.setPortProtocolVersion(2.0);
  unsigned int bps = 0;

  if(stream) stream->print("Detecting Dynamixels... ");
  for(unsigned int i=0; i<numBps; i++) {
    dxl_.begin(bpsList[i]);
    if(dxl_.scan() == true) {
      bps = bpsList[i];
      break;
    }
  }

  if(bps == 0) {
    if(stream) stream->println("none found.");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  if(stream) stream->print(String("found at ") + bps + "bps, enumerating...");

  for (uint8_t id = 1; id < 254; id++) {
    if (dxl_.ping(id)) {
      uint16_t model = dxl_.getModelNumber(id);
      if(stream) stream->print(String("#") + id + ": model #" + model + "...");

      Servo servo = {2048, 2048};

      if(ctrlPresentPos_.addr == 0) {
        ctrlPresentPos_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_POSITION);
        ctrlGoalPos_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_POSITION);
        ctrlGoalVel_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_VELOCITY);
      } else {
        DYNAMIXEL::ControlTableItemInfo_t item;
        item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_POSITION);
        if(item.addr != ctrlPresentPos_.addr || item.addr_length != ctrlPresentPos_.addr_length) {
          if(stream) stream->println("Servos use differing control tables, that is not supported currently!");
          return RES_SUBSYS_HW_DEPENDENCY_MISSING;
        }
        item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_POSITION);
        if(item.addr != ctrlGoalPos_.addr || item.addr_length != ctrlGoalPos_.addr_length) {
          if(stream) stream->println("Servos use differing control tables, that is not supported currently!");
          return RES_SUBSYS_HW_DEPENDENCY_MISSING;
        }
        item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_VELOCITY);
        if(item.addr != ctrlGoalVel_.addr || item.addr_length != ctrlGoalVel_.addr_length) {
          if(stream) stream->println("Servos use differing control tables, that is not supported currently!");
          return RES_SUBSYS_HW_DEPENDENCY_MISSING;
        }
      }
      servos_[id] = servo;
    }
  }
  if(stream) stream->println();

  if(servos_.size() != 4) return RES_SUBSYS_HW_DEPENDENCY_MISSING;

  // Configure servos
  for(auto& s: servos_) dxl_.torqueOff(s.first);
  if(bps != goalBps) {
    for(auto& s: servos_) dxl_.setBaudrate(s.first, goalBps);
    dxl_.begin(goalBps);
    if(dxl_.scan() == false) {
      if(stream) stream->println(String("Could not rescan after switching to ") + goalBps + "bps!");
      return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    }
    bps = goalBps;
  }
  for(auto& s: servos_) {
    dxl_.setOperatingMode(s.first, OP_POSITION);
    dxl_.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, s.first, 10);
  }

  setupSyncBuffers();

  uint8_t recv_cnt = dxl_.syncRead(&srInfos);
  if(recv_cnt != servos_.size()) {
    if(stream) stream->println("Receiving initial position failed!");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  for(auto& s: servos_) {
    if(stream) stream->println(String("Present for #") + s.first + ": " + s.second.present);
    s.second.goal = s.second.present;
  }

  operationStatus_ = RES_OK;
  started_ = true;
  return RES_OK;
}

Result BB8Servos::stop(ConsoleStream *stream) {
  (void)stream;

  teardownSyncBuffers();

  for(auto& s: servos_) {
    dxl_.torqueOff(s.first);
  }
  while(servos_.size()) servos_.erase(servos_.begin());

  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  started_ = false;
  return RES_OK;
}

Result BB8Servos::step() {
  if (!started_ || operationStatus_ != RES_OK) return RES_SUBSYS_NOT_STARTED;
  
  uint8_t num = dxl_.syncRead(&srInfos);
  if(num != servos_.size()) {
    Console::console.printlnBroadcast("Receiving servo position failed!");
    return RES_SUBSYS_COMM_ERROR;
  }

  if(dxl_.syncWrite(&swInfos) == false) {
    Console::console.printlnBroadcast("Sending servo position failed!");
    return RES_SUBSYS_COMM_ERROR;
  }

  return RES_OK;
}

Result BB8Servos::handleConsoleCommand(const std::vector<String> &words, ConsoleStream *stream) {
  (void)stream;
  if (words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;
  if (words[0] == "move") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1].toInt();
    if (id > servos_.size()) return RES_CMD_INVALID_ARGUMENT;
    float angle = words[2].toFloat();
    if (angle < 0 || angle > 360.0) return RES_CMD_INVALID_ARGUMENT;
    if(setGoal(id, angle)) return RES_OK;
    return RES_CMD_INVALID_ARGUMENT;
  } else if (words[0] == "torque") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if (words[1] == "all") {
      for(auto& s: servos_) switchTorque(s.first, words[2] == "on" ? true : false);
      return RES_OK;
    } else {
      int id = words[1].toInt();
      if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
      return switchTorque(id, words[2] == "on" ? true : false);
    }
  } else if (words[0] == "velocity_limit") {
    return handleCtrlTableCommand(ControlTableItem::VELOCITY_LIMIT, words, stream);
  } else if (words[0] == "current_limit") {
    return handleCtrlTableCommand(ControlTableItem::CURRENT_LIMIT, words, stream);
  } else if (words[0] == "profile_acceleration") {
    return handleCtrlTableCommand(ControlTableItem::PROFILE_ACCELERATION, words, stream);
  } else if (words[0] == "goal_velocity") {
    return handleCtrlTableCommand(ControlTableItem::GOAL_VELOCITY, words, stream);
  } else if (words[0] == "operating_mode") {
    return handleCtrlTableCommand(ControlTableItem::OPERATING_MODE, words, stream);
  } else if (words[0] == "info") {
    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    int id = words[1].toInt();
    if (id < 1 || id > 4) return RES_CMD_INVALID_ARGUMENT;
    printStatus(stream, id);
    return RES_OK;
  } else if (words[0] == "test") {
    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    int id = words[1].toInt();
    if (id < 1 || id > 4) return RES_CMD_INVALID_ARGUMENT_COUNT;
    return runServoTest(stream, id);
  } else if (words[0] == "power") {
    if (words.size() == 1) {
      if(BB8ServoPower::power.isOn()) {
        stream->println("On.");
        return RES_OK;
      } else {
        stream->println("Off.");
        return RES_OK;
      }
    } else if(words.size() == 2) {
      if (words[1] == "on") return BB8ServoPower::power.switchOnOff(true);
      else if (words[1] == "off") return BB8ServoPower::power.switchOnOff(false);
      else return RES_CMD_INVALID_ARGUMENT;
    } else return RES_CMD_INVALID_ARGUMENT_COUNT;
  }

  return RES_CMD_UNKNOWN_COMMAND;
}

Result BB8Servos::handleCtrlTableCommand(ControlTableItem::ControlTableItemIndex idx, const std::vector<String> &words, ConsoleStream *stream) {
  if (words.size() < 2 || words.size() > 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
  int id = words[1].toInt();
  if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
  if (words.size() == 2) {
    int val = (int)dxl_.readControlTableItem(idx, id);
    stream->println(words[0] + "=" + val);
  } else {
    int val = words[2].toInt();
    stream->println(String("Setting ") + words[0] + " (" + (int)idx + ") to " + val);
    dxl_.writeControlTableItem(idx, id, val);
    val = (int)dxl_.readControlTableItem(idx, id);
    stream->println(words[0] + "=" + val);
  }
  return RES_OK;
}

Result BB8Servos::runServoTest(ConsoleStream *stream, int id) {
  dxl_.torqueOn(id);
  dxl_.setGoalPosition(id, 180.0, UNIT_DEGREE);
  delay(1000);

  for (float f = 5.0; f < 90.0; f += 5.0) {
    stream->println(String("Moving to ") + (180.0 + f) + "...");
    dxl_.setGoalPosition(id, 180.0 + f, UNIT_DEGREE);
    delay((int)50 * f);
    stream->println("Moving to origin");
    dxl_.setGoalPosition(id, 180.0, UNIT_DEGREE);
    delay((int)50 * f);
    stream->println(String("Moving to ") + (180.0 - f) + "...");
    dxl_.setGoalPosition(id, 180.0 - f, UNIT_DEGREE);
    delay((int)50 * f);
    stream->println("Moving to origin");
    dxl_.setGoalPosition(id, 180.0, UNIT_DEGREE);
    delay((int)50 * f);
  }

  return RES_OK;
}

Result BB8Servos::switchTorque(uint8_t id, bool onoff) {
  if (operationStatus_ != RES_OK || servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
  if (onoff) dxl_.torqueOn(id);
  else dxl_.torqueOff(id);
  return RES_OK;
}

Result BB8Servos::switchTorqueAll(bool onoff) {
  if(operationStatus_ != RES_OK) return RES_CMD_INVALID_ARGUMENT;
  for(auto& s: servos_) {
    Result retval = switchTorque(s.first, onoff);
    if(retval != RES_OK) return retval;
  }
  return RES_OK;
}

bool BB8Servos::isTorqueOn(uint8_t id) {
  if (operationStatus_ != RES_OK || servos_.count(id) == 0) return false;
  return dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id);
}

void BB8Servos::printStatus(ConsoleStream *stream, int id) {
  if (!started_) {
    stream->println("Servo subsystem not started.");
    return;
  }

  stream->print(String("Servo #") + id + ": ");
  if (dxl_.ping(id)) {
    stream->print(String("model #") + dxl_.getModelNumber(id) + ", ");
    stream->print(String("present: ") + present(id) + "deg, ");
    stream->print(String("goal: ") + goal(id) + "deg, ");
    stream->print("hw err: $");
    stream->print(String(dxl_.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id), HEX));
    stream->print(", ");
    stream->print("alarm LED field: $");
    stream->print(String(dxl_.readControlTableItem(ControlTableItem::ALARM_LED, id), HEX));
    stream->print(", ");
    stream->print("alarm shutdown field: $");
    stream->print(String(dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, id), HEX));
    stream->print(", ");
    stream->print("torque limit: ");
    stream->print((int)dxl_.readControlTableItem(ControlTableItem::TORQUE_LIMIT, id));
    stream->print(", ");
    stream->print("max torque: ");
    stream->print((int)dxl_.readControlTableItem(ControlTableItem::MAX_TORQUE, id));
    stream->print(", ");
    stream->print("torque enabled: ");
    stream->print((int)dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id));
    stream->print(", ");
    stream->print("velocity: ");
    stream->print((int)dxl_.readControlTableItem(ControlTableItem::VELOCITY_LIMIT, id));
    stream->println();
  } else {
    stream->print("#");
    stream->print(id);
    stream->print(" not found! ");
  }
}

Result BB8Servos::moveAllServosToOrigin(bool hard) {
  if (operationStatus_ != RES_OK) return operationStatus_;

  for (auto& s: servos_) {
    if(hard) dxl_.setGoalPosition(s.first, 180.0 + servolimits[s.first - 1].offset, UNIT_DEGREE);
    else setGoal(s.first, 180.0);
  }

  return RES_OK;
}

bool BB8Servos::setGoal(uint8_t id, float goal, Unit unit) {
  if(servos_.count(id) == 0) return false;
  if(unit == UNIT_DEGREES) 
    servos_[id].goal = 4096.0*(goal/360.0);
  else
    servos_[id].goal = goal;
  swInfos.is_info_changed = true;

    
  return true;
}

float BB8Servos::goal(uint8_t id, Unit unit) {
  if(servos_.count(id) == 0) return false;
  if(unit == UNIT_DEGREES) 
    return 360.0*(servos_[id].goal/4096.0);
  else
    return servos_[id].goal;
}

float BB8Servos::present(uint8_t id, Unit unit) {
  if (operationStatus_ != RES_OK) return 0.0f;

  if(servos_.count(id) == 0) return 0.0f;
  if(unit == UNIT_DEGREES) return (servos_[id].present / 4096.0) * 360.0;
  else return servos_[id].present;
}

void BB8Servos::setupSyncBuffers() {
  unsigned int i;

  if(!servos_.size()) return;
  if(infoXelsSr != NULL || infoXelsSw != NULL) return;

  srInfos.packet.p_buf = userPktBuf;
  srInfos.packet.buf_capacity = userPktBufCap;
  srInfos.packet.is_completed = false;
  srInfos.addr = ctrlPresentPos_.addr;
  srInfos.addr_length = ctrlPresentPos_.addr_length;
  infoXelsSr = new DYNAMIXEL::XELInfoSyncRead_t[servos_.size()];
  srInfos.p_xels = infoXelsSr;
  i=0;
  for(auto& s: servos_) {
    infoXelsSr[i].id = s.first;
    infoXelsSr[i].p_recv_buf = (uint8_t*)&(s.second.present);
    i++;
  }
  srInfos.xel_count = servos_.size();
  srInfos.is_info_changed = true;

  swInfos.packet.p_buf = NULL;
  swInfos.packet.is_completed = false;
  swInfos.addr = ctrlGoalPos_.addr;
  swInfos.addr_length = ctrlGoalPos_.addr_length;
  infoXelsSw = new DYNAMIXEL::XELInfoSyncWrite_t[servos_.size()];
  swInfos.p_xels = infoXelsSw;
  i=0;
  for(auto& s: servos_) {
    infoXelsSw[i].id = s.first;
    infoXelsSw[i].p_data = (uint8_t*)&(s.second.goal);
    i++;
  }
  swInfos.xel_count = servos_.size();
  swInfos.is_info_changed = true;
}

void BB8Servos::teardownSyncBuffers() {
  delete infoXelsSr; infoXelsSr = NULL;
  delete infoXelsSw; infoXelsSw = NULL;
  srInfos.p_xels = NULL;
  srInfos.xel_count = 0;
  swInfos.p_xels = NULL;
  swInfos.xel_count = 0;
}
