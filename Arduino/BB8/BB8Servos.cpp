#include <Wire.h>
#include "actuator.h"
#include "BB8Servos.h"
#include "BB8Config.h"

static const uint8_t LED_BIT = 0x01;
static const uint8_t SWITCH_BIT = 0x08;
static const uint8_t ADDR = 0x38;
static const uint8_t INPUT_REG = 0x00, OUTPUT_REG = 0x01, INVERT_REG = 0x02, CONFIG_REG = 0x03;

static uint32_t SLOW_VEL = 20;
static const uint8_t MAX_SERVO_ID = 4;

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

bool BB8ServoPower::requestFrom(uint8_t addr, uint8_t reg, uint8_t& byte) {
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
          "\ttorque <servo>|all on|off           Switch torque of <servo> on or off.\r\n"
          "\tmove <servo>|all <angle>            <servo> must be a valid servo number. <angle> is in degrees between -180.0 and 180.0.\r\n"
          "\tmove_slow <servo>|all <angle>       <servo> must be a valid servo number. <angle> is in degrees between -180.0 and 180.0.\r\n"
          "\torigin <servo>|all                  Move <servo> to origin, slowly.\r\n"
          "\tinfo <servo>                        <servo> must be a valid servo number.\r\n"
          "\t<ctrltableitem> <servo>             Return <ctrltableitem>, currently understood: velocity_limit, current_limit, profile_acceleration.\r\n"
          "\t<ctrltableitem> <servo> <value>     Set <ctrltableitem> to <value>, currently understood: velocity_limit, current_limit, profile_acceleration.\r\n"
          "\ttest <servo>                        Test <servo>, going from 180 to +x°/-x° in 5° steps, until it fails\r\n"
          "\tpower on|off                        Switch power on or off\r\n"
          "\reboot <servo>|all                   Reboot <servo>";

  ctrlPresentPos_.addr = 0;
  ctrlGoalPos_.addr = 0;
  ctrlProfileVel_.addr = 0;
  BB8ServoPower::power.initialize();
  return Subsystem::initialize();
}

Result BB8Servos::start(ConsoleStream* stream) {
  if (isStarted()) return RES_SUBSYS_ALREADY_STARTED;

  BB8ServoPower::power.switchOnOff(true);
  delay(500);

  dxl_.setPortProtocolVersion(2.0);
  unsigned int bps = 0;

  if (stream) stream->print("Detecting Dynamixels... ");
  for (unsigned int i = 0; i < numBps; i++) {
    dxl_.begin(bpsList[i]);
    if (dxl_.scan() == false) continue;

    bps = bpsList[i];

    if (stream) stream->print(String("scan successful at ") + bps + "bps, enumerating up to " + MAX_SERVO_ID + "...");

    for (uint8_t id = 1; id <= MAX_SERVO_ID; id++) {
      if (dxl_.ping(id)) {
        uint16_t model = dxl_.getModelNumber(id);
        if (stream) stream->print(String("#") + id + ": model #" + model + "...");

        Servo servo = { 2048, 2048, 0, 4096, 0, false };

        if (ctrlPresentPos_.addr == 0) {
          ctrlPresentPos_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_POSITION);
          ctrlGoalPos_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_POSITION);
          ctrlProfileVel_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PROFILE_VELOCITY);
        } else {
          DYNAMIXEL::ControlTableItemInfo_t item;
          item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_POSITION);
          if (item.addr != ctrlPresentPos_.addr || item.addr_length != ctrlPresentPos_.addr_length) {
            if (stream) stream->println("Servos use differing control tables, that is not supported currently!");
            return RES_SUBSYS_HW_DEPENDENCY_MISSING;
          }
          item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_POSITION);
          if (item.addr != ctrlGoalPos_.addr || item.addr_length != ctrlGoalPos_.addr_length) {
            if (stream) stream->println("Servos use differing control tables, that is not supported currently!");
            return RES_SUBSYS_HW_DEPENDENCY_MISSING;
          }
          item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PROFILE_VELOCITY);
          if (item.addr != ctrlProfileVel_.addr || item.addr_length != ctrlProfileVel_.addr_length) {
            if (stream) stream->println("Servos use differing control tables, that is not supported currently!");
            return RES_SUBSYS_HW_DEPENDENCY_MISSING;
          }
        }
        servos_[id] = servo;
      }
    }

    if (stream) stream->println();
  }

  if (servos_.size() != 4) {
    if (stream) stream->println(String("Only ") + servos_.size() + " servos found, expected 4!");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  if(stream) {
    for(auto& s: servos_) {
      uint8_t err = errorStatus(s.first);
      if(err != 0x0) {
        stream->print(String("Servo #") + s.first + " reports error " + String(err, HEX) + ". Consider rebooting it.");
      }
    }
  }

  // Configure servos
  for (auto& s : servos_) dxl_.torqueOff(s.first);
  if (bps != goalBps) {
    for (auto& s : servos_) dxl_.setBaudrate(s.first, goalBps);
    dxl_.begin(goalBps);
    if (dxl_.scan() == false) {
      if (stream) stream->println(String("Could not rescan after switching to ") + goalBps + "bps!");
      return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    }
    bps = goalBps;
  }

  setupSyncBuffers();

  uint8_t recv_cnt = dxl_.syncRead(&srInfos);
  if (recv_cnt != servos_.size()) {
    if (stream) stream->println("Receiving initial position failed!");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  for (auto& s : servos_) {
    dxl_.setOperatingMode(s.first, OP_POSITION);
    dxl_.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, s.first, 10);
    setGoal(s.first, s.second.present, VALUE_RAW);
    switchTorque(s.first, true);
  }

  operationStatus_ = RES_OK;
  started_ = true;
  return RES_OK;
}

Result BB8Servos::stop(ConsoleStream* stream) {
  (void)stream;

  teardownSyncBuffers();

  for (auto& s : servos_) {
    dxl_.torqueOff(s.first);
  }
  while (servos_.size()) servos_.erase(servos_.begin());

  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  started_ = false;
  return RES_OK;
}

Result BB8Servos::step() {
  static unsigned int failcount = 0;

  if (!started_ || operationStatus_ != RES_OK) return RES_SUBSYS_NOT_STARTED;

  if (failcount > 5) {
    Console::console.printlnBroadcast("Servo communication failed more than 5 times in a row. Stopping subsystem.");
    stop();
    failcount = 0;
    return RES_SUBSYS_COMM_ERROR;
  }

  uint8_t num = dxl_.syncRead(&srInfos);
  if (num != servos_.size()) {
    Console::console.printlnBroadcast("Receiving servo position failed!");
    failcount++;
    return RES_SUBSYS_COMM_ERROR;
  }

  if (dxl_.syncWrite(&swInfos) == false) {
    Console::console.printlnBroadcast("Sending servo position failed!");
    failcount++;
    return RES_SUBSYS_COMM_ERROR;
  }

  failcount = 0;
  return RES_OK;
}

Result BB8Servos::handleConsoleCommand(const std::vector<String>& words, ConsoleStream* stream) {
  (void)stream;
  if (words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if (words[0] == "move") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1].toInt();
    if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
    float angle = words[2].toFloat();
    if(angle < 0 || angle > 360.0) return RES_CMD_INVALID_ARGUMENT;
    if(setGoal(id, angle)) return RES_OK;
    return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "move_slow") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1].toInt();
    if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
    float angle = words[2].toFloat();
    if(angle < 0 || angle > 360.0) return RES_CMD_INVALID_ARGUMENT;
    return moveSlow(id, angle);
  }

  else if (words[0] == "origin") {
    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;

    uint8_t id;
    if (words[1] == "all") id = ID_ALL;
    else {
      id = words[1].toInt();
      if (servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
    }

    return moveSlow(id, 180.0);
  }

  else if (words[0] == "torque") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    uint8_t id;
    if (words[1] == "all") id = ID_ALL;
    else id = words[1].toInt();
    return switchTorque(id, words[2] == "on" ? true : false);
  }

  else if (words[0] == "velocity_limit") {
    return handleCtrlTableCommand(ControlTableItem::VELOCITY_LIMIT, words, stream);
  }

  else if (words[0] == "current_limit") {
    return handleCtrlTableCommand(ControlTableItem::CURRENT_LIMIT, words, stream);
  }

  else if (words[0] == "profile_acceleration") {
    return handleCtrlTableCommand(ControlTableItem::PROFILE_ACCELERATION, words, stream);
  }

  else if (words[0] == "profile_velocity") {
    return handleCtrlTableCommand(ControlTableItem::PROFILE_VELOCITY, words, stream);
  }

  else if (words[0] == "operating_mode") {
    return handleCtrlTableCommand(ControlTableItem::OPERATING_MODE, words, stream);
  }

  else if (words[0] == "position_p_gain") {
    return handleCtrlTableCommand(ControlTableItem::POSITION_P_GAIN, words, stream);
  }

  else if (words[0] == "position_i_gain") {
    return handleCtrlTableCommand(ControlTableItem::POSITION_I_GAIN, words, stream);
  }

  else if (words[0] == "position_d_gain") {
    return handleCtrlTableCommand(ControlTableItem::POSITION_D_GAIN, words, stream);
  }

  else if (words[0] == "info") {
    Runloop::runloop.excuseOverrun();

    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words[1] == "all") {
      for(auto& s: servos_) printStatus(stream, s.first);
      return RES_OK;
    }
    int id = words[1].toInt();
    if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
    printStatus(stream, id);
    return RES_OK;
  }

  else if (words[0] == "test") {
    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    int id = words[1].toInt();
    if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
    return runServoTest(stream, id);
  }

  else if (words[0] == "power") {
    if (words.size() == 1) {
      if (BB8ServoPower::power.isOn()) {
        stream->println("On.");
        return RES_OK;
      } else {
        stream->println("Off.");
        return RES_OK;
      }
    } else if (words.size() == 2) {
      if (words[1] == "on") return BB8ServoPower::power.switchOnOff(true);
      else if (words[1] == "off") return BB8ServoPower::power.switchOnOff(false);
      else return RES_CMD_INVALID_ARGUMENT;
    } else return RES_CMD_INVALID_ARGUMENT_COUNT;
  }

  else if (words[0] == "reboot") {
    Runloop::runloop.excuseOverrun();

    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words[1] == "all") {
      for (auto& s : servos_) {
        if (stream) {
          stream->println(String("Rebooting ") + s.first + "... ");
        }
        dxl_.reboot(s.first);
      }
    } else {
      uint8_t id = words[1].toInt();
      if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
      dxl_.reboot(id);
    }

    delay(1000);

    return RES_OK;
  }

  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result BB8Servos::handleCtrlTableCommand(ControlTableItem::ControlTableItemIndex idx, const std::vector<String>& words, ConsoleStream* stream) {
  if (words.size() < 2 || words.size() > 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
  int id = words[1].toInt();
  if (servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
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

Result BB8Servos::runServoTest(ConsoleStream* stream, int id) {
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
  if (operationStatus_ != RES_OK) return operationStatus_;
  if(id == ID_ALL) {
    for(auto& s: servos_) {
      Result res = switchTorque(s.first, onoff);
      if(res != RES_OK) return res;
    }
    return RES_OK;
  } 
  
  if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
  if(onoff) dxl_.torqueOn(id);
  else dxl_.torqueOff(id);
  return RES_OK;
}

bool BB8Servos::isTorqueOn(uint8_t id) {
  if (operationStatus_ != RES_OK || servos_.count(id) == 0) return false;
  return dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id);
}

void BB8Servos::printStatus(ConsoleStream* stream, int id) {
  if (!started_) {
    stream->println("Servo subsystem not started.");
    return;
  }

  if(servos_.count(id) == 0) {
    stream->println(String("Servo #") + id + "does not exist.");
    return;
  }

  stream->print(String("Servo #") + id + ": ");
  if (dxl_.ping(id)) {
    stream->print(String("model #") + dxl_.getModelNumber(id) + ", ");
    stream->print(String("present: ") + present(id) + "deg (" + (int)present(id, VALUE_RAW) + "), ");
    stream->print(String("goal: ") + goal(id) + "deg (" + (int)goal(id, VALUE_RAW) + "), ");
    
    stream->print(String("range: [") + servos_[id].min + ".." + servos_[id].max +"], ");
    stream->print(String("offset: ") + servos_[id].offset + ", ");
    stream->print(String("invert: ") + servos_[id].invert + ", ");

    stream->print("hw err: $");
    stream->print(String(dxl_.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id), HEX));
    stream->print(", alarm shutdown: $");
    stream->print(String(dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, id), HEX));
    stream->print(", torque limit: ");
    stream->print((int)dxl_.readControlTableItem(ControlTableItem::TORQUE_LIMIT, id));
    stream->print(", max torque: ");
    stream->print((int)dxl_.readControlTableItem(ControlTableItem::MAX_TORQUE, id));
    stream->print(", torque enabled: ");
    stream->print((int)dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id));
    stream->print(", profile vel: ");
    stream->print((int)dxl_.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id));
    stream->print(", profile acc: ");
    stream->print((int)dxl_.readControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id));
    stream->println();
  } else {
    stream->print("#");
    stream->print(id);
    stream->print(" not found! ");
  }
}

Result BB8Servos::moveSlow(int id, float goal, ValueType t) {
  if (operationStatus_ != RES_OK) return operationStatus_;
  if (isnan(goal)) return RES_SUBSYS_HW_DEPENDENCY_MISSING;

  goal = computeRawValue(goal, t);

  float msPerTick = (1000/(SLOW_VEL*15.63));
  float present, delta;
  unsigned int d=0;

  if (id == ID_ALL) {
    for (auto& s : servos_) {
      present = dxl_.getPresentPosition(s.first);
      delta = fabs(present - goal);
      d = max(d, delta*msPerTick);

      dxl_.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, s.first, SLOW_VEL);
      dxl_.setGoalPosition(s.first, goal);
      setGoal(s.first, goal, VALUE_RAW);
    }

    delay(d+100);
    
    for (auto& s : servos_) {
      dxl_.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, s.first, 0);
    }

    for (auto& s : servos_) {
      present = dxl_.getPresentPosition(s.first);
      delta = computeRawValue(1.5, VALUE_DEGREE);
      if (fabs(present - goal) > delta) {
        Console::console.printlnBroadcast(String("Servo ") + s.first + " didn't move to " + goal + ", stuck at " + present);
        return RES_SUBSYS_HW_DEPENDENCY_MISSING;
      }
    }

    return RES_OK;
  }

  if (servos_.count(id) == 0) return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  present = dxl_.getPresentPosition(id);
  delta = fabs(present - goal);
  d = delta * msPerTick;

  dxl_.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, SLOW_VEL);
  dxl_.setGoalPosition(id, goal);
  setGoal(id, goal, VALUE_RAW);

  delay(d+100);

  dxl_.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, 0);

  present = dxl_.getPresentPosition(id);
  delta = computeRawValue(1.5, VALUE_DEGREE);
  if (fabs(present - goal) > delta) {
    Console::console.printlnBroadcast(String("Servo ") + id + " didn't move to " + goal + ", stuck at " + present);
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  return RES_OK;
}

bool BB8Servos::setRange(uint8_t id, float min, float max, ValueType t) {
  if(servos_.count(id) == 0) return false;
  
  if(min > max) {
    float t = min;
    max = min;
    min = t;
  }

  servos_[id].min = computeRawValue(min, t);
  servos_[id].max = computeRawValue(max, t);
  if(goal(id, VALUE_RAW) < servos_[id].min) setGoal(id, servos_[id].min, VALUE_RAW);
  if(goal(id, VALUE_RAW) > servos_[id].max) setGoal(id, servos_[id].max, VALUE_RAW);
  return true;
}

bool BB8Servos::setOffset(uint8_t id, float offset, ValueType t) {
  if(servos_.count(id) == 0) return false;

  if(t == VALUE_DEGREE) {
    servos_[id].offset = (offset * 4096.0) / 360.0;
  } else {
    servos_[id].offset = offset;
  }
  return true;
}

bool BB8Servos::setInvert(uint8_t id, bool invert) {
  if(servos_.count(id) == 0) return false;

  servos_[id].invert = invert;
  return true;
}

bool BB8Servos::setGoal(uint8_t id, float goal, ValueType t) {
  if (isnan(goal)) return false;

  if (id == ID_ALL) {
    for (auto& s : servos_) {
      if (setGoal(s.first, goal, t) != true) return false;
    }
    return true;
  }

  if (servos_.count(id) == 0) return false;
  uint32_t g = computeRawValue(goal, t) + servos_[id].offset;
  if(servos_[id].invert) {
    g = 4096 - g;
  }
  servos_[id].goal = constrain(g, servos_[id].min, servos_[id].max);

  swInfos.is_info_changed = true;

  return true;
}

uint32_t BB8Servos::computeRawValue(float val, ValueType t) {
  int32_t retval;
  switch (t) {
    case VALUE_DEGREE:
      retval = 4096.0 * (val / 360.0);
      break;
    case VALUE_RAW:
    default:
      retval = val;
  }

  if (retval < 0 && retval > 4096) {
    Console::console.printlnBroadcast(String("Capping ") + retval + " (computed from " + val + ") to [0..4096]!");
    retval = constrain(retval, 0, 4096);
  }

  return (uint32_t)retval;
}

bool BB8Servos::setProfileAcceleration(uint8_t id, uint32_t val) {
  dxl_.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, val);
  return true;
}

bool BB8Servos::setProfileVelocity(uint8_t id, uint32_t val) {
  dxl_.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, val);
  return true;
}


float BB8Servos::goal(uint8_t id, ValueType t) {
  if (servos_.count(id) == 0) return false;
  if (t == VALUE_DEGREE)
    return 360.0 * (servos_[id].goal / 4096.0);
  else
    return servos_[id].goal;
}

float BB8Servos::present(uint8_t id, ValueType t) {
  if (operationStatus_ != RES_OK) return 0.0f;

  if (servos_.count(id) == 0) return 0.0f;
  if (t == VALUE_DEGREE) return (servos_[id].present / 4096.0) * 360.0;
  else return servos_[id].present;
}

uint8_t BB8Servos::errorStatus(uint8_t id) {
  if(operationStatus_ != RES_OK) return 0xff;
  if(servos_.count(id) == 0) return 0xff;
  return dxl_.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);
}

void BB8Servos::setupSyncBuffers() {
  unsigned int i;

  if (!servos_.size()) return;
  if (infoXelsSr != NULL || infoXelsSw != NULL) return;

  srInfos.packet.p_buf = userPktBuf;
  srInfos.packet.buf_capacity = userPktBufCap;
  srInfos.packet.is_completed = false;
  srInfos.addr = ctrlPresentPos_.addr;
  srInfos.addr_length = ctrlPresentPos_.addr_length;
  infoXelsSr = new DYNAMIXEL::XELInfoSyncRead_t[servos_.size()];
  srInfos.p_xels = infoXelsSr;
  i = 0;
  for (auto& s : servos_) {
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
  i = 0;
  for (auto& s : servos_) {
    infoXelsSw[i].id = s.first;
    infoXelsSw[i].p_data = (uint8_t*)&(s.second.goal);
    i++;
  }
  swInfos.xel_count = servos_.size();
  swInfos.is_info_changed = true;
}

void BB8Servos::teardownSyncBuffers() {
  delete infoXelsSr;
  infoXelsSr = NULL;
  delete infoXelsSw;
  infoXelsSw = NULL;
  srInfos.p_xels = NULL;
  srInfos.xel_count = 0;
  swInfos.p_xels = NULL;
  swInfos.xel_count = 0;
}
