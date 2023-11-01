#include <Wire.h>
#include "actuator.h"
#include "BB8Servos.h"
#include "BB8Config.h"

static uint32_t SLOW_VEL = 20;
static const uint8_t MAX_SERVO_ID = 4;


BB8Servos BB8Servos::servos;
static const unsigned int bpsList[] = { 57600, 115200, 1000000 };
static const unsigned int numBps = 3;
static const unsigned int goalBps = 1000000;

BB8ServoControlOutput::BB8ServoControlOutput(uint8_t sn, float offset) {
  sn_ = sn;
  offset_ = offset;
}

bb::Result BB8ServoControlOutput::set(float value) {
  if(BB8Servos::servos.setGoal(sn_, value) == true) return RES_OK;
  return RES_CMD_FAILURE;
}

float BB8ServoControlOutput::present() {
  return BB8Servos::servos.present(sn_);
}

BB8Servos::BB8Servos() {
  infoXelsSrPresent = NULL;
  infoXelsSrLoad = NULL;
  infoXelsSwGoal = NULL;
  infoXelsSwVel = NULL;
}

Result BB8Servos::initialize() {
  name_ = "servos";
  description_ = "Dynamixel subsystem";
  help_ = "Dynamixel Subsystem\r\n"
          "Available commands:\r\n"
          "\ttorque <servo>|all on|off           Switch torque.\r\n"
          "\tmove <servo>|all <angle>            <angle>: deg [-180.0..180.0].\r\n"
          "\tset_vel <servo>|all <val>           <val> in deg/s; 0 is infinite speed.\r\n"
          "\tmove_slow <servo>|all <angle>       <angle>: deg [-180.0..180.0].\r\n"
          "\torigin <servo>|all                  Move <servo> to origin, slowly.\r\n"
          "\tinfo <servo>                        Get info on <servo>.\r\n"
          "\t<ctrltableitem> <servo> [<value>]   Get or set, options: velocity_limit, current_limit, profile_acceleration.\r\n"
          "\ttest <servo>                        Test <servo>, going from 180 to +x°/-x° in 5° steps, until it fails\r\n"
          "\reboot <servo>|all                   Reboot <servo>";

  ctrlPresentPos_.addr = 0;
  ctrlPresentLoad_.addr = 0;
  ctrlGoalPos_.addr = 0;
  ctrlProfileVel_.addr = 0;
  return Subsystem::initialize();
}

Result BB8Servos::start(ConsoleStream* stream) {
  if (isStarted()) return RES_SUBSYS_ALREADY_STARTED;

  Runloop::runloop.excuseOverrun();

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

        Servo servo = { 2048, 0, 2048, 0, 0, 4096, 0 };

        if (ctrlPresentPos_.addr == 0) {
          ctrlPresentPos_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_POSITION);
          ctrlGoalPos_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_POSITION);
          ctrlProfileVel_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PROFILE_VELOCITY);
          // PRESENT_LOAD and PRESENT_CURRENT have the same addresses but different names (and units - 0.1% vs mA). Robotis, why do you do this to us?
          ctrlPresentLoad_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_LOAD);
          if(ctrlPresentLoad_.addr == 0) ctrlPresentLoad_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_CURRENT); 
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
          item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_LOAD);
          if(item.addr == 0) item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_CURRENT);
          if (item.addr != ctrlPresentLoad_.addr || item.addr_length != ctrlPresentLoad_.addr_length) {
            if (stream) stream->println(String("Servos use differing control tables, that is not supported currently! (addr ") + item.addr + "!=" + ctrlPresentLoad_.addr + ", length " + item.addr_length + "!=" + ctrlPresentLoad_.addr_length + ")");
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

  uint8_t recv_cnt;
  
  recv_cnt = dxl_.syncRead(&srPresentInfos);
  if (recv_cnt != servos_.size()) {
    if (stream) stream->println("Receiving initial position failed!");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  recv_cnt = dxl_.syncRead(&srLoadInfos);
  if (recv_cnt != servos_.size()) {
    if (stream) stream->println("Receiving initial position failed!");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  for (auto& s : servos_) {
    dxl_.setOperatingMode(s.first, OP_POSITION);
    dxl_.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, s.first, 10);
    dxl_.writeControlTableItem(ControlTableItem::DRIVE_MODE, s.first, 0);
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

  uint8_t num;
  
  num = dxl_.syncRead(&srPresentInfos);
  if (num != servos_.size()) {
    Console::console.printlnBroadcast("Receiving servo position failed!");
    failcount++;
    return RES_SUBSYS_COMM_ERROR;
  }

  num = dxl_.syncRead(&srLoadInfos);
  if (num != servos_.size()) {
    Console::console.printlnBroadcast("Receiving servo load failed!");
    failcount++;
    return RES_SUBSYS_COMM_ERROR;
  }

  if (dxl_.syncWrite(&swGoalInfos) == false) {
    Console::console.printlnBroadcast("Sending servo position goal failed!");
    failcount++;
    return RES_SUBSYS_COMM_ERROR;
  }

  if (dxl_.syncWrite(&swVelInfos) == false) {
    Console::console.printlnBroadcast("Sending servo profile velocity failed!");
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
    unsigned int id = words[1] == "all" ? ID_ALL : words[1].toInt();
    if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
    float angle = words[2].toFloat();
    if(angle < 0 || angle > 360.0) return RES_CMD_INVALID_ARGUMENT;
    if(setGoal(id, angle)) return RES_OK;
    return RES_CMD_INVALID_ARGUMENT;
  }

  if (words[0] == "set_vel") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1] == "all" ? ID_ALL : words[1].toInt();
    if(servos_.count(id) == 0) return RES_CMD_INVALID_ARGUMENT;
    float vel = words[2].toFloat();
    if(vel < 0) return RES_CMD_INVALID_ARGUMENT;
    if(setProfileVelocity(id, vel)) return RES_OK;
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
    stream->print(String("invert: ") + (dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id) & 0x1) + ", ");

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

bool BB8Servos::setInverted(uint8_t id, bool invert) {
  if(servos_.count(id) == 0) return false;

  uint8_t dm = dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  if(invert) dm |= 0x1;
  else dm &= ~0x1;
  dxl_.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, dm);

  return true;
}

bool BB8Servos::inverted(uint8_t id) {
  if(servos_.count(id) == 0) return false;
  return dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id) & 0x1;
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
  servos_[id].goal = constrain(g, servos_[id].min, servos_[id].max);

  swGoalInfos.is_info_changed = true;

  return true;
}

bool BB8Servos::setProfileVelocity(uint8_t id, float vel, ValueType t) {
  if(isnan(vel) || vel<0) return false;

  if (id == ID_ALL) {
    for (auto& s : servos_) {
      if (setProfileVelocity(s.first, vel, t) != true) return false;
    }
    return true;
  }

  if (servos_.count(id) == 0) return false;
  uint32_t v = vel;
  if(t == VALUE_DEGREE) {
    v = (uint32_t)((vel / 6.0f) / 0.229); // deg/s to rev/min, and then multiply with steps of 0.229
  }

  servos_[id].vel = v;
  swVelInfos.is_info_changed = true;

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

float BB8Servos::load(uint8_t id) {
  if (operationStatus_ != RES_OK) return 0.0f;

  if (servos_.count(id) == 0) return 0.0f;
  return servos_[id].load;
}

uint8_t BB8Servos::errorStatus(uint8_t id) {
  if(servos_.count(id) == 0) return 0xff;
  return dxl_.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);
}

bool BB8Servos::loadShutdownEnabled(uint8_t id) {
  if(servos_.count(id) == 0) return false;
  return dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, id) & (1<<5);
}

void BB8Servos::setLoadShutdownEnabled(uint8_t id, bool yesno) {
  if(servos_.count(id) == 0) return;
  uint8_t shutdown = dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, id);
  if(yesno) shutdown |= (1<<5);
  else shutdown &= ~(1<<5);
  dxl_.writeControlTableItem(ControlTableItem::SHUTDOWN, id, shutdown);
}

void BB8Servos::setupSyncBuffers() {
  unsigned int i;

  if (!servos_.size()) return;
  if (infoXelsSrPresent != NULL || infoXelsSrLoad != NULL || infoXelsSwGoal != NULL || infoXelsSwVel != NULL) return;

  srPresentInfos.packet.p_buf = userPktBufPresent;
  srPresentInfos.packet.buf_capacity = userPktBufCap;
  srPresentInfos.packet.is_completed = false;
  srPresentInfos.addr = ctrlPresentPos_.addr;
  srPresentInfos.addr_length = ctrlPresentPos_.addr_length;
  infoXelsSrPresent = new DYNAMIXEL::XELInfoSyncRead_t[servos_.size()];
  srPresentInfos.p_xels = infoXelsSrPresent;
  i = 0;
  for (auto& s : servos_) {
    infoXelsSrPresent[i].id = s.first;
    infoXelsSrPresent[i].p_recv_buf = (uint8_t*)&(s.second.present);
    i++;
  }
  srPresentInfos.xel_count = servos_.size();
  srPresentInfos.is_info_changed = true;

  srLoadInfos.packet.p_buf = userPktBufLoad;
  srLoadInfos.packet.buf_capacity = userPktBufCap;
  srLoadInfos.packet.is_completed = false;
  srLoadInfos.addr = ctrlPresentLoad_.addr;
  srLoadInfos.addr_length = ctrlPresentLoad_.addr_length;
  infoXelsSrLoad = new DYNAMIXEL::XELInfoSyncRead_t[servos_.size()];
  srLoadInfos.p_xels = infoXelsSrLoad;
  i = 0;
  for (auto& s : servos_) {
    infoXelsSrLoad[i].id = s.first;
    infoXelsSrLoad[i].p_recv_buf = (uint8_t*)&(s.second.load);
    i++;
  }
  srLoadInfos.xel_count = servos_.size();
  srLoadInfos.is_info_changed = true;

  swGoalInfos.packet.p_buf = NULL;
  swGoalInfos.packet.is_completed = false;
  swGoalInfos.addr = ctrlGoalPos_.addr;
  swGoalInfos.addr_length = ctrlGoalPos_.addr_length;
  infoXelsSwGoal = new DYNAMIXEL::XELInfoSyncWrite_t[servos_.size()];
  swGoalInfos.p_xels = infoXelsSwGoal;
  i = 0;
  for (auto& s : servos_) {
    infoXelsSwGoal[i].id = s.first;
    infoXelsSwGoal[i].p_data = (uint8_t*)&(s.second.goal);
    i++;
  }
  swGoalInfos.xel_count = servos_.size();
  swGoalInfos.is_info_changed = true;

  swVelInfos.packet.p_buf = NULL;
  swVelInfos.packet.is_completed = false;
  swVelInfos.addr = ctrlProfileVel_.addr;
  swVelInfos.addr_length = ctrlProfileVel_.addr_length;
  infoXelsSwVel = new DYNAMIXEL::XELInfoSyncWrite_t[servos_.size()];
  swVelInfos.p_xels = infoXelsSwVel;
  i = 0;
  for (auto& s : servos_) {
    infoXelsSwVel[i].id = s.first;
    infoXelsSwVel[i].p_data = (uint8_t*)&(s.second.vel);
    i++;
  }
  swVelInfos.xel_count = servos_.size();
  swVelInfos.is_info_changed = true;
}

void BB8Servos::teardownSyncBuffers() {
  delete infoXelsSrPresent;
  infoXelsSrPresent = NULL;
  delete infoXelsSrLoad;
  infoXelsSrLoad = NULL;
  delete infoXelsSwGoal;
  infoXelsSwGoal = NULL;
  delete infoXelsSwVel;
  infoXelsSwVel = NULL;
  srPresentInfos.p_xels = NULL;
  srPresentInfos.xel_count = 0;
  srLoadInfos.p_xels = NULL;
  srLoadInfos.xel_count = 0;
  swGoalInfos.p_xels = NULL;
  swGoalInfos.xel_count = 0;
  swVelInfos.p_xels = NULL;
  swVelInfos.xel_count = 0;
}
