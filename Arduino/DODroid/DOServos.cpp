#include <Wire.h>
#include "actuator.h"
#include "DOServos.h"
#include "DOConfig.h"

static uint32_t SLOW_VEL = 20;
static const uint8_t MAX_SERVO_ID = 4;

DOServos DOServos::servos;
static const unsigned int bpsList[] = { 57600, 115200, 1000000 };
static const unsigned int numBps = 3;
static const unsigned int goalBps = 1000000;

struct StrToCtrlTable {
  const char *str;
  ControlTableItem::ControlTableItemIndex idx;
};

static const StrToCtrlTable strToCtrlTable_[] = {
  {"vel_limit", ControlTableItem::VELOCITY_LIMIT},
  {"current_limit", ControlTableItem::CURRENT_LIMIT},
  {"profile_acc", ControlTableItem::PROFILE_ACCELERATION},
  {"profile_vel", ControlTableItem::PROFILE_VELOCITY},
  {"operating_mode", ControlTableItem::OPERATING_MODE},
  {"pos_p_gain", ControlTableItem::POSITION_P_GAIN},
  {"pos_i_gain", ControlTableItem::POSITION_I_GAIN},
  {"pos_d_gain", ControlTableItem::POSITION_D_GAIN}
};

static const int strToCtrlTableLen_ = 8;

DOServoControlOutput::DOServoControlOutput(uint8_t sn, float offset) {
  sn_ = sn;
  offset_ = offset;
}

bb::Result DOServoControlOutput::set(float value) {
  if(DOServos::servos.setGoal(sn_, value) == true) return RES_OK;
  return RES_CMD_FAILURE;
}

float DOServoControlOutput::present() {
  return DOServos::servos.present(sn_);
}

DOServos::DOServos() {
  infoXelsSrPresent = NULL;
  infoXelsSrLoad = NULL;
  infoXelsSwGoal = NULL;
  infoXelsSwVel = NULL;
}

Result DOServos::initialize() {
  name_ = "servos";
  description_ = "Dynamixel subsystem";
  help_ = "Dynamixel Subsystem\r\n"
          "Available commands:\r\n"
          "\ttorque <servo>|all on|off           Switch torque.\r\n"
          "\tmove <servo>|all <angle>            <angle>: deg [-180.0..180.0].\r\n"
          "\tset_vel <servo>|all <val>           <val> in deg/s; 0 is infinite speed.\r\n"
          "\thome                                Home all servos, slowly.\r\n"
          "\tinfo <servo>                        Get info on <servo>.\r\n"
          "\t<ctrltableitem> <servo> [<value>]   Get or set, options: vel_limit, current_limit, profile_acc.\r\n"
          "\reboot <servo>|all                   Reboot <servo>";

  ctrlPresentPos_.addr = 0;
  ctrlPresentLoad_.addr = 0;
  ctrlGoalPos_.addr = 0;
  ctrlProfileVel_.addr = 0;
  return Subsystem::initialize();
}

Result DOServos::start(ConsoleStream* stream) {
  if (isStarted()) return RES_SUBSYS_ALREADY_STARTED;

  Runloop::runloop.excuseOverrun();

  dxl_.setPortProtocolVersion(2.0);
  unsigned int bps = 0;

  if(stream) stream->printf("Detecting Dynamixels... ");
  for (unsigned int i = 0; i < numBps; i++) {
    dxl_.begin(bpsList[i]);
    if (dxl_.scan() == false) continue;

    bps = bpsList[i];

    if(stream) stream->printf("scan successful at %dbps, enumerating up to %d...", bps, MAX_SERVO_ID);

    for (uint8_t id = 1; id <= MAX_SERVO_ID; id++) {
      if (dxl_.ping(id)) {
        uint16_t model = dxl_.getModelNumber(id);
        if(stream) stream->printf("#%d: model # %d... ", id, model);

        Servo servo;
        servo.id = id;
        servo.goal = dxl_.getPresentPosition(id);
        servo.profileVel = dxl_.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
        servo.present = dxl_.getPresentPosition(id); 
        servo.load = dxl_.readControlTableItem(ControlTableItem::PRESENT_LOAD, id);
        servo.min = dxl_.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
        servo.max = dxl_.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);
        servos_.push_back(servo);

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
            if (stream) stream->printf("Servos use differing control tables, that is not supported currently!\n");
            return RES_SUBSYS_HW_DEPENDENCY_MISSING;
          }
          item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_POSITION);
          if (item.addr != ctrlGoalPos_.addr || item.addr_length != ctrlGoalPos_.addr_length) {
            if (stream) stream->printf("Servos use differing control tables, that is not supported currently!\n");
            return RES_SUBSYS_HW_DEPENDENCY_MISSING;
          }
          item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PROFILE_VELOCITY);
          if (item.addr != ctrlProfileVel_.addr || item.addr_length != ctrlProfileVel_.addr_length) {
            if (stream) stream->printf("Servos use differing control tables, that is not supported currently!\n");
            return RES_SUBSYS_HW_DEPENDENCY_MISSING;
          }
          item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_LOAD);
          if(item.addr == 0) item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_CURRENT);
          if (item.addr != ctrlPresentLoad_.addr || item.addr_length != ctrlPresentLoad_.addr_length) {
            if (stream) stream->printf("Servos use differing control tables, that is not supported currently! (addr %d!=%d, length %d!=%d)\n", item.addr, ctrlPresentLoad_.addr, item.addr_length, ctrlPresentLoad_.addr_length);
            return RES_SUBSYS_HW_DEPENDENCY_MISSING;
          }
        }
      }
    } 

    if (stream) stream->printf("\n");
  }

  for(auto id: requiredIds_) {
    if(servoWithID(id) == NULL) {
      if (stream) stream->printf("Required servo ID %d not found!\n", id);
      return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    }
  }

  if(stream) {
    for(auto& s: servos_) {
      uint8_t err = errorStatus(s.id);
      if(err != 0x0) {
        stream->printf("Servo #%d reports error 0x%x. Consider rebooting it.\n", s.id, err);
      }
    }
  }

  // Configure servos
  if (bps != goalBps) {
    for (auto& s : servos_) {
      dxl_.torqueOff(s.id);
      if(dxl_.setBaudrate(s.id, goalBps) == false) Console::console.printfBroadcast("Failed to set baud rate on #%d to %d\n", s.id, goalBps);
    }
    delay(30);
    dxl_.begin(goalBps);
    if (dxl_.scan() == false) {
      if(stream) stream->printf("Could not rescan after switching to %dbps!\n", goalBps);
      return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    }
    bps = goalBps;
  }

  setupSyncBuffers();
  Result res = syncReadInfo(stream);
  if(res != RES_OK) return res;

  for (auto& s : servos_) {
    dxl_.setOperatingMode(s.id, OP_POSITION);
    dxl_.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, s.id, 10);
    dxl_.writeControlTableItem(ControlTableItem::DRIVE_MODE, s.id, 0);
    dxl_.torqueOn(s.id);
  }

  operationStatus_ = RES_OK;
  started_ = true;
  return RES_OK;
}

Result DOServos::stop(ConsoleStream* stream) {
  (void)stream;

  teardownSyncBuffers();

  for (auto& s : servos_) {
    dxl_.torqueOff(s.id);
  }
  while (servos_.size()) servos_.erase(servos_.begin());

  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  started_ = false;
  return RES_OK;
}

Result DOServos::step() {
  static unsigned int failcount = 0;

  if (!started_ || operationStatus_ != RES_OK) return RES_SUBSYS_NOT_STARTED;

  if (failcount > 5) {
    Console::console.printfBroadcast("Servo communication failed more than 5 times in a row. Stopping subsystem.\n");
    stop();
    failcount = 0;
    return RES_SUBSYS_COMM_ERROR;
  }

  Result res = syncReadInfo();
  if(res!=RES_OK) {
    failcount++;
    return RES_SUBSYS_COMM_ERROR;
  }

  res = syncWriteInfo();
  if(res!=RES_OK) {
    failcount++;
    return RES_SUBSYS_COMM_ERROR;
  }

  failcount = 0;
  return RES_OK;
}

Result DOServos::handleConsoleCommand(const std::vector<String>& words, ConsoleStream* stream) {
  (void)stream;
  if (words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if (words[0] == "move") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1] == "all" ? ID_ALL : words[1].toInt();
    float angle = words[2].toFloat();
    if(angle < 0 || angle > 360.0) return RES_CMD_INVALID_ARGUMENT;
    if(setGoal(id, angle)) return RES_OK;
    return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "set_vel") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1] == "all" ? ID_ALL : words[1].toInt();
    float vel = words[2].toFloat();
    if(vel < 0) return RES_CMD_INVALID_ARGUMENT;
    if(setProfileVelocity(id, vel)) return RES_OK;
    return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "home") {
    if (words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;

    return home(10.0, stream);
  }

  else if (words[0] == "torque") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    uint8_t id;
    if (words[1] == "all") id = ID_ALL;
    else id = words[1].toInt();
    return switchTorque(id, words[2] == "on" ? true : false);
  }

  else if (words[0] == "info") {
    Runloop::runloop.excuseOverrun();

    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words[1] == "all") {
      for(auto& s: servos_) printStatus(stream, s.id);
      return RES_OK;
    }
    int id = words[1].toInt();
    printStatus(stream, id);
    return RES_OK;
  }

  else if (words[0] == "reboot") {
    Runloop::runloop.excuseOverrun();

    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words[1] == "all") {
      for (auto& s : servos_) {
        if (stream) {
          stream->printf("Rebooting %d... ", s.id);
        }
        dxl_.reboot(s.id);
      }
    } else {
      uint8_t id = words[1].toInt();
      if(servoWithID(id) == NULL) return RES_CMD_INVALID_ARGUMENT;
      dxl_.reboot(id);
    }

    delay(1000);

    return RES_OK;
  }

  else {
    for(int i=0; i<strToCtrlTableLen_; i++) {
      if(words[0] == strToCtrlTable_[i].str) {
        return handleCtrlTableCommand(strToCtrlTable_[i].idx, words, stream);
      }
    }
  }

  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result DOServos::handleCtrlTableCommand(ControlTableItem::ControlTableItemIndex idx, const std::vector<String>& words, ConsoleStream* stream) {
  if (words.size() < 2 || words.size() > 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
  int id = words[1].toInt();
  if (words.size() == 2) {
    int val = (int)dxl_.readControlTableItem(idx, id);
    if(stream) stream->printf("%s=%d\n", words[0].c_str(), val);
  } else {
    int val = words[2].toInt();
    if(stream) stream->printf("Setting %s (%d) to %d\n", words[0].c_str(), (int)idx, val);
    dxl_.writeControlTableItem(idx, id, val);
    val = (int)dxl_.readControlTableItem(idx, id);
    if(stream) stream->printf("%s=%d\n", words[0].c_str(), val);
  }
  return RES_OK;
}

Result DOServos::home(float vel, ConsoleStream* stream) {
  if(stream) stream->printf("Homing servos...\n");

  Result res;
  float maxoffs = 0;
  
  res = syncReadInfo();
  if(res != RES_OK) return res;

  uint32_t rawVel = computeRawValue(vel);

  for(auto& s: servos_) {
    s.lastVel = s.profileVel;
    s.profileVel = rawVel;
  }

  res = syncWriteInfo();
  if(res != RES_OK) return res;

  // home servos
  for(auto& s: servos_) {
    s.goal = (s.max - s.min)/2; // FIXME Maybe home pos is not in the middle?
    int offs = s.present - s.goal;
    if(offs < 0) offs = -offs;
    if(offs > maxoffs) maxoffs = offs;
  }
  maxoffs = (maxoffs / 4096.0)*360.0;
  
  res = syncWriteInfo();
  if(res != RES_OK) return res;

  int d = (int)((maxoffs / vel)*1100);
  while(d > 0) {
    res = syncReadInfo();
    if(res != RES_OK) return res;
    for(auto& s: servos_) {
      if(stream) {
        stream->printf("Servo %d: Pos %d Load %d ", s.id, s.present, s.load);
      }
    }
    if(stream) stream->printf("\n");
    delay(1000);
    d-=1000;
  }
  if(stream) {
    stream->printf("Final: ");
    for(auto& s: servos_) {
      if(stream) {
        stream->printf("Servo %d: Pos %d Load %d ", s.id, s.present, s.load);
      }
    }
  }

  // restore old profile velocities
  for(auto& s: servos_) {
    s.profileVel = s.lastVel;
  }
  res = syncWriteInfo();
  if(res != RES_OK) return res;

  return RES_OK;
}


Result DOServos::switchTorque(uint8_t id, bool onoff) {
  if (operationStatus_ != RES_OK) return operationStatus_;
  if(id == ID_ALL) {
    for(auto& s: servos_) {
      if(onoff) dxl_.torqueOn(s.id);
      else dxl_.torqueOff(s.id);
    }
  } else {
    if(onoff) dxl_.torqueOn(id);
    else dxl_.torqueOff(id);
  }
  
  return RES_OK;
}

bool DOServos::isTorqueOn(uint8_t id) {
  return dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id);
}

void DOServos::printStatus(ConsoleStream* stream, int id) {
  if (!started_) {
    stream->printf("Servo subsystem not started.\n");
    return;
  }

  Servo *s = servoWithID(id);
  if(s == NULL) {
    stream->printf("Servo #%d does not exist.\n", id);
    return;
  }

  stream->printf("Servo #%d: ", id);
  if (dxl_.ping(id)) {
    stream->printf("model #%d, present: %f° (%d), goal: %f° (%d), ", dxl_.getModelNumber(id), present(id), s->present, goal(id), s->goal);
    
    stream->printf("range: [%d..%d], offset: %d, invert: %d, ", s->min, s->max, s->offset, dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id) & 0x1);

    stream->printf("hw err: $%x", dxl_.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id));
    stream->printf(", alarm shutdown: $%x", dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, id));
    stream->printf(", torque limit: %d", (int)dxl_.readControlTableItem(ControlTableItem::TORQUE_LIMIT, id));
    stream->printf(", max torque: %d", (int)dxl_.readControlTableItem(ControlTableItem::MAX_TORQUE, id));
    stream->printf(", torque enabled: %d", (int)dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id));
    stream->printf(", drive mode: %d", (int)dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id));
    stream->printf(", profile vel: %d", (int)dxl_.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id));
    stream->printf(", profile acc: %d", (int)dxl_.readControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id));
    stream->printf("\n");
  } else {
    stream->printf("#%d not found! ", id);
  }
}

bool DOServos::setRange(uint8_t id, float min, float max, ValueType t) {
  Servo *s = servoWithID(id);
  if(s == NULL) return false;
  
  if(min > max) {
    float t = min;
    max = min;
    min = t;
  }

  s->min = computeRawValue(min, t);
  s->max = computeRawValue(max, t);
  s->goal = constrain(s->goal, s->min, s->max);

  return true;
}

bool DOServos::setOffset(uint8_t id, float offset, ValueType t) {
  Servo *s = servoWithID(id);
  if(s == NULL) return false;

  if(t == VALUE_DEGREE) {
    s->offset = (offset * 4096.0) / 360.0;
  } else {
    s->offset = offset;
  }
  return true;
}

bool DOServos::setInverted(uint8_t id, bool invert) {
  uint8_t dm = dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  if(invert) dm |= 0x1;
  else dm &= ~0x1;
  dxl_.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, dm);

  return true;
}

bool DOServos::inverted(uint8_t id) {
  return dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id) & 0x1;
}

bool DOServos::setGoal(uint8_t id, float goal, ValueType t) {
  if (isnan(goal)) return false;

  if (id == ID_ALL) {
    for (auto& s : servos_) {
      if (setGoal(s.id, goal, t) != true) return false;
    }
    return true;
  }

  Servo *s = servoWithID(id);
  if (s == NULL) return false;
  uint32_t g = computeRawValue(goal, t) + s->offset;
  s->goal = constrain(g, s->min, s->max);

  swGoalInfos.is_info_changed = true;

  return true;
}

bool DOServos::setProfileVelocity(uint8_t id, float vel, ValueType t) {
  if(isnan(vel) || vel<0) return false;

  if (id == ID_ALL) {
    for (auto& s : servos_) {
      if (setProfileVelocity(s.id, vel, t) != true) return false;
    }
    return true;
  }

  Servo *s = servoWithID(id);
  if(s == NULL) return false;
  uint32_t v = vel;
  if(t == VALUE_DEGREE) {
    v = (uint32_t)((vel / 6.0f) / 0.229); // deg/s to rev/min, and then multiply with steps of 0.229
  }

  Console::console.printfBroadcast("Setting profile velocity to %d", v);
  s->profileVel = v;
  swVelInfos.is_info_changed = true;

  return true;
}

uint32_t DOServos::computeRawValue(float val, ValueType t) {
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
    Console::console.printfBroadcast("Capping %d (computed from %f) to [0..4096]!", retval, val);
    retval = constrain(retval, 0, 4096);
  }

  return (uint32_t)retval;
}

bool DOServos::setProfileAcceleration(uint8_t id, uint32_t val) {
  dxl_.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, val);
  return true;
}

float DOServos::goal(uint8_t id, ValueType t) {
  Servo *s = servoWithID(id);
  if (s == NULL) return 0.0f;

  if (t == VALUE_DEGREE)
    return 360.0 * (s->goal / 4096.0);
  else
    return s->goal;
}

float DOServos::present(uint8_t id, ValueType t) {
  Servo *s = servoWithID(id);
  if (s == NULL) return 0.0f;
  if (t == VALUE_DEGREE) return (s->present / 4096.0) * 360.0;
  else return s->present;
}

float DOServos::load(uint8_t id) {
  if (operationStatus_ != RES_OK) return 0.0f;

  Servo *s = servoWithID(id);
  if (s == NULL) return 0.0f;
  return s->load;
}

uint8_t DOServos::errorStatus(uint8_t id) {
  return dxl_.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);
}

bool DOServos::loadShutdownEnabled(uint8_t id) {
  return dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, id) & (1<<5);
}

void DOServos::setLoadShutdownEnabled(uint8_t id, bool yesno) {
  uint8_t shutdown = dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, id);
  if(yesno) shutdown |= (1<<5);
  else shutdown &= ~(1<<5);
  dxl_.writeControlTableItem(ControlTableItem::SHUTDOWN, id, shutdown);
}

DOServos::Servo* DOServos::servoWithID(uint8_t id) {
  for(auto& s: servos_) {
    if(s.id == id) return &s;
  }
  return NULL;
}

void DOServos::setupSyncBuffers() {
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
    infoXelsSrPresent[i].id = s.id;
    infoXelsSrPresent[i].p_recv_buf = (uint8_t*)&(s.present);
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
    infoXelsSrLoad[i].id = s.id;
    infoXelsSrLoad[i].p_recv_buf = (uint8_t*)&(s.load);
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
    infoXelsSwGoal[i].id = s.id;
    infoXelsSwGoal[i].p_data = (uint8_t*)&(s.goal);
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
    infoXelsSwVel[i].id = s.id;
    infoXelsSwVel[i].p_data = (uint8_t*)&(s.profileVel);
    i++;
  }
  swVelInfos.xel_count = servos_.size();
  swVelInfos.is_info_changed = true;
}

void DOServos::teardownSyncBuffers() {
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

Result DOServos::syncReadInfo(ConsoleStream *stream) {
  uint8_t recv_cnt;
  
  recv_cnt = dxl_.syncRead(&srPresentInfos);
  if(recv_cnt != servos_.size()) {
    if(stream) stream->printf("Receiving position failed!\n");
    else Console::console.printfBroadcast("Receiving position failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  recv_cnt = dxl_.syncRead(&srLoadInfos);
  if(recv_cnt != servos_.size()) {
    if(stream) stream->printf("Receiving initial load failed!\n");
    else Console::console.printfBroadcast("Receiving initial load failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  return RES_OK;
}

Result DOServos::syncWriteInfo(ConsoleStream* stream) {
  if(dxl_.syncWrite(&swGoalInfos) == false) {
    if(stream) stream->printf("Sending servo position goal failed!\n");
    else Console::console.printfBroadcast("Sending servo position goal failed!\n");
    return RES_SUBSYS_COMM_ERROR;
  }

  if(dxl_.syncWrite(&swVelInfos) == false) {
    if(stream) stream->printf("Sending servo profile velocity failed!\n");
    else Console::console.printfBroadcast("Sending servo profile velocity failed!\n");
    return RES_SUBSYS_COMM_ERROR;
  }

  return RES_OK;
}
