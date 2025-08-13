#include <Wire.h>
#include <LibBB.h>

static uint32_t SLOW_VEL = 5;
static const uint8_t MAX_SERVO_ID = 4;

using namespace bb;

bb::Servos bb::Servos::servos;
static const unsigned int bpsList[] = { 1000000, 57600, 115200 };
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
  {"goal_velocity", ControlTableItem::GOAL_VELOCITY},
  {"goal_current", ControlTableItem::GOAL_CURRENT},
  {"pos_p_gain", ControlTableItem::POSITION_P_GAIN},
  {"pos_i_gain", ControlTableItem::POSITION_I_GAIN},
  {"pos_d_gain", ControlTableItem::POSITION_D_GAIN}
};

static const int strToCtrlTableLen_ = 10;

bb::ServoControlOutput::ServoControlOutput(uint8_t sn, float offset) {
  sn_ = sn;
  offset_ = offset;
}

bb::Result bb::ServoControlOutput::set(float value) {
  switch(Servos::servos.controlMode(sn_)) {
  case Servos::CONTROL_POSITION:
    if(Servos::servos.setGoalPos(sn_, value+offset_) == true) return RES_OK;
    break;
  case Servos::CONTROL_VELOCITY:
    if(Servos::servos.setGoalVel(sn_, value+offset_) == true) return RES_OK;
    break;
  case Servos::CONTROL_CURRENT:
    if(Servos::servos.setGoalCur(sn_, value+offset_) == true) return RES_OK;
    break;
  default:
    break;
  }
  return RES_CMD_FAILURE;
}

float bb::ServoControlOutput::present() {
  return Servos::servos.presentPos(sn_)-offset_;
}

bb::Servos::Servos() {
  infoXelsSrPresent = NULL;
  infoXelsSrLoad = NULL;
  infoXelsSwGoalPos = NULL;
  infoXelsSwGoalVel = NULL;
  infoXelsSwGoalCur = NULL;
  infoXelsSwPrfVel = NULL;
}

Result bb::Servos::initialize() {
  name_ = "servos";
  description_ = "Dynamixel subsystem";
  help_ = "Dynamixel Subsystem\r\n"
          "Available commands:\r\n"
          "\ttorque <servo>|all on|off           Switch torque.\r\n"
          "\tmove <servo>|all <angle>            <angle>: deg [-180.0..180.0]. Switches servo to pos ctrl mode.\r\n"
          "\tset_prf_vel <servo>|all <val>       <val> in deg/s; 0 is infinite speed.\r\n"
          "\tset_goal_cur <servo> <val>          <val> in mA [-3000..3000]. Switches servo to current ctrl mode.\r\n"
          "\tset_goal_vel <servo> <val>          <val> in deg/s. Switches servo to velocity ctrl mode.\r\n"
          "\thome                                Home all servos, slowly.\r\n"
          "\tinfo <servo>                        Get info on <servo>.\r\n"
          "\t<ctrltableitem> <servo> [<value>]   Get or set, options: vel_limit, current_limit, profile_acc, profile_vel, operating_mode, pos_p_gain, pos_i_gain, pos_d_gain.\r\n"
          "\reboot <servo>|all                   Reboot <servo>";

  ctrlPresentPos_.addr = 0;
  ctrlPresentLoad_.addr = 0;
  ctrlGoalPos_.addr = 0;
  ctrlProfileVel_.addr = 0;
  ctrlGoalVel_.addr = 0;
  torqueOffOnStop_ = true;
  return Subsystem::initialize();
}

Result bb::Servos::start(ConsoleStream* stream) {
  if (isStarted()) return RES_SUBSYS_ALREADY_STARTED;

  Runloop::runloop.excuseOverrun();

  dxl_.setPortProtocolVersion(2.0);
  unsigned int bps = 0;

  if(stream) stream->printf("Detecting Dynamixels... ");
  for (unsigned int i = 0; i < numBps; i++) {
    dxl_.begin(bpsList[i]);
    if (dxl_.scan() == false) {
      if(stream) stream->printf("scan failed at %dbps...", bpsList[i]);
      continue;
    }

    bps = bpsList[i];

    if(stream) stream->printf("scan successful at %dbps, enumerating up to %d...", bps, MAX_SERVO_ID);

    for (uint8_t id = 1; id <= MAX_SERVO_ID; id++) {
      if (dxl_.ping(id) == false) continue;

      uint16_t model = dxl_.getModelNumber(id);
      if(stream) stream->printf("#%d: model # %d... ", id, model);

      Servo servo;
      servo.id = id;
      servo.mode = CONTROL_POSITION;
      servo.goalPos = dxl_.getPresentPosition(id);
      servo.profileVel = dxl_.readControlTableItem(ControlTableItem::PROFILE_VELOCITY, id);
      servo.presentPos = dxl_.getPresentPosition(id); 
      servo.goalVel = 0;
      servo.goalCur = 0;
      servo.load = dxl_.readControlTableItem(ControlTableItem::PRESENT_LOAD, id);
      servo.min = dxl_.readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
      servo.max = dxl_.readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);
      servo.offset = 0;
      servos_.push_back(servo);

      if (ctrlPresentPos_.addr == 0) {
        ctrlPresentPos_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::PRESENT_POSITION);
        ctrlGoalPos_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_POSITION);
        ctrlGoalVel_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_VELOCITY);
        ctrlGoalCur_ = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_CURRENT);
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
        item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_VELOCITY);
        if (item.addr != ctrlGoalVel_.addr || item.addr_length != ctrlGoalVel_.addr_length) {
          if (stream) stream->printf("Servos use differing control tables, that is not supported currently!\n");
          return RES_SUBSYS_HW_DEPENDENCY_MISSING;
        }
        item = DYNAMIXEL::getControlTableItemInfo(model, ControlTableItem::GOAL_CURRENT);
        if (item.addr != ctrlGoalCur_.addr || item.addr_length != ctrlGoalCur_.addr_length) {
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
    break;
  }

  if (stream) stream->printf("done.\n");

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
    dxl_.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, s.id, 5);
    dxl_.writeControlTableItem(ControlTableItem::DRIVE_MODE, s.id, 0);
    dxl_.torqueOn(s.id);
  }

  operationStatus_ = RES_OK;
  started_ = true;
  return RES_OK;
}

Result bb::Servos::stop(ConsoleStream* stream) {
  (void)stream;

  teardownSyncBuffers();

  if(torqueOffOnStop_) {
    for (auto& s : servos_) {
      dxl_.torqueOff(s.id);
    }
  }
  while (servos_.size()) servos_.erase(servos_.begin());

  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  started_ = false;
  return RES_OK;
}

Result bb::Servos::step() {
  static unsigned int failcount = 0;

  if (!started_ || operationStatus_ != RES_OK) return RES_SUBSYS_NOT_STARTED;

  if (failcount > 100) {
    Console::console.printfBroadcast("Servo communication failed more than %d times in a row. Stopping subsystem.\n", failcount);
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

  for(auto& s: servos_) {
    int maxload = (int)(1193.0*0.8);
    if(s.load > maxload || s.load < -maxload) {
      Console::console.printfBroadcast("Warning - servo %d load %d exceeds 80%%!\n", s.id, s.load);
    }
  }

  failcount = 0;
  return RES_OK;
}

Result bb::Servos::handleConsoleCommand(const std::vector<String>& words, ConsoleStream* stream) {
  (void)stream;
  if (words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if (words[0] == "move") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1] == "all" ? ID_ALL : words[1].toInt();
    float angle = words[2].toFloat();
    if(angle < 0 || angle > 360.0) return RES_CMD_INVALID_ARGUMENT;
    if(controlMode(id) != CONTROL_POSITION) setControlMode(id, CONTROL_POSITION);
    if(setGoalPos(id, angle)) return RES_OK;
    return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "set_prf_vel") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1] == "all" ? ID_ALL : words[1].toInt();
    float vel = words[2].toFloat();
    if(vel < 0) return RES_CMD_INVALID_ARGUMENT;
    if(setProfileVelocity(id, vel)) return RES_OK;
    return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "set_goal_cur") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1] == "all" ? ID_ALL : words[1].toInt();
    int current = words[2].toInt();
    if(current < -3000 || current > 3000) return RES_CMD_INVALID_ARGUMENT;
    if(controlMode(id) != CONTROL_CURRENT) setControlMode(id, CONTROL_CURRENT);
    if(setGoalCur(id, current, VALUE_COOKED)) return RES_OK;
    return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "set_goal_vel") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    unsigned int id = words[1] == "all" ? ID_ALL : words[1].toInt();
    int vel = words[2].toInt();
    if(vel < -300 || vel > 300) return RES_CMD_INVALID_ARGUMENT;
    if(controlMode(id) != CONTROL_VELOCITY) setControlMode(id, CONTROL_VELOCITY);
    if(setGoalVel(id, vel)) return RES_OK;
    return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "home") {
    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;

    if(words[1] == "all") return home(ID_ALL, SLOW_VEL, 50, stream);
    else return home(words[1].toInt(), SLOW_VEL, 50, stream);
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

Result bb::Servos::handleCtrlTableCommand(ControlTableItem::ControlTableItemIndex idx, const std::vector<String>& words, ConsoleStream* stream) {
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

#define ABS(x) (((x)<0?-(x):(x)))

Result bb::Servos::home(uint8_t id, float vel, unsigned int maxLoadPercent, ConsoleStream* stream) {
  Servo *s = NULL;
  if(id != ID_ALL) {
    s = servoWithID(id);
    if(s == NULL) return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  Result res;
  int maxoffs = 0;
  uint32_t rawVel = vel / 0.229; // vel is given in units of 0.229rev/min
  unsigned int maxLoad = maxLoadPercent * 10;

  res = syncReadInfo();
  if(res != RES_OK) return res;

  // store last profile velocity, switch torque on, and set new profile velocity
  if(id == ID_ALL) {
    for(auto& s: servos_) {
      s.lastVel = s.profileVel;
      switchTorque(s.id, true);
      setProfileVelocity(s.id, rawVel, VALUE_RAW);
    }
  } else {
    s->lastVel = s->profileVel;
    switchTorque(id, true);
    setProfileVelocity(id, rawVel, VALUE_RAW);
  }
  res = syncWriteInfo();
  if(res != RES_OK) return res;

  // set goal. Can't do this in one with profile velocity setting.
  if(id == ID_ALL) {
    for(auto& s: servos_) {
      setGoalPos(s.id, s.min + (s.max - s.min)/2, VALUE_RAW); // FIXME Maybe home pos is not in the middle?
      int offs = (int)s.presentPos - (int)s.goalPos;
      if(offs < 0) offs = -offs;
      if(offs > maxoffs) maxoffs = offs;
    }
  } else {
    setGoalPos(id, s->min + (s->max - s->min)/2, VALUE_RAW);
    maxoffs = abs((int)s->presentPos - (int)s->goalPos);
  }      
  res = syncWriteInfo();
  if(res != RES_OK) return res;

  // how many ms should it take to reach the goal?
  float timeToReachGoalMS = maxoffs / ((vel * 4096.0) / 60000);

  int timeRemaining = (int)(2*timeToReachGoalMS);
  bool allReachedGoal = false;
  while(timeRemaining > 0) {
    syncReadInfo();
    allReachedGoal = true;
    if(id == ID_ALL) {
      for(auto& s: servos_) {
        int diff = abs((int)s.presentPos - (int)s.goalPos);
        // Console::console.printfBroadcast("%d Servo %d: Pos %d Goal %d Diff %d Load %d vel %d\n", timeRemaining, s.id, s.present, s.goal, diff, s.load, rawVel);
        if(diff > 10) allReachedGoal = false;
        if(abs(s.load) > maxLoad) {
          Console::console.printfBroadcast("ERROR: MAX LOAD OF %d EXCEEDED BY SERVO %d (%d)!!! SWITCHING OFF.\n",
                                           maxLoad, s.id, s.load);
          switchTorque(s.id, false);
          return RES_SUBSYS_HW_DEPENDENCY_MISSING;
        }
      }
    } else {
      int diff = abs((int)s->presentPos - (int)s->goalPos);
      // Console::console.printfBroadcast("%d Servo %d: Pos %d Offs %d Goal %d Diff %d Load %d vel %d\n", timeRemaining, s->id, s->present, s->offset, s->goal, diff, s->load, rawVel);
      if(diff > 10) allReachedGoal = false;
      if(ABS(s->load) > maxLoad) {
        Console::console.printfBroadcast("ERROR: MAX LOAD OF %d EXCEEDED BY SERVO %d (%d)!!! SWITCHING OFF.", maxLoad, s->id, s->load);
        switchTorque(s->id, false);
        return RES_SUBSYS_HW_DEPENDENCY_MISSING;
      }
    }

    if(allReachedGoal == true) break;
    delay(10);
    timeRemaining -= 10;
  }

  // restore old profile velocities
  if(id == ID_ALL) {
    for(auto& s: servos_) {
      setProfileVelocity(s.id, s.lastVel, VALUE_RAW);
    }
  } else {
    setProfileVelocity(id, s->lastVel, VALUE_RAW);
  }
  res = syncWriteInfo();
  if(res != RES_OK) return res;

  return RES_OK;
}


Result bb::Servos::switchTorque(uint8_t id, bool onoff) {
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

bool bb::Servos::isTorqueOn(uint8_t id) {
  return dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id);
}

void bb::Servos::printStatus(ConsoleStream* stream, int id) {
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
    stream->printf("model #%d, present pos: %.1f° (%d), goal pos: %.1f° (%d), goal vel: %.1f°/s (%d), goal cur: %.1fmA (%d), load: %.1f (%d), ", 
      dxl_.getModelNumber(id), presentPos(id), s->presentPos, goalPos(id), s->goalPos, goalVel(id), s->goalVel, goalCur(id), s->goalCur, load(id), s->load);
    switch(controlMode(id)) {
    case CONTROL_POSITION: stream->printf("pos ctrl mode, "); break;
    case CONTROL_VELOCITY: stream->printf("vel ctrl mode, "); break;
    case CONTROL_CURRENT: stream->printf("cur ctrl mode, "); break;
    default: stream->printf("unknonw ctrl mode, "); break;
    }
    
    stream->printf("range: [%d..%d], offset: %d, invert: %d, ", s->min, s->max, s->offset, dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id) & 0x1);

    stream->printf("hw err: $%x", dxl_.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id));
    uint8_t operatingMode = dxl_.readControlTableItem(ControlTableItem::OPERATING_MODE, id);
    stream->printf(", mode: %d", operatingMode);
    if(operatingMode == OP_CURRENT_BASED_POSITION) {
      stream->printf(", goal current: %d", dxl_.readControlTableItem(ControlTableItem::GOAL_CURRENT, id));
    }
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

bool bb::Servos::hasServoWithID(uint8_t id) {
  return servoWithID(id) != NULL;
}

bool bb::Servos::setRange(uint8_t id, float min, float max, ValueType t) {
  Servo *s = servoWithID(id);
  if(s == NULL) return false;
  
  if(min < max) {
    s->min = computeRawValue(min, t);
    s->max = computeRawValue(max, t);
  } else {
    s->max = computeRawValue(min, t);
    s->min = computeRawValue(max, t);
  }

  setGoalPos(id, constrain(s->goalPos, s->min, s->max), VALUE_RAW);

  return true;
}

bool bb::Servos::setOffset(uint8_t id, float offset, ValueType t) {
  Servo *s = servoWithID(id);
  if(s == NULL) return false;

  if(t == VALUE_DEGREE) {
    s->offset = (offset * 4096.0) / 360.0;
  } else {
    s->offset = offset;
  }
  return true;
}

bool bb::Servos::setInverted(uint8_t id, bool invert) {
  uint8_t dm = dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id);
  if(invert) dm |= 0x1;
  else dm &= ~0x1;
  dxl_.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, dm);

  return true;
}

bool bb::Servos::inverted(uint8_t id) {
  return dxl_.readControlTableItem(ControlTableItem::DRIVE_MODE, id) & 0x1;
}

bool bb::Servos::setControlMode(uint8_t id, ControlMode mode) {
  Servo *s = servoWithID(id);
  if(s == nullptr) return false;
  if(s->mode == mode) return true;

  int32_t op;

  switch(mode) {
  case CONTROL_POSITION: op = OP_POSITION; break;
  case CONTROL_VELOCITY: op = OP_VELOCITY; break;
  case CONTROL_CURRENT: op = OP_CURRENT; break;
  default:
    bb::printf("Can't switch to control mode %d\n", mode);
  }

  bool torque = dxl_.getTorqueEnableStat(s->id);
  if(torque) bb::printf("Need to switch torque off\n");
  else bb::printf("No need to switch torque\n");

  if(torque) dxl_.torqueOff(s->id);
  dxl_.writeControlTableItem(ControlTableItem::OPERATING_MODE, id, op);
  if(dxl_.readControlTableItem(ControlTableItem::OPERATING_MODE, id) == op) {
    bb::printf("Successfully changed operating mode to %d\n", op);
    s->mode = mode;
    if(torque) dxl_.torqueOn(s->id);
    return true;
  }
  bb::printf("Error changing operating mode to %d\n", op);
  return false;
}
  
bb::Servos::ControlMode bb::Servos::controlMode(uint8_t id) {
  Servo *s = servoWithID(id);
  if(s == nullptr) return CONTROL_UNKNOWN; // no such servo

  uint8_t op = dxl_.readControlTableItem(ControlTableItem::OPERATING_MODE, id);
  switch(s->mode) {
  case CONTROL_POSITION:
    if(op == OP_POSITION) return CONTROL_POSITION;
    break;
  case CONTROL_VELOCITY:
    if(op == OP_VELOCITY) return CONTROL_VELOCITY;
    break;
  case CONTROL_CURRENT:
    if(op == OP_CURRENT) return CONTROL_CURRENT;
    break;
  case CONTROL_UNKNOWN:
  default:
    return CONTROL_UNKNOWN;
    break;
  }
  bb::printf("Control mode of servo (%d) and our entry (%d) disagree!\n", op, s->mode);
  return CONTROL_UNKNOWN;
}


bool bb::Servos::setGoalPos(uint8_t id, float goalPos, ValueType t) {
  if (isnan(goalPos)) return false;

  if (id == ID_ALL) {
    for (auto& s : servos_) {
      if (setGoalPos(s.id, goalPos, t) != true) return false;
    }
    return true;
  }

  Servo *s = servoWithID(id);
  if (s == NULL) return false;
  uint32_t g = computeRawValue(goalPos, t);
  s->goalPos = constrain(g, s->min, s->max) + s->offset; // FIXME - s->offset can be negative, is this safe?
  //if(s->id == 4) Console::console.printfBroadcast("Goal: %d Min: %d Max: %d Final: %d\n", g, s->min, s->max, s->goal);
  swGoalPosInfos.is_info_changed = true;

  return true;
}

bool bb::Servos::setGoalVel(uint8_t id, float goalVel, ValueType t) {
  if (isnan(goalVel)) return false;

  if (id == ID_ALL) {
    for (auto& s : servos_) {
      if (setGoalVel(s.id, goalVel, t) != true) return false;
    }
    return true;
  }

  Servo *s = servoWithID(id);
  if (s == NULL) return false;
  int32_t g;
  if(t == VALUE_RAW) g = int32_t(goalVel);
  else { // goal vel is deg/s, convert to units of 0.229rev/min
    g = int32_t(goalVel/(6*0.229));
  }
  s->goalVel = g;
  bb::printf("Goal vel: %d\n", s->goalVel);
  swGoalVelInfos.is_info_changed = true;

  return true;
}

float bb::Servos::goalVel(uint8_t id, ValueType t) {
  Servo *s = servoWithID(id);
  if(s == NULL) return 0;
  if(t == VALUE_RAW) return s->goalVel;
  float g = s->goalVel;
  return (g*(6.0*0.229));
}

bool bb::Servos::setGoalCur(uint8_t id, float goalCur, ValueType t) {
  if (isnan(goalCur)) return false;

  if (id == ID_ALL) {
    for (auto& s : servos_) {
      if (setGoalCur(s.id, goalCur, t) != true) return false;
    }
    return true;
  }

  Servo *s = servoWithID(id);
  if (s == NULL) return false;
  int32_t g;
  if(t == VALUE_RAW) g = int16_t(goalCur);
  else { // goal cur is mA, convert to units of 2.69mA
    g = int16_t(goalCur/2.69);
  }
  s->goalCur = g;
  swGoalCurInfos.is_info_changed = true;

  return true;
}

float bb::Servos::goalCur(uint8_t id, ValueType t) {
  Servo *s = servoWithID(id);
  if(s == NULL) return 0;
  if(t == VALUE_RAW) return s->goalCur;
  float g = s->goalCur;
  return g*2.69;
}

bool bb::Servos::setProfileVelocity(uint8_t id, float vel, ValueType t) {
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

  s->profileVel = v;
  swPrfVelInfos.is_info_changed = true;

  return true;
}

uint32_t bb::Servos::computeRawValue(float val, ValueType t) {
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

bool bb::Servos::setProfileAcceleration(uint8_t id, uint32_t val) {
  dxl_.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, val);
  return true;
}

float bb::Servos::goalPos(uint8_t id, ValueType t) {
  Servo *s = servoWithID(id);
  if (s == NULL) return 0.0f;

  if (t == VALUE_DEGREE)
    return 360.0 * (s->goalPos / 4096.0);
  else
    return s->goalPos;
}

float bb::Servos::presentPos(uint8_t id, ValueType t) {
  Servo *s = servoWithID(id);
  if (s == NULL) return 0.0f;
  if (t == VALUE_DEGREE) return (s->presentPos / 4096.0) * 360.0;
  else return s->presentPos;
}

float bb::Servos::load(uint8_t id) {
  if (operationStatus_ != RES_OK) return 0.0f;

  Servo *s = servoWithID(id);
  if (s == NULL) return 0.0f;
  return s->load;
}

uint8_t bb::Servos::errorStatus(uint8_t id) {
  return dxl_.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id);
}

bool bb::Servos::loadShutdownEnabled(uint8_t id) {
  return dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, id) & (1<<5);
}

void bb::Servos::setLoadShutdownEnabled(uint8_t id, bool yesno) {
  uint8_t shutdown = dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, id);
  if(yesno) shutdown |= (1<<5);
  else shutdown &= ~(1<<5);
  dxl_.writeControlTableItem(ControlTableItem::SHUTDOWN, id, shutdown);
}

bool bb::Servos::setPIDValues(uint8_t id, uint16_t kp, uint16_t ki, uint16_t kd) {
  dxl_.writeControlTableItem(ControlTableItem::P_GAIN, id, kp);
  dxl_.writeControlTableItem(ControlTableItem::I_GAIN, id, ki);
  dxl_.writeControlTableItem(ControlTableItem::D_GAIN, id, kd);
  return true;
}


bb::Servos::Servo* bb::Servos::servoWithID(uint8_t id) {
  for(auto& s: servos_) {
    if(s.id == id) return &s;
  }
  return NULL;
}

void bb::Servos::setupSyncBuffers() {
  unsigned int i;

  if (!servos_.size()) return;
  if (infoXelsSrPresent != NULL || infoXelsSrLoad != NULL || infoXelsSwGoalPos != NULL || infoXelsSwPrfVel != NULL ||
      infoXelsSwGoalVel != NULL || infoXelsSwGoalCur != NULL) {
    bb::printf("Error - servo buffers already set up\n");
    return;
  }

  // Sync buffer for reading present position
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
    infoXelsSrPresent[i].p_recv_buf = (uint8_t*)&(s.presentPos);
    i++;
  }
  srPresentInfos.xel_count = servos_.size();
  srPresentInfos.is_info_changed = true;

  // Sync buffer for reading present load
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

  // Sync buffer for writing goal position
  swGoalPosInfos.packet.p_buf = NULL;
  swGoalPosInfos.packet.is_completed = false;
  swGoalPosInfos.addr = ctrlGoalPos_.addr;
  swGoalPosInfos.addr_length = ctrlGoalPos_.addr_length;
  infoXelsSwGoalPos = new DYNAMIXEL::XELInfoSyncWrite_t[servos_.size()];
  swGoalPosInfos.p_xels = infoXelsSwGoalPos;
  i = 0;
  for (auto& s : servos_) {
    infoXelsSwGoalPos[i].id = s.id;
    infoXelsSwGoalPos[i].p_data = (uint8_t*)&(s.goalPos);
    i++;
  }
  swGoalPosInfos.xel_count = servos_.size();
  swGoalPosInfos.is_info_changed = true;

  // Sync buffer for writing goal velocity
  swGoalVelInfos.packet.p_buf = NULL;
  swGoalVelInfos.packet.is_completed = false;
  swGoalVelInfos.addr = ctrlGoalVel_.addr;
  swGoalVelInfos.addr_length = ctrlGoalVel_.addr_length;
  infoXelsSwGoalVel = new DYNAMIXEL::XELInfoSyncWrite_t[servos_.size()];
  swGoalVelInfos.p_xels = infoXelsSwGoalVel;
  i = 0;
  for (auto& s : servos_) {
    infoXelsSwGoalVel[i].id = s.id;
    infoXelsSwGoalVel[i].p_data = (uint8_t*)&(s.goalVel);
    i++;
  }
  swGoalVelInfos.xel_count = servos_.size();
  swGoalVelInfos.is_info_changed = true;

  // Sync buffer for writing goal current
  swGoalCurInfos.packet.p_buf = NULL;
  swGoalCurInfos.packet.is_completed = false;
  swGoalCurInfos.addr = ctrlGoalCur_.addr;
  swGoalCurInfos.addr_length = ctrlGoalCur_.addr_length;
  infoXelsSwGoalCur = new DYNAMIXEL::XELInfoSyncWrite_t[servos_.size()];
  swGoalCurInfos.p_xels = infoXelsSwGoalCur;
  i = 0;
  for (auto& s : servos_) {
    infoXelsSwGoalCur[i].id = s.id;
    infoXelsSwGoalCur[i].p_data = (uint8_t*)&(s.goalCur);
    i++;
  }
  swGoalCurInfos.xel_count = servos_.size();
  swGoalCurInfos.is_info_changed = true;

  // Sync buffer for writing profile velocity -- not sure if needed?
  swPrfVelInfos.packet.p_buf = NULL;
  swPrfVelInfos.packet.is_completed = false;
  swPrfVelInfos.addr = ctrlProfileVel_.addr;
  swPrfVelInfos.addr_length = ctrlProfileVel_.addr_length;
  infoXelsSwPrfVel = new DYNAMIXEL::XELInfoSyncWrite_t[servos_.size()];
  swPrfVelInfos.p_xels = infoXelsSwPrfVel;
  i = 0;
  for (auto& s : servos_) {
    infoXelsSwPrfVel[i].id = s.id;
    infoXelsSwPrfVel[i].p_data = (uint8_t*)&(s.profileVel);
    i++;
  }
  swPrfVelInfos.xel_count = servos_.size();
  swPrfVelInfos.is_info_changed = true;
}

void bb::Servos::teardownSyncBuffers() {
  delete infoXelsSrPresent;
  infoXelsSrPresent = NULL;
  delete infoXelsSrLoad;
  infoXelsSrLoad = NULL;
  delete infoXelsSwGoalPos;
  infoXelsSwGoalPos = NULL;
  delete infoXelsSwPrfVel;
  infoXelsSwPrfVel = NULL;
  srPresentInfos.p_xels = NULL;
  srPresentInfos.xel_count = 0;
  srLoadInfos.p_xels = NULL;
  srLoadInfos.xel_count = 0;
  swGoalPosInfos.p_xels = NULL;
  swGoalPosInfos.xel_count = 0;
  swPrfVelInfos.p_xels = NULL;
  swPrfVelInfos.xel_count = 0;
}

Result bb::Servos::syncReadInfo(ConsoleStream *stream) {
  uint8_t recv_cnt;
  
  recv_cnt = dxl_.syncRead(&srPresentInfos);
  if(recv_cnt != servos_.size()) {
    if(stream) stream->printf("servo: Receiving position failed!\n");
    else Console::console.printfBroadcast("servo: Receiving position failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  recv_cnt = dxl_.syncRead(&srLoadInfos);
  if(recv_cnt != servos_.size()) {
    if(stream) stream->printf("servo: Receiving initial load failed!\n");
    else Console::console.printfBroadcast("servo: Receiving initial load failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  return RES_OK;
}

Result bb::Servos::syncWriteInfo(ConsoleStream* stream) {
  if(dxl_.syncWrite(&swPrfVelInfos) == false) {
    if(stream) stream->printf("servo: Sending servo profile velocity failed!\n");
    else Console::console.printfBroadcast("servo: Sending servo profile velocity failed!\n");
    return RES_SUBSYS_COMM_ERROR;
  }

  if(swGoalPosInfos.is_info_changed == true) {
    if(dxl_.syncWrite(&swGoalPosInfos) == false) {
      if(stream) stream->printf("servo: Sending servo position goal failed!\n");
      else Console::console.printfBroadcast("servo: Sending servo position goal failed!\n");
      return RES_SUBSYS_COMM_ERROR;
    }
  }

  if(swGoalVelInfos.is_info_changed == true) {
    if(dxl_.syncWrite(&swGoalVelInfos) == false) {
      if(stream) stream->printf("servo: Sending servo velocity goal failed!\n");
      else Console::console.printfBroadcast("servo: Sending servo velocity goal failed!\n");
      return RES_SUBSYS_COMM_ERROR;
    }
    bb::printf("Successfully sent goal vel infos\n");
  }

  if(swGoalCurInfos.is_info_changed == true) {
    if(dxl_.syncWrite(&swGoalCurInfos) == false) {
      if(stream) stream->printf("servo: Sending servo current goal failed!\n");
      else Console::console.printfBroadcast("servo: Sending servo current goal failed!\n");
      return RES_SUBSYS_COMM_ERROR;
    }
    bb::printf("Successfully sent goal current infos\n");
  }

  return RES_OK;
}
