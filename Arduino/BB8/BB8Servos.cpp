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

BB8Servos::BB8Servos() {
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

  memset(servos_, 0, sizeof(servos_));
  BB8ServoPower::power.initialize();
  return Subsystem::initialize();
}

Result BB8Servos::start(ConsoleStream *stream) {
  if(isStarted()) return RES_SUBSYS_ALREADY_STARTED;

  int servosFound = 0;

  BB8ServoPower::power.switchOnOff(true);
  delay(500);

  dxl_.setPortProtocolVersion(2.0);
  dxl_.begin(DYNAMIXEL_BPS);
  dxl_.scan();
  for (int i = 1; i <= 4; i++) {
    if (dxl_.ping(i)) {
      if(stream) stream->print(String("#") + i + ": model #" + dxl_.getModelNumber(i) + "...");
      dxl_.setOperatingMode(i, OP_POSITION);
      dxl_.writeControlTableItem(ControlTableItem::SHUTDOWN, i, 0x14);  // disable overload shutdown -- potentially dangerous
      dxl_.torqueOn(i);

      servos_[i].id = i;
      servos_[i].available = true;
      servos_[i].setpoint = servos_[i].current = dxl_.getPresentPosition(i, UNIT_DEGREE);
      servos_[i].speed = 60.0; // deg/s

      servosFound++;
    } else {
      if(stream) stream->print(String("#") + i + "not found...");
    }
  }
  if(stream) stream->println();

  if (servosFound == 0) return RES_SUBSYS_HW_DEPENDENCY_MISSING;

  operationStatus_ = RES_OK;
  started_ = true;
  return RES_OK;
}

Result BB8Servos::stop(ConsoleStream *stream) {
  (void)stream;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  started_ = false;
  return RES_OK;
}

Result BB8Servos::step() {
  if (!started_ || operationStatus_ != RES_OK) return RES_SUBSYS_NOT_STARTED;

  for (int i = 1; i <= 4; i++) {
    if (servos_[i].available) {
      if(servos_[i].current == servos_[i].setpoint) continue;
      float maxstep = servos_[i].speed / (1000 / Runloop::runloop.cycleTime());
      float delta = servos_[i].setpoint - servos_[i].current;
      if(fabs(delta) > maxstep) {
        servos_[i].current += delta<0 ? -maxstep : maxstep;
        setPosition(i, servos_[i].current);
      } else {
        servos_[i].current = servos_[i].setpoint;
        setPosition(i, servos_[i].current);
      }
    }
  }
  return RES_OK;
}

Result BB8Servos::handleConsoleCommand(const std::vector<String> &words, ConsoleStream *stream) {
  (void)stream;
  if (words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;
  if (words[0] == "move") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    int id = words[1].toInt();
    if (id < 1 || id > 4) return RES_CMD_INVALID_ARGUMENT;
    if (servos_[id].available == false) return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    float angle = words[2].toFloat();
    if (angle < 0 || angle > 360.0) return RES_CMD_INVALID_ARGUMENT;
    servos_[id].setpoint = angle;
    return RES_OK;
  } else if (words[0] == "torque") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if (words[1] == "all") {
      for (int id = 1; id <= 4; id++) switchTorque(id, words[2] == "on" ? true : false);
      return RES_OK;
    } else {
      int id = words[1].toInt();
      if (id < 1 || id > 4) return RES_CMD_INVALID_ARGUMENT;
      if (servos_[id].available == false) return RES_SUBSYS_HW_DEPENDENCY_MISSING;
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
    if (servos_[id].available == false) return RES_SUBSYS_HW_DEPENDENCY_MISSING;
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
  if (id < 1 || id > 4) return RES_CMD_INVALID_ARGUMENT;
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

Result BB8Servos::switchTorque(uint8_t servo, bool onoff) {
  if (operationStatus_ != RES_OK || servo < 1 || servo > 4) return RES_CMD_INVALID_ARGUMENT;
  if (onoff) dxl_.torqueOn(servo);
  else dxl_.torqueOff(servo);
  return RES_OK;
}

Result BB8Servos::switchTorqueAll(bool onoff) {
  if(operationStatus_ != RES_OK) return RES_CMD_INVALID_ARGUMENT;
  for(int i=1; i<=4; i++) {
    Result retval = switchTorque(i, onoff);
    if(retval != RES_OK) return retval;
  }
  return RES_OK;
}

bool BB8Servos::isTorqueOn(uint8_t servo) {
  if (operationStatus_ != RES_OK || servo < 1 || servo > 4) return false;
  return dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, servo);
}

void BB8Servos::printStatus(ConsoleStream *stream, int id) {
  if (!started_) {
    stream->println("Servo subsystem not started.");
    return;
  }

  stream->print("Servo #");
  stream->print(id);
  stream->print(": ");
  if (dxl_.ping(id)) {
    stream->print("model #");
    stream->print(dxl_.getModelNumber(id));
    stream->print(", ");
    stream->print("pos: ");
    stream->print(getPresentPosition(id));
    stream->print("°, ");
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

  for (int i = 1; i <= 4; i++) {
    if(hard) dxl_.setGoalPosition(i, 180.0 + servolimits[i - 1].offset, UNIT_DEGREE);
    else setSetpoint(i, 180.0);
  }

  return RES_OK;
}

bool BB8Servos::setSpeed(uint8_t servo, float speed) {
  if(servo <= 0 || servo > 4) return false;
  servos_[servo].speed = speed;
}
  
bool BB8Servos::setSetpoint(uint8_t servo, float setpoint) {
  if(servo <= 0 || servo > 4) return false;
  servos_[servo].setpoint = setpoint;
  return true;
}


bool BB8Servos::setPosition(uint8_t servo, float goal) {
  if (operationStatus_ != RES_OK) return false;

  if (servo <= 0 || servo > 4) return false;

  goal += servolimits[servo - 1].offset;
  if (goal < servolimits[servo - 1].min) goal = servolimits[servo - 1].min;
  else if (goal > servolimits[servo - 1].max) goal = servolimits[servo - 1].max;

#if 0
  Serial.print("Setting goal position of servo ");
  Serial.print(servo);
  Serial.print(" to ");
  Serial.println(goal, 2);
#endif
  dxl_.setGoalPosition(servo, goal, UNIT_DEGREE);
  return true;
}

float BB8Servos::getPresentPosition(uint8_t servo) {
  if (operationStatus_ != RES_OK) return 0.0f;

  if (servo <= 0 || servo > 4) return false;
  return dxl_.getPresentPosition(servo, UNIT_DEGREE);
}