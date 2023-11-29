#include "DODroid.h"
#include "DOConfig.h"
#include "DOIMU.h"
#include "DOBattStatus.h"

DODroid DODroid::droid;
DODroid::Params DODroid::params_ = {
  .speedKp = SPEED_KP,
  .speedKi = SPEED_KI,
  .speedKd = SPEED_KD,
  .posKp = POS_KP,
  .posKi = POS_KI,
  .posKd = POS_KD,
  .driveAccel = 1
};

DODroid::DODroid():
  leftMotor(P_LEFT_PWMA, P_LEFT_PWMB), 
  leftEncoder(P_LEFT_ENCA, P_LEFT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
  leftController(leftEncoder, leftMotor), 
  rightMotor(P_RIGHT_PWMA, P_RIGHT_PWMB), 
  rightEncoder(P_RIGHT_ENCA, P_RIGHT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
  rightController(rightEncoder, rightMotor)
{
  pinMode(PULL_DOWN_A0, OUTPUT);
  digitalWrite(PULL_DOWN_A0, LOW);

  name_ = "d-o";
  description_ = "D-O Main System";
  help_ = "Available commands:\r\n"\
"\tstatus\tPrint Status\r\n"\
"\tleft pwm|position|speed <val>\tSet left drive motor setpoint\r\n"\
"\tright pwm|position|speed <val>\tSet right drive motor setpoint";

  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
}

Result DODroid::initialize() {
  addParameter("speed_kp", "Proportional constant for speed PID controller", params_.speedKp, 0, INT_MAX);
  addParameter("speed_ki", "Integrative constant for speed PID controller", params_.speedKi, 0, INT_MAX);
  addParameter("speed_kd", "Derivative constant for speed PID controller", params_.speedKd, 0, INT_MAX);
  addParameter("pos_kp", "Proportional constant for position PID controller", params_.posKp, 0, INT_MAX);
  addParameter("pos_ki", "Integrative constant for position PID controller", params_.posKi, 0, INT_MAX);
  addParameter("pos_kd", "Derivative constant for position PID controller", params_.posKd, 0, INT_MAX);
  addParameter("drive_accel", "Acceleration for drive motors", params_.driveAccel, 0, 100);

  return Subsystem::initialize();
}

Result DODroid::start(ConsoleStream* stream) {
  started_ = true;
  operationStatus_ = RES_OK;

  DOIMU::imu.begin();
  DOBattStatus::batt.begin();

  leftMotor.set(0);
  leftEncoder.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  leftEncoder.setMode(bb::Encoder::INPUT_SPEED);
  leftEncoder.setUnit(bb::Encoder::UNIT_MILLIMETERS);
  leftController.setControlParameters(params_.speedKp, params_.speedKi, params_.speedKd);
  rightMotor.set(0);
  rightEncoder.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  rightEncoder.setMode(bb::Encoder::INPUT_SPEED);
  rightEncoder.setUnit(bb::Encoder::UNIT_MILLIMETERS);
  rightController.setControlParameters(params_.speedKp, params_.speedKi, params_.speedKd);

  return RES_OK;
}

Result DODroid::stop(ConsoleStream* stream) {
  (void) stream;
  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;

  return RES_OK;
}

Result DODroid::step() {
  DOIMU::imu.update();

  leftController.update();
  rightController.update();
  DOBattStatus::batt.updateCurrent();
  DOBattStatus::batt.updateVoltage();

  fillAndSendStatusPacket();
  
  return RES_OK;
}

void DODroid::printStatus(ConsoleStream *stream) {
  Serial.println(String("Encoders: R") + rightEncoder.presentPosition() + " L" + leftEncoder.presentPosition());
}

Result DODroid::incomingPacket(const Packet& packet) {
  return RES_OK;
}

Result DODroid::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
  if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if(words[0] == "left") {
    if(words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;

    if(words[1] == "position") {
      leftEncoder.setMode(bb::Encoder::INPUT_POSITION);
      leftController.setControlParameters(params_.posKp, params_.posKi, params_.posKd);
      leftController.setGoal(words[2].toFloat());
      return RES_OK;
    } else if(words[1] == "speed") {
      leftEncoder.setMode(bb::Encoder::INPUT_SPEED);
      leftController.setControlParameters(params_.speedKp, params_.speedKi, params_.speedKd);
      leftController.setGoal(words[2].toFloat());
      return RES_OK;
    } 

    return RES_CMD_INVALID_ARGUMENT;
  }

  if(words[0] == "right") {
    if(words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    
    if(words[1] == "position") {
      leftEncoder.setMode(bb::Encoder::INPUT_POSITION);
      leftController.setControlParameters(params_.posKp, params_.posKi, params_.posKd);
      leftController.setGoal(words[2].toFloat());
      return RES_OK;
    } else if(words[1] == "speed") {
      rightEncoder.setMode(bb::Encoder::INPUT_POSITION);
      rightController.setControlParameters(params_.posKp, params_.posKi, params_.posKd);
      rightController.setGoal(words[2].toFloat());
      return RES_OK;
    }
    return RES_CMD_INVALID_ARGUMENT;
  }

  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result DODroid::setParameterValue(const String& name, const String& stringVal) {
  Result retval = Subsystem::setParameterValue(name, stringVal);
  if(retval != RES_OK) return retval;

  if(leftEncoder.mode() == bb::Encoder::INPUT_POSITION) {
    leftController.setControlParameters(params_.posKp, params_.posKi, params_.posKd);
  } else {
    leftController.setControlParameters(params_.speedKp, params_.speedKi, params_.speedKd);
  }
  if(rightEncoder.mode() == bb::Encoder::INPUT_POSITION) {
    rightController.setControlParameters(params_.posKp, params_.posKi, params_.posKd);
  } else {
    rightController.setControlParameters(params_.speedKp, params_.speedKi, params_.speedKd);
  }
}

Result DODroid::fillAndSendStatusPacket() {
  LargeStatusPacket p;

  p.timestamp = Runloop::runloop.millisSinceStart() / 1000.0;
  p.droidType = DroidType::DROID_DO;
  strncpy(p.droidName, DROID_NAME, sizeof(p.droidName));

  float err, errI, errD, control;

  p.drive[0].errorState = ERROR_OK;
  p.drive[0].controlMode = leftEncoder.mode();
  p.drive[0].presentPWM = leftMotor.present();
  p.drive[0].presentPos = leftEncoder.presentPosition();
  p.drive[0].presentSpeed = leftEncoder.presentSpeed();
  leftController.getControlState(err, errI, errD, control);
  p.drive[0].err = err;
  p.drive[0].errI = errI;
  p.drive[0].errD = errD;
  p.drive[0].control = control;

  p.drive[1].errorState = ERROR_OK;
  p.drive[1].controlMode = rightEncoder.mode();
  p.drive[1].presentPWM = rightMotor.present();
  p.drive[1].presentPos = rightEncoder.presentPosition();
  p.drive[1].presentSpeed = rightEncoder.presentSpeed();
  rightController.getControlState(err, errI, errD, control);
  p.drive[1].err = err;
  p.drive[1].errI = errI;
  p.drive[1].errD = errD;
  p.drive[1].control = control;

  p.drive[2].errorState = ERROR_NOT_PRESENT;

  p.imu[0] = DOIMU::imu.getIMUState();
  p.imu[1].errorState = ERROR_NOT_PRESENT;
  p.imu[2].errorState = ERROR_NOT_PRESENT;

  p.battery[0] = DOBattStatus::batt.getBatteryState();
  p.battery[1].errorState = ERROR_NOT_PRESENT;
  p.battery[2].errorState = ERROR_NOT_PRESENT;

  WifiServer::server.broadcastUDPPacket((const uint8_t*)&p, sizeof(p));

  return RES_OK;
}


