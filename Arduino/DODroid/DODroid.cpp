#include "DODroid.h"
#include "DOConfig.h"
#include "DOIMU.h"
#include "DOBattStatus.h"

DODroid DODroid::droid;
DODroid::Params DODroid::params_;

DODroid::DODroid():
  leftMotor(P_LEFT_PWMA, P_LEFT_PWMB, P_LEFT_ENCA, P_LEFT_ENCB),
  rightMotor(P_RIGHT_PWMA, P_RIGHT_PWMB, P_RIGHT_ENCA, P_RIGHT_ENCB) 
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
  params_.driveKp = 0.001;
  params_.driveKi = 0.0002; 
  params_.driveKd = 0;
  params_.driveAccel = 1;

  addParameter("drive_kp", "Proportional constant for drive PID controller", params_.driveKp, 0, INT_MAX);
  addParameter("drive_ki", "Integrative constant for drive PID controller", params_.driveKi, 0, INT_MAX);
  addParameter("drive_kd", "Derivative constant for drive PID controller", params_.driveKd, 0, INT_MAX);
  addParameter("drive_accel", "Acceleration for drive motors", params_.driveAccel, 0, 100);

  return Subsystem::initialize();
}

Result DODroid::start(ConsoleStream* stream) {
  started_ = true;
  operationStatus_ = RES_OK;

  DOIMU::imu.begin();
  DOBattStatus::batt.begin();

  leftMotor.setDirectionAndSpeed(DCMotor::DCM_IDLE, 0);
  leftMotor.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  leftMotor.setSpeedControlParameters(params_.driveKp, params_.driveKi, params_.driveKd);
  rightMotor.setDirectionAndSpeed(DCMotor::DCM_IDLE, 0);
  rightMotor.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  rightMotor.setSpeedControlParameters(params_.driveKp, params_.driveKi, params_.driveKd);

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

  leftMotor.update();
  rightMotor.update();
  DOBattStatus::batt.updateCurrent();
  DOBattStatus::batt.updateVoltage();

  Serial.println(String("Voltage: ") + DOBattStatus::batt.voltage());
  
  fillAndSendStatusPacket();
  
  return RES_OK;
}

void DODroid::printStatus(ConsoleStream *stream) {
  Serial.println(String("Encoders: R") + rightMotor.getPresentPosition(EncoderMotor::UNIT_TICKS) + " L" + leftMotor.getPresentPosition(EncoderMotor::UNIT_TICKS));
}

Result DODroid::incomingPacket(const Packet& packet) {
  return RES_OK;
}

Result DODroid::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
  if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if(words[0] == "left") {
    if(words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;

    EncoderMotor::ControlMode mode;
    if(words[1] == "pwm") mode = EncoderMotor::CONTROL_PWM;
    else if(words[1] == "position") mode = EncoderMotor::CONTROL_POSITION;
    else if(words[1] == "speed") mode = EncoderMotor::CONTROL_SPEED;
    else return RES_CMD_INVALID_ARGUMENT;

    float sp = words[2].toFloat();    

    leftMotor.setEnabled(true);
    leftMotor.setGoal(sp, mode);
    return RES_OK;
  }

  if(words[0] == "right") {
    if(words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    
    EncoderMotor::ControlMode mode;
    if(words[1] == "pwm") mode = EncoderMotor::CONTROL_PWM;
    else if(words[1] == "position") mode = EncoderMotor::CONTROL_POSITION;
    else if(words[1] == "speed") mode = EncoderMotor::CONTROL_SPEED;
    else return RES_CMD_INVALID_ARGUMENT;

    float sp = words[2].toFloat();    

    rightMotor.setEnabled(true);
    rightMotor.setGoal(sp, mode);
    return RES_OK;
  }

  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result DODroid::setParameterValue(const String& name, const String& stringVal) {
  Result retval = Subsystem::setParameterValue(name, stringVal);
  if(retval != RES_OK) return retval;

  leftMotor.setSpeedControlParameters(params_.driveKp, params_.driveKi, params_.driveKd);
  leftMotor.setAcceleration(params_.driveAccel, EncoderMotor::UNIT_TICKS);
  rightMotor.setSpeedControlParameters(params_.driveKp, params_.driveKi, params_.driveKd);
  rightMotor.setAcceleration(params_.driveAccel, EncoderMotor::UNIT_TICKS);
}

Result DODroid::fillAndSendStatusPacket() {
  LargeStatusPacket packet;

  packet.timestamp = Runloop::runloop.millisSinceStart() / 1000.0;
  packet.droidType = DroidType::DROID_DO;
  strncpy(packet.droidName, DROID_NAME, sizeof(packet.droidName));

  packet.drive[0] = leftMotor.getDriveControlState();
  packet.drive[1] = rightMotor.getDriveControlState();
  packet.drive[2].errorState = ERROR_NOT_PRESENT;

  packet.imu[0] = DOIMU::imu.getIMUState();
  packet.imu[1].errorState = ERROR_NOT_PRESENT;
  packet.imu[2].errorState = ERROR_NOT_PRESENT;

  packet.battery[0] = DOBattStatus::batt.getBatteryState();
  packet.battery[1].errorState = ERROR_NOT_PRESENT;
  packet.battery[2].errorState = ERROR_NOT_PRESENT;

  WifiServer::server.broadcastUDPPacket((const uint8_t*)&packet, sizeof(packet));
  //Console::console.printlnBroadcast(String("Sent packet of size ") + sizeof(packet));

  return RES_OK;
}


