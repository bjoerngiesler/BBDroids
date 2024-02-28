#include "DODroid.h"
#include "DOConfig.h"
#include "DOIMU.h"
#include "DOBattStatus.h"
#include "DOServos.h"

DODroid DODroid::droid;
DODroid::Params DODroid::params_ = {
  .balKp = BAL_KP,
  .balKi = BAL_KD,
  .balKd = BAL_KD,
  .speedKp = SPEED_KP,
  .speedKi = SPEED_KI,
  .speedKd = SPEED_KD,
  .posKp = POS_KP,
  .posKi = POS_KI,
  .posKd = POS_KD,
  .driveAccel = 1
};

DODroid::DODroid():
  leftMotor_(P_LEFT_PWMA, P_LEFT_PWMB), 
  leftEncoder_(P_LEFT_ENCA, P_LEFT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
  rightMotor_(P_RIGHT_PWMA, P_RIGHT_PWMB), 
  rightEncoder_(P_RIGHT_ENCA, P_RIGHT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
  motorsOK_(false)
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
  addParameter("bal_kp", "Proportional constant for balance PID controller", params_.balKp, -INT_MAX, INT_MAX);
  addParameter("bal_ki", "Integrative constant for balance PID controller", params_.balKi, -INT_MAX, INT_MAX);
  addParameter("bal_kd", "Derivative constant for balance PID controller", params_.balKd, -INT_MAX, INT_MAX);
  addParameter("speed_kp", "Proportional constant for speed PID controller", params_.speedKp, 0, INT_MAX);
  addParameter("speed_ki", "Integrative constant for speed PID controller", params_.speedKi, 0, INT_MAX);
  addParameter("speed_kd", "Derivative constant for speed PID controller", params_.speedKd, 0, INT_MAX);
  addParameter("pos_kp", "Proportional constant for position PID controller", params_.posKp, 0, INT_MAX);
  addParameter("pos_ki", "Integrative constant for position PID controller", params_.posKi, 0, INT_MAX);
  addParameter("pos_kd", "Derivative constant for position PID controller", params_.posKd, 0, INT_MAX);
  addParameter("drive_accel", "Acceleration for drive motors", params_.driveAccel, 0, 100);

  balanceInput_ = new DOIMUControlInput(DOIMUControlInput::IMU_PITCH);
  driveOutput_ = new DODriveControlOutput(leftMotor_, rightMotor_);
  balanceController_ = new PIDController(*balanceInput_, *driveOutput_);

  return Subsystem::initialize();
}

Result DODroid::start(ConsoleStream* stream) {
  DOIMU::imu.begin();
  DOBattStatus::batt.begin();

  operationStatus_ = selfTest();
  if(operationStatus_ != RES_OK) return operationStatus_;

  leftMotor_.set(0);
  leftEncoder_.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  leftEncoder_.setMode(bb::Encoder::INPUT_SPEED);
  leftEncoder_.setUnit(bb::Encoder::UNIT_MILLIMETERS);
  rightMotor_.set(0);
  rightEncoder_.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  rightEncoder_.setMode(bb::Encoder::INPUT_SPEED);
  rightEncoder_.setUnit(bb::Encoder::UNIT_MILLIMETERS);

  balanceController_->setControlParameters(params_.balKp, params_.balKi, params_.balKd);

  started_ = true;
  operationStatus_ = RES_OK;
  return operationStatus_;
}

Result DODroid::stop(ConsoleStream* stream) {
  (void) stream;
  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;

  return RES_OK;
}

Result DODroid::step() {
  if(!DOIMU::imu.available() || !DOBattStatus::batt.available()) {
    fillAndSendStatePacket();
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  if((Runloop::runloop.getSequenceNumber() % 100) == 0) {
    DOBattStatus::batt.updateVoltage();
  }
  
  DOIMU::imu.update();

  if(motorsOK_) {
    balanceController_->update();
    float err, errI, errD, control;
    balanceController_->getControlState(err, errI, errD, control);
    //Console::console.printfBroadcast("Balance controller: %f %f %f %f\n", err, errI, errD, control);
  }

  fillAndSendStatePacket();
  
  return RES_OK;
}

void DODroid::printStatus(ConsoleStream *stream) {
  if(!stream) return;
  stream->printf("%s: %s", name_, started_ ? "started" : "not started");
  if(!started_) return;
  
  stream->printf(", status: %s", errorMessage(operationStatus_));
  
  stream->printf(", batt: ");
  if(DOBattStatus::batt.available()) {
    stream->printf("%fV %fmA", DOBattStatus::batt.voltage(), DOBattStatus::batt.current());
  } else {
    stream->printf("not available");
  }

  stream->printf(", servos: %s", DOServos::servos.isStarted() ? "OK" : "not started");

  stream->printf(", motors: ");
  leftEncoder_.update();
  rightEncoder_.update();
  if(motorsOK_) stream->printf(" OK, encoders: R%.1f L%.1f", rightEncoder_.presentPosition(), leftEncoder_.presentPosition());
  else stream->printf("failure");

  stream->printf("\n");
}

Result DODroid::incomingControlPacket(uint16_t station, PacketSource source, uint8_t rssi, const ControlPacket& packet) {
  if(source == PACKET_SOURCE_LEFT_REMOTE) {
    //Console::console.printfBroadcast("Control packet from left remote\n");
    return RES_OK;
  } else if(source == PACKET_SOURCE_RIGHT_REMOTE) {
    //Console::console.printfBroadcast("Control packet from right remote: %.2f %.2f\n", packet.getAxis(0), packet.getAxis(1));
    balanceController_->setGoal(PITCH_REMOTE_FACTOR*packet.getAxis(1));
    driveOutput_->setGoalRotation(PITCH_REMOTE_FACTOR*packet.getAxis(0));
    return RES_OK;
  }
  return RES_OK;
}

Result DODroid::incomingConfigPacket(uint16_t station, PacketSource source, uint8_t rssi, const ConfigPacket& packet) {
  return RES_OK;
}

Result DODroid::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
  if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result DODroid::setParameterValue(const String& name, const String& stringVal) {
  Result retval = Subsystem::setParameterValue(name, stringVal);
  if(retval != RES_OK) return retval;

  balanceController_->setControlParameters(params_.balKp, params_.balKi, params_.balKd);
  balanceController_->reset();
}

Result DODroid::fillAndSendStatePacket() {
  LargeStatePacket p;

  //if((Runloop::runloop.getSequenceNumber() % 4) != 0) return RES_OK;

  p.timestamp = Runloop::runloop.millisSinceStart() / 1000.0;
  p.droidType = DroidType::DROID_DO;
  return RES_OK;

  strncpy(p.droidName, DROID_NAME, sizeof(p.droidName));


  float err, errI, errD, control;

  p.drive[0].errorState = ERROR_OK;
  p.drive[0].controlMode = 0;
  p.drive[0].presentPWM = 0; balanceController_->present();
  p.drive[0].presentPos = leftEncoder_.presentPosition();
  p.drive[0].presentSpeed = leftEncoder_.presentSpeed();
  
  balanceController_->getControlState(err, errI, errD, control);
  p.drive[0].err = err;
  p.drive[0].errI = errI;
  p.drive[0].errD = errD;
  p.drive[0].control = control;

  p.drive[1].errorState = ERROR_NOT_PRESENT;
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

Result DODroid::selfTest() {
  Runloop::runloop.excuseOverrun();

  Console::console.printfBroadcast("D-O Self Test\n=============\n");
  
  // Check battery
  if(!DOBattStatus::batt.available()) {
    Console::console.printfBroadcast("Critical error: Battery monitor not available!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  DOBattStatus::batt.updateVoltage();
  Console::console.printfBroadcast("Battery: %.2fV\n", DOBattStatus::batt.voltage());

  // Check IMU
  if(DOIMU::imu.available() == false) {
    Console::console.printfBroadcast("Critical error: IMU not available!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  DOIMU::imu.update();
  float ax, ay, az;
  uint32_t timestamp;
  DOIMU::imu.getAccelMeasurement(ax, ay, az, timestamp);
  if(fabs(ax) > 1.0 || fabs(ay) > 1.0 || fabs(az) < 9.0) {
    Console::console.printfBroadcast("Critical error: Droid not upright!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  Console::console.printfBroadcast("Down vector: %.2f %.2f %.2f\n", ax, ay, az);

#if 0
  // Check servos
  if(DOServos::servos.isStarted() == false) {
    Console::console.printfBroadcast("Critical error: Servo subsystem not started!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  if(DOServos::servos.hasServoWithID(SERVO_NECK) == false) {
    Console::console.printfBroadcast("Critical error: Neck servo missing!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  if(DOServos::servos.hasServoWithID(SERVO_HEAD_PITCH)) {
    Console::console.printfBroadcast("Degraded: Head pitch servo missing.");
  }
  if(DOServos::servos.hasServoWithID(SERVO_HEAD_HEADING)) {
    Console::console.printfBroadcast("Degraded: Head heading servo missing.");
  }
  if(DOServos::servos.hasServoWithID(SERVO_HEAD_ROLL)) {
    Console::console.printfBroadcast("Degraded: Head roll servo missing.");
  }
#endif

  // Check motors
  motorsOK_ = true;
  static const int testSpeed = 180;
  static const float testExpect = 100.0;
  static const int testDuration = 500;

  leftEncoder_.update(); rightEncoder_.update();
  float l0 = leftEncoder_.presentPosition();
  float r0 = rightEncoder_.presentPosition();

  leftMotor_.setEnabled(true);
  rightMotor_.setEnabled(true);

  leftMotor_.set(testSpeed); rightMotor_.set(-testSpeed);
  delay(testDuration);
  leftMotor_.set(0); rightMotor_.set(0);
  delay(testDuration);

  leftEncoder_.update(); rightEncoder_.update();
  float l1 = leftEncoder_.presentPosition();
  float r1 = rightEncoder_.presentPosition();
  
  leftMotor_.set(-testSpeed); rightMotor_.set(testSpeed);
  delay(testDuration);
  leftMotor_.set(0); rightMotor_.set(0);
  delay(testDuration);

  leftEncoder_.update(); rightEncoder_.update();
  float l2 = leftEncoder_.presentPosition();
  float r2 = rightEncoder_.presentPosition();

  Console::console.printfBroadcast("Encoders: Left %f/%f/%f, right %f/%f/%f\n", l0, l1, l2, r0, r1, r2);
  if(!(l1-l0 > testExpect && l2-l1 < -testExpect)) {
    Console::console.printfBroadcast("Left motor or encoder not working (%.1f/%.1f/%.1f)!\n", l0, l1, l2);
    motorsOK_ = false;
  }
  if(!(r1-r0 < -testExpect && r2-r1 > testExpect)) {
    Console::console.printfBroadcast("Right motor or encoder not working (%.1f/%.1f/%.1f)!\n", r0, r1, r2);
    motorsOK_ = false;
  }
  if(motorsOK_)
    Console::console.printfBroadcast("Motors OK.\n");

  return RES_OK;
}


