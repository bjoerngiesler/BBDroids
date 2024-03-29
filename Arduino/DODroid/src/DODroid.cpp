#include "DODroid.h"
#include "DOConfig.h"
#include "DOIMU.h"
#include "DOBattStatus.h"
#include "DOServos.h"
#include "DOSound.h"
#include "../resources/systemsounds.h"

DODroid DODroid::droid;
DODroid::Params DODroid::params_ = {
  .balKp = BAL_KP,
  .balKi = BAL_KI,
  .balKd = BAL_KD,
  .speedKp = SPEED_KP,
  .speedKi = SPEED_KI,
  .speedKd = SPEED_KD,
  .posKp = POS_KP,
  .posKi = POS_KI,
  .posKd = POS_KD,
  .speedRemoteFactor = SPEED_REMOTE_FACTOR,
  .rotRemoteFactor = ROT_REMOTE_FACTOR
};

DODroid::DODroid():
  leftMotor_(P_LEFT_PWMA, P_LEFT_PWMB), 
  rightMotor_(P_RIGHT_PWMA, P_RIGHT_PWMB), 
  leftEncoder_(P_LEFT_ENCA, P_LEFT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
  rightEncoder_(P_RIGHT_ENCA, P_RIGHT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
  leftMotorStatus_(MOTOR_UNTESTED),
  rightMotorStatus_(MOTOR_UNTESTED),
  servosOK_(false),
  driveMode_(DRIVE_PITCH)
{
  pinMode(PULL_DOWN_15, OUTPUT);
  digitalWrite(PULL_DOWN_15, LOW);
  pinMode(PULL_DOWN_20, OUTPUT);
  digitalWrite(PULL_DOWN_20, LOW);

  name_ = "d-o";

  description_ = "D-O Main System";
  help_ = "Available commands:\r\n"\
"\tstatus\tPrint Status\r\n"\
"\tselftest\tRun self test";
  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
}

Result DODroid::initialize() {
  addParameter("bal_kp", "Proportional constant for balance PID controller", params_.balKp, 0, INT_MAX);
  addParameter("bal_ki", "Integrative constant for balance PID controller", params_.balKi, 0, INT_MAX);
  addParameter("bal_kd", "Derivative constant for balance PID controller", params_.balKd, 0, INT_MAX);
  addParameter("speed_kp", "Proportional constant for speed PID controller", params_.speedKp, 0, INT_MAX);
  addParameter("speed_ki", "Integrative constant for speed PID controller", params_.speedKi, 0, INT_MAX);
  addParameter("speed_kd", "Derivative constant for speed PID controller", params_.speedKd, 0, INT_MAX);
  addParameter("pos_kp", "Proportional constant for position PID controller", params_.posKp, 0, INT_MAX);
  addParameter("pos_ki", "Integrative constant for position PID controller", params_.posKi, 0, INT_MAX);
  addParameter("pos_kd", "Derivative constant for position PID controller", params_.posKd, 0, INT_MAX);
  addParameter("speed_remote_factor", "Amplification factor for remote speed axis", params_.speedRemoteFactor, 0, 255);
  addParameter("rot_remote_factor", "Amplification factor for remote rotation axis", params_.rotRemoteFactor, 0, 255);

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

  DOServos::servos.switchTorque(SERVO_NECK, true);

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

  if(leftMotorStatus_ == MOTOR_OK && rightMotorStatus_ == MOTOR_OK && driveMode_ != DRIVE_OFF) {
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
  if(leftMotorStatus_ == MOTOR_OK) stream->printf("L OK");
  else stream->printf("L Err %d", leftMotorStatus_);
  stream->printf(", enc %.1f", leftEncoder_.presentPosition());

  rightEncoder_.update();
  if(rightMotorStatus_ == MOTOR_OK) stream->printf(", R OK");
  else stream->printf("R Err %d", rightMotorStatus_);
  stream->printf(", enc %.1f", rightEncoder_.presentPosition());

  stream->printf("\n");
}

Result DODroid::incomingControlPacket(uint16_t station, PacketSource source, uint8_t rssi, const ControlPacket& packet) {
  if(source == PACKET_SOURCE_LEFT_REMOTE) {
    //Console::console.printfBroadcast("Control packet from left remote\n");
    return RES_OK;
  } else if(source == PACKET_SOURCE_RIGHT_REMOTE) {
    //Console::console.printfBroadcast("Control packet from right remote: %.2f %.2f\n", packet.getAxis(0), packet.getAxis(1));
    balanceController_->setGoal(params_.speedRemoteFactor*packet.getAxis(1));
    driveOutput_->setGoalRotation(params_.rotRemoteFactor*packet.getAxis(0));
    return RES_OK;
  }
  return RES_OK;
}

Result DODroid::incomingConfigPacket(uint16_t station, PacketSource source, uint8_t rssi, const ConfigPacket& packet) {
  return RES_OK;
}

Result DODroid::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
  if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if(words.size() == 1 && words[0] == "selftest") {
    Runloop::runloop.excuseOverrun();
    return selfTest(stream);
  }

  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result DODroid::setParameterValue(const String& name, const String& stringVal) {
  Result retval = Subsystem::setParameterValue(name, stringVal);
  if(retval != RES_OK) return retval;

  balanceController_->setControlParameters(params_.balKp, params_.balKi, params_.balKd);
  balanceController_->reset();

  return RES_OK;
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

Result DODroid::selfTest(ConsoleStream *stream) {
  Runloop::runloop.excuseOverrun();

  DOSound::sound.playSystemSound(SystemSounds::SELFTEST_STARTING_PLEASE_STAND_CLEAR);
  delay(2000);

  Console::console.printfBroadcast("D-O Self Test\n=============\n");
  
  // Check battery
  DOSound::sound.playSystemSound(SystemSounds::POWER);
  if(!DOBattStatus::batt.available()) {
    Console::console.printfBroadcast("Critical error: Battery monitor not available!\n");
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  DOBattStatus::batt.updateVoltage();
  DOBattStatus::batt.updateCurrent();
  Console::console.printfBroadcast("Battery OK. Voltage: %.2fV, current draw: %.2fmA\n", DOBattStatus::batt.voltage(), DOBattStatus::batt.current());
  if(DOBattStatus::batt.voltage() < 12.0) {
    DOSound::sound.playSystemSound(SystemSounds::VOLTAGE_TOO_LOW);
    return RES_DROID_VOLTAGE_TOO_LOW;
  } else if(DOBattStatus::batt.voltage() > 17.0) {
    DOSound::sound.playSystemSound(SystemSounds::VOLTAGE_TOO_HIGH);
    return RES_DROID_VOLTAGE_TOO_HIGH;
  }
  DOSound::sound.playSystemSound(SystemSounds::OK);


  // Check IMU
  DOSound::sound.playSystemSound(SystemSounds::IMU);
  if(DOIMU::imu.available() == false) {
    Console::console.printfBroadcast("Critical error: IMU not available!\n");
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  DOIMU::imu.update();
  float ax, ay, az;
  uint32_t timestamp;
  DOIMU::imu.getAccelMeasurement(ax, ay, az, timestamp);
  if(fabs(ax) > 1.0 || fabs(ay) > 1.0 || fabs(az) < 9.0) {
    Console::console.printfBroadcast("Critical error: Droid not upright!\n");
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  Console::console.printfBroadcast("IMU OK. Down vector: %.2f %.2f %.2f\n", ax, ay, az);

  DOSound::sound.playSystemSound(SystemSounds::CALIBRATING);
  DOIMU::imu.calibrateGyro(stream);
  Console::console.printfBroadcast("IMU calibrated.\n");
  DOSound::sound.playSystemSound(SystemSounds::OK);

  DOSound::sound.playSystemSound(SystemSounds::SERVOS);
  if(servoTest(stream) != RES_OK) {
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
  } else {
    DOSound::sound.playSystemSound(SystemSounds::OK);
  }

  DOSound::sound.playSystemSound(SystemSounds::LEFT_MOTOR);
  leftMotorStatus_ = singleMotorTest(leftMotor_, leftEncoder_, false, stream);
  switch(leftMotorStatus_) {
  case MOTOR_UNTESTED:
    DOSound::sound.playSystemSound(SystemSounds::UNTESTED);
    break;
  case MOTOR_OK:
    DOSound::sound.playSystemSound(SystemSounds::OK);
    break;
  case MOTOR_DISCONNECTED:
    DOSound::sound.playSystemSound(SystemSounds::DISCONNECTED);
    break;
  case MOTOR_ENC_DISCONNECTED:
    DOSound::sound.playSystemSound(SystemSounds::ENCODER_DISCONNECTED);
    break;
  case MOTOR_REVERSED:
    DOSound::sound.playSystemSound(SystemSounds::REVERSED);
    break;
  case MOTOR_ENC_REVERSED:
    DOSound::sound.playSystemSound(SystemSounds::ENCODER_REVERSED);
    break;
  case MOTOR_BOTH_REVERSED:
    DOSound::sound.playSystemSound(SystemSounds::REVERSED);
    DOSound::sound.playSystemSound(SystemSounds::ENCODER_REVERSED);
    break;
  case MOTOR_BLOCKED:
    DOSound::sound.playSystemSound(SystemSounds::BLOCKED);
    break;
  case MOTOR_OTHER:
  default:
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    break;
  }

  DOSound::sound.playSystemSound(SystemSounds::RIGHT_MOTOR);
  rightMotorStatus_ = singleMotorTest(rightMotor_, rightEncoder_, true, stream);
  switch(rightMotorStatus_) {
  case MOTOR_UNTESTED:
    DOSound::sound.playSystemSound(SystemSounds::UNTESTED);
    break;
  case MOTOR_OK:
    DOSound::sound.playSystemSound(SystemSounds::OK);
    break;
  case MOTOR_DISCONNECTED:
    DOSound::sound.playSystemSound(SystemSounds::DISCONNECTED);
    break;
  case MOTOR_ENC_DISCONNECTED:
    DOSound::sound.playSystemSound(SystemSounds::ENCODER_DISCONNECTED);
    delay(1000);
    break;
  case MOTOR_REVERSED:
    DOSound::sound.playSystemSound(SystemSounds::REVERSED);
    break;
  case MOTOR_ENC_REVERSED:
    DOSound::sound.playSystemSound(SystemSounds::ENCODER_REVERSED);
    break;
  case MOTOR_BOTH_REVERSED:
    DOSound::sound.playSystemSound(SystemSounds::REVERSED);
    DOSound::sound.playSystemSound(SystemSounds::ENCODER_REVERSED);
    break;
  case MOTOR_BLOCKED:
    DOSound::sound.playSystemSound(SystemSounds::BLOCKED);
    break;
  case MOTOR_OTHER:
  default:
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    break;
  }

  return RES_OK;
}

Result DODroid::servoTest(ConsoleStream *stream) {
  // Check servos
  servosOK_ = false;
  if(DOServos::servos.isStarted() == false) {
    Console::console.printfBroadcast("Critical error: Servo subsystem not started!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  if(DOServos::servos.hasServoWithID(SERVO_NECK) == false) {
    Console::console.printfBroadcast("Critical error: Neck servo missing!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  if(DOServos::servos.hasServoWithID(SERVO_HEAD_PITCH) == false) {
    Console::console.printfBroadcast("Degraded: Head pitch servo missing.\n");
  }
  if(DOServos::servos.hasServoWithID(SERVO_HEAD_HEADING) == false) {
    Console::console.printfBroadcast("Degraded: Head heading servo missing.\n");
  }
  if(DOServos::servos.hasServoWithID(SERVO_HEAD_ROLL) == false) {
    Console::console.printfBroadcast("Degraded: Head roll servo missing.\n");
  }
  if(DOServos::servos.home(SERVO_NECK, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing servos failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  servosOK_ = true;
  Console::console.printfBroadcast("Servos OK.\n");
  return RES_OK;
}

DODroid::MotorStatus DODroid::singleMotorTest(bb::DCMotor& mot, bb::Encoder& enc, bool reverse, ConsoleStream *stream) {
  mot.set(0);
  mot.setEnabled(true);

  enc.update();
  enc.setUnit(bb::Encoder::UNIT_MILLIMETERS);
  float startPosition = enc.presentPosition();

  int pwmStep = 1, pwm;

  float r, p, h, h0, ax, ay, az, axmax=0, aymax=0, hdiffmax=0;
  uint32_t timestamp;
  unsigned int blockedcount = 0;
  float distance = 0;

  DOIMU::imu.update();
  DOIMU::imu.getFilteredRPH(r, p, h0);

  unsigned long microsPerLoop = (unsigned long)(1e6 / DOIMU::imu.dataRate());

  for(pwm = ST_MIN_PWM; pwm < ST_MAX_PWM; pwm += pwmStep) {
    unsigned long us0 = micros();
    if(reverse) mot.set(-pwm);
    else mot.set(pwm);

    DOBattStatus::batt.updateCurrent();
    float mA = DOBattStatus::batt.current();
    
    DOIMU::imu.update();
    DOIMU::imu.getFilteredRPH(r, p, h);
    float hdiff = h-h0;
    if(hdiff < -180) hdiff += 360;
    else if(hdiff > 180) hdiff -= 360;
    if(fabs(hdiff) > fabs(hdiffmax)) {
      hdiffmax = hdiff;
    }

    DOIMU::imu.getAccelMeasurement(ax, ay, az, timestamp);
    if(fabs(ax) > fabs(axmax)) {
      axmax = ax;
      aymax = ay;
    }

    enc.update();
    distance = enc.presentPosition()-startPosition;

    Console::console.printfBroadcast("PWM: %d h0: %f h: %f ax: %f ay: %f az: %f Current: %f Enc: %f\n", pwm, h0, h, ax, ay, az, mA, distance);
    if(fabs(mA) > ST_ABORT_MILLIAMPS) { 
      blockedcount++;
    } else {
      blockedcount = 0;
    }

    if(fabs(distance) > ST_ABORT_DISTANCE) {
      Console::console.printfBroadcast("Distance criterion triggered (fabs(%f) > %f)\n", distance, ST_ABORT_DISTANCE);
      break;
    } 
    
    if(fabs(hdiffmax) > ST_ABORT_HEADING_CHANGE) {
      Console::console.printfBroadcast("Heading criterion triggered (fabs(%f) > %f)\n", hdiffmax, ST_ABORT_HEADING_CHANGE);
      break;
    }

    if(fabs(axmax) > ST_ABORT_ACCEL) {
      Console::console.printfBroadcast("X max accel criterion triggered (fabs(%f) > %f)\n", fabs(axmax), ST_ABORT_ACCEL);
      break;
    }
    if(fabs(aymax) > ST_ABORT_ACCEL) {
      Console::console.printfBroadcast("Y max accel criterion triggered (fabs(%f) > %f)\n", fabs(aymax), ST_ABORT_ACCEL);
      break;
    }
    if(blockedcount > 10) {
      Console::console.printfBroadcast("Motor load criterion triggered %d times (%f > %f)\n", blockedcount, mA, ST_ABORT_MILLIAMPS);
      break;
    }

    unsigned long us1 = micros();
    unsigned long tdiff = us1-us0;
    if(microsPerLoop > tdiff) delayMicroseconds(microsPerLoop - tdiff);
  }

  Console::console.printfBroadcast("PWM at end: %d\n", pwm);

  for(; pwm>=ST_MIN_PWM; pwm -= pwmStep) {
    unsigned long us0 = micros();
    if(reverse) mot.set(-pwm);
    else mot.set(pwm);
    delayMicroseconds(microsPerLoop - (micros()-us0));
  }
  mot.set(0.0);

  // Current too high? Motor blocked.
  if(blockedcount > 5) {
    Console::console.printfBroadcast("Motor pulling too much power. Likely blocked!\n");
    return MOTOR_BLOCKED;
  }

  // Not blocked

  // High acceleration in y? Probably IMU is turned by 90°
  if(fabs(aymax) > ST_MIN_ACCEL && (aymax < -fabs(axmax) || aymax > fabs(axmax))) {
    Console::console.printfBroadcast("Accel in Y direction %f higher than in x %f. IMU likely rotated 90°!\n", aymax, axmax);
    return MOTOR_OTHER;
  }

  // Not blocked, and acceleration is along the correct axis

  // Not enough distance returned from the encoder...
  if(fabs(distance) < ST_MIN_DISTANCE) {
    // ...but measured enough acceleration? Motor is connected, encoder likely isn't.
    if(fabs(axmax) > ST_MIN_ACCEL) {
      Console::console.printfBroadcast("Distance of %f (<%f) too low, but accel of %f measured. Encoder likely disconnected!\n",
                                       distance, ST_MIN_DISTANCE, axmax);
      return MOTOR_ENC_DISCONNECTED;
    } else { // ...and not measured enough acceleration? Motor is likely not connected.
      Console::console.printfBroadcast("Distance of %f (<%f) and accel of %f (<%f) both too low. Motor likely disconnected!\n",
                                       distance, ST_MIN_DISTANCE, axmax, ST_MIN_ACCEL);
      return MOTOR_DISCONNECTED;
    }
  }

  // Not blocked, accel axis is OK, enough distance driven

#if 0
  // Not enough acceleration measured? We're likely sitting in the station.
  if(fabs(axmax) < ST_MIN_ACCEL) {
    Console::console.printfBroadcast("Accel %f too small, we're likely in the station, encoder/motor reverse detection cannot be distinguished!\n",
                                     axmax);
  }
#endif
  
  // Not enough heading change observed? We're likely sitting in the station.
  if(fabs(hdiffmax) < ST_MIN_HEADING_CHANGE) {
    Console::console.printfBroadcast("Heading change %f too small, we're likely in the station, encoder/motor reverse detection cannot be distinguished!\n",
                                     hdiffmax);
  }

  // Not blocked, accel axis is OK, enough distance driven, enough accel measured
#if 0
  if(reverse) {
    if(distance < 0 && axmax > 0 && fabs(axmax) > ST_MIN_ACCEL) {
      Console::console.printfBroadcast("Reverse %d, distance %f in right direction, accel %f in wrong direction. Motor likely reversed!\n",
                                       reverse, distance, axmax);
      return MOTOR_REVERSED;
    } else if(distance > 0 && axmax < 0 && fabs(axmax) > ST_MIN_ACCEL) {
      Console::console.printfBroadcast("Reverse %d, distance %f in wrong direction, accel %f in right direction. Encoder likely reversed!\n",
                                       reverse, distance, axmax);
      return MOTOR_ENC_REVERSED;
    } else if(distance > 0 && axmax > 0 && fabs(axmax) > ST_MIN_ACCEL) {
      Console::console.printfBroadcast("Reverse %d, distance %f and accel %f both in wrong direction. Motor and encoder likely reversed!\n",
                                       reverse, distance, axmax);
      return MOTOR_ENC_REVERSED;
    }
  } else {
    if(distance < 0 && axmax > 0 && fabs(axmax) > ST_MIN_ACCEL) {
      Console::console.printfBroadcast("Reverse %d, distance %f in wrong direction, accel %f in right direction. Encoder likely reversed!\n",
                                       reverse, distance, axmax);
      return MOTOR_ENC_REVERSED;
    } else if(distance > 0 && axmax < 0 && fabs(axmax) > ST_MIN_ACCEL) {
      Console::console.printfBroadcast("Reverse %d, distance %f in right direction, accel %f in wrong direction. Motor likely reversed!\n",
                                       reverse, distance, axmax);
      return MOTOR_REVERSED;
    } else if(distance < 0 && axmax < 0 && fabs(axmax) > ST_MIN_ACCEL) {
      Console::console.printfBroadcast("Reverse %d, distance %f and accel %f both in wrong direction. Motor and encoder likely reversed!\n",
                                       reverse, distance, axmax);
      return MOTOR_ENC_REVERSED;
    }
  }
#endif
  
  Console::console.printfBroadcast("Max hdiff: %f\n", hdiffmax);

  if(hdiffmax > 0) { // turned in the wrong direction
    if((reverse && distance >= 0) ||
       (!reverse && distance <= 0)) {
      Console::console.printfBroadcast("%s, turning in wrong direction, distance %f in wrong direction. Motor likely reversed.",
                                       reverse ? "Reverse" : "Forward", distance);
      return MOTOR_REVERSED;
    } else {
      Console::console.printfBroadcast("%s, turning in wrong direction, distance %f in right direction. Motor and encoder likely reversed.",
                                       reverse ? "Reverse" : "Forward", distance);
      return MOTOR_BOTH_REVERSED;
    }
  } else { // turned in the right direction
    if((reverse && distance >= 0) ||
       (!reverse && distance <= 0)) {
      Console::console.printfBroadcast("%s, turning in right direction, distance %f in wrong direction. Encoder likely reversed.",
                                       reverse ? "Reverse" : "Forward", distance);
      return MOTOR_ENC_REVERSED;
    }
  }

  // Not blocked, accel axis is OK, enough distance driven, enough accel generated, accel and distance both in right direction?
  // Looks good. 

  return MOTOR_OK;
}