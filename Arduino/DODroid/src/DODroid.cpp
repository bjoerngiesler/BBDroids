#include "DODroid.h"
#include "DOConfig.h"
#include "DOBattStatus.h"
#include "DOSound.h"
#include "../resources/systemsounds.h"

DODroid DODroid::droid;
DODroid::Params DODroid::params_ = {
  .wheelSpeedKp = WHEEL_SPEED_KP,
  .wheelSpeedKi = WHEEL_SPEED_KI,
  .wheelSpeedKd = WHEEL_SPEED_KD,
  .balKp = BAL_KP,
  .balKi = BAL_KI,
  .balKd = BAL_KD,
  .pwmBalKp = PWM_BAL_KP,
  .pwmBalKi = PWM_BAL_KI,
  .pwmBalKd = PWM_BAL_KD,
  .accel = ACCEL,
  .pwmAccel = PWM_ACCEL,
  .maxSpeed = MAX_SPEED,
  .faNeckAccel = FA_NECK_ACCEL,
  .faNeckSpeed = FA_NECK_SPEED,
  .faHeadRollTurn = FA_HEAD_ROLL_TURN,
  .faHeadHeadingTurn = FA_HEAD_HEADING_TURN
};

DODroid::DODroid():
  imu_(IMU_ADDR),
  leftMotor_(P_LEFT_PWMA, P_LEFT_PWMB), 
  rightMotor_(P_RIGHT_PWMA, P_RIGHT_PWMB), 
  leftEncoder_(P_LEFT_ENCA, P_LEFT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
  rightEncoder_(P_RIGHT_ENCA, P_RIGHT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
  lSpeedController_(leftEncoder_, leftMotor_),
  rSpeedController_(rightEncoder_, rightMotor_),
  balanceInput_(imu_, bb::IMUControlInput::IMU_PITCH),
  driveOutput_(lSpeedController_, rSpeedController_),
  pwmDriveOutput_(leftMotor_, rightMotor_),
  balanceController_(balanceInput_, driveOutput_),
  pwmBalanceController_(balanceInput_, pwmDriveOutput_),
  leftMotorStatus_(MOTOR_UNTESTED),
  rightMotorStatus_(MOTOR_UNTESTED),
  servosOK_(false),
  antennasOK_(false)
{
  pinMode(PULL_DOWN_15, OUTPUT);
  digitalWrite(PULL_DOWN_15, LOW);
  pinMode(PULL_DOWN_20, OUTPUT);
  digitalWrite(PULL_DOWN_20, LOW);

  name_ = "d-o";

  description_ = "D-O Main System";
  help_ = "Available commands:\r\n"\
"\tstatus\tPrint Status\r\n"\
"\tselftest\tRun self test\r\n"\
"\tmode {pwm|speed}\tRun drive system in PWM or speed control mode";
  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
}

Result DODroid::initialize() {
  addParameter("wheel_speed_kp", "Proportional constant for wheel speed PID controller", params_.wheelSpeedKp, -INT_MAX, INT_MAX);
  addParameter("wheel_speed_ki", "Integrative constant for wheel speed PID controller", params_.wheelSpeedKi, -INT_MAX, INT_MAX);
  addParameter("wheel_speed_kd", "Derivative constant for wheel speed PID controller", params_.wheelSpeedKd, -INT_MAX, INT_MAX);
  addParameter("bal_kp", "Proportional constant for balance PID controller", params_.balKp, -INT_MAX, INT_MAX);
  addParameter("bal_ki", "Integrative constant for balance PID controller", params_.balKi, -INT_MAX, INT_MAX);
  addParameter("bal_kd", "Derivative constant for balance PID controller", params_.balKd, -INT_MAX, INT_MAX);
  addParameter("pwm_bal_kp", "Proportional constant for balance PID controller (PWM mode)", params_.pwmBalKp, -INT_MAX, INT_MAX);
  addParameter("pwm_bal_ki", "Integrative constant for balance PID controller (PWM mode)", params_.pwmBalKi, -INT_MAX, INT_MAX);
  addParameter("pwm_bal_kd", "Derivative constant for balance PID controller (PWM mode)", params_.pwmBalKd, -INT_MAX, INT_MAX);
  addParameter("accel", "Acceleration in mm/s^2", params_.accel, -INT_MAX, INT_MAX);
  addParameter("pwmAccel", "Acceleration in pwm/s", params_.accel, -INT_MAX, INT_MAX);
  addParameter("max_speed", "Maximum speed (only honored in speed control mode)", params_.maxSpeed, 0, INT_MAX);
  addParameter("fa_neck_accel", "Free Animation factor - neck on IMU accel", params_.faNeckAccel, -INT_MAX, INT_MAX);
  addParameter("fa_neck_speed", "Free Animation factor - neck on wheel speed", params_.faNeckSpeed, -INT_MAX, INT_MAX);
  addParameter("fa_head_roll_turn", "Free Animation factor - head roll on turn speed", params_.faHeadRollTurn, -INT_MAX, INT_MAX);
  addParameter("fa_head_heading_turn", "Free Animation factor - head heading on turn speed", params_.faHeadHeadingTurn, -INT_MAX, INT_MAX);

  lSpeedController_.setAutoUpdate(false);
  rSpeedController_.setAutoUpdate(false);

  balanceController_.setAutoUpdate(false);
  balanceController_.setReverse(true);
  balanceController_.setControlDeadband(-10.0, 10.0);

  pwmBalanceController_.setAutoUpdate(false);
  pwmBalanceController_.setReverse(true);

  return Subsystem::initialize();
}

Result DODroid::start(ConsoleStream* stream) {
  lastBtn0_ = lastBtn1_ = lastBtn2_ = lastBtn3_ = lastBtn4_ = false;
  imu_.begin();
  DOBattStatus::batt.begin();

  operationStatus_ = selfTest();

  if(operationStatus_ == RES_DROID_VOLTAGE_TOO_LOW) {
    Console::console.printfBroadcast("No power (%.fV), USB only!\n", DOBattStatus::batt.voltage());
    if(DOBattStatus::batt.voltage() < 1.0) {
      started_ = true;
      operationStatus_ = RES_OK;
      return RES_OK;
    }
  } else if(operationStatus_ != RES_OK) {
    return operationStatus_;
  }

  driveOn_ = false;
  pwm_ = false;
  
  leftMotor_.set(0);
  rightMotor_.set(0);

  bb::Servos::servos.switchTorque(SERVO_NECK, true);

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

void DODroid::setControlParameters() {
  leftEncoder_.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  leftEncoder_.setMode(bb::Encoder::INPUT_SPEED);
  leftEncoder_.setUnit(bb::Encoder::UNIT_MILLIMETERS);
  rightEncoder_.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  rightEncoder_.setMode(bb::Encoder::INPUT_SPEED);
  rightEncoder_.setUnit(bb::Encoder::UNIT_MILLIMETERS);

  lSpeedController_.setControlParameters(params_.wheelSpeedKp, params_.wheelSpeedKi, params_.wheelSpeedKd);
  rSpeedController_.setControlParameters(params_.wheelSpeedKp, params_.wheelSpeedKi, params_.wheelSpeedKd);
  lSpeedController_.reset();
  rSpeedController_.reset();

  balanceController_.setControlParameters(params_.balKp, params_.balKi, params_.balKd);
  balanceController_.setRamp(0);
  balanceController_.reset();
  driveOutput_.setAcceleration(params_.accel);

  pwmBalanceController_.setControlParameters(params_.pwmBalKp, params_.pwmBalKi, params_.pwmBalKd);
  pwmBalanceController_.setRamp(0);
  pwmBalanceController_.reset();
  pwmDriveOutput_.setAcceleration(PWM_ACCEL);
}

Result DODroid::step() {
  if(!imu_.available() || !DOBattStatus::batt.available()) {
    fillAndSendStatePacket();
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  if((Runloop::runloop.getSequenceNumber() % 1000) == 0) {
    stepPowerProtect();
  }

  // needed for everything, so we do these here.
  leftEncoder_.update();  
  rightEncoder_.update();  
  imu_.update();

  stepHead();
  stepDrive();

  fillAndSendStatePacket();
  return RES_OK;
}

bb::Result DODroid::stepPowerProtect() {
  DOBattStatus::batt.updateVoltage();
  Console::console.printfBroadcast("Power: %.1fV\n", DOBattStatus::batt.voltage());

  // CRITICAL: Switch everything off (except the neck servo, so that we don't drop the head) and go into endless loop if power 
  if(DOBattStatus::batt.voltage() > 2.0 &&
      DOBattStatus::batt.voltage() < POWER_BATT_MIN) {
    leftMotor_.set(0);
    rightMotor_.set(0);
    bb::Servos::servos.switchTorque(SERVO_HEAD_PITCH, false);
    bb::Servos::servos.switchTorque(SERVO_HEAD_HEADING, false);
    bb::Servos::servos.switchTorque(SERVO_HEAD_ROLL, false);
    while(true) {
      DOSound::sound.playSystemSound(SystemSounds::VOLTAGE_TOO_LOW);
      delay(5000);
    }
  }
  
  return RES_OK;
}

bb::Result DODroid::stepHead() {
 float r, p, h, dr, dp, dh, ax, ay, az;
  if(servosOK_) {
    imu_.getFilteredRPH(r, p, h);
    imu_.getAccelMeasurement(ax, ay, az);
    imu_.getGyroMeasurement(dr, dp, dh);
    float speed = (leftEncoder_.presentSpeed() + rightEncoder_.presentSpeed())/2;

    // float nod = p + params_.faNeckAccel*ax + params_.faNeckSpeed*speed;
    float nod = params_.faNeckAccel*ax + params_.faNeckSpeed*speed;
    bb::Servos::servos.setGoal(SERVO_NECK, 180 + nod);
    bb::Servos::servos.setGoal(SERVO_HEAD_PITCH, 180 - nod);
    
    bb::Servos::servos.setGoal(SERVO_HEAD_HEADING, 180.0 + params_.faHeadHeadingTurn * dh);
    bb::Servos::servos.setGoal(SERVO_HEAD_ROLL, 180.0 + 
    params_.faHeadRollTurn * dh);
  }
  
  return RES_OK;
}

bb::Result DODroid::stepDrive() {
  if(driveOn_ == true && leftMotorStatus_ == MOTOR_OK && rightMotorStatus_ == MOTOR_OK) {
    if(pwm_ == true) {
      pwmBalanceController_.update();
    } else {
      balanceController_.update();
      lSpeedController_.update();
      rSpeedController_.update();
    }
  } else {
    leftMotor_.set(0);
    rightMotor_.set(0);
  }
  return RES_OK;
}

void DODroid::switchDrive(bool onoff) {
  lSpeedController_.reset();
  lSpeedController_.setGoal(0);
  rSpeedController_.reset();
  rSpeedController_.setGoal(0);
  balanceController_.reset();
  pwmBalanceController_.reset();

  driveOn_ = onoff;
}

void DODroid::printStatus(ConsoleStream *stream) {
  if(!stream) return;
  stream->printf("%s: %s", name_, started_ ? "started" : "not started");
  if(!started_) {
    stream->printf("\n");
    return;
  }

  stream->printf(", status: %s", errorMessage(operationStatus_));
  
  stream->printf(", batt: ");
  if(DOBattStatus::batt.available()) {
    stream->printf("%fV %fmA", DOBattStatus::batt.voltage(), DOBattStatus::batt.current());
  } else {
    stream->printf("not available");
  }

  stream->printf(", servos: %s", bb::Servos::servos.isStarted() ? "OK" : "not started");

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
    static int numZero = 0;
    if(EPSILON(packet.getAxis(0)) && EPSILON(packet.getAxis(1))) {
      numZero++;
    } else {
      numZero = 0;
    }

    if(packet.button4 && !lastBtn4_) {
      if(driveOn_ == false) {
        Console::console.printfBroadcast("Switching drive system on\n");
        switchDrive(true);
      } else {
        Console::console.printfBroadcast("Switching drive system off\n");
        switchDrive(false);
      }
    }

    lastBtn0_ = packet.button0;
    lastBtn1_ = packet.button1;
    lastBtn2_ = packet.button2;
    lastBtn3_ = packet.button3;
    lastBtn4_ = packet.button4;

    if(driveOn_ == true) {
      if(pwm_ == true) {
        if(numZero > 5) {
          pwmBalanceController_.reset();
        }
        pwmDriveOutput_.setGoalVelocity(255*packet.getAxis(1));
        pwmDriveOutput_.setGoalRotation(255*packet.getAxis(0));
      } else {
        if(numZero > 5) {
          balanceController_.reset();
        }
        driveOutput_.setGoalVelocity(params_.maxSpeed*packet.getAxis(1));
        driveOutput_.setGoalRotation(params_.maxSpeed*packet.getAxis(0));
      }
    }
  } // right remote
  return RES_OK;
}

Result DODroid::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
  if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if(words[0] == "selftest") {
    if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
    Runloop::runloop.excuseOverrun();
    return selfTest(stream);
  } 
  
  else if(words[0] == "mode") {
    if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;

    if(words[1] == "pwm") {
      if(pwm_ == true) {
        stream->printf("Already in PWM mode.\n");
        return RES_OK;
      }
      pwm_ = true; 
      switchDrive(false);
      return RES_OK;
    } else if(words[1] == "speed") {
      if(pwm_ == false) {
        stream->printf("Already in speed control mode.\n");
        return RES_OK;
      }
      pwm_ = false;
      switchDrive(false);
      return RES_OK;
    }
  }

  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result DODroid::setParameterValue(const String& name, const String& stringVal) {
  Result retval = Subsystem::setParameterValue(name, stringVal);
  if(retval != RES_OK) return retval;

  setControlParameters();

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
  p.drive[0].presentPWM = 0; balanceController_.present();
  p.drive[0].presentPos = leftEncoder_.presentPosition();
  p.drive[0].presentSpeed = leftEncoder_.presentSpeed();
  
  balanceController_.getControlState(err, errI, errD, control);
  p.drive[0].err = err;
  p.drive[0].errI = errI;
  p.drive[0].errD = errD;
  p.drive[0].control = control;

  p.drive[1].errorState = ERROR_NOT_PRESENT;
  p.drive[2].errorState = ERROR_NOT_PRESENT;

  p.imu[0] = imu_.getIMUState();
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
  
  // Check IMU
  DOSound::sound.playSystemSound(SystemSounds::IMU);
  if(imu_.available() == false) {
    Console::console.printfBroadcast("Critical error: IMU not available!\n");
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  imu_.update(true);
  float ax, ay, az;
  imu_.getAccelMeasurement(ax, ay, az);
  if(fabs(ax) > 0.1 || fabs(ay) > 0.1 || fabs(az) < 0.9) {
    Console::console.printfBroadcast("Critical error: Droid not upright (ax %f, ay %f, az %f)!\n", ax, ay, az);
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  Console::console.printfBroadcast("IMU OK. Down vector: %.2f %.2f %.2f\n", ax, ay, az);

  DOSound::sound.playSystemSound(SystemSounds::CALIBRATING);
  imu_.calibrateGyro(stream);
  for(int i=0; i<100; i++) {
    imu_.update(true);
  }
  float r, p, h;
  imu_.getFilteredRPH(r, p, h);
  balanceController_.setGoal(-p);
  Console::console.printfBroadcast("IMU calibrated. Pitch angle at rest: %f\n", p);
  DOSound::sound.playSystemSound(SystemSounds::OK);

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

  // Check Servos
  DOSound::sound.playSystemSound(SystemSounds::SERVOS);
  if(servoTest(stream) != RES_OK) {
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
  } else {
    DOSound::sound.playSystemSound(SystemSounds::OK);
  }

  // Check Antennas
  Wire.beginTransmission(0x17);
  uint8_t antErr = Wire.endTransmission();
  if(antErr != 0) {
    Console::console.printfBroadcast("Antenna error: 0x%x\n", antErr);
  } else {
    uint8_t step=4, d=25;
    antennasOK_ = true;
    for(uint8_t val = 127; val < 255; val+=step) {
      setAntennas(val, val, val);
      delay(d);
    }
    for(uint8_t val = 255; val > 64; val-=step) {
      setAntennas(val, val, val);
      delay(d);
    }
    for(uint8_t val = 64; val < 127; val+=step) {
      setAntennas(val, val, val);
      delay(d);
    }
    setAntennas(127, 127, 127);
  }

  // Check Motors
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
  rightMotorStatus_ = singleMotorTest(rightMotor_, rightEncoder_, false, stream);
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
  if(bb::Servos::servos.isStarted() == false) {
    Console::console.printfBroadcast("Critical error: Servo subsystem not started!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  if(bb::Servos::servos.hasServoWithID(SERVO_NECK) == false) {
    Console::console.printfBroadcast("Critical error: Neck servo missing!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else if(bb::Servos::servos.home(SERVO_NECK, 5.0, 95, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing servos failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    bb::Servos::servos.setRange(SERVO_NECK, 180-NECK_RANGE, 180+NECK_RANGE);
    bb::Servos::servos.setOffset(SERVO_NECK, NECK_OFFSET);
    bb::Servos::servos.setProfileVelocity(SERVO_NECK, 50);
  }

  if(bb::Servos::servos.hasServoWithID(SERVO_HEAD_PITCH) == false) {
    Console::console.printfBroadcast("Degraded: Head pitch servo missing.\n");
  } else if(bb::Servos::servos.home(SERVO_HEAD_PITCH, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing servos failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    bb::Servos::servos.setRange(SERVO_HEAD_PITCH, 180-HEAD_PITCH_RANGE, 180+HEAD_PITCH_RANGE);
    bb::Servos::servos.setOffset(SERVO_HEAD_PITCH, HEAD_PITCH_OFFSET);
    bb::Servos::servos.setProfileVelocity(SERVO_HEAD_PITCH, 50);
  }

  if(bb::Servos::servos.hasServoWithID(SERVO_HEAD_HEADING) == false) {
    Console::console.printfBroadcast("Degraded: Head heading servo missing.\n");
  } else if(bb::Servos::servos.home(SERVO_HEAD_HEADING, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing servos failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    bb::Servos::servos.setRange(SERVO_HEAD_HEADING, 180-HEAD_HEADING_RANGE, 180+HEAD_HEADING_RANGE);
    bb::Servos::servos.setOffset(SERVO_HEAD_HEADING, HEAD_HEADING_OFFSET);
  }


  if(bb::Servos::servos.hasServoWithID(SERVO_HEAD_ROLL) == false) {
    Console::console.printfBroadcast("Degraded: Head roll servo missing.\n");
  } else if(bb::Servos::servos.home(SERVO_HEAD_ROLL, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing servos failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    bb::Servos::servos.setRange(SERVO_HEAD_ROLL, 180-HEAD_ROLL_RANGE, 180+HEAD_ROLL_RANGE);
    bb::Servos::servos.setOffset(SERVO_HEAD_ROLL, HEAD_ROLL_OFFSET);
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
  unsigned int blockedcount = 0;
  float distance = 0;

  imu_.update();
  imu_.getFilteredRPH(r, p, h0);

  unsigned long microsPerLoop = (unsigned long)(1e6 / imu_.dataRate());

  for(pwm = ST_MIN_PWM; pwm < ST_MAX_PWM; pwm += pwmStep) {
    unsigned long us0 = micros();
    if(reverse) mot.set(-pwm);
    else mot.set(pwm);

    DOBattStatus::batt.updateCurrent();
    float mA = DOBattStatus::batt.current();
    
    imu_.update();
    imu_.getFilteredRPH(r, p, h);
    float hdiff = h-h0;
    if(hdiff < -180) hdiff += 360;
    else if(hdiff > 180) hdiff -= 360;
    if(fabs(hdiff) > fabs(hdiffmax)) {
      hdiffmax = hdiff;
    }

    imu_.getAccelMeasurement(ax, ay, az);
    if(fabs(ax) > fabs(axmax)) {
      axmax = ax;
      aymax = ay;
    }

    enc.update();
    distance = enc.presentPosition()-startPosition;

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

  Console::console.printfBroadcast("Max hdiff: %f\n", hdiffmax);

  // Not enough heading change observed? We're likely sitting in the station.
  if(fabs(hdiffmax) < ST_MIN_HEADING_CHANGE) {
    Console::console.printfBroadcast("Heading change %f too small, we're likely in the station, encoder/motor reverse detection cannot be distinguished!\n",
                                     hdiffmax);
  } else if(hdiffmax > 0) { // turned in the wrong direction
    if((reverse && distance >= 0) ||
       (!reverse && distance <= 0)) {
      Console::console.printfBroadcast("%s, turning in wrong direction, distance %f in wrong direction. Motor likely reversed.\n",
                                       reverse ? "Reverse" : "Forward", distance);
      return MOTOR_REVERSED;
    } else {
      Console::console.printfBroadcast("%s, turning in wrong direction, distance %f in right direction. Motor and encoder likely reversed.\n",
                                       reverse ? "Reverse" : "Forward", distance);
      return MOTOR_BOTH_REVERSED;
    }
  } else { // turned in the right direction
    if((reverse && distance >= 0) ||
       (!reverse && distance <= 0)) {
      Console::console.printfBroadcast("%s, turning in right direction, distance %f in wrong direction. Encoder likely reversed.\n",
                                       reverse ? "Reverse" : "Forward", distance);
      return MOTOR_ENC_REVERSED;
    }
  }

  // Not blocked, accel axis is OK, enough distance driven, enough accel generated, accel and distance both in right direction?
  // Looks good. 

  return MOTOR_OK;
}

bool DODroid::setAntennas(uint8_t a1, uint8_t a2, uint8_t a3) {
  if(antennasOK_ == false) return false;
  uint8_t antennas[3] = {a1, a2, a3};
  Wire.beginTransmission(0x17);
  Wire.write(antennas, sizeof(antennas));
  if(Wire.endTransmission() == 0) return true;
  return false;
}

bool DODroid::getAntennas(uint8_t& a1, uint8_t& a2, uint8_t& a3) {
  if(antennasOK_ == false) return false;
  int timeout;
  Wire.requestFrom(0x17, 3);

  for(timeout=100; timeout>0 && !Wire.available(); timeout--);
  if(timeout < 0) return false;
  a1 = Wire.read();

  for(timeout=100; timeout>0 && !Wire.available(); timeout--);
  if(timeout < 0) return false;
  a2 = Wire.read();

  for(timeout=100; timeout>0 && !Wire.available(); timeout--);
  if(timeout < 0) return false;
  a3 = Wire.read();

  return true;
}