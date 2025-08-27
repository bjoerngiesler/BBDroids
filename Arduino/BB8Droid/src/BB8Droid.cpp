#include "BB8Droid.h"
#include "BB8Config.h"
#include "BB8BattStatus.h"
#include "BB8Sound.h"
#include <SAMD_PWM.h>


BB8 BB8::bb8;
BB8Params BB8::params_;

//#define ROLL_VELOCITY
#define ROLL_POSITION

static float minPwmFreq = 200.0f;
static float maxPwmFreq = 500.0f;
static std::map<uint8_t, SAMD_PWM*> pwms;

static void configureTimers() {
  pwms[P_DRIVE_PWM] = new SAMD_PWM(P_DRIVE_PWM, minPwmFreq, 0);
  pwms[P_YAW_PWM] = new SAMD_PWM(P_YAW_PWM, minPwmFreq, 0);
  pwms[P_DRIVE_PWM]->setPWM();
  pwms[P_YAW_PWM]->setPWM();
}

static void customAnalogWrite(uint8_t pin, uint8_t dutycycle) {
  SAMD_PWM* pwm = pwms[pin];
  if(pwm == nullptr) {
    return;
  }
  float f = map(dutycycle, 0, 255, minPwmFreq, maxPwmFreq);
  pwm->setPWM(pin, f, map(dutycycle, 0, 255, 0.0, 100.0));
}


BB8::BB8(): 
    imu_(IMU_ADDR),
    driveMotor_(P_DRIVE_A, P_DRIVE_B, P_DRIVE_PWM, P_DRIVE_EN),
    yawMotor_(P_YAW_A, P_YAW_B, P_YAW_PWM, P_YAW_EN),
    driveEncoder_(P_DRIVEENC_A, P_DRIVEENC_B, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
    balanceInput_(imu_, IMUControlInput::IMU_PITCH),
    rollInput_(imu_, IMUControlInput::IMU_ROLL, false),
#if defined(ROLL_VELOCITY)
    rollOutput_(BODY_ROLL_SERVO, 0, Servos::CONTROL_VELOCITY),
#elif defined(ROLL_POSITION)
    rollOutput_(BODY_ROLL_SERVO, 0, Servos::CONTROL_POSITION),
#endif
    driveController_(driveEncoder_, driveMotor_),
    balanceController_(balanceInput_, driveController_),
    rollController_(rollInput_, rollOutput_),
    posControlOutput_(balanceController_),
    autoPosController_(driveEncoder_, posControlOutput_),
    posController_(driveEncoder_, posControlOutput_),
    statusPixels_(3, P_STATUS_NEOPIXEL, NEO_GRB+NEO_KHZ800)
{
  name_ = "bb8";
  description_ = "BB8 Main System";
  help_ = "BB8 Main System\r\n"
          "Available commands:\r\n"
          "        status                          Print status\r\n"
          "        running_status on|off           Continuously print status\r\n"
          "        play_sound <folder> <num>       Play sound\r\n"
          "        calibrate                       Calibrate gyro\r\n"
          "        drive pwm|speed|position <val>  Set drive motor setpoint\r\n"
          "        mode off|roll|speed|speed_roll|pos|kiosk|calib  Set drive mode\r\n"
          "        set_pixel <num> <r> <g> <b>     Set Neopixel <num> (1-3) to rgb color";
  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;

  statusPixels_.begin();
  statusPixels_.setBrightness(2);
  statusPixels_.show();

  setLED(LED_STATUS, WHITE);
  setLED(LED_COMM, OFF);
  setLED(LED_DRIVE, OFF);
  commLEDOn_ = false;
  msLastLeftCtrlPacket_ = msLastRightCtrlPacket_ = msLastPrimaryCtrlPacket_ = 0;
}

Result BB8::initialize() {
  addParameter("body_roll_range", "Body roll servo plus-minus range", params_.bodyRollRange);
  addParameter("body_roll_offset", "Body roll servo offset", params_.bodyRollOffset);
  addParameter("body_roll_vel", "Velocity for roll servo", params_.bodyRollVel);
  addParameter("body_roll_accel", "Acceleration for roll servo", params_.bodyRollAccel);
  addParameter("dome_pitch_range", "Body roll servo plus-minus range",  params_.domePitchRange);
  addParameter("dome_pitch_offset", "Plus-minus range for the body roll servo",  params_.domePitchOffset);
  addParameter("dome_pitch_vel", "Velocity for dome pitch servo", params_.domePitchVel);
  addParameter("dome_heading_range", "Plus-minus range for the body roll servo",  params_.domeHeadingRange);
  addParameter("dome_heading_offset", "Plus-minus range for the body roll servo",  params_.domeHeadingOffset);
  addParameter("dome_heading_vel", "Velocity for dome heading servo", params_.domeHeadingVel);
  addParameter("dome_roll_range", "Plus-minus range for the body roll servo",  params_.domeRollRange);
  addParameter("dome_roll_offset", "Plus-minus range for the body roll servo",  params_.domeRollOffset);
  addParameter("dome_roll_vel", "Velocity for dome roll servo", params_.domeRollVel);

  addParameter("drive_speed_kp", "Proportional constant for drive speed controller", params_.driveSpeedKp);
  addParameter("drive_speed_ki", "Integral constant for drive speed controller", params_.driveSpeedKi);
  addParameter("drive_speed_kd", "Derivative constant for drive speed controller", params_.driveSpeedKd);

  addParameter("bal_kp", "Proportional constant for drive balance controller", params_.balKp);
  addParameter("bal_ki", "Integral constant for drive balance controller", params_.balKi);
  addParameter("bal_kd", "Derivative constant for drive balance controller", params_.balKd);

  addParameter("roll_kp", "Proportional constant for roll controller", params_.rollKp);
  addParameter("roll_ki", "Integral constant for roll controller", params_.rollKi);
  addParameter("roll_kd", "Derivative constant for roll controller", params_.rollKd);

  addParameter("auto_pos_kp", "Proportional constant for position PID controller", params_.autoPosKp, -INT_MAX, INT_MAX);
  addParameter("auto_pos_ki", "Integrative constant for position PID controller", params_.autoPosKi, -INT_MAX, INT_MAX);
  addParameter("auto_pos_kd", "Derivative constant for position PID controller", params_.autoPosKd, -INT_MAX, INT_MAX);

  addParameter("pos_kp", "Proportional constant for position PID controller", params_.posKp, -INT_MAX, INT_MAX);
  addParameter("pos_ki", "Integrative constant for position PID controller", params_.posKi, -INT_MAX, INT_MAX);
  addParameter("pos_kd", "Derivative constant for position PID controller", params_.posKd, -INT_MAX, INT_MAX);

  addParameter("roll_direct", "Set to true for direct roll servo control, false for PID control", params_.rollDirect);
  addParameter("roll_inhibit", "Set to true to inhibit roll controller output (but continue updating, for debugging)", params_.rollInhibit);
  addParameter("roll_debug", "Set to true to debug roll controller", params_.rollDebug);
  addParameter("auto_pos_control", "Automatically switch to position control", params_.autoPosControl);

  addParameter("drive_speed_deadband", "Deadband for drive speed controller", params_.driveSpeedDeadband);
  addParameter("drive_speed_max", "Max speed for drive controller", params_.driveSpeedMax);
  addParameter("roll_angle_max", "Maximum roll angle settable by remote", params_.rollAngleMax);

  addParameter("roll_imax", "Max integral error for roll controller", params_.rollIMax);
  addParameter("roll_torque_percent", "Abort roll motion if this percentage of torque is exceeded", params_.rollTorquePercent);

  addParameter("dome_heading_servo_reverse", "Reverse the dome heading servo", params_.domeHeadingServoReverse);
  addParameter("dome_roll_servo_reverse", "Reverse the dome roll servo", params_.domeRollServoReverse);
  addParameter("dome_pitch_servo_reverse", "Reverse the dome pitch servo", params_.domePitchServoReverse);
  addParameter("body_roll_servo_reverse", "Reverse the body roll servo", params_.bodyRollServoReverse);

  addParameter("fa_speed_sp_to_dome", "Factor to add forward speed setpoint to dome pitch", params_.faSpeedSPToDome);
  addParameter("fa_roll_sp_to_dome", "Factor to add roll setpoint to dome roll", params_.faRollSPToDome);
  addParameter("fa_dome_pitch_to_weight", "Factor to add dome pitch to weight fwd/back setpoint", params_.faDomePitchToWeight);
  addParameter("fa_dome_roll_to_weight", "Factor to add roll setpoint to dome roll", params_.faDomeRollToWeight);

  configureTimers();
  driveMotor_.setCustomAnalogWrite(&customAnalogWrite);

  driveController_.setAutoUpdate(false);
  //driveController_.setDebug(true);
  balanceController_.setAutoUpdate(false);
  balanceController_.setRamp(100);
  //balanceController_.setDebug(true);
  rollController_.setAutoUpdate(false);
#if defined(ROLL_POSITION)
  rollController_.setControlOffset(180.0);
#endif
  rollController_.setGoal(0.0);
  //rollController_.setRamp(30);  

  imu_.setRotationAroundZ(bb::IMU::ROTATE_90);

  setPacketSource(PACKET_SOURCE_DROID);

  return Subsystem::initialize();
}

Result BB8::start(ConsoleStream *stream) {
  started_ = true;
  runningStatus_ = false;
  operationStatus_ = RES_OK;
  mode_ = MODE_SPEED_ROLL_CONTROL;
  bb::XBee::xbee.addPacketReceiver(this);
  packetsReceived_ = packetsMissed_ = 0;
  memset((uint8_t *)&lastPacket_, 0, sizeof(lastPacket_));
  packetTimeout_ = 0;

  imu_.begin();
  BB8BattStatus::batt.begin();

  operationStatus_ = selfTest();

  driveMode_ = DRIVE_OFF;
  maxRollLoad_ = 0;

  pwmControl_ = false;
  driveMotor_.set(0.0f);
  driveEncoder_.setMillimetersPerTick(BODY_CIRCUMFERENCE / DRIVE_MOTOR_TICKS_PER_TURN);
  driveMotor_.setReverse(true);
  driveMotor_.setEnabled(true);

  if (!bb::Servos::servos.isStarted()) {
    if (bb::Servos::servos.start(stream) != RES_OK) {
      return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    }
  }

  setControlParameters();
  annealH_ = annealP_ = annealR_ = 0;

#if defined(ROLL_VELOCITY)
  bb::Servos::servos.setControlMode(BODY_ROLL_SERVO, bb::Servos::CONTROL_VELOCITY);
#elif defined(ROLL_POSITION)
  bb::Servos::servos.setControlMode(BODY_ROLL_SERVO, bb::Servos::CONTROL_POSITION);
#endif
  bb::Servos::servos.switchTorque(bb::Servos::ID_ALL, true);

  return RES_OK;
}

Result BB8::stop(ConsoleStream *stream) {
  (void)stream;
  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  bb::XBee::xbee.removePacketReceiver(this);
  return RES_OK;
}

Result BB8::step() {
  static int stepcount = 0;
  Runloop::runloop.excuseOverrun();

  imu_.update();

  driveEncoder_.update();

  stepcount++;
  if (stepcount == 1 && BB8BattStatus::batt.available(BB8BattStatus::BATT_1)) BB8BattStatus::batt.updateVoltage(BB8BattStatus::BATT_1);
  if (stepcount == 2 && BB8BattStatus::batt.available(BB8BattStatus::BATT_1)) BB8BattStatus::batt.updateCurrent(BB8BattStatus::BATT_1);
  if (stepcount == 3 && BB8BattStatus::batt.available(BB8BattStatus::BATT_2)) BB8BattStatus::batt.updateVoltage(BB8BattStatus::BATT_2);
  if (stepcount == 4 && BB8BattStatus::batt.available(BB8BattStatus::BATT_2)) BB8BattStatus::batt.updateCurrent(BB8BattStatus::BATT_2);
  if (stepcount == 100) stepcount = 0;

  Result res;
  res = stepDriveMotor();
  if (res != RES_OK) {
    Console::console.printfBroadcast("stepDriveMotor() failed!\n");
    return res;
  }

  res = stepRollMotor();
  if (res != RES_OK) {
    Console::console.printfBroadcast("stepRollMotor() failed!\n");
    return res;
  }
  res = stepDome();
  if (res != RES_OK) {
    Console::console.printfBroadcast("stepDome() failed!\n");
    return res;
  }

  if (runningStatus_) {
    printCurrentSystemStatus();
    Console::console.printfBroadcast("\r");
  }

  if (packetTimeout_ > 0) packetTimeout_--;

  fillAndSendStatePacket();

  return RES_OK;
}

bb::Result BB8::stepIfNotStarted() {
  if(imu_.available()) imu_.update();
#if 0
  leftEncoder_.update();
  rightEncoder_.update();
  if((Runloop::runloop.getSequenceNumber() % 4) == 0) {
    fillAndSendStatePacket();
  }
  if((Runloop::runloop.getSequenceNumber() % 100) == 0 && DOBattStatus::batt.available()) {
    DOBattStatus::batt.updateVoltage();
    DOBattStatus::batt.updateCurrent();
  }
#endif
  return RES_OK;
}


Result BB8::stepDriveMotor() {
  unsigned long timeSinceLastPrimary = WRAPPEDDIFF(millis(), msLastPrimaryCtrlPacket_, ULONG_MAX);
  if(timeSinceLastPrimary > 500 && driveMode_ != DRIVE_OFF) {
    LOG(LOG_INFO, "No control packet from primary in %ldms. Switching drive off.\n", timeSinceLastPrimary);
    switchDrive(DRIVE_OFF);
  }

  if(driveMode_ == DRIVE_OFF) {
    driveMotor_.set(0);
    return RES_OK;
  }

  balanceController_.update();
  driveController_.update();
  if(driveMode_ == DRIVE_POS) posController_.update();
  else if(driveMode_ == DRIVE_AUTO_POS) autoPosController_.update();
  return RES_OK;
}

Result BB8::stepRollMotor() {
  if(params_.rollDirect == false)
    rollController_.update();
  float load = Servos::servos.load(BODY_ROLL_SERVO);
  if(load > maxRollLoad_) maxRollLoad_ = load;
  return RES_OK;
}

Result BB8::stepDome() {
  float p, r, h, dr, dp, dh, ax, ay, az;
  imu_.getFilteredPRH(p, r, h);
  imu_.getAccelMeasurement(ax, ay, az);
  imu_.getGyroMeasurement(dp, dr, dh);

  LOG(LOG_DEBUG, "R:%f P:%f H:%f AX:%f AY:%f AZ:%f DR:%f DP:%f DH:%f\n", r, p, h, ax, ay, az, dr, dp, dh);

  float domePitch = 180 + remoteP_ + params_.faSpeedSPToDome*balanceController_.controlOffset();
  float domeRoll = 180 + remoteR_ + params_.faRollSPToDome*rollController_.goal();
  float domeHeading = 180 + remoteH_;

  if(servosOK_) {
    bb::Servos::servos.setGoalPos(DOME_PITCH_SERVO, domePitch);
    bb::Servos::servos.setGoalPos(DOME_HEADING_SERVO, domeHeading);
    bb::Servos::servos.setGoalPos(DOME_ROLL_SERVO, domeRoll);
  }
  
  if(lastPrimaryCtrlPacket_.button3 == false && (float(millis())/1000.0f > annealTime_ + params_.faDomeAnnealDelay)) {
    if(remoteP_ > 0.5) {
      remoteP_ -= annealP_;
      if(remoteP_ < 0) remoteP_ = 0;
    } else if(remoteP_ < -0.5) {
      remoteP_ += annealP_;
      if(remoteP_ > 0) remoteP_ = 0;
    } else remoteP_ = 0;

    if(remoteH_ > 0.5) {
      remoteH_ -= annealH_;
      if(remoteH_ < 0) remoteH_ = 0;
    } else if(remoteH_ < -0.5) {
      remoteH_ += annealH_;
      if(remoteH_ > 0) remoteH_ = 0;
    } else remoteH_ = 0;

    if(remoteR_ > 0.5) {
      remoteR_ -= annealR_;
      if(remoteR_ < 0) remoteR_ = 0;
    } else if(remoteR_ < -0.5) {
      remoteR_ += annealR_;
      if(remoteR_ > 0) remoteR_ = 0;
    } else remoteR_ = 0;
  } else if(lastPrimaryCtrlPacket_.button3 == true){
    annealTime_ = float(millis()) / 1000.0f;
  }

  return RES_OK;
}

Result BB8::setMode(BB8::Mode mode) {
  if (mode_ == mode) return RES_OK;
  mode_ = mode;

  if (mode_ == MODE_OFF) {
    bb::Servos::servos.switchTorque(DOME_HEADING_SERVO, false);
    bb::Servos::servos.switchTorque(DOME_ROLL_SERVO, false);
    bb::Servos::servos.switchTorque(DOME_PITCH_SERVO, false);
    bb::Servos::servos.switchTorque(BODY_ROLL_SERVO, false);
  }

  driveController_.reset();

  return RES_OK;
}

Result BB8::setParameterValue(const String& name, const String& stringVal) {
  Result retval = Subsystem::setParameterValue(name, stringVal);
  if(retval != RES_OK) return retval;

  setControlParameters();
  setServoParameters();

  return RES_OK;
}

void BB8::setControlParameters() {
  driveEncoder_.setMillimetersPerTick(BODY_CIRCUMFERENCE / DRIVE_MOTOR_TICKS_PER_TURN);
  driveEncoder_.setMode(bb::Encoder::INPUT_SPEED);
  driveEncoder_.setUnit(bb::Encoder::UNIT_MILLIMETERS);

  driveController_.setControlParameters(params_.driveSpeedKp, params_.driveSpeedKi, params_.driveSpeedKd);
  driveController_.setIBounds(-255, 255);
  driveController_.reset();

  balanceController_.setControlParameters(params_.balKp, params_.balKi, params_.balKd);
  balanceController_.reset();

  if(params_.rollDebug) rollController_.setDebug(true);
  else rollController_.setDebug(false);

  rollController_.setControlParameters(params_.rollKp, params_.rollKi, params_.rollKd);
  rollController_.setIBounds(-params_.rollIMax, params_.rollIMax);
  rollController_.setInhibit(params_.rollInhibit);
#if defined(ROLL_VELOCITY)
  rollController_.setControlBounds(-params_.rollServoVel, params_.rollServoVel);
  rollController_.setRamp(params_.rollServoAccel);
#else
  rollController_.setControlUnbounded();
  rollController_.setRamp(0.0);
#endif
  rollController_.reset();

  autoPosController_.setControlParameters(params_.autoPosKp, params_.autoPosKi, params_.autoPosKd);
  posController_.setControlParameters(params_.posKp, params_.posKi, params_.posKd);
}

void BB8::setServoParameters() {
  bb::Servos::servos.setOffset(BODY_ROLL_SERVO, params_.bodyRollOffset);
  bb::Servos::servos.setRange(BODY_ROLL_SERVO, 180-(params_.bodyRollRange*ROLL_REDUCTION), 180+(params_.bodyRollRange*ROLL_REDUCTION));
  bb::Servos::servos.setProfileVelocity(BODY_ROLL_SERVO, params_.bodyRollVel);
  bb::Servos::servos.setProfileAcceleration(BODY_ROLL_SERVO, params_.bodyRollAccel);
  bb::Servos::servos.setLoadShutdownEnabled(BODY_ROLL_SERVO, true);
  bb::Servos::servos.setInverted(BODY_ROLL_SERVO, params_.bodyRollServoReverse);

  bb::Servos::servos.setOffset(DOME_PITCH_SERVO, params_.domePitchOffset);
  bb::Servos::servos.setRange(DOME_PITCH_SERVO, 180-params_.domePitchRange, 180+params_.domePitchRange);
  bb::Servos::servos.setProfileVelocity(DOME_PITCH_SERVO, params_.domePitchVel);
  bb::Servos::servos.setInverted(DOME_PITCH_SERVO, params_.domePitchServoReverse);

  bb::Servos::servos.setOffset(DOME_HEADING_SERVO, params_.domeHeadingOffset);
  bb::Servos::servos.setRange(DOME_HEADING_SERVO, 180-params_.domeHeadingRange, 180+params_.domeHeadingRange);
  bb::Servos::servos.setProfileVelocity(DOME_HEADING_SERVO, params_.domeHeadingVel);
  bb::Servos::servos.setInverted(DOME_HEADING_SERVO, params_.domeHeadingServoReverse);

  bb::Servos::servos.setOffset(DOME_ROLL_SERVO, params_.domeRollOffset);
  bb::Servos::servos.setRange(DOME_ROLL_SERVO, 180-params_.domeRollRange, 180+params_.domeRollRange);
  bb::Servos::servos.setProfileVelocity(DOME_ROLL_SERVO, params_.domeRollVel);
  bb::Servos::servos.setInverted(DOME_ROLL_SERVO, params_.domeRollServoReverse);
}

Result BB8::selfTest(ConsoleStream *stream) {
  Runloop::runloop.excuseOverrun();

  Console::console.printfBroadcast("BB8 Self Test\n=============\n");

  bb::printf("Servos\n");
  Result res = servoTest(stream);
  if(res != RES_OK) {
    bb::printf("Servo test failed.\n");
    return res;
  }

  float p, r, h;
  if(imu_.available() == false) {
    Console::console.printfBroadcast("Critical error: IMU not available!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  for(unsigned int i=0; i<100; i++) {
    imu_.update(true);
  }
  imu_.getFilteredPRH(p, r, h);
  if(fabs(p) > 5 || fabs(r) > 5) {
    LOG(LOG_FATAL, "Fatal: Droid not upright (pitch %.2f, roll %.2f)!\n", p, r);
    //return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  LOG(LOG_INFO, "IMU OK. Pitch %.2f, Roll %.2f.\n", p, r);

  if(BB8BattStatus::batt.available(BB8BattStatus::BATT_1) == false) {
    Console::console.printfBroadcast("Critical error: Battery monitor not available!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  BB8BattStatus::batt.updateVoltage();
  BB8BattStatus::batt.updateCurrent();
  Console::console.printfBroadcast("Battery monitor OK. Battery 1: %fV %fmA Battery 2: %fV %fmA\n", 
                                   BB8BattStatus::batt.voltage(BB8BattStatus::BATT_1), BB8BattStatus::batt.current(BB8BattStatus::BATT_1),
                                   BB8BattStatus::batt.voltage(BB8BattStatus::BATT_2), BB8BattStatus::batt.current(BB8BattStatus::BATT_2));

  return RES_OK;
}

Result BB8::servoTest(ConsoleStream *stream) {
  // Check servos
  servosOK_ = false;
  if(bb::Servos::servos.isStarted() == false) {
    Console::console.printfBroadcast("Critical error: Servo subsystem not started!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  bb::Servos::servos.switchTorque(Servos::ID_ALL, false);
  setServoParameters();
  bb::Servos::servos.switchTorque(Servos::ID_ALL, true);

  if(bb::Servos::servos.home(BODY_ROLL_SERVO, 5.0, 95, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing body roll servo failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }   
  rollController_.setGoal(0.0);

  if(bb::Servos::servos.home(DOME_PITCH_SERVO, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing dome pitch failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  if(bb::Servos::servos.home(DOME_ROLL_SERVO, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing dome roll servo failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  if(bb::Servos::servos.home(DOME_HEADING_SERVO, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing dome heading servo failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  servosOK_ = true;
  Console::console.printfBroadcast("Servos OK.\n");
  return RES_OK;
}


void BB8::printExtendedStatus(ConsoleStream *stream) {
  if (stream == NULL) return;

  stream->printf("%s (%s): ", name(), description());

  if (isStarted()) {
    stream->printf("started, ");
    switch (operationStatus()) {
      case RES_OK:
        stream->printf("operational");
        break;
      default:
        stream->printf("not operational: ");
        stream->printf(errorMessage(operationStatus()));
        break;
    }
  } else stream->printf("stopped");

  float p, r, h;
  if(imu_.available()) {
    imu_.getFilteredPRH(p, r, h);
    stream->printf(", IMU: P%.1f R%.1f H%.1f", p, r, h);
  } else {
    stream->printf(", IMU n/a");
  }

  if (BB8BattStatus::batt.available(BB8BattStatus::BATT_1)) {
    stream->printf(", batt1: %fV %fmA", BB8BattStatus::batt.voltage(BB8BattStatus::BATT_1), BB8BattStatus::batt.current(BB8BattStatus::BATT_1));
  } else {
    stream->printf(", batt1 n/a");
  }

  if (BB8BattStatus::batt.available(BB8BattStatus::BATT_2)) {
    stream->printf(", batt2: %fV %fmA", BB8BattStatus::batt.voltage(BB8BattStatus::BATT_2), BB8BattStatus::batt.current(BB8BattStatus::BATT_2));
  } else {
    stream->printf(", batt2 n/a");
  }

  stream->printf(", roll servo max load %f\n", maxRollLoad_);
}

void BB8::switchDrive(DriveMode mode) {
  driveController_.reset();
  driveController_.setGoal(0);

  balanceController_.reset();
  balanceController_.setControlOffset(0);
  balanceController_.setGoal(0);

  autoPosController_.reset();
  autoPosController_.setPresentAsGoal();

  posController_.reset();
  posController_.setPresentAsGoal();
  posControllerZero_ = posController_.present();

  driveMode_ = mode;
  switch(driveMode_) {
    case DRIVE_OFF: 
      setLED(LED_DRIVE, OFF);
      LOG(LOG_INFO, "Switched drive mode to off.\n");
      break;
    
    case DRIVE_VEL: 
      setLED(LED_DRIVE, GREEN);
      LOG(LOG_INFO, "Switched drive mode to velocity.\n");
      break;

    case DRIVE_POS: 
      setLED(LED_DRIVE, YELLOW);
      LOG(LOG_INFO, "Switched drive mode to position.\n");
      break;

    case DRIVE_AUTO_POS:
    default: 
      setLED(LED_DRIVE, BLUE);
      LOG(LOG_INFO, "Switched drive mode to auto_position.\n");
      break;
  }
}


Result BB8::incomingControlPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const ControlPacket& packet) {
  if(commLEDOn_ == false) {
    unsigned long ms = millis();
    if(ms - msLastLeftCtrlPacket_ < 100) {
      if(ms - msLastRightCtrlPacket_ < 100) {
        // both coming in - white
        setLED(LED_COMM, WHITE);
      } else {
        // Only left coming in - blue
        setLED(LED_COMM, BLUE);
      }
    } else {
      // Only right coming in - green
      setLED(LED_COMM, GREEN);
    }
  
    commLEDOn_ = true;
    Runloop::runloop.scheduleTimedCallback(100, [=]{ setLED(LED_COMM, OFF); commLEDOn_ = false; });
  }

  // Check for duplicates and lost packets
  if(source == PACKET_SOURCE_LEFT_REMOTE) {
    if(seqnum == lastLeftSeqnum_) return RES_OK; // duplicate
    if(WRAPPEDDIFF(seqnum, lastLeftSeqnum_, 8) != 1) {
      LOG(LOG_WARN, "Left control packet: seqnum %d, last %d, lost %d packets!\n", 
                    seqnum, lastLeftSeqnum_, WRAPPEDDIFF(seqnum, lastLeftSeqnum_, 8)-1);
    }
    lastLeftSeqnum_ = seqnum;
    numLeftCtrlPackets_++;
    msLastLeftCtrlPacket_ = millis();
  } else if(source == PACKET_SOURCE_RIGHT_REMOTE) {
    if(seqnum == lastRightSeqnum_) return RES_OK; // duplicate
    if(WRAPPEDDIFF(seqnum, lastRightSeqnum_, 8) != 1) {
      LOG(LOG_WARN, "Right control packet: seqnum %d, last %d, lost %d packets!\n", 
                    seqnum, lastRightSeqnum_, WRAPPEDDIFF(seqnum, lastRightSeqnum_, 8)-1);
    }
    lastRightSeqnum_ = seqnum;
    numRightCtrlPackets_++;
    msLastRightCtrlPacket_ = millis();
  } else {
    LOG(LOG_ERROR, "Control packet from unknown source %d\n", source);
    return RES_SUBSYS_COMM_ERROR;
  }

  if(packet.primary == true) {
    msLastPrimaryCtrlPacket_ = millis();
    
    // Joystick button toggles drive mode: OFF/VEL or OFF/AUTO_POS (if enabled)
    if(packet.button4 && !lastPrimaryCtrlPacket_.button4) {
      if(driveMode_ == DRIVE_OFF || driveMode_ == DRIVE_POS) {
        switchDrive(params_.autoPosControl == true ? DRIVE_AUTO_POS : DRIVE_VEL);
      } else {
        switchDrive(DRIVE_OFF);
      }
    }
  
    if(driveMode_ != DRIVE_OFF) {
      float vel = packet.getAxis(1);

      // In DRIVE_VEL, if no drive input for a while, switch to AUTO_POS if enabled
      if(EPSILON(vel) && params_.autoPosControl == true) { 
        if(WRAPPEDDIFF(millis(), msSinceDriveInput_, ULONG_MAX) > 500 && 
           driveMode_ == DRIVE_VEL) {
          switchDrive(DRIVE_AUTO_POS);
        }
      } else { // We have drive input - switch to velocity control mode
        if(driveMode_ == DRIVE_AUTO_POS) {
          switchDrive(DRIVE_VEL);
        }
        msSinceDriveInput_ = millis();
      }

      if(driveMode_ == DRIVE_VEL) {
        if(fabs(vel) <= params_.driveSpeedDeadband) {
          vel = 0;
        } else {
          vel *= params_.driveSpeedMax;
        }

        balanceController_.setControlOffset(vel);
      } else if(driveMode_ == DRIVE_POS) {
        float posDelta = packet.getAxis(1) * 1000;
        posController_.setGoal(posControllerZero_ + posDelta);
      }
    } else {
      setLED(LED_DRIVE, OFF);
    }

    float bodyRollInput = -packet.getAxis(0) * params_.rollAngleMax;
    if(params_.rollDirect) Servos::servos.setGoalPos(BODY_ROLL_SERVO, bodyRollInput * ROLL_REDUCTION + 180.0);
    else {
      rollController_.setGoal(bodyRollInput);
    }

    if(packet.button3) {
      remoteR_ = packet.getAxis(2, ControlPacket::UNIT_DEGREES_CENTERED);
      remoteP_ = packet.getAxis(3, ControlPacket::UNIT_DEGREES_CENTERED);
      remoteH_ = packet.getAxis(4, ControlPacket::UNIT_DEGREES_CENTERED);
      annealR_ = fabs(remoteR_ / (params_.faDomeAnnealTime / bb::Runloop::runloop.cycleTimeSeconds()));
      annealP_ = fabs(remoteP_ / (params_.faDomeAnnealTime / bb::Runloop::runloop.cycleTimeSeconds()));
      annealH_ = fabs(remoteH_ / (params_.faDomeAnnealTime / bb::Runloop::runloop.cycleTimeSeconds()));
    }

    lastPrimaryCtrlPacket_ = packet;
  }

  return RES_OK;
}

Result BB8::handleConsoleCommand(const std::vector<String> &words, ConsoleStream *stream) {
  if (words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if(words[0] == "selftest") {
    if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
    Result res = selfTest(stream);
    stream->printf("%s\n", errorMessage(res));
    return RES_OK;
  }

  else if (words[0] == "running_status") {
    if (words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if (words[1] == "on" || words[1] == "true") {
      runningStatus_ = true;
      return RES_OK;
    } else if (words[1] == "off" || words[1] == "false") {
      runningStatus_ = false;
      return RES_OK;
    }
    return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "play_sound") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    int folder = words[1].toInt();
    int sound = words[2].toInt();
    if (BB8Sound::sound.playFolder(folder, sound) == false) return RES_CMD_FAILURE;
    return RES_OK;
  }

  else if (words[0] == "calibrate") {
    imu_.calibrate(stream);
  }

  else if (words[0] == "drive") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;

    if (words[1] == "pwm") {
      pwmControl_ = true;
      driveMotor_.set(words[2].toFloat());
      driveMotor_.setEnabled(true);
      return RES_OK;
    } else if (words[1] == "position") {
      pwmControl_ = false;
      driveEncoder_.setMode(bb::Encoder::INPUT_POSITION);
      driveController_.setGoal(words[2].toFloat());
      driveMotor_.setEnabled(true);
      return RES_OK;
    } else if (words[1] == "speed") {
      pwmControl_ = false;
      driveEncoder_.setMode(bb::Encoder::INPUT_SPEED);
      driveController_.setGoal(words[2].toFloat());
      return RES_OK;
    } else return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "mode") {
    if (words.size() == 1) {
      switch (mode_) {
        case MODE_OFF:
          if (stream) stream->printf("off\n");
          break;
        case MODE_ROLL_CONTROL_ONLY:
          if (stream) stream->printf("roll\n");
          break;
        case MODE_SPEED_CONTROL_ONLY:
          if (stream) stream->printf("speed\n");
          break;
        case MODE_SPEED_ROLL_CONTROL:
          if (stream) stream->printf("speed_roll\n");
          break;
        case MODE_POS_CONTROL:
          if (stream) stream->printf("pos\n");
          break;
        case MODE_KIOSK:
          if (stream) stream->printf("kiosk\n");
          break;
        case MODE_CALIB:
          if (stream) stream->printf("calib\n");
          break;
        default:
          if (stream) stream->printf("unknown\n");
          break;
      }
    } else if (words.size() == 2) {
      if (words[1] == "off") {
        mode_ = MODE_OFF;
        return RES_OK;
      } else if (words[1] == "roll") {
        mode_ = MODE_ROLL_CONTROL_ONLY;
        return RES_OK;
      } else if (words[1] == "speed") {
        mode_ = MODE_SPEED_CONTROL_ONLY;
        return RES_OK;
      } else if (words[1] == "speed_roll") {
        mode_ = MODE_SPEED_ROLL_CONTROL;
        return RES_OK;
      } else if (words[1] == "pos") {
        mode_ = MODE_POS_CONTROL;
        return RES_OK;
      } else if (words[1] == "kiosk") {
        mode_ = MODE_KIOSK;
        return RES_OK;
      } else if (words[1] == "calib") {
        mode_ = MODE_CALIB;
        return RES_OK;
      } else return RES_CMD_INVALID_ARGUMENT;
    } else
      return RES_CMD_INVALID_ARGUMENT_COUNT;
  }

  else if (words[0] == "set_pixel") {
    if (words.size() != 5) return RES_CMD_INVALID_ARGUMENT_COUNT;
    uint8_t p, r, g, b;
    p = words[1].toInt();
    r = words[2].toInt();
    g = words[3].toInt();
    b = words[4].toInt();

    setLED(WhichLED(p), r, g, b);

    return RES_OK;
  }

  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result BB8::fillAndSendStatePacket() {
  return RES_OK;
}

void BB8::printCurrentSystemStatus(ConsoleStream *stream) {
  float r, p, h;
  imu_.getFilteredPRH(p, r, h);
  String str = "Buttons: ";
  lastPacket_.button0 ? str += 'X' : str += '_';
  lastPacket_.button1 ? str += 'X' : str += '_';
  lastPacket_.button2 ? str += 'X' : str += '_';
  lastPacket_.button3 ? str += 'X' : str += '_';
  lastPacket_.button4 ? str += 'X' : str += '_';
  str += " Axes: ";
  for (int i = 0; i < 5; i++) {
    str += lastPacket_.getAxis(i);
    str += " ";
  }

  str += String("Gyro: R") + r + " P" + p + " H" + h;

  if (stream != NULL) stream->printf("%s\n", str.c_str());
  else Console::console.printfBroadcast("%s\n", str.c_str());
}

Result BB8::setLED(WhichLED which, uint8_t r, uint8_t g, uint8_t b) {
  statusPixels_.setPixelColor(int(which), r, g, b);
  statusPixels_.show();
  return RES_OK;
}

Result BB8::setLED(WhichLED which, WhatColor color) {
  switch(color) {
    case RED: return setLED(which, 255, 0, 0); break;
    case GREEN: return setLED(which, 0, 255, 0); break;
    case BLUE: return setLED(which, 0, 0, 255); break;
    case YELLOW: return setLED(which, 255, 255, 0); break;
    case WHITE: return setLED(which, 255, 255, 255); break;
    case OFF: default: return setLED(which, 0, 0, 0); break;
  }
  return RES_COMMON_NOT_IN_LIST;
}

void BB8::setLEDBrightness(uint8_t brightness) {
  statusPixels_.setBrightness(brightness);
  statusPixels_.show();
}