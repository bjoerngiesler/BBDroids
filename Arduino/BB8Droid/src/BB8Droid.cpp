#include "BB8Droid.h"
#include "BB8Config.h"
#include "BB8StatusPixels.h"
#include "BB8BattStatus.h"
#include "BB8Sound.h"

#define USE_ROLL_CONTROLLER

BB8 BB8::bb8;
BB8Params BB8::params_;

ServoLimits servolimits[] = {
  { 0.0f, 360.0f, 0.0f, 60.0 },
  { 120.0f, 240.0f, 0.0f, 60.0 },
  { 120.0f, 240.0f, 0.0f, 60.0 },
  { 160.0f, 200.0f, 0.0f, 80.0 }
};

BB8::BB8(): 
    imu_(IMU_ADDR),
    driveMotor_(P_DRIVE_A, P_DRIVE_B, P_DRIVE_PWM, P_DRIVE_EN),
    yawMotor_(P_YAW_A, P_YAW_B, P_YAW_PWM, P_YAW_EN),
    driveEncoder_(P_DRIVEENC_A, P_DRIVEENC_B, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
    balanceInput_(imu_, IMUControlInput::IMU_PITCH),
    rollInput_(imu_, IMUControlInput::IMU_ROLL, true),
    rollOutput_(BODY_ROLL_SERVO),
    driveController_(driveEncoder_, driveMotor_),
    balanceController_(balanceInput_, driveController_),
    rollController_(rollInput_, rollOutput_) {
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
}

Result BB8::initialize() {
  addParameter("body_roll_range", "Body roll servo plus-minus range", params_.bodyRollRange);
  addParameter("body_roll_offset", "Body roll servo offset", params_.bodyRollOffset);
  addParameter("dome_pitch_range", "Body roll servo plus-minus range",  params_.domePitchRange);
  addParameter("dome_pitch_offset", "Plus-minus range for the body roll servo",  params_.domePitchOffset);
  addParameter("dome_heading_range", "Plus-minus range for the body roll servo",  params_.domeHeadingRange);
  addParameter("dome_heading_offset", "Plus-minus range for the body roll servo",  params_.domeHeadingOffset);
  addParameter("dome_roll_range", "Plus-minus range for the body roll servo",  params_.domeRollRange);
  addParameter("dome_roll_offset", "Plus-minus range for the body roll servo",  params_.domeRollOffset);

  addParameter("drive_speed_kp", "Proportional constant for drive speed controller", params_.driveSpeedKp);
  addParameter("drive_speed_ki", "Integral constant for drive speed controller", params_.driveSpeedKi);
  addParameter("drive_speed_kd", "Derivative constant for drive speed controller", params_.driveSpeedKd);
  addParameter("bal_kp", "Proportional constant for drive balance controller", params_.balKp);
  addParameter("bal_ki", "Integral constant for drive balance controller", params_.balKi);
  addParameter("bal_kd", "Derivative constant for drive balance controller", params_.balKd);
  addParameter("roll_kp", "Proportional constant for roll controller", params_.rollKp);
  addParameter("roll_ki", "Integral constant for roll controller", params_.rollKi);
  addParameter("roll_kd", "Derivative constant for roll controller", params_.rollKd);
  addParameter("roll_servo_vel", "Velocity for roll servo", params_.rollServoVel);
  addParameter("roll_servo_accel", "Acceleration for roll servo", params_.rollServoAccel);

  addParameter("drive_speed_imax", "Max integral error for drive speed controller", params_.driveSpeedIMax);
  addParameter("drive_speed_deadband", "Deadband for drive speed controller", params_.driveSpeedDeadband);
  addParameter("drive_speed_iabort", "Drive speed controller aborts if integral error exceeds this", params_.driveSpeedIAbort);
  addParameter("drive_speed_max", "Max speed for drive controller", params_.driveSpeedMax);

  addParameter("roll_imax", "Max integral error for roll controller", params_.rollIMax);
  addParameter("roll_torque_percent", "Abort roll motion if this percentage of torque is exceeded", params_.rollTorquePercent);
  addParameter("dome_max_vel", "Max velocity for dome motion", params_.domeMaxVel);

  addParameter("dome_heading_servo_reverse", "Reverse the dome heading servo", params_.domeHeadingServoReverse);
  addParameter("dome_roll_servo_reverse", "Reverse the dome roll servo", params_.domeRollServoReverse);
  addParameter("dome_pitch_servo_reverse", "Reverse the dome pitch servo", params_.domePitchServoReverse);
  addParameter("body_roll_servo_reverse", "Reverse the body roll servo", params_.bodyRollServoReverse);

  driveController_.setAutoUpdate(false);
  //driveController_.setDebug(true);
  balanceController_.setAutoUpdate(false);
  balanceController_.setRamp(100);
  //balanceController_.setDebug(true);
  rollController_.setAutoUpdate(false);
  rollController_.setControlOffset(180.0);
  rollController_.setGoal(0.0);
  rollController_.setRamp(30);
  //rollController_.setDebug(true);

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
  BB8StatusPixels::statusPixels.update();

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
  //driveMotor_.brake();
  balanceController_.update();
  driveController_.update();
  //Console::console.printfBroadcast("Drive motor setpoint: %f\n", driveMotor_.present());

  return RES_OK;
}

Result BB8::stepRollMotor() {
  rollController_.update();
  return RES_OK;
}

Result BB8::stepDome() {
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
  driveController_.setControlParameters(params_.driveSpeedKp, params_.driveSpeedKi, params_.driveSpeedKd);
  driveController_.setIBounds(-params_.driveSpeedIMax, params_.driveSpeedIMax);
  driveController_.setGoal(0);
  driveController_.reset();

  balanceController_.setControlParameters(params_.balKp, params_.balKi, params_.balKd);
  balanceController_.reset();

  rollController_.setControlParameters(params_.rollKp, params_.rollKi, params_.rollKd);
  rollController_.setIBounds(-params_.rollIMax, params_.rollIMax);
  rollController_.reset();
}

void BB8::setServoParameters() {
  bb::Servos::servos.setOffset(BODY_ROLL_SERVO, params_.bodyRollOffset);
  bb::Servos::servos.setRange(BODY_ROLL_SERVO, 180-params_.bodyRollRange, 180+params_.bodyRollRange);
  bb::Servos::servos.setProfileVelocity(BODY_ROLL_SERVO, params_.rollServoVel);
  bb::Servos::servos.setProfileAcceleration(BODY_ROLL_SERVO, params_.rollServoAccel);
  bb::Servos::servos.setLoadShutdownEnabled(BODY_ROLL_SERVO, true);

  bb::Servos::servos.setOffset(DOME_PITCH_SERVO, params_.domePitchOffset);
  bb::Servos::servos.setRange(DOME_PITCH_SERVO, 180-params_.domePitchRange, 180+params_.domePitchRange);
  bb::Servos::servos.setProfileVelocity(DOME_PITCH_SERVO, params_.domeMaxVel);

  bb::Servos::servos.setOffset(DOME_HEADING_SERVO, params_.domeHeadingOffset);
  bb::Servos::servos.setRange(DOME_HEADING_SERVO, 180-params_.domeHeadingRange, 180+params_.domeHeadingRange);
  bb::Servos::servos.setProfileVelocity(DOME_HEADING_SERVO, params_.domeMaxVel);

  bb::Servos::servos.setOffset(DOME_ROLL_SERVO, params_.domeRollOffset);
  bb::Servos::servos.setRange(DOME_ROLL_SERVO, 180-params_.domeRollRange, 180+params_.domeRollRange);
  bb::Servos::servos.setProfileVelocity(DOME_ROLL_SERVO, params_.domeMaxVel);
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

  if(bb::Servos::servos.home(DOME_HEADING_SERVO, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing dome heading servo failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  if(bb::Servos::servos.home(DOME_ROLL_SERVO, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing dome roll servo failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  servosOK_ = true;
  Console::console.printfBroadcast("Servos OK.\n");
  return RES_OK;
}


void BB8::printStatus(ConsoleStream *stream) {
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

  stream->printf("\n");
}

Result BB8::incomingControlPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const ControlPacket& packet) {
#if defined(USE_ROLL_CONTROLLER)
  float bodyRollInput = packet.getAxis(0) * 25.0;
  //bb::printf("Body roll: %f\n", bodyRollInput);
  rollController_.setGoal(bodyRollInput);
  return RES_OK;
#else
  float bodyRollInput = packet.getAxis(0) * BODY_ROLL_RANGE;
  Servos::servos.setProfileVelocity(BODY_ROLL_SERVO, 100);
  Servos::servos.setGoal(BODY_ROLL_SERVO, 180+bodyRollInput);
  return RES_OK;
#endif

  float velInput = packet.getAxis(1);
  float domeRollInput = packet.getAxis(2);
  float domePitchInput = packet.getAxis(3);
  float domeHeadingInput = packet.getAxis(4);
  
  if(packet.button1) {
    float vel = params_.driveSpeedMax * velInput;  // magic
    driveEncoder_.setMode(bb::Encoder::INPUT_SPEED);
    //Console::console.printfBroadcast("Setting velocity to %f\n", vel);
    balanceController_.setControlOffset(-vel);
    
    float roll = bodyRollInput * 20.0;
    Console::console.printfBroadcast("Setting body roll goal to %f\n", roll);
    rollController_.setGoal(roll);    
    //rollController_.setControlOffset(roll+180.0);
  } 
  else {
    driveEncoder_.setMode(bb::Encoder::INPUT_SPEED);
    driveController_.setGoal(0.0f);
    rollController_.setGoal(0.0);
    //rollController_.setControlOffset(180.0);
  }

  float r, p, h;
  imu_.getFilteredPRH(p, r, h);
  static float domeRollZero = 0, domePitchZero = 0, domeHeadingZero = 0;

  if(packet.button2) {
    if(!lastPacket_.button2) { // fresh press - use current values as zero
      domeRollZero = domeRollInput;
      domePitchZero = domePitchInput;
      domeHeadingZero = domeHeadingInput;
    }
    domeRollInput -= domeRollZero;
    domePitchInput -= domePitchZero;
    domeHeadingInput -= domeHeadingZero;

    
    if(params_.domeRollServoReverse) domeRollInput = 180.0 - ((domeRollInput*30.0)*4 - 2*(bb::Servos::servos.present(BODY_ROLL_SERVO)-180.0));
    else domeRollInput = 180.0 + ((domeRollInput*30.0)*4 - 2*(bb::Servos::servos.present(BODY_ROLL_SERVO)-180.0));
    if(params_.domePitchServoReverse) domePitchInput = 180.0 - ((domePitchInput*30.0)*4);
    else domePitchInput = 180.0 + ((domePitchInput*30.0)*4);
    if(params_.domeHeadingServoReverse) domeHeadingInput = 180.0 - ((domeHeadingInput*30.0)*4);
    else domeHeadingInput = 180.0 + ((domeHeadingInput*30.0)*4);
      
    bb::Servos::servos.setGoal(DOME_ROLL_SERVO, domeRollInput);
    bb::Servos::servos.setGoal(DOME_PITCH_SERVO, domePitchInput);
    bb::Servos::servos.setGoal(DOME_HEADING_SERVO, domeHeadingInput);
  } else {
    bb::Servos::servos.setGoal(DOME_ROLL_SERVO, 180.0);
    bb::Servos::servos.setGoal(DOME_PITCH_SERVO, 180.0);
    bb::Servos::servos.setGoal(DOME_HEADING_SERVO, 180.0);
  }

  packetTimeout_ = 3;
  lastPacket_ = packet;

  return RES_OK;
}

Result BB8::handleConsoleCommand(const std::vector<String> &words, ConsoleStream *stream) {
  if (words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if (words[0] == "status") {
    if (words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
    printCurrentSystemStatus(stream);
    stream->printf("\n");
    return RES_OK;
  }

  else if(words[0] == "selftest") {
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

    BB8StatusPixels::statusPixels.setPixel(p, r, g, b);
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