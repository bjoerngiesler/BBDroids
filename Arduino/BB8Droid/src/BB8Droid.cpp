#include "BB8Droid.h"
#include "BB8Config.h"
#include "BB8StatusPixels.h"
#include "BB8BattStatus.h"
#include "BB8Sound.h"

BB8 BB8::bb8;

ServoLimits servolimits[] = {
  { 0.0f, 360.0f, 0.0f, 60.0 },
  { 120.0f, 240.0f, 0.0f, 60.0 },
  { 120.0f, 240.0f, 0.0f, 60.0 },
  { 160.0f, 200.0f, 0.0f, 80.0 }
};

BB8::BB8Params BB8::params_ = {
  .driveSpeedKp = DRIVE_SPEED_KP, 
  .driveSpeedKi = DRIVE_SPEED_KI,
  .driveSpeedKd = DRIVE_SPEED_KD,
  .balKp = BAL_KP,
  .balKi = BAL_KI, 
  .balKd = BAL_KD,
  .rollKp = ROLL_KP,
  .rollKi = ROLL_KI,
  .rollKd = ROLL_KD,
  .rollServoKp = ROLL_SERVO_KP,
  .rollServoKi = ROLL_SERVO_KI,
  .rollServoKd = ROLL_SERVO_KD,
  .rollServoVel = ROLL_SERVO_VEL
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
          "        set_pixel <num> <r> <g> <b>     Set Neopixel <num> (1-3) to rgb color\r\n"
          "        scan_i2c                        Scan the i2c bus";
  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
}

Result BB8::initialize() {
  addParameter("drive_speed_kp", "Proportional constant for drive speed controller", params_.driveSpeedKp);
  addParameter("drive_speed_ki", "Integral constant for drive speed controller", params_.driveSpeedKi);
  addParameter("drive_speed_kd", "Derivative constant for drive speed controller", params_.driveSpeedKd);
  addParameter("bal_kp", "Proportional constant for drive balance controller", params_.balKp);
  addParameter("bal_ki", "Integral constant for drive balance controller", params_.balKi);
  addParameter("bal_kd", "Derivative constant for drive balance controller", params_.balKd);
  addParameter("roll_kp", "Proportional constant for roll controller", params_.rollKp);
  addParameter("roll_ki", "Integral constant for roll controller", params_.rollKi);
  addParameter("roll_kd", "Derivative constant for roll controller", params_.rollKd);
  addParameter("roll_servo_kp", "Proportional constant for roll servo", params_.rollServoKp);
  addParameter("roll_servo_ki", "Integral constant for roll servo", params_.rollServoKi);
  addParameter("roll_servo_kd", "Derivative constant for roll servo", params_.rollServoKd);
  addParameter("roll_servo_vel", "Velocity for roll servo", params_.rollServoVel);
  addParameter("drive_speed_goal", "Drive speed goal", driveSpeedGoal_);

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
  //rollInput_.setBias(180.0);

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
    Console::console.printfBroadcast("stepDriveMotor() failed!\n");
    return res;
  }
//  res = stepDome();
  if (res != RES_OK) {
    Console::console.printfBroadcast("stepDriveMotor() failed!\n");
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

  return RES_OK;
}

void BB8::setControlParameters() {
  driveController_.setControlParameters(params_.driveSpeedKp, params_.driveSpeedKi, params_.driveSpeedKd);
  //driveController_.setIBounds(-100.0, 100.0);
  driveController_.setGoal(0);
  driveController_.reset();

  balanceController_.setControlParameters(params_.balKp, params_.balKi, params_.balKd);
  balanceController_.reset();

  rollController_.setControlParameters(params_.rollKp, params_.rollKi, params_.rollKd);
  rollController_.setIBounds(-ROLL_IMAX, ROLL_IMAX);
  rollController_.reset();

  Servos::servos.switchTorque(BODY_ROLL_SERVO, false);
  Servos::servos.setPIDValues(BODY_ROLL_SERVO, params_.rollServoKp, params_.rollServoKi, params_.rollServoKd);
  Servos::servos.setProfileVelocity(BODY_ROLL_SERVO, params_.rollServoVel);
  Servos::servos.switchTorque(BODY_ROLL_SERVO, true);
}


Result BB8::selfTest(ConsoleStream *stream) {
  Runloop::runloop.excuseOverrun();

  Console::console.printfBroadcast("BB8 Self Test\n=============\n");

  if(imu_.available() == false) {
    Console::console.printfBroadcast("Critical error: IMU not available!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  imu_.update(true);
  float ax, ay, az;

  imu_.getAccelMeasurement(ax, ay, az);
#if 0 // FIXME need to move this until *after* servo selftest for BB-8
  if(fabs(ax) > 0.15 || fabs(ay) > 0.15 || fabs(az) < 0.9) {
    Console::console.printfBroadcast("Critical error: Droid not upright (ax %f, ay %f, az %f)!\n", ax, ay, az);
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
#endif
  Console::console.printfBroadcast("IMU OK. Down vector: %.2f %.2f %.2f\n", ax, ay, az);

  if(BB8BattStatus::batt.available(BB8BattStatus::BATT_1) == false) {
    Console::console.printfBroadcast("Critical error: Battery monitor not available!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  BB8BattStatus::batt.updateVoltage();
  BB8BattStatus::batt.updateCurrent();
  Console::console.printfBroadcast("Battery monitor OK. Battery 1: %fV %fmA Battery 2: %fV %fmA\n", 
                                   BB8BattStatus::batt.voltage(BB8BattStatus::BATT_1), BB8BattStatus::batt.current(BB8BattStatus::BATT_1),
                                   BB8BattStatus::batt.voltage(BB8BattStatus::BATT_2), BB8BattStatus::batt.current(BB8BattStatus::BATT_2));

  Result res = servoTest(stream);  

  return RES_OK;
}

Result BB8::servoTest(ConsoleStream *stream) {
  // Check servos
  servosOK_ = false;
  if(bb::Servos::servos.isStarted() == false) {
    Console::console.printfBroadcast("Critical error: Servo subsystem not started!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  if(bb::Servos::servos.hasServoWithID(BODY_ROLL_SERVO) == false) {
    Console::console.printfBroadcast("Critical error: Body roll servo missing!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else if(bb::Servos::servos.home(BODY_ROLL_SERVO, 5.0, 95, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing body roll servo failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    bb::Servos::servos.switchTorque(BODY_ROLL_SERVO, false);
    bb::Servos::servos.setRange(BODY_ROLL_SERVO, 180-BODY_ROLL_RANGE, 180+BODY_ROLL_RANGE);
    bb::Servos::servos.setOffset(BODY_ROLL_SERVO, BODY_ROLL_OFFSET);
    bb::Servos::servos.setProfileVelocity(BODY_ROLL_SERVO, params_.rollServoVel);
    bb::Servos::servos.setCompliantMode(BODY_ROLL_SERVO, ROLL_TORQUE_PERCENT);
    bb::Servos::servos.setPIDValues(BODY_ROLL_SERVO, params_.rollServoKp, params_.rollServoKi, params_.rollServoKd);
    bb::Servos::servos.setLoadShutdownEnabled(BODY_ROLL_SERVO, false);
    bb::Servos::servos.switchTorque(BODY_ROLL_SERVO, true);
    rollController_.setGoal(0.0);
  }

  if(bb::Servos::servos.hasServoWithID(DOME_PITCH_SERVO) == false) {
    Console::console.printfBroadcast("Degraded: Dome pitch servo missing.\n");
  } else if(bb::Servos::servos.home(DOME_PITCH_SERVO, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing dome pitch failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    bb::Servos::servos.setRange(DOME_PITCH_SERVO, 180-DOME_PITCH_RANGE, 180+DOME_PITCH_RANGE);
    bb::Servos::servos.setOffset(DOME_PITCH_SERVO, DOME_PITCH_OFFSET);
    bb::Servos::servos.setProfileVelocity(DOME_PITCH_SERVO, 50);
  }

  if(bb::Servos::servos.hasServoWithID(DOME_HEADING_SERVO) == false) {
    Console::console.printfBroadcast("Degraded: Dome heading servo missing.\n");
  } else if(bb::Servos::servos.home(DOME_HEADING_SERVO, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing dome heading servo failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    bb::Servos::servos.setRange(DOME_HEADING_SERVO, 180-DOME_HEADING_RANGE, 180+DOME_HEADING_RANGE);
    bb::Servos::servos.setOffset(DOME_HEADING_SERVO, DOME_HEADING_OFFSET);
  }

  if(bb::Servos::servos.hasServoWithID(DOME_ROLL_SERVO) == false) {
    Console::console.printfBroadcast("Degraded: Dome roll servo missing.\n");
  } else if(bb::Servos::servos.home(DOME_ROLL_SERVO, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing dome roll servo failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    bb::Servos::servos.setRange(DOME_ROLL_SERVO, 180-DOME_ROLL_RANGE, 180+DOME_ROLL_RANGE);
    bb::Servos::servos.setOffset(DOME_ROLL_SERVO, DOME_ROLL_OFFSET);
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

Result BB8::incomingControlPacket(uint16_t station, PacketSource source, uint8_t rssi, const ControlPacket& packet) {
  if (source == PACKET_SOURCE_TEST_ONLY) {
    Console::console.printfBroadcast("Test packet received!\n");
    return RES_OK;
  }

  float bodyRollInput = packet.getAxis(0);
  float velInput = packet.getAxis(1);
  float domeRollInput = packet.getAxis(2);
  float domePitchInput = packet.getAxis(3);
  float domeHeadingInput = packet.getAxis(4);
  
  if(packet.button1) {
    float vel = DRIVE_SPEED_MAX * velInput;  // magic
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
  imu_.getFilteredRPH(r, p, h);
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

    
    if(DOME_ROLL_SERVO_REVERSE) domeRollInput = 180.0 - ((domeRollInput*30.0)*4/AXIS_MAX - 2*(bb::Servos::servos.present(BODY_ROLL_SERVO)-180.0));
    else domeRollInput = 180.0 + ((domeRollInput*30.0)*4/AXIS_MAX - 2*(bb::Servos::servos.present(BODY_ROLL_SERVO)-180.0));
    if(DOME_PITCH_SERVO_REVERSE) domePitchInput = 180.0 - ((domePitchInput*30.0)*4/AXIS_MAX);
    else domePitchInput = 180.0 + ((domePitchInput*30.0)*4/AXIS_MAX);
    if(DOME_HEADING_SERVO_REVERSE) domeHeadingInput = 180.0 - ((domeHeadingInput*30.0)*4/AXIS_MAX);
    else domeHeadingInput = 180.0 + ((domeHeadingInput*30.0)*4/AXIS_MAX);
      
    bb::Servos::servos.setGoal(DOME_ROLL_SERVO, domeRollInput);
    bb::Servos::servos.setGoal(DOME_PITCH_SERVO, domePitchInput);
    bb::Servos::servos.setGoal(DOME_HEADING_SERVO, domeHeadingInput);
  } else {
    bb::Servos::servos.setGoal(DOME_ROLL_SERVO, 180.0);
    bb::Servos::servos.setGoal(DOME_PITCH_SERVO, 180.0);
    bb::Servos::servos.setGoal(DOME_HEADING_SERVO, 180.0);
  }

  bb::Servos::servos.setProfileVelocity(DOME_ROLL_SERVO, DOME_MAX_VELOCITY);
  bb::Servos::servos.setProfileVelocity(DOME_PITCH_SERVO, DOME_MAX_VELOCITY);
  bb::Servos::servos.setProfileVelocity(DOME_HEADING_SERVO, DOME_MAX_VELOCITY);

  packetTimeout_ = 3;
  lastPacket_ = packet;

  return RES_OK;
}

Result BB8::handleConsoleCommand(const std::vector<String> &words, ConsoleStream *stream) {
  if (words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if (words[0] == "status") {
    if (words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
    printStatus(stream);
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
    imu_.calibrateGyro(stream);
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

  else if(words[0] == "scan_i2c") {
    bb::Runloop::runloop.excuseOverrun();
    for(uint addr=0x8; addr<=0x77; addr++) {
      Wire.beginTransmission(addr);
      uint8_t result = Wire.endTransmission();
      if(result == 0) stream->printf("Found device at 0x%x\n", addr);
    }
    return RES_OK;
  }

  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result BB8::fillAndSendStatePacket() {
  bb::LargeStatePacket p;
  memset(&p, 0, sizeof(bb::LargeStatePacket));

  p.timestamp = Runloop::runloop.millisSinceStart() / 1000.0;
  p.droidType = DroidType::DROID_BB8;

  float err, errI, errD, control;

  p.drive[0].errorState = ERROR_OK;
  p.drive[0].controlMode = driveEncoder_.mode();
  p.drive[0].presentPWM = driveMotor_.present();
  p.drive[0].presentPos = driveEncoder_.presentPosition();
  p.drive[0].presentSpeed = driveEncoder_.presentSpeed();
  driveController_.getControlState(err, errI, errD, control);
  p.drive[0].err = err;
  p.drive[0].errI = errI;
  p.drive[0].errD = errD;
  p.drive[0].control = control;

  p.drive[1].errorState = ERROR_OK;
  p.drive[1].controlMode = bb::ControlMode::CONTROL_AUTOMATIC;
  p.drive[2].goal = balanceController_.goal();
  balanceController_.getControlState(err, errI, errD, control);
  p.drive[1].err = err;
  p.drive[1].errI = errI;
  p.drive[1].errD = errD;
  p.drive[1].control = control;

  p.drive[2].errorState = ERROR_OK;
  p.drive[2].controlMode = bb::ControlMode::CONTROL_AUTOMATIC;
  p.drive[2].goal = rollController_.goal();
  p.drive[2].presentSpeed = rollController_.present();
  rollController_.getControlState(err, errI, errD, control);
  p.drive[2].err = err;
  p.drive[2].errI = errI;
  p.drive[2].errD = errD;
  p.drive[2].control = control;

  p.imu[0] = imu_.getIMUState();
  p.imu[1].errorState = ERROR_NOT_PRESENT;
  p.imu[2].errorState = ERROR_NOT_PRESENT;

  p.battery[0] = BB8BattStatus::batt.getBatteryState(BB8BattStatus::BATT_1);
  p.battery[1] = BB8BattStatus::batt.getBatteryState(BB8BattStatus::BATT_2);
  p.battery[2].errorState = ERROR_NOT_PRESENT;

  WifiServer::server.broadcastUDPPacket((const uint8_t *)&p, sizeof(p));

  return RES_OK;
}

void BB8::printCurrentSystemStatus(ConsoleStream *stream) {
  float r, p, h;
  imu_.getFilteredRPH(r, p, h);
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