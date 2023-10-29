#include "BB8Droid.h"
#include "BB8Config.h"
#include "BB8Servos.h"
#include "BB8StatusPixels.h"
#include "BB8IMU.h"
#include "BB8BattStatus.h"
#include "BB8Sound.h"

BB8 BB8::bb8;

static const String driveSpeedKpStr_ = "drive_speed_kp";
static const String driveSpeedKiStr_ = "drive_speed_ki";
static const String driveSpeedKdStr_ = "drive_speed_kd";
static const String bodyRollKpStr_ = "body_roll_kp";
static const String bodyRollKiStr_ = "body_roll_ki";
static const String bodyRollKdStr_ = "body_roll_kd";

ServoLimits servolimits[] = {
  { 0.0f, 360.0f, 0.0f, 60.0 },
  { 120.0f, 240.0f, 0.0f, 60.0 },
  { 120.0f, 240.0f, 0.0f, 60.0 },
  { 160.0f, 200.0f, 0.0f, 80.0 }
};

bb::DCMotor driveMotor(P_DRIVE_A, P_DRIVE_B, P_DRIVE_PWM, P_DRIVE_EN);
bb::DCMotor yawMotor(P_YAW_A, P_YAW_B, P_YAW_PWM, P_YAW_EN);

BB8::BB8Params BB8::params_;

BB8::BB8()
  : rollControlInput_(BB8IMUControlInput::IMU_ROLL),
    rollControlOutput_(BODY_ROLL_SERVO, 180.0),
    rollController_(rollControlInput_, rollControlOutput_),
    driveControlInput_(P_DRIVEENC_A, P_DRIVEENC_B, bb::Encoder::INPUT_POSITION, bb::Encoder::UNIT_MILLIMETERS),
    driveController_(driveControlInput_, driveMotor) {
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

  addParameter(driveSpeedKpStr_, "Proportional constant for drive speed controller", params_.driveSpeedKp);
  addParameter(driveSpeedKiStr_, "Integral constant for drive speed controller", params_.driveSpeedKi);
  addParameter(driveSpeedKdStr_, "Derivative constant for drive speed controller", params_.driveSpeedKd);
  addParameter("drive_speed_goal", "Drive speed goal", driveSpeedGoal_);

  addParameter(bodyRollKpStr_, "Proportional constant for body roll controller", params_.bodyRollKp);
  addParameter(bodyRollKiStr_, "Integral constant for body roll controller", params_.bodyRollKi);
  addParameter(bodyRollKdStr_, "Derivative constant for body roll controller", params_.bodyRollKd);
  addParameter("body_roll_goal", "Body roll goal", bodyRollGoal_);

  addParameter("dome_roll_kp", "Proportional constant for dome pitch controller", params_.domePitchKp);
  addParameter("dome_roll_ki", "Integral constant for dome pitch controller", params_.domePitchKi);
  addParameter("dome_roll_kd", "Derivative constant for dome pitch controller", params_.domePitchKd);
  addParameter("dome_roll_goal", "Dome pitch goal", domePitchGoal_);

  addParameter("dome_roll_kp", "Proportional constant for dome roll controller", params_.domeRollKp);
  addParameter("dome_roll_ki", "Integral constant for dome roll controller", params_.domeRollKi);
  addParameter("dome_roll_kd", "Derivative constant for dome roll controller", params_.domeRollKd);
  addParameter("dome_roll_goal", "Dome roll goal", domeRollGoal_);
}

Result BB8::initialize() {
  paramsHandle_ = ConfigStorage::storage.reserveBlock(sizeof(params_));
  if (ConfigStorage::storage.blockIsValid(paramsHandle_)) {
    ConfigStorage::storage.readBlock(paramsHandle_, (uint8_t *)&params_);

    servolimits[BODY_ROLL_SERVO - 1].min = params_.body_roll_servo_min;
    servolimits[BODY_ROLL_SERVO - 1].max = params_.body_roll_servo_max;
    servolimits[BODY_ROLL_SERVO - 1].offset = params_.body_roll_servo_offset;
    servolimits[BODY_ROLL_SERVO - 1].speed = params_.body_roll_servo_speed;

    servolimits[BODY_ROLL_SERVO - 1].min = params_.dome_pitch_servo_min;
    servolimits[BODY_ROLL_SERVO - 1].max = params_.dome_pitch_servo_max;
    servolimits[BODY_ROLL_SERVO - 1].offset = params_.dome_pitch_servo_offset;
    servolimits[BODY_ROLL_SERVO - 1].speed = params_.dome_pitch_servo_speed;

    servolimits[DOME_ROLL_SERVO - 1].min = params_.dome_roll_servo_min;
    servolimits[DOME_ROLL_SERVO - 1].max = params_.dome_roll_servo_max;
    servolimits[DOME_ROLL_SERVO - 1].offset = params_.dome_roll_servo_offset;
    servolimits[DOME_ROLL_SERVO - 1].speed = params_.dome_roll_servo_speed;

    servolimits[DOME_HEADING_SERVO - 1].min = params_.dome_heading_servo_min;
    servolimits[DOME_HEADING_SERVO - 1].max = params_.dome_heading_servo_max;
    servolimits[DOME_HEADING_SERVO - 1].offset = params_.dome_heading_servo_offset;
    servolimits[DOME_HEADING_SERVO - 1].speed = params_.dome_heading_servo_speed;
  } else {
    params_.drive_speed_factor = 2.0;  //0.5;
    params_.turn_speed_factor = 1.0;

    params_.body_roll_servo_min = servolimits[BODY_ROLL_SERVO - 1].min;
    params_.body_roll_servo_max = servolimits[BODY_ROLL_SERVO - 1].max;
    params_.body_roll_servo_offset = servolimits[BODY_ROLL_SERVO - 1].offset;
    params_.body_roll_servo_speed = servolimits[BODY_ROLL_SERVO - 1].speed;
    params_.body_roll_servo_invert = true;

    params_.dome_pitch_servo_min = servolimits[DOME_PITCH_SERVO - 1].min;
    params_.dome_pitch_servo_max = servolimits[DOME_PITCH_SERVO - 1].max;
    params_.dome_pitch_servo_offset = servolimits[DOME_PITCH_SERVO - 1].offset;
    params_.dome_pitch_servo_speed = servolimits[DOME_PITCH_SERVO - 1].speed;
    params_.dome_pitch_servo_invert = false;

    params_.dome_roll_servo_min = servolimits[DOME_ROLL_SERVO - 1].min;
    params_.dome_roll_servo_max = servolimits[DOME_ROLL_SERVO - 1].max;
    params_.dome_roll_servo_offset = servolimits[DOME_ROLL_SERVO - 1].offset;
    params_.dome_roll_servo_speed = servolimits[DOME_ROLL_SERVO - 1].speed;
    params_.dome_roll_servo_invert = false;

    params_.dome_heading_servo_min = servolimits[DOME_HEADING_SERVO - 1].min;
    params_.dome_heading_servo_max = servolimits[DOME_HEADING_SERVO - 1].max;
    params_.dome_heading_servo_offset = servolimits[DOME_HEADING_SERVO - 1].offset;
    params_.dome_heading_servo_speed = servolimits[DOME_HEADING_SERVO - 1].speed;
    params_.dome_heading_servo_invert = false;
  }

  params_.domeRollKp = 1.0;
  params_.domeRollKi = 0.0;
  params_.domeRollKd = 0.0;
  params_.domePitchKp = 1.0;
  params_.domePitchKi = 0.0;
  params_.domePitchKd = 0.0;

  return Subsystem::initialize();
}

Result BB8::start(ConsoleStream *stream) {
  started_ = true;
  runningStatus_ = false;
  operationStatus_ = RES_OK;
  mode_ = MODE_OFF;
  bb::XBee::xbee.addPacketReceiver(this);
  packetsReceived_ = packetsMissed_ = 0;
  memset((uint8_t *)&lastPacket_, 0, sizeof(lastPacket_));
  packetTimeout_ = 0;

  BB8IMU::imu.begin();
  BB8BattStatus::batt.begin();

  pwmControl_ = false;
  driveMotor.set(0.0f);
  driveControlInput_.setMillimetersPerTick(BODY_CIRCUMFERENCE / DRIVE_MOTOR_TICKS_PER_TURN);
  // driveControlInput_.setMaxSpeed(DRIVE_MOTOR_MAX_SPEED_M_PER_S);
  driveMotor.setReverse(true);
  driveMotor.setEnabled(true);

  if (!BB8Servos::servos.isStarted()) {
    if (BB8Servos::servos.start(stream) != RES_OK) {
      return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    }
  }

  BB8Servos::servos.setRange(BODY_ROLL_SERVO, params_.body_roll_servo_min, params_.body_roll_servo_max);
  BB8Servos::servos.setOffset(BODY_ROLL_SERVO, params_.body_roll_servo_offset);
  BB8Servos::servos.setInverted(BODY_ROLL_SERVO, params_.body_roll_servo_invert);

  BB8Servos::servos.setRange(DOME_ROLL_SERVO, params_.dome_roll_servo_min, params_.dome_roll_servo_max);
  BB8Servos::servos.setOffset(DOME_ROLL_SERVO, params_.dome_roll_servo_offset);
  BB8Servos::servos.setInverted(DOME_ROLL_SERVO, params_.dome_roll_servo_invert);

  BB8Servos::servos.setRange(DOME_PITCH_SERVO, params_.dome_pitch_servo_min, params_.dome_pitch_servo_max);
  BB8Servos::servos.setOffset(DOME_PITCH_SERVO, params_.dome_pitch_servo_offset);
  BB8Servos::servos.setInverted(DOME_PITCH_SERVO, params_.dome_pitch_servo_invert);

  BB8Servos::servos.setRange(DOME_HEADING_SERVO, params_.dome_heading_servo_min, params_.dome_heading_servo_max);
  BB8Servos::servos.setOffset(DOME_HEADING_SERVO, params_.dome_heading_servo_offset);
  BB8Servos::servos.setInverted(DOME_HEADING_SERVO, params_.dome_heading_servo_invert);

  BB8Servos::servos.switchTorque(BB8Servos::ID_ALL, true);

  rollController_.setControlParameters(1.0, 0.0, 0.0);
  rollController_.setIBounds(-180.0, 180.0);
  rollController_.setGoal(0.0);

  driveController_.setControlParameters(1.0, 0.0, 0.0);
  driveController_.setIBounds(-100.0, 100.0);
  driveController_.setGoal(0);

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
  if (packetTimeout_ > 0) BB8StatusPixels::statusPixels.setPixel(STATUSPIXEL_NETWORK, 0, 255, 0);
  else BB8StatusPixels::statusPixels.setPixel(STATUSPIXEL_NETWORK, 0, 0, 150);

  BB8IMU::imu.update();

  stepcount++;
  if (stepcount == 1 && BB8BattStatus::batt.available(BB8BattStatus::BATT_1)) BB8BattStatus::batt.updateVoltage(BB8BattStatus::BATT_1);
  if (stepcount == 2 && BB8BattStatus::batt.available(BB8BattStatus::BATT_1)) BB8BattStatus::batt.updateCurrent(BB8BattStatus::BATT_1);
  //if (stepcount == 3 && BB8BattStatus::batt.available(BB8BattStatus::BATT_2)) BB8BattStatus::batt.updateVoltage(BB8BattStatus::BATT_2);
  //if (stepcount == 4 && BB8BattStatus::batt.available(BB8BattStatus::BATT_2)) BB8BattStatus::batt.updateCurrent(BB8BattStatus::BATT_2);
  if (stepcount == 100) stepcount = 0;

  Result res;
  res = stepDriveMotor();
  if (res != RES_OK) {
    Console::console.printlnBroadcast("stepDriveMotor() failed!");
    return res;
  }
  res = stepRollMotor();
  if (res != RES_OK) {
    Console::console.printlnBroadcast("stepDriveMotor() failed!");
    return res;
  }
  res = stepDome();
  if (res != RES_OK) {
    Console::console.printlnBroadcast("stepDriveMotor() failed!");
    return res;
  }

  if (runningStatus_) {
    printCurrentSystemStatus();
    Console::console.printBroadcast("\r");
  }

  if (packetTimeout_ > 0) packetTimeout_--;

  fillAndSendStatusPacket();

  return RES_OK;
}

Result BB8::stepDriveMotor() {
  if (mode_ == MODE_OFF || mode_ == MODE_ROLL_CONTROL_ONLY || mode_ == MODE_KIOSK) {
    if (driveMotor.isEnabled()) driveMotor.setEnabled(false);
    return RES_OK;
  }

  if (mode_ == MODE_SPEED_CONTROL_ONLY || mode_ == MODE_SPEED_ROLL_CONTROL) {
    driveController_.update();
    return RES_OK;
  }

  return RES_OK;
}

Result BB8::stepRollMotor() {
  return RES_OK;
}

Result BB8::stepDome() {
  if (mode_ == MODE_OFF) {
    return RES_OK;
  }

  else if (mode_ == MODE_KIOSK) {
    if (kioskDelay_ < Runloop::runloop.cycleTime()) {
      if (random(0, 2) != 0) {
        BB8Servos::servos.setGoal(DOME_HEADING_SERVO, (float)random(servolimits[DOME_HEADING_SERVO].min, servolimits[DOME_HEADING_SERVO].max));
      }
      if (random(0, 2) != 0) {
        BB8Servos::servos.setGoal(DOME_ROLL_SERVO, (float)random(servolimits[DOME_ROLL_SERVO].min, servolimits[DOME_ROLL_SERVO].max));
      }
      if (random(0, 2) != 0) {
        BB8Servos::servos.setGoal(DOME_PITCH_SERVO, (float)random(servolimits[DOME_PITCH_SERVO].min, servolimits[DOME_PITCH_SERVO].max));
      }
      kioskDelay_ = random(2000000, 5000000);
    } else kioskDelay_ -= Runloop::runloop.cycleTime();
  }

  else {
    float r, p, h, gr, gp, gh, goal;
    BB8IMU::imu.getFilteredRPH(r, p, h);
    BB8IMU::imu.getGyroMeasurement(gr, gp, gh);

    goal = params_.domeRollKp * 2 * r + params_.domeRollKd * gr + 180.0;
    if (goal < 0 || goal > 360.0) {
      Console::console.printlnBroadcast(String("Roll out of range (") + goal + ")");
    } else {
      if (BB8Servos::servos.setGoal(DOME_ROLL_SERVO, goal) == false) {
        Console::console.printlnBroadcast(String("Huh? ") + r + " " + params_.domeRollKp + " " + params_.domeRollKd + " " + r + " " + gr);
      }
    }

    goal = params_.domePitchKp * 2 * p + params_.domePitchKd * gr + 180.0;
    if (goal < 0 || goal > 360.0) {
      Console::console.printlnBroadcast(String("Pitch out of range (") + goal + ")");
    } else {
      if (BB8Servos::servos.setGoal(DOME_PITCH_SERVO, goal) == false) {
        Console::console.printlnBroadcast(String("Huh? ") + goal + " " + params_.domePitchKp + " " + params_.domePitchKd + " " + p + " " + gp);
      }
    }
  }

  return RES_OK;
}

Result BB8::setMode(BB8::Mode mode) {
  if (mode_ == mode) return RES_OK;
  mode_ = mode;

  if (mode_ == MODE_OFF) {
    BB8Servos::servos.switchTorque(DOME_HEADING_SERVO, false);
    BB8Servos::servos.switchTorque(DOME_ROLL_SERVO, false);
    BB8Servos::servos.switchTorque(DOME_PITCH_SERVO, false);
    BB8Servos::servos.switchTorque(BODY_ROLL_SERVO, false);
  }

  rollController_.reset();
  driveController_.reset();

  return RES_OK;
}

void BB8::printStatus(ConsoleStream *stream) {
  if (stream == NULL) return;

  stream->print(name() + " (" + description() + "): ");

  if (isStarted()) {
    stream->print("started, ");
    switch (operationStatus()) {
      case RES_OK:
        stream->print("operational");
        break;
      default:
        stream->print("not operational: ");
        stream->print(errorMessage(operationStatus()));
        break;
    }
  } else stream->print("stopped");

  if (BB8BattStatus::batt.available(BB8BattStatus::BATT_1)) {
    stream->print(String(", batt1: ") + BB8BattStatus::batt.voltage(BB8BattStatus::BATT_1) + "V " + BB8BattStatus::batt.current(BB8BattStatus::BATT_1) + "mA");
  } else {
    stream->print(", batt1 n/a");
  }

  if (BB8BattStatus::batt.available(BB8BattStatus::BATT_2)) {
    stream->print(String(", batt2: ") + BB8BattStatus::batt.voltage(BB8BattStatus::BATT_2) + "V " + BB8BattStatus::batt.current(BB8BattStatus::BATT_2) + "mA");
  } else {
    stream->print(", batt2 n/a");
  }

  stream->println();
}

Result BB8::incomingPacket(const Packet &packet) {
  if (packet.source == PACKET_SOURCE_TEST_ONLY) {
    Console::console.printlnBroadcast("Test packet received!");
    return RES_OK;
  }

  if (packet.type != PACKET_TYPE_COMMAND) return RES_SUBSYS_PROTOCOL_ERROR;

  packetsReceived_++;
  if (packet.seqnum != (lastPacket_.seqnum + 1) % MAX_SEQUENCE_NUMBER) packetsMissed_++;  // FIXME not correct - should count based on seqnum
  lastPacket_ = packet;

  float r, p, h;
  BB8IMU::imu.getFilteredRPH(r, p, h);

#if 0
  if(packet.payload.cmd.button2) {
    //Console::console.printlnBroadcast("Updating servos");
    float axis2, axis3, axis4;
    if(params_.dome_roll_servo_invert)
      axis2 = 180.0 - ((packet.payload.cmd.getAxis(2)*30.0)*4/127.0 - 2*(BB8Servos::servos.present(BODY_ROLL_SERVO)-180.0) + 2*r);
    else 
      axis2 = 180.0 + ((packet.payload.cmd.getAxis(2)*30.0)*4/127.0 - 2*(BB8Servos::servos.present(BODY_ROLL_SERVO)-180.0) + 2*r);
    if(params_.dome_pitch_servo_invert)
      axis3 = 180.0 - ((packet.payload.cmd.getAxis(3)*30.0)*4/127.0 + 2*p);
    else 
      axis3 = 180.0 + ((packet.payload.cmd.getAxis(3)*30.0)*4/127.0 + 2*p);
    if(params_.dome_heading_servo_invert)
      axis4 = 180.0 - ((packet.payload.cmd.getAxis(4)*30.0)*4/127.0);
    else
      axis4 = 180.0 + ((packet.payload.cmd.getAxis(4)*30.0)*4/127.0);
      
    BB8Servos::servos.setGoal(DOME_ROLL_SERVO, axis2);
    BB8Servos::servos.setGoal(DOME_PITCH_SERVO, axis3);
    BB8Servos::servos.setGoal(DOME_HEADING_SERVO, axis4);
  } else {
    //BB8Servos::servos.setSetpoint(DOME_ROLL_SERVO, 180.0+2*r);
    if(params_.dome_roll_servo_invert)
      BB8Servos::servos.setGoal(DOME_ROLL_SERVO, 180.0 - (2*r - 2*(BB8Servos::servos.present(BODY_ROLL_SERVO)-180.0)));
    else
      BB8Servos::servos.setGoal(DOME_ROLL_SERVO, 180.0 + (2*r - 2*(BB8Servos::servos.present(BODY_ROLL_SERVO)-180.0)));

    if(params_.dome_pitch_servo_invert)
      BB8Servos::servos.setGoal(DOME_PITCH_SERVO, 180.0-2*p);
    else 
      BB8Servos::servos.setGoal(DOME_PITCH_SERVO, 180.0+2*p);
    BB8Servos::servos.setGoal(DOME_HEADING_SERVO, 180.0);
  }
#endif

  if (packet.payload.cmd.button1) {
    int8_t axis1 = packet.payload.cmd.getAxis(1);
    float speed = (800.0 * axis1 / 127.0);  // magic
    driveController_.setGoal(speed);
    //    if(axis1 < 0) driveMotor.setDirectionAndSpeed(DCMotor::DCM_BACKWARD, -axis1 * params_.drive_speed_factor);
    //    else driveMotor.setDirectionAndSpeed(DCMotor::DCM_FORWARD, axis1 * params_.drive_speed_factor);

    float axis0;
    if (params_.body_roll_servo_invert)
      axis0 = 180.0 + (packet.payload.cmd.getAxis(0) * 20.0) / 127.0;
    else
      axis0 = 180.0 - (packet.payload.cmd.getAxis(0) * 20.0) / 127.0;
    BB8Servos::servos.setGoal(BODY_ROLL_SERVO, axis0);
  } else {
    driveController_.setGoal(0.0f);
    //BB8Servos::servos.setGoal(BODY_ROLL_SERVO, 180.0);
  }

  packetTimeout_ = 3;

  return RES_OK;
}

Result BB8::handleConsoleCommand(const std::vector<String> &words, ConsoleStream *stream) {
  if (words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if (words[0] == "status") {
    if (words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
    printStatus(stream);
    stream->println();
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
    BB8IMU::imu.calibrateGyro(stream);
  }

  else if (words[0] == "drive") {
    if (words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;

    if (words[1] == "pwm") {
      pwmControl_ = true;
      driveMotor.set(words[2].toFloat());
      driveMotor.setEnabled(true);
      return RES_OK;
    } else if (words[1] == "position") {
      pwmControl_ = false;
      driveControlInput_.setMode(bb::Encoder::INPUT_POSITION);
      driveController_.setGoal(words[2].toFloat());
      driveMotor.setEnabled(true);
      return RES_OK;
    } else if (words[1] == "speed") {
      pwmControl_ = false;
      driveControlInput_.setMode(bb::Encoder::INPUT_SPEED);
      driveController_.setGoal(words[2].toFloat());
      driveMotor.setEnabled(true);
      return RES_OK;
    } else return RES_CMD_INVALID_ARGUMENT;
  }

  else if (words[0] == "mode") {
    if (words.size() == 1) {
      switch (mode_) {
        case MODE_OFF:
          if (stream) stream->println("off");
          break;
        case MODE_ROLL_CONTROL_ONLY:
          if (stream) stream->println("roll");
          break;
        case MODE_SPEED_CONTROL_ONLY:
          if (stream) stream->println("speed");
          break;
        case MODE_SPEED_ROLL_CONTROL:
          if (stream) stream->println("speed_roll");
          break;
        case MODE_POS_CONTROL:
          if (stream) stream->println("pos");
          break;
        case MODE_KIOSK:
          if (stream) stream->println("kiosk");
          break;
        case MODE_CALIB:
          if (stream) stream->println("calib");
          break;
        default:
          if (stream) stream->println("unknown");
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

Result BB8::fillAndSendStatusPacket() {
  LargeStatusPacket p;

  p.timestamp = Runloop::runloop.millisSinceStart() / 1000.0;
  p.droidType = DroidType::DROID_BB8;

  float err, errI, errD, control;

  p.drive[0].errorState = ERROR_OK;
  p.drive[0].controlMode = driveControlInput_.mode();
  p.drive[0].presentPWM = driveMotor.present();
  p.drive[0].presentPos = driveControlInput_.presentPosition();
  p.drive[0].presentSpeed = driveControlInput_.presentSpeed();
  driveController_.getControlState(err, errI, errD, control);
  p.drive[0].err = err;
  p.drive[0].errI = errI;
  p.drive[0].errD = errD;
  p.drive[0].control = control;

  p.drive[1].errorState = ERROR_NOT_PRESENT;  // Not quite true, but we have no encoder info for this motor.
  p.drive[2].errorState = ERROR_NOT_PRESENT;

  p.imu[0] = BB8IMU::imu.getIMUState();
  p.imu[1].errorState = ERROR_NOT_PRESENT;
  p.imu[2].errorState = ERROR_NOT_PRESENT;

  p.battery[0] = BB8BattStatus::batt.getBatteryState(BB8BattStatus::BATT_1);
  p.battery[1] = BB8BattStatus::batt.getBatteryState(BB8BattStatus::BATT_2);
  p.battery[2].errorState = ERROR_NOT_PRESENT;

  WifiServer::server.broadcastUDPPacket((const uint8_t *)&p, sizeof(p));

  return RES_OK;
}

void BB8::parameterChangedCallback(const String &name) {
  Result res;
  if (name == driveSpeedKpStr_ || name == driveSpeedKiStr_ || name == driveSpeedKdStr_) {
    driveController_.setControlParameters(params_.driveSpeedKp, params_.driveSpeedKi, params_.driveSpeedKd);
  } else if (name == "drive_speed_goal") {
    driveController_.setGoal(driveSpeedGoal_);
    driveControlInput_.setMode(bb::Encoder::INPUT_SPEED);
  } else if (name == bodyRollKpStr_ || name == bodyRollKiStr_ || name == bodyRollKdStr_) {
    rollController_.setControlParameters(params_.driveSpeedKp, params_.driveSpeedKi, params_.driveSpeedKd);
  } else if (name == "body_roll_goal") {
    rollController_.setGoal(bodyRollGoal_);
  }

  if (res == RES_OK) ConfigStorage::storage.writeBlock(paramsHandle_, (uint8_t *)&params_);
}


void BB8::printCurrentSystemStatus(ConsoleStream *stream) {
  char buf[255];

  float r, p, h;
  BB8IMU::imu.getFilteredRPH(r, p, h);
  String str = String("Received: ") + packetsReceived_ + "Missed: " + packetsMissed_ + "Seq: " + lastPacket_.seqnum;
  str += "Buttons: ";
  lastPacket_.payload.cmd.button0 ? str += 'X' : str += '_';
  lastPacket_.payload.cmd.button1 ? str += 'X' : str += '_';
  lastPacket_.payload.cmd.button2 ? str += 'X' : str += '_';
  lastPacket_.payload.cmd.button3 ? str += 'X' : str += '_';
  lastPacket_.payload.cmd.button4 ? str += 'X' : str += '_';
  str += "Axes: ";
  for (int i = 0; i < 5; i++) {
    str += lastPacket_.payload.cmd.getAxis(0);
    str += " ";
  }

  str += String("Gyro: R") + r + " P" + p + " H" + h;

  if (stream != NULL) stream->print(buf);
  else Console::console.printBroadcast(buf);
}