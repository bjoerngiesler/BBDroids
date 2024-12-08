#include "DODroid.h"
#include "DOConfig.h"
#include "DOBattStatus.h"
#include "DOSound.h"
#include "../resources/systemsounds.h"

DODroid DODroid::droid;
DOParams DODroid::params_;

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

  balanceController_(balanceInput_, driveOutput_)
{
  // Pull down the ENABLE pins for the motor controllers. FIXME I *think* this isn't needed for D-Ov2Evo, test!
  pinMode(PULL_DOWN_15, OUTPUT);
  digitalWrite(PULL_DOWN_15, LOW);
  pinMode(PULL_DOWN_20, OUTPUT);
  digitalWrite(PULL_DOWN_20, LOW);

  name_ = "d-o";

  description_ = "D-O Main System";
  help_ = "Available commands:\r\n"\
"\tstatus\t\tPrint Status\r\n"\
"\tselftest\tRun self test\r\n"\
"\tplay_sound [<folder>] <num>\tPlay sound\r\n";
  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
}

Result DODroid::initialize() {
  addParameter("neck_range", "Neck servo movement range", params_.neckRange, -INT_MAX, INT_MAX);
  addParameter("neck_offset", "Neck servo offset", params_.neckOffset, -INT_MAX, INT_MAX);
  addParameter("head_roll_range", "Head roll servo movement range", params_.headRollRange, -INT_MAX, INT_MAX);
  addParameter("head_roll_offset", "Head roll servo servo offset", params_.headRollOffset, -INT_MAX, INT_MAX);
  addParameter("head_pitch_range", "Head pitch servo movement range", params_.headPitchRange, -INT_MAX, INT_MAX);
  addParameter("head_pitch_offset", "Head pitch servo servo offset", params_.headPitchOffset, -INT_MAX, INT_MAX);
  addParameter("head_heading_range", "Head heading servo movement range", params_.headHeadingRange, -INT_MAX, INT_MAX);
  addParameter("head_heading_offset", "Head heading servo servo offset", params_.headHeadingOffset, -INT_MAX, INT_MAX);

  addParameter("wheel_kp", "Proportional constant for wheel speed PID controller", params_.wheelSpeedKp, -INT_MAX, INT_MAX);
  addParameter("wheel_ki", "Integrative constant for wheel speed PID controller", params_.wheelSpeedKi, -INT_MAX, INT_MAX);
  addParameter("wheel_kd", "Derivative constant for wheel speed PID controller", params_.wheelSpeedKd, -INT_MAX, INT_MAX);
  addParameter("wheel_imax", "Max I aggregate for wheel speed PID controller", params_.wheelSpeedImax, -INT_MAX, INT_MAX);

  addParameter("bal_kp", "Proportional constant for balance PID controller", params_.balKp, -INT_MAX, INT_MAX);
  addParameter("bal_ki", "Integrative constant for balance PID controller", params_.balKi, -INT_MAX, INT_MAX);
  addParameter("bal_kd", "Derivative constant for balance PID controller", params_.balKd, -INT_MAX, INT_MAX);
  addParameter("bal_neck_mix", "Mix neck SP into balance SP to avoid inducing motion", params_.balNeckMix, -INT_MAX, INT_MAX);

  addParameter("accel", "Acceleration in mm/s^2", params_.accel, -INT_MAX, INT_MAX);
  addParameter("max_speed", "Maximum speed (only honored in speed control mode)", params_.maxSpeed, 0, INT_MAX);
  addParameter("speed_axis_gain", "Gain for controller speed axis", params_.speedAxisGain, -INT_MAX, INT_MAX);
  addParameter("speed_axis_deadband", "Deadband for controller speed axis", params_.speedAxisGain, -INT_MAX, INT_MAX);
  addParameter("rot_axis_gain", "Gain for controller rot axis", params_.speedAxisGain, -INT_MAX, INT_MAX);
  addParameter("rot_axis_deadband", "Deadband for controller rot axis", params_.speedAxisGain, -INT_MAX, INT_MAX);

  addParameter("antenna_offset", "Offset for antennas", params_.antennaOffset, -INT_MAX, INT_MAX);
  addParameter("gyro_pitch_deadband", "Deadband for gyro pitch", params_.gyroPitchDeadband, -INT_MAX, INT_MAX);

  addParameter("fa_neck_imu_accel", "Free Anim - neck on IMU accel", params_.faNeckIMUAccel, -INT_MAX, INT_MAX);
  addParameter("fa_neck_sp_accel", "Free Anim - neck on accel setpoint", params_.faNeckSPAccel, -INT_MAX, INT_MAX);
  addParameter("fa_neck_speed", "Free Anim - neck on wheel speed", params_.faNeckSpeed, -INT_MAX, INT_MAX);
  addParameter("fa_neck_speed_sp", "Free Anim - neck on wheel speed setpoint", params_.faNeckSpeedSP, -INT_MAX, INT_MAX);
  addParameter("fa_head_pitch_speed_sp", "Free Anim - head pitch on wheel speed setpoint", params_.faHeadPitchSpeedSP, -INT_MAX, INT_MAX);
  addParameter("fa_head_roll_turn", "Free Anim: Head roll on turn speed", params_.faHeadRollTurn, -INT_MAX, INT_MAX);
  addParameter("fa_head_heading_turn", "Free Anim: Head heading on turn speed", params_.faHeadHeadingTurn, -INT_MAX, INT_MAX);
  addParameter("fa_antenna_speed", "Free Anim: Antenna position on wheel speed setpoint", params_.faAntennaSpeedSP, -INT_MAX, INT_MAX);
  addParameter("fa_head_anneal_time", "Free Anim: Head anneal time", params_.faHeadAnnealTime, -INT_MAX, INT_MAX);

  lSpeedController_.setAutoUpdate(false);
  rSpeedController_.setAutoUpdate(false);

  balanceController_.setAutoUpdate(false);
  balanceController_.setReverse(true); // FIXME Not quite sure anymore why we're reversing here, we should be forwarding. Check!
  balanceController_.setDebug(false);
 
  imu_.setRotationAroundZ(bb::IMU::ROTATE_90);

  return Subsystem::initialize();
}

Result DODroid::start(ConsoleStream* stream) {
  lastBtn0_ = lastBtn1_ = lastBtn2_ = lastBtn3_ = lastBtn4_ = false;
  leftMotorStatus_ = MOTOR_UNTESTED;
  rightMotorStatus_ = MOTOR_UNTESTED;
  servosOK_ = false;
  antennasOK_ = false;
  driveOn_ = false;

  imu_.begin();
  DOBattStatus::batt.begin();

  operationStatus_ = selfTest();
  

  if(operationStatus_ == RES_DROID_VOLTAGE_TOO_LOW) {
    if(DOBattStatus::batt.voltage() < 1.0) {
      Console::console.printfBroadcast("No power (%.fV), USB only!\n", DOBattStatus::batt.voltage());
      started_ = true;
      operationStatus_ = RES_OK;
      return RES_OK;
    }
  } else if(operationStatus_ != RES_OK) {
    return operationStatus_;
  }

  setControlParameters();

  annealH_ = annealP_ = annealR_ = 0;

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
  lSpeedController_.setIBounds(-params_.wheelSpeedImax, params_.wheelSpeedImax);
  rSpeedController_.setIBounds(-params_.wheelSpeedImax, params_.wheelSpeedImax);
  lSpeedController_.reset();
  rSpeedController_.reset();

  balanceController_.setControlParameters(params_.balKp, params_.balKi, params_.balKd);
  balanceController_.setRamp(0);
  balanceController_.setErrorDeadband(-1.0, 1.0);
  balanceController_.reset();
  driveOutput_.setAcceleration(params_.accel);
  driveOutput_.setMaxSpeed(params_.maxSpeed);
}

Result DODroid::step() {
  if(!imu_.available() || !DOBattStatus::batt.available()) {
    fillAndSendStatePacket();
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  if((Runloop::runloop.getSequenceNumber() % 1000) == 0) {
    stepPowerProtect();
  }

  Runloop::runloop.excuseOverrun();

  // Encoder and IMU updates are needed for everything, so we do them here.
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
  float p, r, h, dr, dp, dh, ax, ay, az;
  imu_.getFilteredPRH(p, r, h);
  imu_.getAccelMeasurement(ax, ay, az);
  imu_.getGyroMeasurement(dp, dr, dh);

  //Console::console.printfBroadcast("R:%f P:%f H:%f AX:%f AY:%f AZ:%f DR:%f DP:%f DH:%f\n", r, p, h, ax, ay, az, dr, dp, dh);

  float speed = (leftEncoder_.presentSpeed() + rightEncoder_.presentSpeed())/2;
  float speedSP = (lSpeedController_.goal() + rSpeedController_.goal())/2;
  float accelSP = (speedSP-speed);
  if(speedSP == 0) accelSP = 0;

  if(servosOK_) {
    float nod = params_.faNeckIMUAccel*ax + params_.faNeckSPAccel*accelSP + params_.faNeckSpeed*speed;
    if(speedSP < 0) nod += params_.faNeckSpeedSP*speedSP/2;
    else nod += params_.faNeckSpeedSP*speedSP/2;
    nod += params_.neckOffset;
    bb::Servos::servos.setGoal(SERVO_NECK, 180 + nod);

    float headPitch = -nod + remoteP_ - params_.neckOffset - params_.headPitchOffset;
    headPitch += params_.faHeadPitchSpeedSP*speedSP;
    bb::Servos::servos.setGoal(SERVO_HEAD_PITCH, 180 + headPitch);
    bb::Servos::servos.setGoal(SERVO_HEAD_HEADING, 180.0 + params_.faHeadHeadingTurn * dh - remoteH_ + params_.headHeadingOffset);
    bb::Servos::servos.setGoal(SERVO_HEAD_ROLL, 180.0 - params_.faHeadRollTurn * dh - remoteR_ + params_.headRollOffset);
  }

  if(antennasOK_) {
    float ant = 90 + params_.antennaOffset + speedSP*params_.faAntennaSpeedSP;
    ant = constrain(ant, 0, 180);
    setAntennas(ant, ant, ant);
  }

  if(lastBtn3_ == false && (float(millis())/1000.0f > annealTime_ + params_.faHeadAnnealDelay)) {
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
  } else if(lastBtn3_ == true){
    annealTime_ = float(millis()) / 1000.0f;
  }
  
  return RES_OK;
}

bb::Result DODroid::stepDrive() {
  if(driveOn_ == true && leftMotorStatus_ == MOTOR_OK && rightMotorStatus_ == MOTOR_OK) {
    balanceController_.update();
    lSpeedController_.update();
    rSpeedController_.update();
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
  balanceController_.setGoal(0);

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

  float p, r, h;
  imu_.getFilteredPRH(p, r, h);
  stream->printf(", IMU: P%f R%f H%f", p, r, h);

  stream->printf("\n");
}

Result DODroid::incomingControlPacket(uint64_t srcAddr, PacketSource source, uint8_t rssi, const ControlPacket& packet) {
  if(source == PACKET_SOURCE_LEFT_REMOTE) {
    Console::console.printfBroadcast("Control packet from left remote (but not primary)\n");
  } else if(source == PACKET_SOURCE_RIGHT_REMOTE) {    
    Console::console.printfBroadcast("Control packet from right remote (but not primary)\n");
  } 
  if(packet.primary == true) {
    //Console::console.printfBroadcast("Control packet from primary remote\n");
    static int numZero = 0;
    if(EPSILON(packet.getAxis(0)) && EPSILON(packet.getAxis(1))) {
      numZero++;
    } else {
      numZero = 0;
    }

    if(packet.button4 && !lastBtn4_) {
      if(driveOn_ == false) {
        switchDrive(true);
      } else {
        switchDrive(false);
      }
    }

    if(packet.button3) {
      remoteR_ = -packet.getAxis(2, ControlPacket::UNIT_DEGREES_CENTERED);
      remoteP_ = packet.getAxis(3, ControlPacket::UNIT_DEGREES_CENTERED);
      remoteH_ = -packet.getAxis(4, ControlPacket::UNIT_DEGREES_CENTERED);
      annealR_ = fabs(remoteR_ / (params_.faHeadAnnealTime / bb::Runloop::runloop.cycleTimeSeconds()));
      annealP_ = fabs(remoteP_ / (params_.faHeadAnnealTime / bb::Runloop::runloop.cycleTimeSeconds()));
      annealH_ = fabs(remoteH_ / (params_.faHeadAnnealTime / bb::Runloop::runloop.cycleTimeSeconds()));
    }

    if(driveOn_ == false) { // doesn't work while driving
      if(packet.button0 && !lastBtn0_) {
        DOSound::sound.playFolderRandom(DOSound::FOLDER_GREETING, false);
      } 
      else if(packet.button1 && !lastBtn1_) DOSound::sound.playFolderRandom(DOSound::FOLDER_POSITIVE, false);
      else if(packet.button2 && !lastBtn2_) DOSound::sound.playFolderRandom(DOSound::FOLDER_NEGATIVE, false);
    }

    lastBtn0_ = packet.button0;
    lastBtn1_ = packet.button1;
    lastBtn2_ = packet.button2;
    lastBtn3_ = packet.button3;
    lastBtn4_ = packet.button4;

    if(driveOn_ == true) {
      float vel = packet.getAxis(1);
      float rot = packet.getAxis(0);
      if(fabs(vel)<params_.speedAxisDeadband) vel = 0;
      if(fabs(rot)<params_.rotAxisDeadband) rot = 0;

      if(numZero > 5) {
        balanceController_.reset();
      }
      
      vel = constrain(vel * params_.maxSpeed * params_.speedAxisGain, -params_.maxSpeed, params_.maxSpeed);
      rot = constrain(rot * params_.maxSpeed * params_.rotAxisGain, -params_.maxSpeed, params_.maxSpeed);
      driveOutput_.setGoalVelocity(vel);
      driveOutput_.setGoalRotation(rot);
    }

    DOSound::sound.setVolume(int(30.0 * packet.getAxis(8)));
  } // Primary Remote

  return RES_OK;
}

Result DODroid::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
  if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if(words[0] == "selftest") {
    if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
    Runloop::runloop.excuseOverrun();
    return selfTest(stream);
  } else if(words[0] == "play_sound") {
    if(words.size() == 1 || words.size() > 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words.size() == 2) {
      bool retval = DOSound::sound.playSound(words[1].toInt());
      if(retval == false) stream->printf("Error\n");      
    } if(words.size() == 3) {
      bool retval = DOSound::sound.playFolder(DOSound::Folder(words[1].toInt()), words[2].toInt());
      if(retval == false) stream->printf("Error\n");
    }
    return RES_OK;
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
  memset(&p, 0, sizeof(LargeStatePacket));

  if((Runloop::runloop.getSequenceNumber() % 5) != 0) return RES_OK;

  Runloop::runloop.excuseOverrun(); // not nice but sending the UDP packet takes about 3ms

  p.timestamp = Runloop::runloop.millisSinceStart() / 1000.0;
  p.droidType = DroidType::DROID_DO;

  strncpy(p.droidName, DROID_NAME, sizeof(p.droidName));

  float err, errI, errD, control;

  if(leftMotorStatus_ == MOTOR_OK && rightMotorStatus_ == MOTOR_OK) {
    p.drive[0].errorState = ERROR_OK;   
    p.drive[0].controlMode = driveOn_ ? bb::StatePacket::CONTROL_RC : bb::StatePacket::CONTROL_OFF;

    p.drive[0].presentPWM = balanceController_.present();
    p.drive[0].presentPos = leftEncoder_.presentPosition();
    p.drive[0].presentSpeed = leftEncoder_.presentSpeed();
    balanceController_.getControlState(err, errI, errD, control);
    p.drive[0].goal = balanceController_.goal();

    p.drive[0].err = err;
    p.drive[0].errI = errI;
    p.drive[0].errD = errD;
    p.drive[0].control = control;
  }

  if(leftMotorStatus_ == MOTOR_OK) {
    p.drive[1].errorState = ERROR_OK;
    p.drive[1].controlMode = driveOn_ ? bb::StatePacket::CONTROL_RC : bb::StatePacket::CONTROL_OFF;
    p.drive[1].presentPWM = leftMotor_.present();
    p.drive[1].presentPos = leftEncoder_.presentPosition();
    p.drive[1].presentSpeed = leftEncoder_.presentSpeed();
    lSpeedController_.getControlState(err, errI, errD, control);
    p.drive[1].goal = lSpeedController_.goal();
    p.drive[1].err = err;
    p.drive[1].errI = errI;
    p.drive[1].errD = errD;
    p.drive[1].control = control;
  } else {
    p.drive[1].errorState = ERROR_NOT_PRESENT;
  }

  if(rightMotorStatus_ == MOTOR_OK) {
    p.drive[2].errorState = ERROR_OK;
    p.drive[2].controlMode = driveOn_ ? bb::StatePacket::CONTROL_RC : bb::StatePacket::CONTROL_OFF;
    p.drive[2].presentPWM = rightMotor_.present();
    p.drive[2].presentPos = rightEncoder_.presentPosition();
    p.drive[2].presentSpeed = rightEncoder_.presentSpeed();
    rSpeedController_.getControlState(err, errI, errD, control);
    p.drive[2].goal = rSpeedController_.goal();
    p.drive[2].err = err;
    p.drive[2].errI = errI;
    p.drive[2].errD = errD;
    p.drive[2].control = control;
  } else {
     p.drive[2].errorState = ERROR_NOT_PRESENT;
  }

  p.imu[0] = imu_.getIMUState();
  p.imu[1].errorState = ERROR_NOT_PRESENT;
  p.imu[2].errorState = ERROR_NOT_PRESENT;

  p.battery[0] = DOBattStatus::batt.getBatteryState();
  p.battery[1].errorState = ERROR_NOT_PRESENT;
  p.battery[2].errorState = ERROR_NOT_PRESENT;

  WifiServer::server.broadcastUDPPacket((const uint8_t*)&p, sizeof(p));

  return RES_OK;
}


bool DODroid::setAntennas(uint8_t a1, uint8_t a2, uint8_t a3) {
  if(antennasOK_ == false) return false;
  uint8_t antennas[3] = {a1, a2, a3};
  Wire.beginTransmission(0x17);
  Wire.write(antennas, sizeof(antennas));
  if(Wire.endTransmission() == 0) return true;
  Console::console.printfBroadcast("Error moving antennas\n");
  antennasOK_ = false;
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