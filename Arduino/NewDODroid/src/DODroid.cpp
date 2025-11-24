#include "DODroid.h"
#include "DOConfig.h"
#include "DOBattStatus.h"
#include "DOSound.h"
#include <SAMD_PWM.h>
#include <map>
#include "../resources/systemsounds.h"

#include "DOHeadInterface.h"

#include <MCS/BBRMPacket.h>

DODroid DODroid::droid;
DOParams DODroid::params_;
bb::ConfigStorage::HANDLE DODroid::paramsHandle_;
static head::Parameters headParameters_;
static int eyesOffset_ = 0;

static float minPwmFreq = 200.0f;
static float maxPwmFreq = 500.0f;
static float detune = 0.0f;
static std::map<uint8_t, SAMD_PWM*> pwms;

static void configureTimers() {
  pwms[P_LEFT_PWMA] = new SAMD_PWM(P_LEFT_PWMA, minPwmFreq, 0);
  pwms[P_LEFT_PWMB] = new SAMD_PWM(P_LEFT_PWMB, minPwmFreq, 0);;
  pwms[P_RIGHT_PWMA] = new SAMD_PWM(P_RIGHT_PWMA, minPwmFreq, 0);
  pwms[P_RIGHT_PWMB] = new SAMD_PWM(P_RIGHT_PWMB, minPwmFreq, 0);
  pwms[P_LEFT_PWMA]->setPWM();
  pwms[P_LEFT_PWMB]->setPWM();
  pwms[P_RIGHT_PWMA]->setPWM();
  pwms[P_RIGHT_PWMB]->setPWM();
}

static void customAnalogWrite(uint8_t pin, uint8_t dutycycle) {
  SAMD_PWM* pwm = pwms[pin];
  if(pwm == nullptr) {
    return;
  }
  float f = map(dutycycle, 0, 255, minPwmFreq, maxPwmFreq);
  if(pin == P_RIGHT_PWMA || pin == P_RIGHT_PWMB) f += detune/2;
  else f -= detune/2;
  pwm->setPWM(pin, f, map(dutycycle, 0, 255, 0.0, 100.0));
}

DODroid::DODroid():
  imu_(),

  leftMotor_(P_LEFT_PWMA, P_LEFT_PWMB), 
  rightMotor_(P_RIGHT_PWMA, P_RIGHT_PWMB), 
  leftEncoder_(P_LEFT_ENCA, P_LEFT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),
  rightEncoder_(P_RIGHT_ENCA, P_RIGHT_ENCB, bb::Encoder::INPUT_SPEED, bb::Encoder::UNIT_MILLIMETERS),

  lSpeedController_(leftEncoder_, leftMotor_),
  rSpeedController_(rightEncoder_, rightMotor_),

  balanceInput_(imu_, bb::IMUControlInput::IMU_PITCH),

  velOutput_(lSpeedController_, rSpeedController_),

  balanceController_(balanceInput_, velOutput_),

  posInput_(leftEncoder_, rightEncoder_),
  posOutput_(velOutput_),
  autoPosController_(posInput_, posOutput_),
  posController_(posInput_, posOutput_),

  statusPixels_(3, P_STATUS_NEOPIXEL, NEO_GRB+NEO_KHZ800),

  receiver_(nullptr)
{
  // Pull down the GND pins for the motor controllers.
  pinMode(PULL_DOWN_15, OUTPUT);
  digitalWrite(PULL_DOWN_15, LOW);
  pinMode(PULL_DOWN_20, OUTPUT);
  digitalWrite(PULL_DOWN_20, LOW);

  statusPixels_.begin();
  statusPixels_.setBrightness(2);
  statusPixels_.show();

  name_ = "d-o";

  description_ = "D-O Main System";
  help_ = "Available commands:\n"\
"\tstatus\t\tPrint Status\n"\
"\tselftest\tRun self test\n"\
"\tsafety {on|off}\tSwitch safety functions on/off\n"\
"\tdrive {off|pos|vel}\tSwitch drive system to off, position control, or velocity control\n"\
"\tplay_sound [<folder>] <num>\tPlay sound\n"\
"\tset_aerials A1 [A2 A3]\tMove aerials. A1, A2, A3: Angle between 0 and 180\n";
  started_ = false;

  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  setLED(LED_STATUS, WHITE);
  setLED(LED_COMM, OFF);
  setLED(LED_DRIVE, OFF);

  commLEDOn_ = false;
  msLastLeftCtrlPacket_ = msLastRightCtrlPacket_ = msLastPrimaryCtrlPacket_ = 0;
}

Result DODroid::initialize(Uart* remoteUart) {
  addParameter("neck_range", "Neck servo movement range", params_.neckRange, -INT_MAX, INT_MAX);
  addParameter("neck_offset", "Neck servo offset", params_.neckOffset, -INT_MAX, INT_MAX);
  addParameter("head_roll_range", "Head roll servo movement range", params_.headRollRange, -INT_MAX, INT_MAX);
  addParameter("head_roll_offset", "Head roll servo servo offset", params_.headRollOffset, -INT_MAX, INT_MAX);
  addParameter("head_pitch_range", "Head pitch servo movement range", params_.headPitchRange, -INT_MAX, INT_MAX);
  addParameter("head_pitch_offset", "Head pitch servo offset", params_.headPitchOffset, -INT_MAX, INT_MAX);
  addParameter("head_heading_range", "Head heading servo movement range", params_.headHeadingRange, -INT_MAX, INT_MAX);
  addParameter("head_heading_offset", "Head heading servo servo offset", params_.headHeadingOffset, -INT_MAX, INT_MAX);

  addParameter("mot_deadband", "Deadband for drive motors", params_.motorDeadband, 0, 255);

  addParameter("wheel_kp", "Proportional constant for wheel speed PID controller", params_.wheelKp, -INT_MAX, INT_MAX);
  addParameter("wheel_ki", "Integrative constant for wheel speed PID controller", params_.wheelKi, -INT_MAX, INT_MAX);
  addParameter("wheel_kd", "Derivative constant for wheel speed PID controller", params_.wheelKd, -INT_MAX, INT_MAX);

  addParameter("bal_kp", "Proportional constant for balance PID controller", params_.balKp, -INT_MAX, INT_MAX);
  addParameter("bal_ki", "Integrative constant for balance PID controller", params_.balKi, -INT_MAX, INT_MAX);
  addParameter("bal_kd", "Derivative constant for balance PID controller", params_.balKd, -INT_MAX, INT_MAX);

  addParameter("auto_pos_kp", "Proportional constant for position PID controller", params_.autoPosKp, -INT_MAX, INT_MAX);
  addParameter("auto_pos_ki", "Integrative constant for position PID controller", params_.autoPosKi, -INT_MAX, INT_MAX);
  addParameter("auto_pos_kd", "Derivative constant for position PID controller", params_.autoPosKd, -INT_MAX, INT_MAX);

  addParameter("pos_kp", "Proportional constant for position PID controller", params_.posKp, -INT_MAX, INT_MAX);
  addParameter("pos_ki", "Integrative constant for position PID controller", params_.posKi, -INT_MAX, INT_MAX);
  addParameter("pos_kd", "Derivative constant for position PID controller", params_.posKd, -INT_MAX, INT_MAX);

  addParameter("accel", "Acceleration in mm/s^2", params_.accel, -INT_MAX, INT_MAX);
  addParameter("max_speed", "Maximum speed (only honored in speed control mode)", params_.maxSpeed, 0, INT_MAX);
  addParameter("speed_axis_gain", "Gain for controller speed axis", params_.speedAxisGain, -INT_MAX, INT_MAX);
  addParameter("rot_axis_gain", "Gain for controller rot axis", params_.rotAxisGain, -INT_MAX, INT_MAX);

  addParameter("lean_head_to_body", "Lean multiplier to counter head motion with body motion", params_.leanHeadToBody, -INT_MAX, INT_MAX);

  addParameter("aerial_offset", "Offset for aerials", params_.aerialOffset, -INT_MAX, INT_MAX);
  addParameter("aerial_anim", "Animation angle for aerials", params_.aerialAnim, -INT_MAX, INT_MAX);

  addParameter("fa_neck_imu_pitch", "Free Anim - neck on IMU pitch", params_.faNeckIMUPitch, -INT_MAX, INT_MAX);
  addParameter("fa_neck_imu_accel", "Free Anim - neck on IMU accel", params_.faNeckIMUAccel, -INT_MAX, INT_MAX);
  addParameter("fa_neck_sp_accel", "Free Anim - neck on accel setpoint", params_.faNeckSPAccel, -INT_MAX, INT_MAX);
  addParameter("fa_neck_speed", "Free Anim - neck on wheel speed", params_.faNeckSpeed, -INT_MAX, INT_MAX);
  addParameter("fa_neck_speed_sp", "Free Anim - neck on wheel speed setpoint", params_.faNeckSpeedSP, -INT_MAX, INT_MAX);
  addParameter("fa_head_pitch_speed_sp", "Free Anim - head pitch on wheel speed setpoint", params_.faHeadPitchSpeedSP, -INT_MAX, INT_MAX);
  addParameter("fa_head_roll_turn", "Free Anim: Head roll on turn speed", params_.faHeadRollTurn, -INT_MAX, INT_MAX);
  addParameter("fa_head_heading_turn", "Free Anim: Head heading on turn speed", params_.faHeadHeadingTurn, -INT_MAX, INT_MAX);
  addParameter("fa_aerial_speed", "Free Anim: Aerial position on wheel speed setpoint", params_.faAerialSpeedSP, -INT_MAX, INT_MAX);
  addParameter("fa_head_anneal_time", "Free Anim: Head anneal time", params_.faHeadAnnealTime, -INT_MAX, INT_MAX);

  addParameter("auto_pos_control", "Automatically switch to position control", params_.autoPosControl);

  configureTimers();
  leftMotor_.setCustomAnalogWrite(&customAnalogWrite);
  rightMotor_.setCustomAnalogWrite(&customAnalogWrite);

  paramsHandle_ = ConfigStorage::storage.reserveBlock("d-o", sizeof(params_), (uint8_t*)&params_);
  if(ConfigStorage::storage.blockIsValid(paramsHandle_)) {
    LOG(LOG_INFO, "Storage block 0x%x is valid.\n", paramsHandle_);
    ConfigStorage::storage.readBlock(paramsHandle_);
    LOG(LOG_INFO, "Left Address: 0x%lx:%lx\n", params_.leftRemoteAddress.addrHi, params_.leftRemoteAddress.addrLo);
  } else {
    LOG(LOG_INFO, "Remote: Storage block 0x%x is invalid, using initialized parameters.\n", paramsHandle_);
    ConfigStorage::storage.writeBlock(paramsHandle_);
  }

  lSpeedController_.setAutoUpdate(false);
  rSpeedController_.setAutoUpdate(false);

  balanceController_.setAutoUpdate(false);
  balanceController_.setReverse(true); // FIXME Not quite sure anymore why we're reversing here, we should be forwarding. Check!
  //balanceController_.setDebug(true);
 
  imu_.setRotationAroundZ(bb::IMU::ROTATE_90);

  //setPacketSource(PACKET_SOURCE_DROID);

  protocol_.init("DODroid", DEFAULT_CHAN, DEFAULT_PAN, remoteUart);
  receiver_ = protocol_.createReceiver();
  if(receiver_ == nullptr) {
    bb::printf("Could not create receiver\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  uint8_t turnInput = receiver_->addInput("turn", remTurn_);
  uint8_t velInput = receiver_->addInput("vel", remVel_);
  uint8_t headPInput = receiver_->addInput("headP", remP_);
  uint8_t headRInput = receiver_->addInput("headR", remR_);
  uint8_t headHInput = receiver_->addInput("headH", remH_);
  uint8_t nodInput = receiver_->addInput("nod", remLean_);
  uint8_t volInput = receiver_->addInput("vol", remVol_);
  uint8_t primDMI = receiver_->addInput("drvVel", [this](float v){primaryDriveButtonCB(v);});
  uint8_t secDMI = receiver_->addInput("drvPos", [this](float v){secondaryDriveButtonCB(v);});
  uint8_t headAnnealInput = receiver_->addInput("headAnneal", [this](float v){headAnnealCB(v);});
  uint8_t play1Input = receiver_->addInput("playGreet", [this](float v){playSoundCB(v, 2);});
  uint8_t play2Input = receiver_->addInput("playHappy", [this](float v){playSoundCB(v, 3);});
  uint8_t play3Input = receiver_->addInput("playSad", [this](float v){playSoundCB(v, 4);});
  uint8_t twitch1Input = receiver_->addInput("twitch1", remAerial1_);
  uint8_t twitch2Input = receiver_->addInput("twitch2", remAerial2_);
  uint8_t twitch3Input = receiver_->addInput("twitch3", remAerial3_);

  protocol_.setCommTimeoutWatchdog(0.5, [this](Protocol* p, float s) {commTimeoutCB(p, s);});

  receiver_->setMix(turnInput, AxisMix(0, INTERP_LIN_CENTERED));
  receiver_->setMix(velInput, AxisMix(1, INTERP_LIN_CENTERED));
  receiver_->setMix(headPInput, AxisMix(2, INTERP_LIN_CENTERED));
  receiver_->setMix(headRInput, AxisMix(3, INTERP_LIN_CENTERED));
  receiver_->setMix(headHInput, AxisMix(4, INTERP_LIN_CENTERED));
  receiver_->setMix(volInput, AxisMix(8, INTERP_LIN_POSITIVE));
  receiver_->setMix(play1Input, AxisMix(11, INTERP_LIN_POSITIVE));
  receiver_->setMix(play2Input, AxisMix(12, INTERP_LIN_POSITIVE));
  receiver_->setMix(headAnnealInput, AxisMix(13, INTERP_LIN_POSITIVE));
  receiver_->setMix(play3Input, AxisMix(14, INTERP_LIN_POSITIVE));
  receiver_->setMix(primDMI, AxisMix(15, INTERP_LIN_POSITIVE));
  
  receiver_->setMix(nodInput, AxisMix(1+SECONDARY_ADD, INTERP_LIN_CENTERED));
  receiver_->setMix(secDMI, AxisMix(15+SECONDARY_ADD, INTERP_LIN_POSITIVE));
  receiver_->setMix(twitch1Input, AxisMix(16+SECONDARY_ADD, INTERP_LIN_POSITIVE));
  receiver_->setMix(twitch2Input, AxisMix(17+SECONDARY_ADD, INTERP_LIN_POSITIVE));
  receiver_->setMix(twitch3Input, AxisMix(18+SECONDARY_ADD, INTERP_LIN_POSITIVE));

  receiver_->setDataFinishedCallback([this](const NodeAddr& addr, uint8_t seqnum) { dataFinishedCB(addr, seqnum); });

  return Subsystem::initialize();
}

Result DODroid::start(ConsoleStream* stream) {
  leftMotorStatus_ = MOTOR_UNTESTED;
  rightMotorStatus_ = MOTOR_UNTESTED;
  servosOK_ = false;
  aerialsOK_ = false;
  driveMode_ = DRIVE_OFF;
  driveSafety_ = true;
  headIsOn_ = false;
  pitchAtRest_ = 0;
  lastLeftSeqnum_ = 255;
  lastRightSeqnum_ = 255;

  DOSound::sound.begin();
  imu_.begin(IMU_ADDR);
  DOBattStatus::batt.begin();

  //operationStatus_ = RES_OK;
  operationStatus_ = selfTest();
  if(bb::Servos::servos.hasServoWithID(SERVO_HEAD_HEADING) || 
     bb::Servos::servos.hasServoWithID(SERVO_HEAD_PITCH) || 
     bb::Servos::servos.hasServoWithID(SERVO_HEAD_ROLL)) {
    LOG(LOG_INFO, "Head is on\n");
    headIsOn_ = true;
  } else {
    LOG(LOG_INFO, "Head is not on\n");
    headIsOn_ = false;
  }

  if(operationStatus_ == RES_DROID_VOLTAGE_TOO_LOW) {
    if(DOBattStatus::batt.voltage() < 1.0) {
      LOG(LOG_ERROR, "No power (%.fV), USB only!\n", DOBattStatus::batt.voltage());
      started_ = true;
      operationStatus_ = RES_OK;
      setLED(LED_STATUS, BLUE);
      return RES_OK;
    }
  } else if(operationStatus_ != RES_OK) {
    setLED(LED_STATUS, RED);
    setControlStrip(head::CONTROL_STRIP_ERROR, true);
    return operationStatus_;
  }

  setControlParameters();

  annealH_ = annealP_ = annealR_ = 0;

  started_ = true;
  operationStatus_ = RES_OK;
  setLED(LED_STATUS, GREEN);
  setEyes(head::EYE_COLOR_WHITE, 10, 150, head::EYE_COLOR_WHITE, 10, 150);
  updateHead();
  statusPixels_.show();

  return operationStatus_;
}

Result DODroid::stop(ConsoleStream* stream) {
  (void) stream;
  started_ = false;
  operationStatus_ = RES_SUBSYS_NOT_STARTED;
  setLED(LED_STATUS, WHITE); 

  return RES_OK;
}

void DODroid::setControlParameters() {
  leftMotor_.setDeadband(params_.motorDeadband);
  rightMotor_.setDeadband(params_.motorDeadband);

  leftEncoder_.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  leftEncoder_.setMode(bb::Encoder::INPUT_SPEED);
  leftEncoder_.setUnit(bb::Encoder::UNIT_MILLIMETERS);
  rightEncoder_.setMillimetersPerTick(WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_TURN);
  rightEncoder_.setMode(bb::Encoder::INPUT_SPEED);
  rightEncoder_.setUnit(bb::Encoder::UNIT_MILLIMETERS);

  lSpeedController_.setControlParameters(params_.wheelKp, params_.wheelKi, params_.wheelKd);
  rSpeedController_.setControlParameters(params_.wheelKp, params_.wheelKi, params_.wheelKd);
  lSpeedController_.setIBounds(-255, 255);
  rSpeedController_.setIBounds(-255, 255);
  lSpeedController_.reset();
  rSpeedController_.reset();

  balanceController_.setControlParameters(params_.balKp, params_.balKi, params_.balKd);
  balanceController_.setRamp(0);
  balanceController_.setErrorDeadband(-1.0, 1.0);
  balanceController_.reset();
  velOutput_.setAcceleration(params_.accel);
  velOutput_.setMaxSpeed(params_.maxSpeed);

  autoPosController_.setControlParameters(params_.autoPosKp, params_.autoPosKi, params_.autoPosKd);
  posController_.setControlParameters(params_.posKp, params_.posKi, params_.posKd);
}

Result DODroid::step() {
  protocol_.step();

  unsigned long seqnum = Runloop::runloop.getSequenceNumber();
  static unsigned long lastSDCardCheck = millis();

  // We're broken; still send out the state packet.
  if(!imu_.available() || !DOBattStatus::batt.available()) {
    if((seqnum % 4) == 0) {
      sendTelemetry();
    }
    if(!imu_.available()) {
      LOG(LOG_FATAL, "IMU missing - critical error\n");
      leftMotor_.set(0);
      rightMotor_.set(0);
    }
    if(!DOBattStatus::batt.available()) {
      LOG(LOG_FATAL, "Battery missing - critical error\n");
      leftMotor_.set(0);
      rightMotor_.set(0);
    }
    
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  // Look for battery undervoltage
  if((Runloop::runloop.getSequenceNumber() % 100) == 0) {
    Runloop::runloop.excuseOverrun();
    stepPowerProtect();
  }

  // Check if SD card was changed
  if(WRAPPEDDIFF(millis(), lastSDCardCheck, ULONG_MAX) > 1000 && driveMode_ == DRIVE_OFF) {
    Runloop::runloop.excuseOverrun();
    DOSound::sound.checkSDCard();
    lastSDCardCheck = millis();
  }

  // Encoder and IMU updates are needed for everything, so we do them here.
  leftEncoder_.update();  
  rightEncoder_.update();  
  imu_.update();

  // We only run head and drive control at 50Hz
  if((seqnum % 2) == 0) {
    stepHead();
  } else {
    // stepDrive() takes less time than stepHead()
    stepDrive();
    updateHead();
  }

  if((seqnum % 4) == 0) {
    sendTelemetry();
    DOSound::sound.setVolume(remVol_*30);
    if(Servos::servos.isStarted()) setLED(LED_STATUS, GREEN, false);
    else setLED(LED_STATUS, YELLOW, false);
  }

  statusPixels_.show();

  return RES_OK;
}

bb::Result DODroid::stepPowerProtect() {
  DOBattStatus::batt.updateVoltage();
  DOBattStatus::batt.updateCurrent();

  // CRITICAL: Switch everything off (except the neck servo, so that we don't drop the head) and go into endless loop if power 
  if(DOBattStatus::batt.voltage() > 2.0 &&
      DOBattStatus::batt.voltage() < POWER_BATT_MIN) {
    leftMotor_.set(0);
    rightMotor_.set(0);
    bb::Servos::servos.switchTorque(SERVO_HEAD_PITCH, false);
    bb::Servos::servos.switchTorque(SERVO_HEAD_HEADING, false);
    bb::Servos::servos.switchTorque(SERVO_HEAD_ROLL, false);

    setLED(LED_STATUS, RED);
    operationStatus_ = RES_SUBSYS_HW_DEPENDENCY_MISSING;
    while(true) {
      int i=0;
      if(i%5 == 0) {
        DOSound::sound.playSystemSound(SystemSounds::VOLTAGE_TOO_LOW);
        LOG(LOG_ERROR, "Voltage too low (%.fV), system disabled!\n", DOBattStatus::batt.voltage());
        sendTelemetry();
      }
      i++;
      delay(1000);
    }
  }
  
  return RES_OK;
}

bb::Result DODroid::stepHead() {
  float p, r, h, dr, dp, dh, ax, ay, az;
  imu_.getFilteredPRH(p, r, h);
  imu_.getAccelMeasurement(ax, ay, az);
  imu_.getGyroMeasurement(dp, dr, dh);

  LOG(LOG_DEBUG, "R:%f P:%f H:%f AX:%f AY:%f AZ:%f DR:%f DP:%f DH:%f\n", r, p, h, ax, ay, az, dr, dp, dh);

  float speed = (leftEncoder_.presentSpeed() + rightEncoder_.presentSpeed())/2;
  float speedSP = velOutput_.goalVelocity();
  float accelSP = (speedSP-speed);
  if(speedSP == 0) accelSP = 0;

  if(servosOK_ && headIsOn_) {
    float nod;
    if(HEAD_COUNTERWEIGHT == false) {
      nod = params_.faNeckIMUAccel*ax + params_.faNeckSPAccel*accelSP + params_.faNeckSpeed*speed;
      if(speedSP < 0) nod += params_.faNeckSpeedSP*speedSP/2;
      else nod += params_.faNeckSpeedSP*speedSP;
    } else {
      nod = params_.faNeckIMUAccel*ax + params_.faNeckSPAccel*accelSP + params_.faNeckSpeed*speed;
      if(speedSP < 0) nod += params_.faNeckSpeedSP*speedSP/2;
      else nod += params_.faNeckSpeedSP*speedSP;
    }

    lean_ = leanFilter_.filter(-1 * remLean_ * params_.neckRange);
    if(headIsOn_) {
      balanceController_.setGoal(params_.leanHeadToBody*lean_-pitchAtRest_);
    }
    nod += lean_ + pitchAtRest_;

    float neck = nod + params_.faNeckIMUPitch*p;
    neck = constrain(neck, -params_.neckRange, params_.neckRange);
    bb::Servos::servos.setGoalPos(SERVO_NECK, 180 + neck);

    float headPitch = -nod;
    headPitch += remoteP_;
    headPitch += params_.faHeadPitchSpeedSP*speedSP;
    bb::Servos::servos.setGoalPos(SERVO_HEAD_PITCH, 180 + headPitch);
    bb::Servos::servos.setGoalPos(SERVO_HEAD_HEADING, 180.0 + params_.faHeadHeadingTurn * dh + remoteH_);
    bb::Servos::servos.setGoalPos(SERVO_HEAD_ROLL, 180.0 - params_.faHeadRollTurn * dh + remoteR_);
  } else {
    bb::Servos::servos.setGoalPos(SERVO_NECK, 180);
  }
    
  if(aerialsOK_) {
    float aer = 90 + params_.aerialOffset + speedSP*params_.faAerialSpeedSP;

    aer = constrain(aer, 0, 180);
    setAerials(constrain(aer + remAerial1_*params_.aerialAnim, 0, 180), 
               constrain(aer + remAerial2_*params_.aerialAnim, 0, 180),
               constrain(aer + remAerial3_*params_.aerialAnim, 0, 180));

    int eyePosInt = 150 + uint8_t(100.0*(speed / params_.maxSpeed)) + eyesOffset_;
    if(speed<0) eyePosInt = 150 + eyesOffset_;

    uint8_t eyePos = constrain(eyePosInt, 0, 255);
    uint8_t eyeSize = 10 - uint8_t(8.0*(speed / params_.maxSpeed));
    setEyes(head::EYE_COLOR_WHITE, eyeSize, eyePos, head::EYE_COLOR_WHITE, eyeSize, eyePos);

    // updateHead() is done in step(), in the stepDrive() part of the loop, for performance reasons
    // updateHead();
  }
  
  return RES_OK;
}

bb::Result DODroid::stepDrive() {
  if(EPSILON(remVel_) && EPSILON(remTurn_) && params_.autoPosControl == true) { 
      if(WRAPPEDDIFF(millis(), msSinceDriveInput_, ULONG_MAX) > 500 && 
          driveMode_ == DRIVE_VEL) {
        switchDrive(DRIVE_AUTO_POS);
      }
    } else { // We have drive input - switch to velocity control modex
      if(driveMode_ == DRIVE_AUTO_POS) {
        switchDrive(DRIVE_VEL);
      }
      msSinceDriveInput_ = millis();
    }

  if(driveMode_ == DRIVE_VEL) {
    float vel = remVel_, rot = remTurn_;

    vel = constrain(vel * params_.maxSpeed * params_.speedAxisGain, -params_.maxSpeed, params_.maxSpeed);
    rot = constrain(rot * params_.maxSpeed * params_.rotAxisGain, -params_.maxSpeed, params_.maxSpeed);

    velOutput_.setGoalVelocity(vel);
    velOutput_.setGoalRotation(rot);
  } else if(driveMode_ == DRIVE_POS) {
    float posDelta = remPos_ * 1000; // FIXME - should be a constant
    posController_.setGoal(posControllerZero_ + posDelta);
  }


  if(driveMode_ != DRIVE_OFF && leftMotorStatus_ == MOTOR_OK && rightMotorStatus_ == MOTOR_OK) {
    balanceController_.update();
    lSpeedController_.update();
    rSpeedController_.update();
    if(driveMode_ == DRIVE_POS) posController_.update();
    else if(driveMode_ == DRIVE_AUTO_POS) autoPosController_.update();
  } else {
    leftMotor_.set(0);
    rightMotor_.set(0);
  }
  return RES_OK;
}

bb::Result DODroid::stepIfNotStarted() {
  protocol_.step();
  if(imu_.available()) imu_.update();
  leftEncoder_.update();
  rightEncoder_.update();
  if((Runloop::runloop.getSequenceNumber() % 4) == 0) {
    sendTelemetry();
  }
  if((Runloop::runloop.getSequenceNumber() % 100) == 0 && DOBattStatus::batt.available()) {
    DOBattStatus::batt.updateVoltage();
    DOBattStatus::batt.updateCurrent();
  }

  statusPixels_.show();

  return RES_OK;
}

void DODroid::primaryDriveButtonCB(float val) {
  static bool wasPressed = false;
  if(wasPressed == false && val > 0.5) {
    wasPressed = true;

    if(driveMode_ != DRIVE_VEL) switchDrive(DRIVE_VEL);
    else switchDrive(DRIVE_OFF);
  } else if(wasPressed == true && val < 0.5) {
    wasPressed = false;
  }
}

void DODroid::secondaryDriveButtonCB(float val) {
  static bool wasPressed = false;
  if(wasPressed == false && val > 0.5) {
    wasPressed = true;

    if(driveMode_ != DRIVE_POS) switchDrive(DRIVE_POS);
    else switchDrive(DRIVE_OFF);
  } else if(wasPressed == true && val < 0.5) {
    bb::printf("2 Released now\n", val);
    wasPressed = false;
  }
}

void DODroid::playSoundCB(float val, uint8_t folder) {
  static bool wasPressed = false;
  if(wasPressed == false && val > 0.5) {
    wasPressed = true;

    DOSound::sound.playFolderNext(folder);
  } else if(wasPressed == true && val < 0.5) {
    bb::printf("2 Released now\n", val);
    wasPressed = false;
  }
}

void DODroid::headAnnealCB(float val) {
  if(val > 0.5) {
    annealR_ = 0;
    annealP_ = 0;
    annealH_ = 0;
    remoteP_ = remP_ * 180.0;
    remoteR_ = remR_ * 180.0;
    remoteH_ = remH_ * 180.0;
    annealDelay_ = params_.faHeadAnnealDelay;
  } else {
    if(annealDelay_ > 0) {
      annealDelay_ -= Runloop::runloop.cycleTimeSeconds()*4;
      if(annealDelay_ < 0) {
        annealR_ = remoteR_ / (params_.faHeadAnnealTime / (bb::Runloop::runloop.cycleTimeSeconds()*4));
        annealP_ = remoteP_ / (params_.faHeadAnnealTime / (bb::Runloop::runloop.cycleTimeSeconds()*4));
        annealH_ = remoteH_ / (params_.faHeadAnnealTime / (bb::Runloop::runloop.cycleTimeSeconds()*4));
      } 
    }
    
    if(fabs(remoteR_ - annealR_) <= fabs(annealR_)) remoteR_ = annealR_ = 0;
    else remoteR_ -= annealR_;
    if(fabs(remoteP_ - annealP_) <= fabs(annealP_)) remoteP_ = annealP_ = 0;
    else remoteP_ -= annealP_;
    if(fabs(remoteH_ - annealH_) <= fabs(annealH_)) remoteH_ = annealH_ = 0;
    else remoteH_ -= annealH_;
  }
}

void DODroid::commTimeoutCB(Protocol* proto, float seconds) {
  if(driveMode_ != DRIVE_OFF) {
    LOG(LOG_ERROR, "No primary control packet since %f seconds - switching drive system off\n", seconds);
    switchDrive(DRIVE_OFF);
  }
  setLED(LED_COMM, RED, false);
}

void DODroid::dataFinishedCB(const NodeAddr& addr, uint8_t seqnum) {
  static unsigned long last = millis();
  static unsigned long flashDelay = 250;
  static bool on = false;

  if(on == false && WRAPPEDDIFF(millis(), last, ULONG_MAX) > flashDelay) {
    on = true;
    setLED(LED_COMM, WHITE, false);
    last = millis();
  } else if(on == true && WRAPPEDDIFF(millis(), last, ULONG_MAX) > flashDelay) {
    on = false;
    setLED(LED_COMM, OFF, false);
    last = millis();
  }
}

void DODroid::switchDrive(DriveMode mode) {
  lSpeedController_.reset();
  lSpeedController_.setGoal(0);
  rSpeedController_.reset();
  rSpeedController_.setGoal(0);
  balanceController_.reset();
  balanceController_.setGoal(-pitchAtRest_);
  autoPosController_.reset();
  autoPosController_.setPresentAsGoal();
  posController_.reset();
  posController_.setPresentAsGoal();
  posControllerZero_ = posController_.present();

  driveMode_ = mode;
  switch(driveMode_) {
    case DRIVE_OFF: 
      setLED(LED_DRIVE, OFF, false);
      setControlStrip(head::CONTROL_STRIP_OFF, true);
      LOG(LOG_INFO, "Switched drive mode to off.\n");
      break;
    
    case DRIVE_VEL: 
      setLED(LED_DRIVE, GREEN, false);
      setControlStrip(head::CONTROL_STRIP_VEL, true);
      LOG(LOG_INFO, "Switched drive mode to velocity.\n");
      break;

    case DRIVE_POS: 
      setLED(LED_DRIVE, YELLOW, false);
      setControlStrip(head::CONTROL_STRIP_MAN_POS, true);
      LOG(LOG_INFO, "Switched drive mode to position.\n");
      break;

    case DRIVE_AUTO_POS:
    default: 
      setLED(LED_DRIVE, BLUE, false);
      setControlStrip(head::CONTROL_STRIP_AUTO_POS, true);
      LOG(LOG_INFO, "Switched drive mode to auto_position.\n");
      break;
  }
}

String DODroid::statusLine() {
  String str = bb::Subsystem::statusLine();
  str += ", batt: ";
  if(DOBattStatus::batt.available()) {
    str = str + DOBattStatus::batt.voltage() + "V " + DOBattStatus::batt.current() + "mA";
  } else {
    str += "not available";
  }

  if(Servos::servos.isStarted()) str += ", servos OK";
  else str += ", servos not started";

  str += ", motors: ";

  leftEncoder_.update();
  if(leftMotorStatus_ == MOTOR_OK) str += "L OK";
  else str = str + "L Err " + leftMotorStatus_;

  rightEncoder_.update();
  if(rightMotorStatus_ == MOTOR_OK) str += ", R OK";
  else str = str + ", R Err " + rightMotorStatus_;

  if(imu_.available()) str = str + ", IMU OK";
  else str = str + ", IMU Error";  

  return str;
}

void DODroid::printExtendedStatus(ConsoleStream *stream) {
  stream->printf("Started: %s\n", started_?"yes":"no");
  stream->printf("Operation status: %s\n", errorMessage(operationStatus_));

  stream->printf("Control packets received:\n");
  stream->printf("\tFrom left remote: %d\n", numLeftCtrlPackets_);
  stream->printf("\tFrom right remote: %d\n", numRightCtrlPackets_);

  stream->printf("Motors:\n");
  leftEncoder_.update();
  if(leftMotorStatus_ == MOTOR_OK) stream->printf("\tLeft OK, encoder %f\n", leftEncoder_.presentPosition());
  else stream->printf("\tLeft status: %s, encoder at %.2fmm\n", motorStatusToString(leftMotorStatus_), leftEncoder_.presentPosition());
  rightEncoder_.update();
  if(rightMotorStatus_ == MOTOR_OK) stream->printf("\tRight OK, encoder %f\n", rightEncoder_.presentPosition());
  else stream->printf("\tRight status: %s, encoder at %.2fmm\n", motorStatusToString(rightMotorStatus_), rightEncoder_.presentPosition());

  float p, r, h, ax, ay, az;
  imu_.getFilteredPRH(p, r, h);
  imu_.getAccelMeasurement(ax, ay, az);
  stream->printf("IMU:\n");
  stream->printf("\tPitch %.2f Roll %.2f Heading %.2f\n", p, r, h);
  stream->printf("\tAx %.2f Ay %.2f Az %.2f\n", ax, ay, az);

  DOBattStatus::batt.updateCurrent();
  DOBattStatus::batt.updateVoltage();
  stream->printf("Battery: %.2fmA, %.2fV\n", DOBattStatus::batt.current(), DOBattStatus::batt.voltage());
}

Result DODroid::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
  if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;

  if(words[0] == "selftest") {
    if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
    Runloop::runloop.excuseOverrun();
    return selfTest(stream);
  } 
  
  else if(words[0] == "play_sound") {
    if(words.size() == 1 || words.size() > 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words.size() == 2) {
      bool retval = DOSound::sound.playSound(words[1].toInt());
      if(retval == false) stream->printf("Error\n");      
    } if(words.size() == 3) {
      bool retval = DOSound::sound.playFolder((unsigned int)(words[1].toInt()), words[2].toInt());
      if(retval == false) stream->printf("Error\n");
    }
    return RES_OK;
  }

  else if(words[0] == "safety") {
    if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words[1] == "off") {
      driveSafety_ = false;
      return RES_OK;
    } else if(words[1] == "on") {
      driveSafety_ = true;
      return RES_OK;
    } else return RES_CMD_INVALID_ARGUMENT;
  }

  else if(words[0] == "drive") {
    if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
    if(words[1] == "off") {
      switchDrive(DRIVE_OFF);
      return RES_OK;
    } else if(words[1] == "pos") {
      switchDrive(DRIVE_POS);
      return RES_OK;
    } else if(words[1] == "vel") {
      switchDrive(DRIVE_VEL);
      return RES_OK;
    } else return RES_CMD_INVALID_ARGUMENT;
  }

  else if(words[0] == "set_aerials") {
    if(words.size() == 2) {
      float angle = words[1].toFloat();
      if(setAerials(angle, angle, angle) == true) return RES_OK;
      return RES_CMD_FAILURE;
    } else if(words.size() == 4) {
      float a1 = words[1].toFloat(), a2 = words[2].toFloat(), a3 = words[3].toFloat();
      if(setAerials(a1, a2, a3) == true) return RES_OK;
      return RES_CMD_FAILURE;
    }

    return RES_CMD_INVALID_ARGUMENT_COUNT;
  }
  
  return bb::Subsystem::handleConsoleCommand(words, stream);
}

Result DODroid::setParameterValue(const String& name, const String& stringVal) {
  Result retval = Subsystem::setParameterValue(name, stringVal);
  if(retval != RES_OK) return retval;

  setControlParameters();
  bb::Servos::servos.setOffset(SERVO_NECK, params_.neckOffset);
  bb::Servos::servos.setOffset(SERVO_HEAD_PITCH, params_.headPitchOffset);
  bb::Servos::servos.setOffset(SERVO_HEAD_HEADING, params_.headHeadingOffset);
  bb::Servos::servos.setOffset(SERVO_HEAD_ROLL, params_.headRollOffset);

  bb::Servos::servos.setRange(SERVO_NECK, 180-params_.neckRange, 180+params_.neckRange);
  bb::Servos::servos.setRange(SERVO_HEAD_PITCH, 180-params_.headPitchRange, 180+params_.headPitchRange);
  bb::Servos::servos.setRange(SERVO_HEAD_HEADING, 180-params_.headHeadingRange, 180+params_.headHeadingRange);
  bb::Servos::servos.setRange(SERVO_HEAD_ROLL, 180-params_.headRollRange, 180+params_.headRollRange);

  return RES_OK;
}

Result DODroid::sendTelemetry() {
  Telemetry telem;

  telem.overallStatus = (operationStatus_ == RES_OK) ? Telemetry::STATUS_OK : Telemetry::STATUS_ERROR;

  if(DOBattStatus::batt.available() == false) {
    telem.batteryStatus = Telemetry::STATUS_ERROR;
    telem.batteryVoltage = 0;
  } else {
    if(DOBattStatus::batt.voltage() < 3.0) telem.batteryStatus = Telemetry::STATUS_DEGRADED;
    else telem.batteryStatus = Telemetry::STATUS_OK;

    telem.batteryVoltage = DOBattStatus::batt.voltage();
    telem.batteryCurrent = DOBattStatus::batt.current();
  }

  if(leftMotorStatus_ == MOTOR_OK && rightMotorStatus_ == MOTOR_OK) {
    telem.driveStatus = Telemetry::STATUS_OK;
  } else {
    telem.driveStatus = Telemetry::STATUS_ERROR;
  }

  if(!Servos::servos.isStarted()) {
    telem.servoStatus = Telemetry::STATUS_ERROR;
  } else if(Servos::servos.hasServoWithID(SERVO_NECK)) {
    if(Servos::servos.hasServoWithID(SERVO_HEAD_HEADING) &&
       Servos::servos.hasServoWithID(SERVO_HEAD_PITCH) &&
       Servos::servos.hasServoWithID(SERVO_HEAD_ROLL)) {
      telem.servoStatus = Telemetry::STATUS_OK;
    } else {
      telem.servoStatus = Telemetry::STATUS_DEGRADED;
    }
  }

  // drive mode
  switch(driveMode_) {
  case DRIVE_VEL:
    telem.driveMode = Telemetry::DRIVE_VEL;
    break;
  case DRIVE_AUTO_POS:
    telem.driveMode = Telemetry::DRIVE_AUTO_POS;
    break;
  case DRIVE_POS:
    telem.driveMode = Telemetry::DRIVE_POS;
    break;
  case DRIVE_OFF:
  default:
    telem.driveMode = Telemetry::DRIVE_OFF;
    break;
  }

  if(imu_.available()) {
    imu_.getFilteredPRH(telem.imuPitch, telem.imuRoll, telem.imuHeading);
  } else {
    telem.imuPitch = telem.imuRoll = telem.imuHeading = 0;
  }

  // Speed in m/s
  telem.speed = ((leftEncoder_.present() + rightEncoder_.present())/2) / 1000;

  if(protocol_.sendTelemetry(telem) == true) return RES_OK;
  return RES_SUBSYS_COMM_ERROR;
}

bool DODroid::setAerials(uint8_t a1, uint8_t a2, uint8_t a3, bool update) {
  //if(aerialsOK_ == false) return false;
  headParameters_.servoSetpoints[0] = a1;
  headParameters_.servoSetpoints[1] = a2;
  headParameters_.servoSetpoints[2] = a3;
  if(update) return updateHead();
  return true;
}

bool DODroid::setEyes(uint8_t lCol, uint8_t lSize, uint8_t lPos, uint8_t rCol, uint8_t rSize, uint8_t rPos, bool update) {
  headParameters_.eyes[0] = lCol | lSize;
  headParameters_.eyes[1] = rCol | rSize;
  headParameters_.eyePos[0] = lPos;
  headParameters_.eyePos[1] = rPos;
  if(update) return updateHead();
  return true;
}

bool DODroid::setControlStrip(uint8_t strip, bool update) {
  headParameters_.control = strip | 1 << head::CONTROL_AUTOBLINK_SHIFT | 4;
  if(update) return updateHead();
  return true;
}

bool DODroid::updateHead() {
  if(!aerialsOK_) return false;

  Wire.beginTransmission(AERIAL_ADDR);
  Wire.write(0);
  Wire.write((uint8_t*)(&headParameters_), sizeof(headParameters_));
  int retval = Wire.endTransmission();
  if(retval == 0) return true;
  LOG(LOG_ERROR, "Error %d setting control strip\n", retval);
  return false;
}


bool DODroid::getAerials(uint8_t& a1, uint8_t& a2, uint8_t& a3) {
  if(aerialsOK_ == false) return false;
  int timeout;

  Wire.beginTransmission(AERIAL_ADDR);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(AERIAL_ADDR, 3);

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

Result DODroid::setLED(WhichLED which, uint8_t r, uint8_t g, uint8_t b, bool autoshow) {
  statusPixels_.setPixelColor(int(which), r, g, b);
  if(autoshow)   statusPixels_.show();
  return RES_OK;
}

Result DODroid::setLED(WhichLED which, WhatColor color, bool autoshow) {
  switch(color) {
    case RED: return setLED(which, 255, 0, 0, autoshow); break;
    case GREEN: return setLED(which, 0, 255, 0, autoshow); break;
    case BLUE: return setLED(which, 0, 0, 255, autoshow); break;
    case YELLOW: return setLED(which, 255, 255, 0, autoshow); break;
    case WHITE: return setLED(which, 255, 255, 255, autoshow); break;
    case OFF: default: return setLED(which, 0, 0, 0, autoshow); break;
  }
  return RES_COMMON_NOT_IN_LIST;
}

void DODroid::setLEDBrightness(uint8_t brightness) {
  statusPixels_.setBrightness(brightness);
  statusPixels_.show();
}