#include "DODroid.h"
#include "DOConfig.h"
#include "DOBattStatus.h"
#include "DOSound.h"
#include "../resources/systemsounds.h"

DODroid DODroid::droid;
DOParams DODroid::params_;
bb::ConfigStorage::HANDLE DODroid::paramsHandle_;

static uint16_t pwmPeriod;

static void configureTimers() {
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(4);
  while(GCLK->STATUS.bit.SYNCBUSY); 

  // Set the clock source, duty cycle, and enable GCLK5  
  REG_GCLK_GENCTRL = GCLK_GENCTRL_SRC_DFLL48M |  // Set 48MHz source
                     GCLK_GENCTRL_IDC |          // Improve Duty Cycle
                     GCLK_GENCTRL_GENEN |        // Enable GCLK
                     GCLK_GENCTRL_ID(4);         // For GLCK5    
  while(GCLK->STATUS.bit.SYNCBUSY);    

  // Route GLCK5 to TCC0 & TCC1, and enable the clock.
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_ID_TCC0_TCC1 | // Route GCLK5 to TCC0 & 1
                     GCLK_CLKCTRL_CLKEN |        // Enable the clock
                     GCLK_CLKCTRL_GEN_GCLK4;     // Select GCLK5
  while(GCLK->STATUS.bit.SYNCBUSY);  

  // Route to multiplexer...
  PORT->Group[g_APinDescription[P_LEFT_PWMA].ulPort].PINCFG[g_APinDescription[P_LEFT_PWMA].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[P_LEFT_PWMB].ulPort].PINCFG[g_APinDescription[P_LEFT_PWMB].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[P_RIGHT_PWMA].ulPort].PINCFG[g_APinDescription[P_RIGHT_PWMA].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[P_RIGHT_PWMB].ulPort].PINCFG[g_APinDescription[P_RIGHT_PWMB].ulPin].bit.PMUXEN = 1;
  // ...and in the MUX to peripheral function F. Here comes the messy bit (data sheet page 30, table 7.1)
  // LEFT_PWMA is Pin 18 -> PA04, which gets TCC0/WO[0] in port function E
  PORT->Group[g_APinDescription[P_LEFT_PWMA].ulPort].PMUX[g_APinDescription[P_LEFT_PWMA].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E;
  // LEFT_PWMB is Pin 19 -> PA05, which gets TCC0/WO[1] in port function E
  PORT->Group[g_APinDescription[P_LEFT_PWMB].ulPort].PMUX[g_APinDescription[P_LEFT_PWMB].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
  // RIGHT_PWMA is Pin 3 -> PA11, which gets TCC0/WO[3] in port function F
  PORT->Group[g_APinDescription[P_RIGHT_PWMA].ulPort].PMUX[g_APinDescription[P_RIGHT_PWMA].ulPin >> 1].reg |= PORT_PMUX_PMUXE_F;
  // RIGHT_PWMB is Pin 2 -> PA10, which gets TCC0/WO[2] in port function F
  PORT->Group[g_APinDescription[P_RIGHT_PWMB].ulPort].PMUX[g_APinDescription[P_RIGHT_PWMB].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;  

  double fPWM = 20000;
  double fBus = 48000000;
  pwmPeriod = int(fBus / fPWM)-1;

  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  REG_TCC0_PER = pwmPeriod;
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;     // Requires SYNC on CTRLA
  while(TCC0->SYNCBUSY.bit.ENABLE || TCC0->SYNCBUSY.bit.WAVE || TCC0->SYNCBUSY.bit.PER);
}

static void customAnalogWrite(uint8_t pin, uint8_t dutycycle) {
  uint16_t dc = map(dutycycle, 0, 255, 0, pwmPeriod);

  switch(pin) {
  case P_LEFT_PWMA: // WO[0] -> CCB0
    REG_TCC0_CCB0 = dc;
    while(TCC0->SYNCBUSY.bit.CCB0);
    break;
  case P_LEFT_PWMB: // WO[1] -> CCB1
    REG_TCC0_CCB1 = dc;
    while(TCC0->SYNCBUSY.bit.CCB1);
    break;
  case P_RIGHT_PWMA: // WO[3] -> CCB3
    REG_TCC0_CCB3 = dc;
    while(TCC0->SYNCBUSY.bit.CCB3);
    break;
  case P_RIGHT_PWMB: // WO[2] -> CCB2
    REG_TCC0_CCB2 = dc;
    while(TCC0->SYNCBUSY.bit.CCB2);
    break;
  default:
    bb::printf("ERROR - Unknown pin %d in customAnalogWrite()\n", pin);
  }
}

DODroid::DODroid():
  imu_(IMU_ADDR),

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
  posController_(posInput_, posOutput_),

  statusPixels_(3, P_STATUS_NEOPIXEL, NEO_GRB+NEO_KHZ800)
{
  // Pull down the ENABLE pins for the motor controllers. FIXME I *think* this isn't needed for D-Ov2Evo, test!
  pinMode(PULL_DOWN_15, OUTPUT);
  digitalWrite(PULL_DOWN_15, LOW);
  pinMode(PULL_DOWN_20, OUTPUT);
  digitalWrite(PULL_DOWN_20, LOW);

  configureTimers();
  leftMotor_.setCustomAnalogWrite(&customAnalogWrite);
  rightMotor_.setCustomAnalogWrite(&customAnalogWrite);

  statusPixels_.begin();
  statusPixels_.setBrightness(10);
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

Result DODroid::initialize() {
  addParameter("neck_range", "Neck servo movement range", params_.neckRange, -INT_MAX, INT_MAX);
  addParameter("neck_offset", "Neck servo offset", params_.neckOffset, -INT_MAX, INT_MAX);
  addParameter("head_roll_range", "Head roll servo movement range", params_.headRollRange, -INT_MAX, INT_MAX);
  addParameter("head_roll_offset", "Head roll servo servo offset", params_.headRollOffset, -INT_MAX, INT_MAX);
  addParameter("head_pitch_range", "Head pitch servo movement range", params_.headPitchRange, -INT_MAX, INT_MAX);
  addParameter("head_pitch_offset", "Head pitch servo servo offset", params_.headPitchOffset, -INT_MAX, INT_MAX);
  addParameter("head_heading_range", "Head heading servo movement range", params_.headHeadingRange, -INT_MAX, INT_MAX);
  addParameter("head_heading_offset", "Head heading servo servo offset", params_.headHeadingOffset, -INT_MAX, INT_MAX);

  addParameter("mot_deadband", "Deadband for drive motors", params_.motorDeadband, 0, 255);

  addParameter("wheel_kp", "Proportional constant for wheel speed PID controller", params_.wheelKp, -INT_MAX, INT_MAX);
  addParameter("wheel_ki", "Integrative constant for wheel speed PID controller", params_.wheelKi, -INT_MAX, INT_MAX);
  addParameter("wheel_kd", "Derivative constant for wheel speed PID controller", params_.wheelKd, -INT_MAX, INT_MAX);

  addParameter("bal_kp", "Proportional constant for balance PID controller", params_.balKp, -INT_MAX, INT_MAX);
  addParameter("bal_ki", "Integrative constant for balance PID controller", params_.balKi, -INT_MAX, INT_MAX);
  addParameter("bal_kd", "Derivative constant for balance PID controller", params_.balKd, -INT_MAX, INT_MAX);

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

  setPacketSource(PACKET_SOURCE_DROID);

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

  imu_.begin();
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
    return operationStatus_;
  }

  setControlParameters();

  annealH_ = annealP_ = annealR_ = 0;

  started_ = true;
  operationStatus_ = RES_OK;
  setLED(LED_STATUS, GREEN);
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

  posController_.setControlParameters(params_.posKp, params_.posKi, params_.posKd);
}

Result DODroid::step() {
  unsigned long seqnum = Runloop::runloop.getSequenceNumber();
  
  // We're broken; still send out the state packet.
  if(!imu_.available() || !DOBattStatus::batt.available()) {
    if((seqnum % 4) == 0) {
      fillAndSendStatePacket();
    }
    LOG(LOG_FATAL, "IMU or battery missing - critical error\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  // Look for battery undervoltage
  if((Runloop::runloop.getSequenceNumber() % 1000) == 0) {
    stepPowerProtect();
  }

  // Check if SD card was changed
  if((seqnum % 1000) == 0 && driveMode_ == DRIVE_OFF) {
    Runloop::runloop.excuseOverrun();
    DOSound::sound.checkSDCard();
  }

  // Encoder and IMU updates are needed for everything, so we do them here.
  leftEncoder_.update();  
  rightEncoder_.update();  
  imu_.update();

  // We only run head and drive control at 50Hz
  if((seqnum % 2) == 0) {
    stepHead();
  } else {
    stepDrive();
  }

  if((seqnum % 4) == 0) {
    fillAndSendStatePacket();
    if(XBee::xbee.isStarted() && Servos::servos.isStarted()) setLED(LED_STATUS, GREEN);
    else setLED(LED_STATUS, YELLOW);
  }

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
      int i=0;
      if(i%2 == 0) setLED(LED_STATUS, RED);
      else setLED(LED_STATUS, OFF);
      if(i%5 == 0) DOSound::sound.playSystemSound(SystemSounds::VOLTAGE_TOO_LOW);
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

  //Console::console.printfBroadcast("R:%f P:%f H:%f AX:%f AY:%f AZ:%f RAX:%f RAY: %f RAZ: %f DR:%f DP:%f DH:%f\n", r, p, h, ax, ay, az, rax, ray, raz, dr, dp, dh);

  float speed = (leftEncoder_.presentSpeed() + rightEncoder_.presentSpeed())/2;
  float speedSP = velOutput_.goalVelocity();
  float accelSP = (speedSP-speed);
  if(speedSP == 0) accelSP = 0;

  if(servosOK_) {
    if(headIsOn_) {
      float nod;
      if(HEAD_COUNTERWEIGHT == false) {
        nod = params_.faNeckIMUAccel*ax + params_.faNeckSPAccel*accelSP + params_.faNeckSpeed*speed;
        if(speedSP < 0) nod += params_.faNeckSpeedSP*speedSP/2;
        else nod += params_.faNeckSpeedSP*speedSP;
      } else {
        nod = params_.faNeckIMUPitch*p + params_.faNeckIMUAccel*ax + params_.faNeckSPAccel*accelSP + params_.faNeckSpeed*speed;
        if(speedSP < 0) nod += params_.faNeckSpeedSP*speedSP/2;
        else nod += params_.faNeckSpeedSP*speedSP;

      }

      nod += lean_;
      nod = constrain(nod, -params_.neckRange, params_.neckRange);
      bb::Servos::servos.setGoal(SERVO_NECK, 180 + nod);

      float headPitch = -nod;
      //headPitch -= params_.leanHeadToBody*lean_;
      headPitch += remoteP_;
      headPitch += params_.faHeadPitchSpeedSP*speedSP;
      bb::Servos::servos.setGoal(SERVO_HEAD_PITCH, 180 + headPitch);
      bb::Servos::servos.setGoal(SERVO_HEAD_HEADING, 180.0 + params_.faHeadHeadingTurn * dh + remoteH_);
      bb::Servos::servos.setGoal(SERVO_HEAD_ROLL, 180.0 - params_.faHeadRollTurn * dh + remoteR_);
    } else {
      bb::Servos::servos.setGoal(SERVO_NECK, 180);
    }
  }
  
  if(aerialsOK_) {
    float aer = 90 + params_.aerialOffset + speedSP*params_.faAerialSpeedSP;

    aer = constrain(aer, 0, 180);
    //LOG(LOG_INFO, "Setting aerials to %f %f %f\n", aer+remoteAerial1_, aer+remoteAerial2_, aer+remoteAerial3_);
    setAerials(constrain(aer + remoteAerial1_, 0, 180), 
               constrain(aer + remoteAerial2_, 0, 180),
               constrain(aer + remoteAerial3_, 0, 180));
  }

  if(lastPrimaryCtrlPacket_.button3 == false && (float(millis())/1000.0f > annealTime_ + params_.faHeadAnnealDelay)) {
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

bb::Result DODroid::stepDrive() {
  unsigned long timeSinceLastPrimary = WRAPPEDDIFF(millis(), msLastPrimaryCtrlPacket_, ULONG_MAX);
  if(timeSinceLastPrimary > 500 && driveMode_ != DRIVE_OFF && driveSafety_ == true) {
    Console::console.printfBroadcast("No control packet from primary in %dms. Switching drive off.\n", timeSinceLastPrimary);
    switchDrive(DRIVE_OFF);
    DOSound::sound.playSystemSound(SystemSounds::DISCONNECTED);
  }

  if(driveMode_ != DRIVE_OFF && leftMotorStatus_ == MOTOR_OK && rightMotorStatus_ == MOTOR_OK) {
    balanceController_.update();
    lSpeedController_.update();
    rSpeedController_.update();
    if(driveMode_ == DRIVE_POSITION) posController_.update();
  } else {
    leftMotor_.set(0);
    rightMotor_.set(0);
  }
  return RES_OK;
}

void DODroid::switchDrive(DriveMode mode) {
  lSpeedController_.reset();
  lSpeedController_.setGoal(0);
  rSpeedController_.reset();
  rSpeedController_.setGoal(0);
  balanceController_.reset();
  balanceController_.setGoal(-pitchAtRest_);
  posController_.reset();
  posController_.setPresentAsGoal();

  driveMode_ = mode;
  Console::console.printfBroadcast("Drive mode: ");
  switch(driveMode_) {
    case DRIVE_OFF: 
      setLED(LED_DRIVE, OFF);
      Console::console.printfBroadcast("off\n"); 
      break;
    
    case DRIVE_VELOCITY: 
      setLED(LED_DRIVE, GREEN);
      Console::console.printfBroadcast("velocity\n"); 
      break;

    case DRIVE_POSITION: default: 
      setLED(LED_DRIVE, BLUE);
      Console::console.printfBroadcast("position\n"); 
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
  stream->printf("Started: %s, Operation status: %s\n", started_?"yes":"no", errorMessage(operationStatus_));

  stream->printf("Control packets received:\n");
  stream->printf("\tFrom left remote: %d\n", numLeftCtrlPackets_);
  stream->printf("\tFrom right remote: %d\n", numRightCtrlPackets_);

  stream->printf("Motors:\n");
  leftEncoder_.update();
  if(leftMotorStatus_ == MOTOR_OK) stream->printf("\tLeft OK, encoder %f\n", leftEncoder_.presentPosition());
  else stream->printf("\tLeft Err %d, encoder %f\n", leftMotorStatus_, leftEncoder_.presentPosition());
  rightEncoder_.update();
  if(rightMotorStatus_ == MOTOR_OK) stream->printf("\tRight OK, encoder %f\n", rightEncoder_.presentPosition());
  else stream->printf("\tRight Err %d, encoder %f\n", rightMotorStatus_, rightEncoder_.presentPosition());

  float p, r, h;
  imu_.getFilteredPRH(p, r, h);
  stream->printf("IMU: P%.2f R%.2f H%.2f\n", p, r, h);
  DOBattStatus::batt.updateCurrent();
  DOBattStatus::batt.updateVoltage();
  stream->printf("Battery: %fmA, %fV\n", DOBattStatus::batt.current(), DOBattStatus::batt.voltage());
}

Result DODroid::incomingConfigPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, ConfigPacket& packet) {
  if(source == PACKET_SOURCE_LEFT_REMOTE && packet.type == ConfigPacket::CONFIG_SET_LEFT_REMOTE_ID) {
    Console::console.printfBroadcast("setting left remote id to 0x%lx:%lx.\n", packet.cfgPayload.address.addrHi, packet.cfgPayload.address.addrLo);
    params_.leftRemoteAddress = packet.cfgPayload.address;
    ConfigStorage::storage.writeBlock(paramsHandle_);
    ConfigStorage::storage.commit();
    return RES_OK;
  }
  Console::console.printfBroadcast("Error - packet of type %d from source %d\n", packet.type, source);
  return RES_SUBSYS_COMM_ERROR;
}

Result DODroid::incomingControlPacket(const HWAddress& srcAddr, PacketSource source, uint8_t rssi, uint8_t seqnum, const ControlPacket& packet) {
  if(commLEDOn_ == false) {
    unsigned long ms = millis();
    if(ms - msLastLeftCtrlPacket_ < 100) {
      if(ms - msLastRightCtrlPacket_ < 100) {
        setLED(LED_COMM, WHITE);
      } else {
        setLED(LED_COMM, BLUE);
      }
    } else setLED(LED_COMM, GREEN);
  
    commLEDOn_ = true;
    Runloop::runloop.scheduleTimedCallback(100, [=]{ setLED(LED_COMM, OFF); commLEDOn_ = false; });
  }

  // Check for duplicates and lost packets
  if(source == PACKET_SOURCE_LEFT_REMOTE) {
    if(seqnum == lastLeftSeqnum_) return RES_OK; // duplicate
    if(WRAPPEDDIFF(seqnum, lastLeftSeqnum_, 8) != 1) {
      Console::console.printfBroadcast("Left control packet: seqnum %d, last %d, lost %d packets!\n", 
                                       seqnum, lastLeftSeqnum_, WRAPPEDDIFF(seqnum, lastLeftSeqnum_, 8)-1);
    }
    lastLeftSeqnum_ = seqnum;
    numLeftCtrlPackets_++;
    msLastLeftCtrlPacket_ = millis();
  } else if(source == PACKET_SOURCE_RIGHT_REMOTE) {
    if(seqnum == lastRightSeqnum_) return RES_OK; // duplicate
    if(WRAPPEDDIFF(seqnum, lastRightSeqnum_, 8) != 1) {
      Console::console.printfBroadcast("Right control packet: seqnum %d, last %d, lost %d packets!\n", 
                                       seqnum, lastRightSeqnum_, WRAPPEDDIFF(seqnum, lastRightSeqnum_, 8)-1);
    }
    lastRightSeqnum_ = seqnum;
    numRightCtrlPackets_++;
    msLastRightCtrlPacket_ = millis();
  } else {
    Console::console.printfBroadcast("Control packet from unknown source %d\n", source);
    return RES_SUBSYS_COMM_ERROR;
  }

  // Hardcoded axis / trigger mapping starts here
  if(packet.primary == true) {
    msLastPrimaryCtrlPacket_ = millis();
    
    // Joystick button switches drive mode
    if(packet.button4 && !lastPrimaryCtrlPacket_.button4) {
      if(driveMode_ == DRIVE_OFF) {
        if(params_.autoPosControl == true) {
          switchDrive(DRIVE_POSITION);
        } else {
          switchDrive(DRIVE_VELOCITY);
        }
      } else {
        switchDrive(DRIVE_OFF);
      }
    }

    // If button 3 is held, axes 2, 3, 4 (IMU rot) control the head
    if(packet.button3) {
      remoteR_ = packet.getAxis(2, ControlPacket::UNIT_DEGREES_CENTERED);
      remoteP_ = packet.getAxis(3, ControlPacket::UNIT_DEGREES_CENTERED);
      remoteH_ = packet.getAxis(4, ControlPacket::UNIT_DEGREES_CENTERED);
      annealR_ = fabs(remoteR_ / (params_.faHeadAnnealTime / bb::Runloop::runloop.cycleTimeSeconds()));
      annealP_ = fabs(remoteP_ / (params_.faHeadAnnealTime / bb::Runloop::runloop.cycleTimeSeconds()));
      annealH_ = fabs(remoteH_ / (params_.faHeadAnnealTime / bb::Runloop::runloop.cycleTimeSeconds()));
    }

    // Button 0, 1, 2 play different sounds
    if(packet.button0 && !lastPrimaryCtrlPacket_.button0) DOSound::sound.playFolderNext(2);
    else if(packet.button1 && !lastPrimaryCtrlPacket_.button1) DOSound::sound.playFolderNext(3);
    else if(packet.button2 && !lastPrimaryCtrlPacket_.button2) DOSound::sound.playFolderNext(4);

    // If drive system is on, joystick controls rotation and speed
    if(driveMode_ != DRIVE_OFF) {
      float rot = packet.getAxis(0);
      float vel = packet.getAxis(1);

      // Joystick at zero for >500ms? Switch to position control mode
      if(EPSILON(vel) && EPSILON(rot) && params_.autoPosControl == true) { 
        if(WRAPPEDDIFF(millis(), msSinceDriveInput_, ULONG_MAX) > 500 && driveMode_ == DRIVE_VELOCITY) {
          switchDrive(DRIVE_POSITION);
        }
      } else { // We have drive input - switch to velocity control modex
        if(driveMode_ == DRIVE_POSITION) {
          switchDrive(DRIVE_VELOCITY);
        }
        msSinceDriveInput_ = millis();
      }

      if(driveMode_ == DRIVE_VELOCITY) {
        vel = constrain(vel * params_.maxSpeed * params_.speedAxisGain, -params_.maxSpeed, params_.maxSpeed);
        rot = constrain(rot * params_.maxSpeed * params_.rotAxisGain, -params_.maxSpeed, params_.maxSpeed);
        velOutput_.setGoalVelocity(vel);
        velOutput_.setGoalRotation(rot);
      }

    } else {
      setLED(LED_DRIVE, OFF);
    }

    DOSound::sound.setVolume(int(30.0 * packet.getAxis(8)));
    lastPrimaryCtrlPacket_ = packet;
  } else { // secondary remote
    lean_ = leanFilter_.filter(-1 * packet.getAxis(1, bb::ControlPacket::UNIT_UNITY_CENTERED) * params_.neckRange);

    if(headIsOn_) {
      balanceController_.setGoal(params_.leanHeadToBody*lean_-pitchAtRest_);
    }

    if(packet.button0 && !lastSecondaryCtrlPacket_.button0) DOSound::sound.playFolderNext(5);
    else if(packet.button1 && !lastSecondaryCtrlPacket_.button1) DOSound::sound.playFolderNext(6);
    else if(packet.button2 && !lastSecondaryCtrlPacket_.button2) DOSound::sound.playFolderNext(7);

    lastSecondaryCtrlPacket_ = packet;
  }

  if(source == PACKET_SOURCE_RIGHT_REMOTE) {
    if(packet.button5) remoteAerial3_ = params_.aerialAnim;
    else remoteAerial3_ = 0;
    if(packet.button6) remoteAerial2_ = params_.aerialAnim;
    else remoteAerial2_ = 0;
    if(packet.button7) remoteAerial1_ = params_.aerialAnim;
    else remoteAerial1_ = 0;
  }

  return RES_OK;
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
      switchDrive(DRIVE_POSITION);
      return RES_OK;
    } else if(words[1] == "vel") {
      switchDrive(DRIVE_VELOCITY);
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

  return RES_OK;
}

Result DODroid::fillAndSendStatePacket() {
  if(params_.leftRemoteAddress.isZero()) return RES_OK;

  Packet packet(PACKET_TYPE_STATE, PACKET_SOURCE_DROID, sequenceNumber());
  packet.payload.state.battery = DOBattStatus::batt.available() ? StatePacket::STATUS_OK : StatePacket::STATUS_ERROR;
  XBee::xbee.sendTo(params_.leftRemoteAddress, packet, false);
  return RES_OK;

  LargeStatePacket p;
  memset(&p, 0, sizeof(LargeStatePacket));

  if((Runloop::runloop.getSequenceNumber() % 5) != 0) return RES_OK;

  p.timestamp = Runloop::runloop.millisSinceStart() / 1000.0;
  p.droidType = DroidType::DROID_DO;

  strncpy(p.droidName, DROID_NAME, sizeof(p.droidName));

  float err, errI, errD, control;

  if(leftMotorStatus_ == MOTOR_OK && rightMotorStatus_ == MOTOR_OK) {
    p.drive[0].errorState = ERROR_OK;   
    p.drive[0].controlMode = (driveMode_ != DRIVE_OFF) ? bb::StatePacket::CONTROL_RC : bb::StatePacket::CONTROL_OFF;

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
    p.drive[1].controlMode = (driveMode_ != DRIVE_OFF) ? bb::StatePacket::CONTROL_RC : bb::StatePacket::CONTROL_OFF;
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
    p.drive[2].controlMode = (driveMode_ != DRIVE_OFF) ? bb::StatePacket::CONTROL_RC : bb::StatePacket::CONTROL_OFF;
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


bool DODroid::setAerials(uint8_t a1, uint8_t a2, uint8_t a3) {
  //if(aerialsOK_ == false) return false;

  int retval;
  uint8_t aerials[3] = {a1, a2, a3};
  Wire.beginTransmission(AERIAL_ADDR);
  Wire.write(aerials, sizeof(aerials));
  retval = Wire.endTransmission();
  if(retval == 0) return true;
  LOG(LOG_ERROR, "Error %d moving aerials\n", retval);
  return false;
}

bool DODroid::getAerials(uint8_t& a1, uint8_t& a2, uint8_t& a3) {
  if(aerialsOK_ == false) return false;
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

Result DODroid::setLED(WhichLED which, uint8_t r, uint8_t g, uint8_t b) {
  statusPixels_.setPixelColor(int(which), r, g, b);
  statusPixels_.show();
  return RES_OK;
}

Result DODroid::setLED(WhichLED which, WhatColor color) {
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

void DODroid::setLEDBrightness(uint8_t brightness) {
  statusPixels_.setBrightness(brightness);
  statusPixels_.show();
}