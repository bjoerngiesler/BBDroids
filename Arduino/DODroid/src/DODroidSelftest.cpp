#include "DODroid.h"
#include "DOBattStatus.h"
#include "DOSound.h"
#include "../resources/systemsounds.h"

Result DODroid::selfTest(ConsoleStream *stream) {
  Result res;

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
    // FIXME -- Temporarily disabling low-voltage checks
    //return RES_DROID_VOLTAGE_TOO_LOW;
  } else if(DOBattStatus::batt.voltage() > 17.0) {
    DOSound::sound.playSystemSound(SystemSounds::VOLTAGE_TOO_HIGH);
    return RES_DROID_VOLTAGE_TOO_HIGH;
  }
  DOSound::sound.playSystemSound(SystemSounds::OK);

  // Check Servos
  DOSound::sound.playSystemSound(SystemSounds::SERVOS);
  res = servoTest(stream);
  if(res != RES_OK) {
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    return res;
  } else {
    DOSound::sound.playSystemSound(SystemSounds::OK);
  }

  // Check Aerials
  Wire.beginTransmission(AERIAL_ADDR);
  uint8_t antErr = Wire.endTransmission();
  if(antErr != 0) {
    Console::console.printfBroadcast("Aerial error: 0x%x\n", antErr);
  } else {
    uint8_t step=1, d=5;
    uint8_t min = 30, max = 150, mid = 90;
    aerialsOK_ = true;
    uint8_t val = mid;
    for(; val < max; val+=step) {
      setAerials(val, val, val);
      delay(d);
    }
    delay(10*d);
    for(; val > min; val-=step) {
      setAerials(val, val, val);
      delay(d);
    }
    delay(10*d);
    for(; val < mid; val+=step) {
      setAerials(val, val, val);
      delay(d);
    }
    delay(10*d);
    setAerials(mid, mid, mid);
  }

  // Check IMU
  DOSound::sound.playSystemSound(SystemSounds::IMU);
  if(imu_.available() == false) {
    bb::printf("Critical error: IMU not available!\n");
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  float p, r, h;
  for(unsigned int i=0; i<100; i++) {
    imu_.update(true);
  }
  imu_.getFilteredPRH(p, r, h);
  if(fabs(p) > 5 || fabs(r) > 5) {
    bb::printf("Critical error: Droid not upright (p %f, r %f)!\n", p, r);
    DOSound::sound.playSystemSound(SystemSounds::FAILURE);
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }
  bb::printf("IMU OK. Pitch %.2f, Roll %.2f\n", p, r);

  DOSound::sound.playSystemSound(SystemSounds::CALIBRATING);
  imu_.calibrate(stream);

  for(int i=0; i<100; i++) {
    imu_.update(true);
  }
  imu_.getFilteredPRH(p, r, h);
  pitchAtRest_ = p;
  balanceController_.setGoal(-pitchAtRest_);
  bb::printf("IMU calibrated. Pitch angle at rest: %f\n", pitchAtRest_);
  DOSound::sound.playSystemSound(SystemSounds::OK);


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

  bb::Servos::servos.setOffset(SERVO_NECK, params_.neckOffset);
  bb::Servos::servos.setOffset(SERVO_HEAD_PITCH, params_.headPitchOffset);
  bb::Servos::servos.setOffset(SERVO_HEAD_HEADING, params_.headHeadingOffset);
  bb::Servos::servos.setOffset(SERVO_HEAD_ROLL, params_.headRollOffset);

  bb::Servos::servos.setRange(SERVO_NECK, 180-params_.neckRange, 180+params_.neckRange);
  bb::Servos::servos.setRange(SERVO_HEAD_PITCH, 180-params_.headPitchRange, 180+params_.headPitchRange);
  bb::Servos::servos.setRange(SERVO_HEAD_HEADING, 180-params_.headHeadingRange, 180+params_.headHeadingRange);
  bb::Servos::servos.setRange(SERVO_HEAD_ROLL, 180-params_.headRollRange, 180+params_.headRollRange);

  if(bb::Servos::servos.hasServoWithID(SERVO_NECK) == false) {
    Console::console.printfBroadcast("Critical error: Neck servo missing!\n");
    return bb::RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else if(bb::Servos::servos.home(SERVO_NECK, 5.0, 95, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing servos failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  bb::Servos::servos.setOffset(SERVO_HEAD_PITCH, params_.headPitchOffset);
  if(bb::Servos::servos.hasServoWithID(SERVO_HEAD_PITCH) == false) {
    Console::console.printfBroadcast("Degraded: Head pitch servo missing.\n");
  } else if(bb::Servos::servos.home(SERVO_HEAD_PITCH, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing servos failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    headIsOn_ = true;
  }

  if(bb::Servos::servos.hasServoWithID(SERVO_HEAD_HEADING) == false) {
    Console::console.printfBroadcast("Degraded: Head heading servo missing.\n");
  } else if(bb::Servos::servos.home(SERVO_HEAD_HEADING, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing servos failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    headIsOn_ = true;
  }

  if(bb::Servos::servos.hasServoWithID(SERVO_HEAD_ROLL) == false) {
    Console::console.printfBroadcast("Degraded: Head roll servo missing.\n");
  } else if(bb::Servos::servos.home(SERVO_HEAD_ROLL, 5.0, 50, stream) != RES_OK) {
    Console::console.printfBroadcast("Homing servos failed!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  } else {
    headIsOn_ = true;
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

  float p, r, h, h0, ax, ay, az, axmax=0, aymax=0, hdiffmax=0;
  unsigned int blockedcount = 0;
  float distance = 0;

  imu_.update();
  imu_.getFilteredPRH(p, r, h0);

  unsigned long microsPerLoop = (unsigned long)(1e6 / imu_.dataRate());

  for(pwm = ST_MIN_PWM; pwm < ST_MAX_PWM; pwm += pwmStep) {
    unsigned long us0 = micros();
    if(reverse) mot.set(-pwm);
    else mot.set(pwm);

    DOBattStatus::batt.updateCurrent();
    DOBattStatus::batt.updateVoltage();
    float mA = DOBattStatus::batt.current();
    float V = DOBattStatus::batt.voltage();
    
    imu_.update();
    imu_.getFilteredPRH(p, r, h);
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
      LOG(LOG_WARN, "Current meter reports %.1fmA/%.1fV of current, higher than threshold of %.1fmA\n", mA, V, ST_ABORT_MILLIAMPS);
      blockedcount++;
    }

    if(fabs(distance) > ST_ABORT_DISTANCE) {
      LOG(LOG_INFO, "Distance criterion triggered (fabs(%f) > %f)\n", distance, ST_ABORT_DISTANCE);
      break;
    } 
    
    if(fabs(hdiffmax) > ST_ABORT_HEADING_CHANGE) {
      LOG(LOG_INFO, "Heading criterion triggered (fabs(%f) > %f)\n", hdiffmax, ST_ABORT_HEADING_CHANGE);
      break;
    }

    if(fabs(axmax) > ST_ABORT_ACCEL) {
      LOG(LOG_INFO, "X max accel criterion triggered (fabs(%f) > %f)\n", fabs(axmax), ST_ABORT_ACCEL);
      break;
    }
    if(fabs(aymax) > ST_ABORT_ACCEL) {
      LOG(LOG_INFO, "Y max accel criterion triggered (fabs(%f) > %f)\n", fabs(aymax), ST_ABORT_ACCEL);
      break;
    }
    if(blockedcount > 10) {
      LOG(LOG_INFO, "Motor load criterion triggered %d times (%f > %f)\n", blockedcount, mA, ST_ABORT_MILLIAMPS);
      break;
    }

    unsigned long us1 = micros();
    unsigned long tdiff = us1-us0;
    if(microsPerLoop > tdiff) delayMicroseconds(microsPerLoop - tdiff);
  }

  for(; pwm>=ST_MIN_PWM; pwm -= pwmStep) {
    unsigned long us0 = micros();
    if(reverse) mot.set(-pwm);
    else mot.set(pwm);
    delayMicroseconds(microsPerLoop - (micros()-us0));
  }
  mot.set(0.0);

  // Current too high? Motor blocked.
  if(blockedcount > 5) {
    LOG(LOG_ERROR, "Motor pulling too much power. Likely blocked!\n");
    return MOTOR_BLOCKED;
  }

  // Not blocked

  // High acceleration in y? Probably IMU is turned by 90°
  if(fabs(aymax) > ST_MIN_ACCEL && (aymax < -fabs(axmax) || aymax > fabs(axmax))) {
    LOG(LOG_ERROR, "Accel in Y direction %f higher than in x %f. IMU likely rotated 90°!\n", aymax, axmax);
    return MOTOR_OTHER;
  }

  // Not blocked, and acceleration is along the correct axis

  // Not enough distance returned from the encoder...
  if(fabs(distance) < ST_MIN_DISTANCE) {
    // ...but measured enough acceleration? Motor is connected, encoder likely isn't.
    if(fabs(axmax) > ST_MIN_ACCEL) {
      LOG(LOG_ERROR, "Distance of %f (<%f) too low, but accel of %f measured. Encoder likely disconnected!\n",
          distance, ST_MIN_DISTANCE, axmax);
      return MOTOR_ENC_DISCONNECTED;
    } else { // ...and not measured enough acceleration? Motor is likely not connected.
      LOG(LOG_ERROR, "Distance of %f (<%f) and accel of %f (<%f) both too low. Motor likely disconnected!\n",
          distance, ST_MIN_DISTANCE, axmax, ST_MIN_ACCEL);
      return MOTOR_DISCONNECTED;
    }
  }

  // Not blocked, accel axis is OK, enough distance driven

  Console::console.printfBroadcast("Max hdiff: %f\n", hdiffmax);

  // Not enough heading change observed? We're likely sitting in the station.
  if(fabs(hdiffmax) < ST_MIN_HEADING_CHANGE) {
    Console::console.printfBroadcast("Heading change %f too small, we're likely in the station, can't distinguish if encoder and motor are both reversed!\n",
                                     hdiffmax);
    if((reverse && distance >= 0) ||
       (!reverse && distance <= 0)) {
      Console::console.printfBroadcast("%s, motor and encoder direction disagree. Encoder or motor likely reversed.\n",
                                        reverse ? "Reverse" : "Forward");
      return MOTOR_ENC_REVERSED;
    }
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
