#include "MotorControl.h"
#include "Config.h"

MotorControl::MotorControl() {
  drive_stepper_ = AccelStepper(AccelStepper::DRIVER, DRIVE_STEPPER_DIR_PIN, DRIVE_STEPPER_STEP_PIN);
  drive_stepper_.setMaxSpeed(DRIVE_STEPPER_MAX_SPEED);
  drive_stepper_.setAcceleration(DRIVE_STEPPER_ACCELERATION);
  spot_turn_stepper_ = AccelStepper(AccelStepper::DRIVER, SPOT_TURN_STEPPER_DIR_PIN, SPOT_TURN_STEPPER_STEP_PIN);
}

void MotorControl::setDriveSpeed(float speed) {
  Serial.print("Set speed: "); Serial.println(speed);
  drive_stepper_.setSpeed(speed);
}

void MotorControl::control() {
  drive_stepper_.runSpeed();
}
