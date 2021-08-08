#if !defined(MOTORCONTROL_H)
#define MOTORCONTROL_H

#include <AccelStepper.h>

class MotorControl {
  public:
  MotorControl();

  void setDriveSpeed(float speed);

  void control();
  
  protected:
  AccelStepper drive_stepper_, spot_turn_stepper_;
};

#endif // !defined(MOTORCONTROL_H)
