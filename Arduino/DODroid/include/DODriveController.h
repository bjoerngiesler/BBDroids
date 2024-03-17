#if !defined(DODRIVECONTROLLER_H)
#define DODRIVECONTROLLER_H

#include <LibBB.h>

using namespace bb;

class DODriveControlOutput: public bb::ControlOutput {
public:
  DODriveControlOutput(bb::ControlOutput& left, bb::ControlOutput& right);

  void setGoalVelocity(float goalVel);
  void setAcceleration(float accel);
  void setGoalRotation(float goalRot);
  void setDeadband(float deadband);

  // These are used by the controller framework, do not call directly.
  virtual Result set(float value); 
  virtual float present();

protected:
  float goalVel_, goalRot_;
  float accel_;
  float deadband_;
  float presentGoalVelL_, presentGoalVelR_;
  bb::ControlOutput &left_, &right_;
  unsigned long lastCycleUS_;
};

#endif // DODRIVECONTROLLER_H