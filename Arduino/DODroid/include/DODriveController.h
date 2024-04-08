#if !defined(DODRIVECONTROLLER_H)
#define DODRIVECONTROLLER_H

#include <LibBB.h>

using namespace bb;



class DODriveControlOutput: public bb::ControlOutput {
public:
  DODriveControlOutput(bb::ControlOutput& left, bb::ControlOutput& right);

  virtual void setGoalVelocity(float goalVel);
  virtual void setAcceleration(float accel);
  virtual void setGoalRotation(float goalRot);
  void setDeadband(float deadband);

  // These are used by the controller framework, do not call directly.
  virtual float present();
  virtual Result set(float value); 

protected:
  float goalVel_, goalRot_;
  float accel_;
  float deadband_;
  float presentGoalVelL_, presentGoalVelR_;
  bb::ControlOutput &left_, &right_;
  unsigned long lastCycleUS_;
};

class DODriveControlInput: public bb::ControlInput {
public:
  DODriveControlInput(bb::ControlInput& left, bb::ControlInput& right);

  virtual Result update();
  virtual float present();
protected:
  bb::ControlInput &left_, &right_;
  unsigned long lastCycleUS_;
};

#endif // DODRIVECONTROLLER_H