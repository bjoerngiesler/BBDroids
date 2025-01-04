#if !defined(DODRIVECONTROLLER_H)
#define DODRIVECONTROLLER_H

#include <LibBB.h>

using namespace bb;



class DODriveControlOutput: public bb::ControlOutput {
public:
  DODriveControlOutput(bb::ControlOutput& left, bb::ControlOutput& right);

  virtual void setGoalVelocity(float goalVel);
  virtual float goalVelocity() { return goalVel_; }
  virtual void setAcceleration(float accel);
  virtual void setGoalRotation(float goalRot);
  void setDeadband(float deadband);

  bool useControlInput() { return useControlInput_; }
  void setUseControlInput(bool yesno) { useControlInput_ = yesno; }

  // These are used by the controller framework, do not call directly.
  virtual float present();
  virtual Result set(float value); 
  virtual float maxSpeed() { return maxSpeed_; }
  virtual void setMaxSpeed(float m) { maxSpeed_ = m; }

protected:
  float goalVel_, goalRot_, curVel_, curRot_;
  float accel_;
  float deadband_;
  bb::ControlOutput &left_, &right_;
  unsigned long lastCycleUS_;
  bool useControlInput_;
  float maxSpeed_;
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