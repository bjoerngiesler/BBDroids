#if !defined(DODRIVECONTROLLER_H)
#define DODRIVECONTROLLER_H

#include <LibBB.h>

using namespace bb;

class DOVelControlOutput: public bb::ControlOutput {
public:
  DOVelControlOutput(bb::ControlOutput& left, bb::ControlOutput& right);

  virtual void setGoalVelocity(float goalVel);
  virtual float goalVelocity() { return goalVel_; }
  virtual void setAcceleration(float accel);
  virtual void setGoalRotation(float goalRot);
  void setDeadband(float deadband);

  // These are used by the controller framework, do not call directly.
  virtual float present();
  virtual Result set(float value); // set velocity difference
  virtual float maxSpeed() { return maxSpeed_; }
  virtual void setMaxSpeed(float m) { maxSpeed_ = m; }

protected:
  float goalVel_, goalRot_, curVel_, curRot_;
  float accel_;
  float deadband_;
  bb::ControlOutput &left_, &right_;
  unsigned long lastCycleUS_;
  float maxSpeed_;
};

class DOVelControlInput: public bb::ControlInput {
public:
  DOVelControlInput(bb::ControlInput& left, bb::ControlInput& right);

  virtual float present(); // outputs velocity
  virtual Result update() { return RES_OK; } // we call update() one by one in DODroid::step()
protected:
  bb::ControlInput& left_;
  bb::ControlInput& right_;
  unsigned long lastCycleUS_;
};

class DOPosControlOutput: public bb::ControlOutput {
public:
  DOPosControlOutput(DOVelControlOutput& velOut);

  virtual float present();
  virtual Result set(float value);

protected:
  DOVelControlOutput& velOut_;
};

class DOPosControlInput: public bb::ControlInput {
public:
  DOPosControlInput(bb::Encoder& left, bb::Encoder& right);

  virtual float present(); // outputs position
  virtual Result update() { return RES_OK; }

protected:
  bb::Encoder& left_;
  bb::Encoder& right_;
};

#endif // DODRIVECONTROLLER_H