#if !defined(REMOTEINPUT_H)
#define REMOTEINPUT_H

class RemoteInput {
public:
  static RemoteInput input;

  bool begin();
  void update();
  void printOnSerial();

  bool anyButtonPressed();

  float pot1, pot2;
  float battery;
  float joyH, joyV;
  bool btnPinky;
  bool btnIndex;
  bool btnJoy;
  bool btnL;
  bool btnR;
  bool btnConfirm;
  bool btnTopL;
  bool btnTopR;

  bool btnTopLChanged, btnTopRChanged, btnConfirmChanged;

  class Delegate {
  public:
    virtual void buttonTopLeftPressed() = 0;
    virtual void buttonTopRightPressed() = 0;
    virtual void buttonConfirmPressed() = 0;
    virtual void buttonTopLeftReleased() = 0;
    virtual void buttonTopRightReleased() = 0;
    virtual void buttonConfirmReleased() = 0;
  };

  void setDelegate(Delegate *d);

protected:
  RemoteInput();
  Delegate *delegate_;
};

#endif // REMOTEINPUT_H