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
    virtual void buttonTopLeftPressed() {}
    virtual void buttonTopRightPressed() {}
    virtual void buttonConfirmPressed() {}
    virtual void buttonTopLeftReleased() {}
    virtual void buttonTopRightReleased() {}
    virtual void buttonConfirmReleased() {}
  };

  void setDelegate(Delegate *d);

protected:
  RemoteInput();
  Delegate *delegate_;
  uint16_t zeroVertical_, zeroHorizontal_;
};

#endif // REMOTEINPUT_H