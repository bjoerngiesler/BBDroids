#include "StatePacket.h"

class RemoteState {
  public:
  static RemoteState* getSharedInstance();
  void update();

  void printOnSerial(); 

  bool isTopLeftButtonPressed();
  bool isTopRightButtonPressed();
  bool isRightPinkyButtonPressed();
  bool isRightIndexButtonPressed();
  bool isTopPCBLeftButtonPressed();
  bool isTopPCBRightButtonPressed();
  bool isJoystickButtonPressed();
  float getJoystickHorizontalAxis();
  float getJoystickVerticalAxis();

  void fillStatePacket(StatePacket& packet);

protected:
  RemoteState(); // uses the RemoteState* pins defined in Config.h
};
