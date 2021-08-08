#include "StatePacket.h"

class RemoteState {
  public:
  RemoteState(); // uses the RemoteState* pins defined in Config.h

  void update();

  void printOnSerial(); 

  bool isTopLeftButtonPressed() { return button_top_left_; }
  bool isTopRightButtonPressed() { return button_top_right_; }
  bool isRightPinkyButtonPressed() { return button_right_pinky_; }
  bool isRightIndexButtonPressed() { return button_right_index_; }
  bool isTopPCBLeftButtonPressed() { return button_top_pcb_left_; }
  bool isTopPCBRightButtonPressed() { return button_top_pcb_right_; }
  bool isJoystickButtonPressed() { return button_joystick_; }
  short getJoystickHorizontalAxis() { return joystick_horizontal_; }
  short getJoystickVerticalAxis() { return joystick_vertical_; }

  void fillStatePacket(StatePacket& packet);

  protected:
  bool button_top_left_;
  bool button_top_right_;
  bool button_right_pinky_;
  bool button_right_index_;
  bool button_top_pcb_left_;
  bool button_top_pcb_right_;

  bool button_joystick_;
  short joystick_horizontal_, joystick_vertical_;    
};
