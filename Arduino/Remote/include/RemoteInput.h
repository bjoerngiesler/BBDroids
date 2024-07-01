#if !defined(REMOTEINPUT_H)
#define REMOTEINPUT_H

#include <Adafruit_MCP23X17.h>
#include "Config.h"
#include <array>

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

  // Weird order given by ESP32 layout
  enum ButtonIndex {
    BUTTON_PINKY     = 5,
    BUTTON_INDEX     = 4,
    BUTTON_JOY       = 1,
    BUTTON_LEFT      = 3,
    BUTTON_RIGHT     = 2,
    BUTTON_CONFIRM   = 7,
    BUTTON_TOP_LEFT  = 6,
    BUTTON_TOP_RIGHT = 0
  };

  std::array<bool,8> buttons;

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
#if defined(ESP32_REMOTE)
  Adafruit_MCP23X17 mcp_; bool mcpOK_;
#endif
};

#endif // REMOTEINPUT_H