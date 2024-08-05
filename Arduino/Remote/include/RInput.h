#if !defined(RINPUT_H)
#define RINPUT_H

#include <Adafruit_MCP23X17.h>
#include "Config.h"
#include <array>

class RInput {
public:
  static RInput input;

  struct AxisCalib {
  public:
    AxisCalib(): min(0), center(2048), max(4096) {}
    uint16_t min, center, max;
  };

  bool begin();
  void update();
  void printOnSerial();

  void setCalibration(const AxisCalib& hCalib, const AxisCalib& vCalib) { hCalib_ = hCalib; vCalib_ = vCalib; }

  bool anyButtonPressed();

  float pot1, pot2;
  float battery;
  float joyH, joyV;
  uint16_t joyRawH, joyRawV;

  // Weird order given by ESP32 layout
#if defined(ESP32_REMOTE)
#if defined(LEFT_REMOTE)
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
#else
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
#endif
#endif

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

  void setDelegate(Delegate* d);
  void clearDelegate();

protected:
  RInput();
  Delegate* delegate_;
  AxisCalib hCalib_, vCalib_;

#if defined(ESP32_REMOTE)
  Adafruit_MCP23X17 mcp_; bool mcpOK_;
#endif
};

#endif // REMOTEINPUT_H