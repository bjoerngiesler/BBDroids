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

  float pot1, pot2; // range: 0 .. 1.0
  float battery;    // range: 0 .. 1.0
  float joyH, joyV; // range: -1.0 .. 1.0
  uint16_t joyRawH, joyRawV, battRaw;

  // Weird order given by ESP32 layout
#if defined(ESP32_REMOTE)
#if defined(LEFT_REMOTE)
  enum ButtonIndex {
    BUTTON_PINKY     = 2, // correct
    BUTTON_INDEX     = 4, // correct
    BUTTON_JOY       = 5, // correct
    BUTTON_LEFT      = 7, // correct
    BUTTON_RIGHT     = 6, // correct
    BUTTON_CONFIRM   = 3, // correct
    BUTTON_TOP_LEFT  = 1, // correct
    BUTTON_TOP_RIGHT = 0  // correct
  };
#else
  enum ButtonIndex {
    BUTTON_PINKY     = 5,
    BUTTON_INDEX     = 4,
    BUTTON_JOY       = 1,
    BUTTON_LEFT      = 2,
    BUTTON_RIGHT     = 3,
    BUTTON_CONFIRM   = 7,
    BUTTON_TOP_LEFT  = 6,
    BUTTON_TOP_RIGHT = 0
  };
#endif

  bool initMCP();
#endif

  std::array<bool,8> buttons;

  bool btnTopLChanged, btnTopRChanged, btnConfirmChanged;
  
  void setTopLeftShortPressCallback(std::function<void(void)> cb) { tlShortPressCB_ = cb; }
  void setTopLeftLongPressCallback(std::function<void(void)> cb) { tlLongPressCB_ = cb; }
  void setTopRightShortPressCallback(std::function<void(void)> cb) { trShortPressCB_ = cb; }
  void setTopRightLongPressCallback(std::function<void(void)> cb) { trLongPressCB_ = cb; }
  void setConfirmShortPressCallback(std::function<void(void)> cb) { cShortPressCB_ = cb; }
  void setConfirmLongPressCallback(std::function<void(void)> cb) { cLongPressCB_ = cb; }
  void clearCallbacks();

protected:
  RInput();
  AxisCalib hCalib_, vCalib_;

#if defined(ESP32_REMOTE)
  Adafruit_MCP23X17 mcp_; bool mcpOK_;
#endif

  void btnTopLeftPressed();
  void btnTopLeftReleased();
  void btnTopRightPressed();
  void btnTopRightReleased();
  void btnConfirmPressed();
  void btnConfirmReleased();

  unsigned long tlms_, trms_, cms_;
  std::function<void(void)> tlShortPressCB_, tlLongPressCB_, trShortPressCB_, trLongPressCB_, cShortPressCB_, cLongPressCB_;
  unsigned long longPressThresh_;
};

#endif // REMOTEINPUT_H