#if !defined(RINPUT_H)
#define RINPUT_H

#include <LibBB.h>
#include <Adafruit_MCP23X17.h>
#include "Config.h"
#include <array>

using namespace bb;

class RInput {
public:
  static RInput input;

  // Weird order given by ESP32 layout
#if defined(ARDUINO_ARCH_ESP32)
#if defined(LEFT_REMOTE)
  enum ButtonIndex {
    BUTTON_PINKY     = 2,
    BUTTON_INDEX     = 4,
    BUTTON_JOY       = 5,
    BUTTON_LEFT      = 7,
    BUTTON_RIGHT     = 6,
    BUTTON_CONFIRM   = 3,
    BUTTON_TOP_LEFT  = 1,
    BUTTON_TOP_RIGHT = 0,
    BUTTON_NONE      = 255
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
    BUTTON_TOP_RIGHT = 0,
    BUTTON_NONE      = 255
  };
#endif

  bool initMCP();
#else
  enum ButtonIndex {
    BUTTON_PINKY     = 0,
    BUTTON_INDEX     = 1,
    BUTTON_JOY       = 2,
    BUTTON_LEFT      = 3,
    BUTTON_RIGHT     = 4,
    BUTTON_CONFIRM   = 5,
    BUTTON_TOP_LEFT  = 6,
    BUTTON_TOP_RIGHT = 7,
    BUTTON_NONE      = 255
  };
#endif
  struct AxisCalib {
  public:
    AxisCalib(): min(0), max(4096), center(2048) {}
    uint16_t min, max, center;
  };

  bool begin();
  void update();
  void printOnSerial();
  bool isOK();
  Result fillControlPacket(ControlPacket& packet);
  bb::IMU& imu() { return imu_; }

  void setCalibration(const AxisCalib& hc, const AxisCalib& vc) { hCalib = hc; vCalib = vc; }
  void setIncrementalPos(ButtonIndex btn);
  void resetIncrementalPos();
  void setIncrementalRot(ButtonIndex btn);

  bool anyButtonPressed();

  void testMatrix();

  float pot1, pot2; // range: 0 .. 1.0
  float battery;    // range: 0 .. 1.0
  float joyH, joyV; // range: -1.0 .. 1.0
  uint16_t joyRawH, joyRawV, battRaw;
  uint16_t minJoyRawH, maxJoyRawH, minJoyRawV, maxJoyRawV;
  AxisCalib hCalib, vCalib;

  std::array<bool,8> buttons, buttonsChanged;

  bool btnTopLChanged, btnTopRChanged, btnConfirmChanged;
  
  void setTopLeftShortPressCallback(std::function<void(void)> cb) { tlShortPressCB_ = cb; }
  void setTopLeftLongPressCallback(std::function<void(void)> cb) { tlLongPressCB_ = cb; }
  void setTopRightShortPressCallback(std::function<void(void)> cb) { trShortPressCB_ = cb; }
  void setTopRightLongPressCallback(std::function<void(void)> cb) { trLongPressCB_ = cb; }
  void setConfirmShortPressCallback(std::function<void(void)> cb) { cShortPressCB_ = cb; }
  void setConfirmLongPressCallback(std::function<void(void)> cb) { cLongPressCB_ = cb; }
  void setAllCallbacks(std::function<void(void)> cb) {
    tlShortPressCB_ = cb;
    tlLongPressCB_ = cb;
    trShortPressCB_ = cb;
    trLongPressCB_ = cb;
    cShortPressCB_ = cb;
    cLongPressCB_ = cb;
  }
  void clearCallbacks();

protected:
  RInput();

#if defined(ARDUINO_ARCH_ESP32)
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
  ButtonIndex incrementalPos_, incrementalRot_; 
  float incRotR_, incRotP_, incRotH_;
  
  unsigned long lastIncPosMicros_;
  double incAccX_, incVelX_, incPosX_;
  double incAccY_, incVelY_, incPosY_;
  double incAccZ_, incVelZ_, incPosZ_;
  bb::HighPassFilter accXFilter_, accYFilter_, accZFilter_;
  bb::IMU imu_;
};

#endif // REMOTEINPUT_H