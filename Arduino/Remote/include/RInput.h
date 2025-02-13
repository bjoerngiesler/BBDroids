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

  enum Button {
    BUTTON_1       = 0,
    BUTTON_2       = 1,
    BUTTON_3       = 2,
    BUTTON_4       = 3,
    BUTTON_JOY     = 4,
    BUTTON_CONFIRM = 5,
    BUTTON_LEFT    = 6,
    BUTTON_RIGHT   = 7,
    BUTTON_NONE    = 8
  };

  // Weird order given by ESP32 layout
#if defined(ARDUINO_ARCH_ESP32)
#if defined(LEFT_REMOTE)
  enum ButtonPin {
    BUTTON_PIN_1       = 2,
    BUTTON_PIN_2       = 4,
    BUTTON_PIN_3       = 6,
    BUTTON_PIN_4       = 7,
    BUTTON_PIN_JOY     = 5,
    BUTTON_PIN_CONFIRM = 3,
    BUTTON_PIN_LEFT    = 1,
    BUTTON_PIN_RIGHT   = 0
  };
#else
  enum ButtonPin {
    BUTTON_PIN_1       = 5,
    BUTTON_PIN_2       = 4,
    BUTTON_PIN_3       = 1,
    BUTTON_PIN_4       = 2,
    BUTTON_PIN_JOY     = 3,
    BUTTON_PIN_CONFIRM = 7,
    BUTTON_PIN_LEFT    = 6,
    BUTTON_PIN_RIGHT   = 0
  };
#endif

#else
  enum ButtonPin {
    BUTTON_PIN_1       = 0,
    BUTTON_PIN_2       = 1,
    BUTTON_PIN_3       = 2,
    BUTTON_PIN_4       = 3,
    BUTTON_PIN_JOY     = 4,
    BUTTON_PIN_CONFIRM = 5,
    BUTTON_PIN_LEFT    = 6,
    BUTTON_PIN_RIGHT   = 7
  };
#endif
  struct AxisCalib {
  public:
    AxisCalib(): min(0), max(4095), center(2048) {}
    uint16_t min, max, center;
  };

  static Button pinToButton(ButtonPin index);
  static ButtonPin buttonToPin(Button index);

  bool begin();
  void update();
  void printOnSerial();
  bool initIMU();
  bool imuOK() { return imu_.available(); }
  bool initMCP();
  bool mcpOK() { return mcpOK_; }

  Result fillControlPacket(ControlPacket& packet);
  bb::IMU& imu() { return imu_; }

  void setCalibration(const AxisCalib& hc, const AxisCalib& vc) { hCalib = hc; vCalib = vc; }
  void setDeadbandPercent(unsigned long db) { deadbandPercent_ = db; }
  void setIncrementalPos(Button btn);
  void resetIncrementalPos();
  void setIncrementalRot(Button btn);

  bool anyButtonPressed();

  bool joyAtZero() { return joyAtZero_; }

  float secondsSinceLastMotion();

  void testMatrix();

  float pot1, pot2; // range: 0 .. 1.0
  float battery;    // range: 0 .. 1.0
  float joyH, joyV; // range: -1.0 .. 1.0
  uint16_t joyRawH, joyRawV, battRaw;
  uint16_t minJoyRawH, maxJoyRawH, minJoyRawV, maxJoyRawV;
  AxisCalib hCalib, vCalib;

  std::map<Button,bool> buttons, buttonsChanged;

  bool btnLChanged, btnRChanged, btnConfirmChanged;
  
  void setLeftShortPressCallback(std::function<void(void)> cb) { lShortPressCB_ = cb; }
  void setLeftLongPressCallback(std::function<void(void)> cb) { lLongPressCB_ = cb; }
  void setLeftPressCallback(std::function<void(void)> cb) { lPressCB_ = cb; }
  void setLeftReleaseCallback(std::function<void(void)> cb) { lReleaseCB_ = cb; }

  void setRightShortPressCallback(std::function<void(void)> cb) { rShortPressCB_ = cb; }
  void setRightLongPressCallback(std::function<void(void)> cb) { rLongPressCB_ = cb; }
  void setRightPressCallback(std::function<void(void)> cb) { rPressCB_ = cb; }
  void setRightReleaseCallback(std::function<void(void)> cb) { rReleaseCB_ = cb; }

  void setConfirmShortPressCallback(std::function<void(void)> cb) { cShortPressCB_ = cb; }
  void setConfirmLongPressCallback(std::function<void(void)> cb) { cLongPressCB_ = cb; }
  void setConfirmPressCallback(std::function<void(void)> cb) { cPressCB_ = cb; }
  void setConfirmReleaseCallback(std::function<void(void)> cb) { cReleaseCB_ = cb; }

  void setEncTurnCallback(std::function<void(float)> cb) { encTurnCB_ = cb; }

  void setAllCallbacks(std::function<void(void)> cb) {
    lShortPressCB_ = cb;
    lLongPressCB_ = cb;
    lPressCB_ = cb;
    lReleaseCB_ = cb;

    rShortPressCB_ = cb;
    rLongPressCB_ = cb;
    rPressCB_ = cb;
    rReleaseCB_ = cb;

    cShortPressCB_ = cb;
    cLongPressCB_ = cb;
    cPressCB_ = cb;
    cReleaseCB_ = cb;
  }
  void clearCallbacks();

protected:
  RInput();

#if defined(ARDUINO_ARCH_ESP32)
  Adafruit_MCP23X17 mcp_; bool mcpOK_;
#endif

  void btnLeftPressed();
  void btnLeftReleased();
  void btnRightPressed();
  void btnRightReleased();
  void btnConfirmPressed();
  void btnConfirmReleased();
  void processEncoder();

  unsigned long lms_, rms_, cms_;
  std::function<void(void)> lShortPressCB_, lLongPressCB_, lPressCB_, lReleaseCB_;
  std::function<void(void)> rShortPressCB_, rLongPressCB_, rPressCB_, rReleaseCB_;
  std::function<void(void)> cShortPressCB_, cLongPressCB_, cPressCB_, cReleaseCB_;
  std::function<void(float)> encTurnCB_;

  unsigned long longPressThresh_;
  Button incrementalPos_, incrementalRot_; 
  float incRotR_, incRotP_, incRotH_;
  unsigned long deadbandPercent_;
  
  unsigned long lastIncPosMicros_;
  double incAccX_, incVelX_, incPosX_;
  double incAccY_, incVelY_, incPosY_;
  double incAccZ_, incVelZ_, incPosZ_;
  bb::HighPassFilter accXFilter_, accYFilter_, accZFilter_;
  bb::LowPassFilter joyHFilter_, joyVFilter_;
  bb::IMU imu_;
  unsigned long lastMotionMS_;
  bool joyAtZero_;

#if defined(LEFT_REMOTE)
  float lastEncDeg_;
  bb::LowPassFilter encTurnFilter_;
#endif
};

#endif // REMOTEINPUT_H