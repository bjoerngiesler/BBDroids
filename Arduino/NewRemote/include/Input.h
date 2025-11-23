#if !defined(INPUT_H)
#define INPUT_H

#include <LibBB.h>
#include <LibBBRemotes.h>

#include <Adafruit_MCP23X17.h>
#include "Config.h"
#include <array>

using namespace bb;
using namespace bb::rmt;

class Input {
public:
  static Input inst;

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
  static const uint8_t NUM_BUTTONS = 8;

  // Weird order given by ESP32 layout
#if defined(ARDUINO_ARCH_ESP32)
  typedef uint8_t ButtonPin;
  static ButtonPin BUTTON_PIN_1;
  static ButtonPin BUTTON_PIN_2;
  static ButtonPin BUTTON_PIN_3;
  static ButtonPin BUTTON_PIN_4;
  static ButtonPin BUTTON_PIN_JOY;
  static ButtonPin BUTTON_PIN_CONFIRM;
  static ButtonPin BUTTON_PIN_LEFT;
  static ButtonPin BUTTON_PIN_RIGHT;
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

  static void initButtonPinMapping();
  static Button pinToButton(ButtonPin index);
  static ButtonPin buttonToPin(Button index);

  bool begin(const Pins& pins);
  void update();
  void printOnSerial();
  bool initIMU();
  bool imuOK() { return imu_.available(); }
  bool initMCP();
  bool mcpOK() { return mcpOK_; }

  Result setupAxesFromTransmitter(Transmitter *trans);
  Result updateTransmitter(bb::rmt::Transmitter* trans);
  bb::IMU& imu() { return imu_; }

  void setCalibration(const AxisCalib& hc, const AxisCalib& vc) { hCalib = hc; vCalib = vc; }
  void setDeadbandPercent(unsigned long db) { deadbandPercent_ = db; }
  void setIncrementalPos(Button btn);
  void resetIncrementalPos();
  void setIncrementalRot(Button btn);

  bool anyButtonPressed();

  bool joyAtZero() { return joyAtZero_; }

  float secondsSinceLastMotion();

  float joyH, joyV; // range: -1.0 .. 1.0
  float rotR, rotP, rotH; // range: 0..360
  float aX, aY, aZ;
  float pot1, pot2; // range: 0 .. 1.0
  uint16_t pot1Raw, pot2Raw;
  float battery;    // range: 0 .. 1.0
  uint16_t joyRawH, joyRawV, battRaw;
  uint16_t minJoyRawH, maxJoyRawH, minJoyRawV, maxJoyRawV;
  AxisCalib hCalib, vCalib;

  std::map<Button,bool> buttons, buttonsChanged;

  bool btnLChanged, btnRChanged, btnConfirmChanged;

  void setFaceButtonsLocked(bool yesno);
  bool faceButtonsLocked() { return faceButtonsLocked_; }
  
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
  Input();

#if defined(ARDUINO_ARCH_ESP32)
  Adafruit_MCP23X17 mcp_; bool mcpOK_;
#endif

  AxisID joyHAxis = AXIS_INVALID, joyVAxis = AXIS_INVALID;
  AxisID rotPAxis = AXIS_INVALID, rotHAxis = AXIS_INVALID, rotRAxis = AXIS_INVALID;
  AxisID accXAxis = AXIS_INVALID, accYAxis = AXIS_INVALID, accZAxis = AXIS_INVALID;
  AxisID pot1Axis = AXIS_INVALID, pot2Axis = AXIS_INVALID;
  AxisID btn1Axis = AXIS_INVALID, btn2Axis = AXIS_INVALID, btn3Axis = AXIS_INVALID,
         btn4Axis = AXIS_INVALID, btnJoyAxis = AXIS_INVALID;
  AxisID btnLAxis = AXIS_INVALID, btnRAxis = AXIS_INVALID, btnConfirmAxis = AXIS_INVALID;
  AxisID battAxis = AXIS_INVALID;

  void btnLeftPressed();
  void btnLeftReleased();
  void btnRightPressed();
  void btnRightReleased();
  void btnConfirmPressed();
  void btnConfirmReleased();

  unsigned long lms_, rms_, cms_;
  std::function<void(void)> lShortPressCB_, lLongPressCB_, lPressCB_, lReleaseCB_;
  std::function<void(void)> rShortPressCB_, rLongPressCB_, rPressCB_, rReleaseCB_;
  std::function<void(void)> cShortPressCB_, cLongPressCB_, cPressCB_, cReleaseCB_;
  std::function<void(float)> encTurnCB_;

  bool faceButtonsLocked_;

  unsigned long longPressThresh_;
  Button incrementalPos_, incrementalRot_; 
  unsigned long deadbandPercent_;
  
  unsigned long lastIncPosMicros_;
  double incAccX_, incVelX_, incPosX_;
  double incAccY_, incVelY_, incPosY_;
  double incAccZ_, incVelZ_, incPosZ_;
  float incRotP_, incRotR_, incRotH_;
  bb::HighPassFilter accXFilter_, accYFilter_, accZFilter_;
  bb::LowPassFilter joyHFilter_, joyVFilter_;
  bb::IMU imu_;
  unsigned long lastMotionMS_;
  bool joyAtZero_;

  float lastEncDeg_;
  bb::LowPassFilter encTurnFilter_;
  void processEncoder();

  Pins pins_;
};

#endif // REMOTEINPUT_H