#include <LibBB.h>

#include "Input.h"
#include "Config.h"

#include <deque>

using namespace bb;

#if !defined(ARDUINO_ARCH_ESP32)
static void prepareInterruptPin(int pin, void (*isr)(void)) {
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE);
}

void b1ISR(void) { 
  Input::input.buttons[Input::BUTTON_1] = !digitalRead(P_D_BTN_1); 
}

void b2ISR(void) { 
  Input::input.buttons[Input::BUTTON_2] = !digitalRead(P_D_BTN_2); 
}

void b3ISR(void) { 
  Input::input.buttons[Input::BUTTON_3] = !digitalRead(P_D_BTN_3); 
}

void b4ISR(void) { 
  Input::input.buttons[Input::BUTTON_4] = !digitalRead(P_D_BTN_4); 
}

void bJoyISR(void) { 
  Input::input.buttons[Input::BUTTON_JOY] = !digitalRead(P_D_BTN_JOY); 
}

void bConfirmISR(void) { 
  Input::input.buttons[Input::BUTTON_CONFIRM] = !digitalRead(P_D_BTN_CONFIRM); 
  Input::input.btnConfirmChanged = true;
}

void bLISR(void) { 
  Input::input.buttons[Input::BUTTON_LEFT] = !digitalRead(P_D_BTN_L); 
  Input::input.btnLChanged = true;
}

void bRISR(void) { 
  Input::input.buttons[Input::BUTTON_RIGHT] = !digitalRead(P_D_BTN_R); 
  Input::input.btnRChanged = true;
}
#endif // ARDUINO_ARCH_ESP32

Input Input::inst;

Input::ButtonPin Input::BUTTON_PIN_1;
Input::ButtonPin Input::BUTTON_PIN_2;
Input::ButtonPin Input::BUTTON_PIN_3;
Input::ButtonPin Input::BUTTON_PIN_4;
Input::ButtonPin Input::BUTTON_PIN_JOY;
Input::ButtonPin Input::BUTTON_PIN_CONFIRM;
Input::ButtonPin Input::BUTTON_PIN_LEFT;
Input::ButtonPin Input::BUTTON_PIN_RIGHT;

void Input::initButtonPinMapping() {
  if(isLeftRemote) {
    BUTTON_PIN_1       = 2;
    BUTTON_PIN_2       = 4;
    BUTTON_PIN_3       = 6;
    BUTTON_PIN_4       = 7;
    BUTTON_PIN_JOY     = 5;
    BUTTON_PIN_CONFIRM = 3;
    BUTTON_PIN_LEFT    = 1;
    BUTTON_PIN_RIGHT   = 0;
  } else {
    BUTTON_PIN_1       = 5;
    BUTTON_PIN_2       = 4;
    BUTTON_PIN_3       = 3;
    BUTTON_PIN_4       = 2;
    BUTTON_PIN_JOY     = 1;
    BUTTON_PIN_CONFIRM = 7;
    BUTTON_PIN_LEFT    = 6;
    BUTTON_PIN_RIGHT   = 0;
  };
}

Input::Button Input::pinToButton(Input::ButtonPin pin) {
  if(pin==BUTTON_PIN_1) return BUTTON_1;
  else if(pin==BUTTON_PIN_2) return BUTTON_2; 
  else if(pin==BUTTON_PIN_3) return BUTTON_3;
  else if(pin==BUTTON_PIN_4) return BUTTON_4;
  else if(pin==BUTTON_PIN_JOY) return BUTTON_JOY;
  else if(pin==BUTTON_PIN_CONFIRM) return BUTTON_CONFIRM;
  else if(pin==BUTTON_PIN_LEFT) return BUTTON_LEFT;
  else if(pin==BUTTON_PIN_RIGHT) return BUTTON_RIGHT;
  return BUTTON_RIGHT;
}

Input::ButtonPin Input::buttonToPin(Input::Button button) {
  switch(button) {
    case BUTTON_1:       return BUTTON_PIN_1; break;
    case BUTTON_2:       return BUTTON_PIN_2; break;
    case BUTTON_3:       return BUTTON_PIN_3; break;
    case BUTTON_4:       return BUTTON_PIN_4; break;
    case BUTTON_JOY:     return BUTTON_PIN_JOY; break;
    case BUTTON_CONFIRM: return BUTTON_PIN_CONFIRM; break;
    case BUTTON_LEFT:    return BUTTON_PIN_LEFT; break;
    case BUTTON_RIGHT:   default: return BUTTON_PIN_RIGHT; break;
  }
}

bool Input::initIMU() {
  for(auto addr: IMU_ADDRESSES) {
    if(imu_.begin(addr) == true) {
      float dr = imu_.dataRate();
      Console::console.printfBroadcast("Successfully initialized IMU; data rate: %f\n", dr);
      Runloop::runloop.setCycleTimeMicros(1e6/dr);
      return true;
    }
  }

  Console::console.printfBroadcast("IMU not available\n");
  return false;
}

bool Input::initMCP() {
  bb::printf("Initializing MCP... ");

  mcpOK_ = false;

  // determine addr
  uint8_t addr = 0x00;
  Wire.beginTransmission(MCP_ADDR1);
  if(Wire.endTransmission() == 0) {
    addr = MCP_ADDR1;
  } else {
    Wire.beginTransmission(MCP_ADDR2);
    if(Wire.endTransmission() == 0) {
      addr = MCP_ADDR2;
    }
  }

  if(addr == 0x0) {
    bb::printf("failed (not found at 0x%x or 0x%x).\n", MCP_ADDR1, MCP_ADDR2);
    return false;
  }

  if(mcp_.begin_I2C(addr) == false) {
    bb::printf("failed on 0x%x\n", addr);
    return false;
  }
  
  for (uint8_t i = 0; i < 8; i++) {
    mcp_.pinMode(i, INPUT_PULLUP);
  }

  bb::printf("success at 0x%x\n", addr);
  mcpOK_ = true;
  return true;
}

Input::Input(): joyHFilter_(100, 0.01), joyVFilter_(100, 0.01) {
  for(auto& b: buttons) b.second = false;
  lms_ = rms_ = cms_ = 0;
  longPressThresh_ = 500;
  minJoyRawH = 2048;
  maxJoyRawH = 2048;
  minJoyRawV = 2048;
  maxJoyRawV = 2048;
  incrementalPos_ = BUTTON_NONE;
  incrementalRot_ = BUTTON_3;
  joyAtZero_ = false;
  imu_.setRotationAroundZ(bb::IMU::ROTATE_180);
  pot1 = 0.5;
  pot2 = 0.5;
  deadbandPercent_ = 5;
}

bool Input::begin(const Pins& pins) {
  pins_ = pins;

  // calibrate joystick
  float hval = 0, vval = 0;
  for(int i=0; i<100; i++) {
    if(isLeftRemote) {
      hval += float(analogRead(pins_.P_A_JOY_HOR))/100.0;
      vval += float(4095-analogRead(pins_.P_A_JOY_VER))/100.0;
    } else {
      hval += float(4095-analogRead(pins_.P_A_JOY_HOR))/100.0;
      vval += float(analogRead(pins_.P_A_JOY_VER))/100.0;
    }
  }
  bb::printf("Calibration values: %f, %f\n", hval, vval);
  hCalib.center = uint16_t(hval);
  vCalib.center = uint16_t(vval);


  initButtonPinMapping();
  for(int i=0; i<10; i++) {
    if(initIMU() == true) break;
    delay(100);
  }
  for(int i=0; i<10; i++) {
    if(initMCP() == true) break;
    delay(100);
  }
  if(!imuOK()) bb::printf("Error: Could not initialize IMU\n");
  if(!mcpOK()) bb::printf("Error: Could not initialize MCP\n");

  return imuOK() && mcpOK();
}

void Input::setIncrementalPos(Button btn) {
  incrementalPos_ = btn;
}

void Input::resetIncrementalPos() {
  incAccX_ = 0; incAccY_ = 0; incAccZ_ = 0;
  incVelX_ = 0; incVelY_ = 0; incVelZ_ = 0;
  incPosX_ = 0; incPosY_ = 0; incPosZ_ = 0;
  lastIncPosMicros_ = 0;
}
  
void Input::setIncrementalRot(Button btn) {
  incrementalRot_ = btn;
}

void Input::update() {
  if(isLeftRemote) {
    joyRawH = analogRead(pins_.P_A_JOY_HOR);
    joyRawV = 4095 - analogRead(pins_.P_A_JOY_VER);
  } else {
    joyRawH = 4095 - analogRead(pins_.P_A_JOY_HOR);
    joyRawV = analogRead(pins_.P_A_JOY_VER);
  }

  minJoyRawH = min(minJoyRawH, joyRawH);
  maxJoyRawH = max(maxJoyRawH, joyRawH);
  minJoyRawV = min(minJoyRawV, joyRawV);
  maxJoyRawV = max(maxJoyRawV, joyRawV);

  //float joyFilteredH = joyHFilter_.filter(joyRawH);
  //float joyFilteredV = joyVFilter_.filter(joyRawV);
  float joyFilteredH = joyRawH;
  float joyFilteredV = joyRawV;
  unsigned int deadbandAbs = rint(4096*(deadbandPercent_/100.0f)/2.0f);

  joyAtZero_ = true;

  if(joyFilteredH < hCalib.center - deadbandAbs) {
    joyH = float(map(joyFilteredH, hCalib.min, hCalib.center-deadbandAbs, 0, 2047)-2047) / 2048.0f;
    joyAtZero_ = false;
  } else if(joyFilteredH > hCalib.center + deadbandAbs) {
    joyAtZero_ = false;
    joyH = float(map(joyFilteredH, hCalib.center+deadbandAbs, hCalib.max, 2047, 4095)-2047) / 2048.0f;
  } else {
    joyH = 0;
  }
  joyH = constrain(joyH, -1.0f, 1.0f);
  //bb::printf("Updated joyH to %f\n", joyH);

  if(joyFilteredV < vCalib.center-deadbandAbs) {
    joyAtZero_ = false;
    joyV = float(2047-map(joyFilteredV, vCalib.min, vCalib.center-deadbandAbs, 0, 2047)) / 2048.0f;
  } else if(joyFilteredV > vCalib.center+deadbandAbs) {
    joyAtZero_ = false;
    joyV = float(2047-map(joyFilteredV, vCalib.center+deadbandAbs, vCalib.max, 2047, 4095)) / 2048.0f;
  } else {
    joyV = 0;
  }
  joyV = constrain(joyV, -1.0f, 1.0f);
  
  battRaw = analogRead(pins_.P_A_BATT_CHECK);
  float battCooked = (battRaw/4095.0)*3.1;
  (void)battCooked;
  battRaw = constrain(battRaw, MIN_ANALOG_IN_VDIV, MAX_ANALOG_IN_VDIV);
  battery = float(battRaw-MIN_ANALOG_IN_VDIV)/float(MAX_ANALOG_IN_VDIV-MIN_ANALOG_IN_VDIV);
  
  if(isLeftRemote) {
    processEncoder();
  } else {
    pot1Raw = analogRead(pins_.P_A_POT1);
    pot1 = (float)(4095-pot1Raw) / 4096.0f; 
    pot2Raw = analogRead(pins_.P_A_POT2);
    pot2 = (float)(4095-pot2Raw) / 4096.0f;
  }

  if(imu_.available()) {
    imu_.update();

    imu_.getGravCorrectedAccel(aX, aY, aZ);
    float absAccel = sqrt(sq(aX)+sq(aY)+sq(aZ));
    if(absAccel>.045) {
      lastMotionMS_ = millis();
    }

    incAccX_ = accXFilter_.filter(aX); if(fabs(incAccX_)<.1) incAccX_=0; 
    incAccY_ = accYFilter_.filter(aY); if(fabs(incAccY_)<.1) incAccY_=0; 
    incAccZ_ = accZFilter_.filter(aZ); if(fabs(incAccZ_)<.1) incAccZ_=0; 
        
    if(lastIncPosMicros_ != 0) {
      double dt = (micros() - lastIncPosMicros_)/1e6;
      incVelX_ += incAccX_*dt; incVelY_ += incAccY_*dt; incVelZ_ += incAccZ_*dt;
      incPosX_ += incVelX_*dt + (incAccX_*dt)*(incAccX_*dt)/2.0;
      incPosY_ += incVelY_*dt + (incAccY_*dt)*(incAccY_*dt)/2.0;
      incPosZ_ += incVelZ_*dt + (incAccZ_*dt)*(incAccZ_*dt)/2.0;
    }
    lastIncPosMicros_ = micros();
  }

  #if defined(ARDUINO_ARCH_ESP32) // ESP reads buttons from the MCP expander. Non-ESP does it from the interrupt routine block above.
  if(mcpOK_) {
    for(uint8_t i = 0; i < NUM_BUTTONS; i++) {
      Button b = Button(i);
      ButtonPin pin = buttonToPin(b);
      if (mcp_.digitalRead(int(pin)) == LOW) {
        if(buttons[b] == false) buttonsChanged[b] = true;
        else buttonsChanged[b] = false;
        buttons[b] = true;
      } else {
        if(buttons[b] == true) buttonsChanged[b] = true;
        else buttonsChanged[b] = false;
        buttons[b] = false;
      }
    }
  } else {
    bb::printf("MCP not OK\n");
  }
#endif // ARDUINO_ARCH_ESP32
  if(buttonsChanged[BUTTON_LEFT]) {
    if(buttons[BUTTON_LEFT]) btnLeftPressed();
    else btnLeftReleased();
  }
  if(buttonsChanged[BUTTON_RIGHT]) {
    if(buttons[BUTTON_RIGHT]) btnRightPressed();
    else btnRightReleased();
  }
  if(buttonsChanged[BUTTON_CONFIRM]) {
    if(buttons[BUTTON_CONFIRM]) btnConfirmPressed();
    else btnConfirmReleased();
  }

  if(incrementalPos_ < buttons.size()) {
    if(buttonsChanged[incrementalPos_]) {
      resetIncrementalPos();
    } else if(buttons[incrementalPos_]) {
    }
  }

  if(imu_.available()) {
    imu_.getFilteredPRH(rotP, rotR, rotH);
  }

  if(incrementalRot_ < buttons.size()) {
    // Changed? Remember current rotation
    if(buttonsChanged[incrementalRot_]) {
      if(buttons[incrementalRot_]) {
        if(imu_.available()) {
          float r = rotR; float p = rotP; float h = rotH;
          imu_.getFilteredPRH(incRotP_, incRotR_, incRotH_);
          transformRotation(p, r, h, incRotP_, incRotR_, incRotH_, rotP, rotR, rotH, true);
        } else {
          Console::console.printfBroadcast("IMU not available!\n");
        }
      } else {
        incRotP_ = incRotH_ = incRotR_ = 0;
      }
    } else if(buttons[incrementalRot_]) { // pressed
      float r = rotR; float p = rotP; float h = rotH;
      transformRotation(p, r, h, incRotP_, incRotR_, incRotH_, rotP, rotR, rotH, true);
    } else {
      rotP = 0; rotR = 0; rotH = 0;
    }
  }
}

float Input::secondsSinceLastMotion() {
  return WRAPPEDDIFF(millis(), lastMotionMS_, ULONG_MAX) / 1e3;
}


bool Input::anyButtonPressed() {
  for(auto& b: buttons) if(b.second == true) return true;
  return false;
}

void Input::btnLeftPressed() {
  lms_ = millis();
  if(lPressCB_ != nullptr && faceButtonsLocked_ == false) lPressCB_();
}

void Input::btnLeftReleased() {
  if(faceButtonsLocked_ == true) return;

  if(lReleaseCB_ != nullptr) lReleaseCB_();
  
  if(millis() - lms_ < longPressThresh_ || lLongPressCB_ == nullptr) {
    if(lShortPressCB_ != nullptr) {
      lShortPressCB_();
    }
  } else if(lLongPressCB_ != nullptr) {
    lLongPressCB_();
  }
}

void Input::btnRightPressed() {
  rms_ = millis();
  if(rPressCB_ != nullptr && faceButtonsLocked_ == false) rPressCB_();
}

void Input::btnRightReleased() {
  if(faceButtonsLocked_ == true) return;

  if(rReleaseCB_ != nullptr) rReleaseCB_();

  if(millis() - rms_ < longPressThresh_ || rLongPressCB_ == nullptr) {
    if(rShortPressCB_ != nullptr) {
      rShortPressCB_();
    }
  } else if(rLongPressCB_ != nullptr) {
    rLongPressCB_();
  }
}

void Input::btnConfirmPressed() {
  cms_ = millis();
  if(cPressCB_ != nullptr && faceButtonsLocked_ == false) cPressCB_();
}

void Input::btnConfirmReleased() {
  if(faceButtonsLocked_ == false) {
    if(cReleaseCB_ != nullptr) cReleaseCB_();

    if(millis() - cms_ < longPressThresh_ || cLongPressCB_ == nullptr) {
      if(cShortPressCB_ != nullptr) {
        cShortPressCB_();
      } 
    } else if(cLongPressCB_ != nullptr){
      cLongPressCB_();
    }
  } else if(millis() - cms_ >= longPressThresh_ && cLongPressCB_ != nullptr) {
    cLongPressCB_();
  }
}

void Input::processEncoder() {
  uint16_t enc = analogRead(pins_.P_A_POT1);

  // Remove spurious stuff arount zero
  static uint16_t deadband = 70;
  if(enc < deadband) return;
  
  // median filter to reduce noise
  static std::deque<uint16_t> meas;
  meas.push_back(enc);
  while(meas.size() > 3) meas.pop_front();
  std::deque<uint16_t> measTmp = meas;
  std::sort(measTmp.begin(), measTmp.end());
  enc = measTmp[measTmp.size()/2];

  float encDeg = rint((enc/4096.0)*360.0);
  float encDiff = lastEncDeg_ - encDeg;

  if(lastEncDeg_ > 270 && encDeg < 90) {
    encDiff -= 360;
  } else if(lastEncDeg_ < 90 && encDeg > 270) {
    encDiff += 360;
  }

  if(fabs(encDiff) > 5) {
    if(encTurnCB_ != nullptr) encTurnCB_(encDiff);
    lastEncDeg_ = encDeg;
  }
}

void Input::setFaceButtonsLocked(bool yn) {
  faceButtonsLocked_ = yn;
}

void Input::clearCallbacks() {
  setAllCallbacks(nullptr);
}

Result Input::setupAxesFromTransmitter(Transmitter* trans) {
  joyHAxis = joyVAxis = AXIS_INVALID;
  rotPAxis = rotHAxis = rotRAxis = AXIS_INVALID;
  accXAxis = accYAxis = accZAxis = AXIS_INVALID;
  pot1Axis = pot2Axis = AXIS_INVALID;
  btn1Axis = btn2Axis = btn3Axis = btn4Axis = btnJoyAxis = AXIS_INVALID;
  btnLAxis = btnRAxis = btnConfirmAxis = AXIS_INVALID;
  battAxis = AXIS_INVALID;

  uint8_t axis = 0, numAxes = trans->numAxes();
  #define MAPAXIS(here, minbits, maxbits, name) { if(axis < numAxes && trans->bitDepthForAxis(axis) >= minbits) {here = axis; bb::printf("Mapped %s to %s\n", name, trans->axisName(axis).c_str()); axis++;} else if(trans->canAddAxes()) { here = axis; trans->addAxis(name, maxbits); bb::printf("Mapped %s to new axis\n", name); axis++; } else {bb::printf("Not mapped %s\n", name);} }

  MAPAXIS(joyHAxis, 4, 12, "JoyH");
  MAPAXIS(joyVAxis, 4, 12, "JoyV");
  MAPAXIS(rotRAxis, 4, 12, "RotR");
  MAPAXIS(rotPAxis, 4, 12, "RotP");
  MAPAXIS(rotHAxis, 4, 12, "RotH");
  MAPAXIS(accXAxis, 4, 12, "AccX");
  MAPAXIS(accYAxis, 4, 12, "AccY");
  MAPAXIS(accZAxis, 4, 12, "AccZ");
  MAPAXIS(pot1Axis, 4, 12, "Pot1");
  MAPAXIS(pot2Axis, 4, 12, "Pot2");
  MAPAXIS(battAxis, 4, 12, "Batt");
  MAPAXIS(btn1Axis, 1, 1, "Btn1");
  MAPAXIS(btn2Axis, 1, 1, "Btn2");
  MAPAXIS(btn3Axis, 1, 1, "Btn3");
  MAPAXIS(btn4Axis, 1, 1, "Btn4");
  MAPAXIS(btnJoyAxis, 1, 1, "BtnJoy");
  MAPAXIS(btnLAxis, 1, 1, "BtnL");
  MAPAXIS(btnRAxis, 1, 1, "BtnR");
  MAPAXIS(btnConfirmAxis, 1, 1, "BtnConf");
  return RES_OK;
}

Result Input::updateTransmitter(bb::rmt::Transmitter* trans) {
  if(trans == nullptr) {
    bb::printf("Transmitter is nullptr!\n");
    return RES_SUBSYS_HW_DEPENDENCY_MISSING;
  }

  //bb::printf("Setting axis value for %d to %f\n", joyHAxis, joyH);
  trans->setAxisValue(joyHAxis, joyH, bb::rmt::UNIT_UNITY_CENTERED);
  trans->setAxisValue(joyVAxis, joyV, bb::rmt::UNIT_UNITY_CENTERED);
  trans->setAxisValue(rotPAxis, 0, bb::rmt::UNIT_DEGREES_CENTERED); 
  trans->setAxisValue(rotRAxis, 0, bb::rmt::UNIT_DEGREES_CENTERED);
  trans->setAxisValue(rotHAxis, 0, bb::rmt::UNIT_DEGREES);

  trans->setAxisValue(rotRAxis, rotR, bb::rmt::UNIT_DEGREES_CENTERED);
  trans->setAxisValue(rotPAxis, rotP, bb::rmt::UNIT_DEGREES_CENTERED);
  trans->setAxisValue(rotHAxis, rotH, bb::rmt::UNIT_DEGREES_CENTERED);
    
  trans->setAxisValue(accXAxis, aX, bb::rmt::UNIT_UNITY_CENTERED);
  trans->setAxisValue(accYAxis, aY, bb::rmt::UNIT_UNITY_CENTERED);
  trans->setAxisValue(accZAxis, aZ, bb::rmt::UNIT_UNITY_CENTERED);

  trans->setAxisValue(pot1Axis, pot1, bb::rmt::UNIT_UNITY);
  trans->setAxisValue(pot2Axis, pot2, bb::rmt::UNIT_UNITY);

  trans->setAxisValue(btn1Axis, buttons[BUTTON_1], bb::rmt::UNIT_RAW);
  trans->setAxisValue(btn2Axis, buttons[BUTTON_2], bb::rmt::UNIT_RAW);
  trans->setAxisValue(btn3Axis, buttons[BUTTON_3], bb::rmt::UNIT_RAW);
  trans->setAxisValue(btn4Axis, buttons[BUTTON_4], bb::rmt::UNIT_RAW);
  trans->setAxisValue(btnJoyAxis, buttons[BUTTON_JOY], bb::rmt::UNIT_RAW);
  trans->setAxisValue(btnLAxis, buttons[BUTTON_LEFT], bb::rmt::UNIT_RAW);
  trans->setAxisValue(btnRAxis, buttons[BUTTON_RIGHT], bb::rmt::UNIT_RAW);
  trans->setAxisValue(btnConfirmAxis, buttons[BUTTON_CONFIRM], bb::rmt::UNIT_RAW);

  return RES_OK;
}

