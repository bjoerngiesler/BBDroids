#include <LibBB.h>

#include "RInput.h"
#include "Config.h"

#include <deque>

using namespace bb;

RInput RInput::input;

#if !defined(ARDUINO_ARCH_ESP32)
static void prepareInterruptPin(int pin, void (*isr)(void)) {
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE);
}

void b1ISR(void) { 
  RInput::input.buttons[RInput::BUTTON_1] = !digitalRead(P_D_BTN_1); 
}

void b2ISR(void) { 
  RInput::input.buttons[RInput::BUTTON_2] = !digitalRead(P_D_BTN_2); 
}

void b3ISR(void) { 
  RInput::input.buttons[RInput::BUTTON_3] = !digitalRead(P_D_BTN_3); 
}

void b4ISR(void) { 
  RInput::input.buttons[RInput::BUTTON_4] = !digitalRead(P_D_BTN_4); 
}

void bJoyISR(void) { 
  RInput::input.buttons[RInput::BUTTON_JOY] = !digitalRead(P_D_BTN_JOY); 
}

void bConfirmISR(void) { 
  RInput::input.buttons[RInput::BUTTON_CONFIRM] = !digitalRead(P_D_BTN_CONFIRM); 
  RInput::input.btnConfirmChanged = true;
}

void bLISR(void) { 
  RInput::input.buttons[RInput::BUTTON_LEFT] = !digitalRead(P_D_BTN_L); 
  RInput::input.btnLChanged = true;
}

void bRISR(void) { 
  RInput::input.buttons[RInput::BUTTON_RIGHT] = !digitalRead(P_D_BTN_R); 
  RInput::input.btnRChanged = true;
}
#endif // ARDUINO_ARCH_ESP32

RInput::ButtonPin RInput::BUTTON_PIN_1;
RInput::ButtonPin RInput::BUTTON_PIN_2;
RInput::ButtonPin RInput::BUTTON_PIN_3;
RInput::ButtonPin RInput::BUTTON_PIN_4;
RInput::ButtonPin RInput::BUTTON_PIN_JOY;
RInput::ButtonPin RInput::BUTTON_PIN_CONFIRM;
RInput::ButtonPin RInput::BUTTON_PIN_LEFT;
RInput::ButtonPin RInput::BUTTON_PIN_RIGHT;

void RInput::initButtonPinMapping() {
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

RInput::Button RInput::pinToButton(RInput::ButtonPin pin) {
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

RInput::ButtonPin RInput::buttonToPin(RInput::Button button) {
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

bool RInput::initIMU() {
  if(imu_.begin() == true) {
    float dr = imu_.dataRate();
    Console::console.printfBroadcast("Successfully initialized IMU; data rate: %f\n", dr);
    Runloop::runloop.setCycleTimeMicros(1e6/dr);
    return true;
  }

  Console::console.printfBroadcast("IMU not available\n");
  return false;
}

bool RInput::initMCP() {
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

RInput::RInput(): joyHFilter_(100, 0.01), joyVFilter_(100, 0.01), imu_(IMU_ADDR) {
  for(auto& b: buttons) b.second = false;
  lms_ = rms_ = cms_ = 0;
  longPressThresh_ = 500;
  minJoyRawH = 2048;
  maxJoyRawH = 2048;
  minJoyRawV = 2048;
  maxJoyRawV = 2048;
  incrementalPos_ = BUTTON_NONE;
  incrementalRot_ = BUTTON_NONE;
  incRotR_ = 0; incRotP_ = 0; incRotH_ = 0;
  joyAtZero_ = false;
  imu_.setRotationAroundZ(bb::IMU::ROTATE_180);
}

bool RInput::begin() {
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

void RInput::setIncrementalPos(Button btn) {
  incrementalPos_ = btn;
}

void RInput::resetIncrementalPos() {
  incAccX_ = 0; incAccY_ = 0; incAccZ_ = 0;
  incVelX_ = 0; incVelY_ = 0; incVelZ_ = 0;
  incPosX_ = 0; incPosY_ = 0; incPosZ_ = 0;
  lastIncPosMicros_ = 0;
}
  
void RInput::setIncrementalRot(Button btn) {
  incrementalRot_ = btn;
}

void RInput::testMatrix() {
  float r, p, h, rXf, pXf, hXf, r1, p1, h1;

  Console::console.printfBroadcast("Testing eulerToRot() and rotToEuler\n");
  r = 45; p = -45; h = 10;
  Console::console.printfBroadcast("In: r: %f p: %f h: %f\n", r, p, h);
  BLA::Matrix<3,3> R1 = eulerToRot(r, p, h);
  R1.printTo(Serial);
  Serial.println();
  r1 = 0; p1 = 0; h1 = 0;
  rotToEuler(R1, r1, p1, h1);
  Console::console.printfBroadcast("Out: r: %f p: %f h: %f\n", r1, p1, h1);

  Console::console.printfBroadcast("Rotating by 90° around z axis\n");
  r = 0; p = 0; h = 0;
  rXf = 0; pXf = 0; hXf = 90;
  r1 = 0; p1 = 0; h1 = 0;
  transformRotation(r, p, h, rXf, pXf, hXf, r1, p1, h1, false);
  Console::console.printfBroadcast("Out: r: %f p: %f h: %f\n", r1, p1, h1);
}

void RInput::update() {
  if(isLeftRemote) {
    joyRawH = analogRead(pins.P_A_JOY_HOR);
    joyRawV = 4095 - analogRead(pins.P_A_JOY_VER);
  } else {
    joyRawH = 4095 - analogRead(pins.P_A_JOY_HOR);
    joyRawV = analogRead(pins.P_A_JOY_VER);
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
  
  battRaw = analogRead(pins.P_A_BATT_CHECK);
  float battCooked = (battRaw/4095.0)*3.1;
  (void)battCooked;
  battRaw = constrain(battRaw, MIN_ANALOG_IN_VDIV, MAX_ANALOG_IN_VDIV);
  battery = float(battRaw-MIN_ANALOG_IN_VDIV)/float(MAX_ANALOG_IN_VDIV-MIN_ANALOG_IN_VDIV);
  
  if(isLeftRemote) {
    processEncoder();
  } else {
    pot1Raw = analogRead(pins.P_A_POT1);
    pot1 = (float)(4095-pot1Raw) / 4096.0f; 
    pot2Raw = analogRead(pins.P_A_POT2);
    pot2 = (float)(4095-pot2Raw) / 4096.0f;
  }

  if(imu_.available()) {
    imu_.update();

    float ax, ay, az;
    imu_.getGravCorrectedAccel(ax, ay, az);
    float absAccel = sqrt(sq(ax)+sq(ay)+sq(az));
    if(absAccel>.045) {
      lastMotionMS_ = millis();
    }

    incAccX_ = accXFilter_.filter(ax); if(fabs(incAccX_)<.1) incAccX_=0; 
    incAccY_ = accYFilter_.filter(ay); if(fabs(incAccY_)<.1) incAccY_=0; 
    incAccZ_ = accZFilter_.filter(az); if(fabs(incAccZ_)<.1) incAccZ_=0; 
        
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
  
  if(incrementalRot_ < buttons.size()) {
    if(buttonsChanged[incrementalRot_]) {
      incRotP_ = 0; incRotR_ = 0; incRotH_ = 0;
      if(buttons[incrementalRot_]) {
        if(imu_.available()) {
          imu_.getFilteredPRH(incRotP_, incRotR_, incRotH_);
        } else {
          Console::console.printfBroadcast("IMU not available!\n");
        }
      }
    }
  }
}

float RInput::secondsSinceLastMotion() {
  return WRAPPEDDIFF(millis(), lastMotionMS_, ULONG_MAX) / 1e3;
}


bool RInput::anyButtonPressed() {
  for(auto& b: buttons) if(b.second == true) return true;
  return false;
}

void RInput::btnLeftPressed() {
  lms_ = millis();
  if(lPressCB_ != nullptr && faceButtonsLocked_ == false) lPressCB_();
}

void RInput::btnLeftReleased() {
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

void RInput::btnRightPressed() {
  rms_ = millis();
  if(rPressCB_ != nullptr && faceButtonsLocked_ == false) rPressCB_();
}

void RInput::btnRightReleased() {
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

void RInput::btnConfirmPressed() {
  cms_ = millis();
  if(cPressCB_ != nullptr && faceButtonsLocked_ == false) cPressCB_();
}

void RInput::btnConfirmReleased() {
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

void RInput::processEncoder() {
  uint16_t enc = analogRead(pins.P_A_POT1);

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

void RInput::setFaceButtonsLocked(bool yn) {
  faceButtonsLocked_ = yn;
}

void RInput::clearCallbacks() {
  setAllCallbacks(nullptr);
}

Result RInput::fillControlPacket(ControlPacket& packet) {
  packet.button0 = buttons[BUTTON_1];
  packet.button1 = buttons[BUTTON_2];
  packet.button2 = buttons[BUTTON_3];
  packet.button3 = buttons[BUTTON_4];
  packet.button4 = buttons[BUTTON_JOY];

  if(isLeftRemote && faceButtonsLocked() == false) {
    packet.button5 = false;
    packet.button6 = false;
    packet.button7 = false;
  } else {
    packet.button5 = RInput::input.buttons[BUTTON_LEFT];
    packet.button6 = RInput::input.buttons[BUTTON_RIGHT];
    packet.button7 = RInput::input.buttons[BUTTON_CONFIRM];
  }

  packet.setAxis(0, joyH, bb::ControlPacket::UNIT_UNITY_CENTERED);
  packet.setAxis(1, joyV, bb::ControlPacket::UNIT_UNITY_CENTERED);

  if (imu_.available()) {
    float pitch, roll, heading;
    float ax, ay, az;
    imu_.getFilteredPRH(pitch, roll, heading);
    imu_.getAccelMeasurement(ax, ay, az);

    if(incrementalRot_ < buttons.size()) {
      if(buttons[incrementalRot_]) {
        float pitchOut, rollOut, headingOut;
        transformRotation(pitch, roll, heading, incRotP_, incRotR_, incRotH_, pitchOut, rollOut, headingOut, true);
        packet.setAxis(2, pitchOut, bb::ControlPacket::UNIT_DEGREES_CENTERED); 
        packet.setAxis(3, rollOut, bb::ControlPacket::UNIT_DEGREES_CENTERED);
        packet.setAxis(4, headingOut, bb::ControlPacket::UNIT_DEGREES_CENTERED);
      } else {
        packet.setAxis(2, 0, bb::ControlPacket::UNIT_DEGREES_CENTERED); 
        packet.setAxis(3, 0, bb::ControlPacket::UNIT_DEGREES_CENTERED);
        packet.setAxis(4, 0, bb::ControlPacket::UNIT_DEGREES);
      }
    } else {
      packet.setAxis(2, pitch, bb::ControlPacket::UNIT_DEGREES_CENTERED); 
      packet.setAxis(3, roll, bb::ControlPacket::UNIT_DEGREES_CENTERED);
      packet.setAxis(4, heading, bb::ControlPacket::UNIT_DEGREES);
    }
    packet.setAxis(5, ax, bb::ControlPacket::UNIT_UNITY_CENTERED);
    packet.setAxis(6, ay, bb::ControlPacket::UNIT_UNITY_CENTERED);
    packet.setAxis(7, az, bb::ControlPacket::UNIT_UNITY_CENTERED);
  }

  packet.setAxis(8, RInput::input.pot1, bb::ControlPacket::UNIT_UNITY);
  packet.setAxis(9, RInput::input.pot2, bb::ControlPacket::UNIT_UNITY);

  packet.battery = RInput::input.battery * BATTERY_MAX;
  
  return RES_OK;
}

