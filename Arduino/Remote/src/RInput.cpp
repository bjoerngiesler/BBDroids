#include <LibBB.h>

#include "RInput.h"
#include "Config.h"

using namespace bb;

RInput RInput::input;

#if !defined(ARDUINO_ARCH_ESP32)
static void prepareInterruptPin(int pin, void (*isr)(void)) {
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE);
}

void bPinkyISR(void) { 
  RInput::input.buttons[RInput::BUTTON_PINKY] = !digitalRead(P_D_BTN_PINKY); 
}

void bIndexISR(void) { 
  RInput::input.buttons[RInput::BUTTON_INDEX] = !digitalRead(P_D_BTN_INDEX); 
}

void bJoyISR(void) { 
  RInput::input.buttons[RInput::BUTTON_JOY] = !digitalRead(P_D_BTN_JOY); 
}

void bLISR(void) { 
  RInput::input.buttons[RInput::BUTTON_LEFT] = !digitalRead(P_D_BTN_L); 
}

void bRISR(void) { 
  RInput::input.buttons[RInput::BUTTON_RIGHT] = !digitalRead(P_D_BTN_R); 
}

void bConfirmISR(void) { 
  RInput::input.buttons[RInput::BUTTON_CONFIRM] = !digitalRead(P_D_BTN_CONFIRM); 
  RInput::input.btnConfirmChanged = true;
}

void bTopLISR(void) { 
  RInput::input.buttons[RInput::BUTTON_TOP_LEFT] = !digitalRead(P_D_BTN_TOP_L); 
  RInput::input.btnTopLChanged = true;
}

void bTopRISR(void) { 
  RInput::input.buttons[RInput::BUTTON_TOP_RIGHT] = !digitalRead(P_D_BTN_TOP_R); 
  RInput::input.btnTopRChanged = true;
}
#endif // ARDUINO_ARCH_ESP32

RInput::Button RInput::pinToButton(RInput::ButtonPin pin) {
  switch(pin) {
    case BUTTON_PIN_PINKY:     return BUTTON_PINKY; break;
    case BUTTON_PIN_INDEX:     return BUTTON_INDEX; break;
    case BUTTON_PIN_JOY:       return BUTTON_JOY; break;
    case BUTTON_PIN_LEFT:      return BUTTON_LEFT; break;
    case BUTTON_PIN_RIGHT:     return BUTTON_RIGHT; break;
    case BUTTON_PIN_CONFIRM:   return BUTTON_CONFIRM; break;
    case BUTTON_PIN_TOP_LEFT:  return BUTTON_TOP_LEFT; break;
    case BUTTON_PIN_TOP_RIGHT: default: return BUTTON_TOP_RIGHT; break;
  }
}

RInput::ButtonPin RInput::buttonToPin(RInput::Button button) {
  switch(button) {
    case BUTTON_PINKY: return BUTTON_PIN_PINKY; break;
    case BUTTON_INDEX: return BUTTON_PIN_INDEX; break;
    case BUTTON_JOY: return BUTTON_PIN_JOY; break;
    case BUTTON_LEFT: return BUTTON_PIN_LEFT; break;
    case BUTTON_RIGHT: return BUTTON_PIN_RIGHT; break;
    case BUTTON_CONFIRM: return BUTTON_PIN_CONFIRM; break;
    case BUTTON_TOP_LEFT: return BUTTON_PIN_TOP_LEFT; break;
    case BUTTON_TOP_RIGHT: default: return BUTTON_PIN_TOP_RIGHT; break;
  }
}


bool RInput::initMCP() {
  if(mcp_.begin_I2C(MCP_ADDR1) == 0) {
    if(mcp_.begin_I2C(MCP_ADDR2) == 0) { // HAAAAAACK!!! to fix my stupid right remote with a bad solder joint
      Console::console.printfBroadcast("Couldn't initialize MCP!\n");
      return false;
    }
  }
  for (uint8_t i = 0; i < 8; i++) {
    mcp_.pinMode(i, INPUT_PULLUP);
  }
  return true;
}

RInput::RInput(): imu_(IMU_ADDR) {
  for(auto& b: buttons) b.second = false;
  tlms_ = trms_ = cms_ = 0;
  longPressThresh_ = 500;
  minJoyRawH = 2048;
  maxJoyRawH = 2048;
  minJoyRawV = 2048;
  maxJoyRawV = 2048;
  incrementalPos_ = BUTTON_NONE;
  incrementalRot_ = BUTTON_NONE;
  incRotR_ = 0; incRotP_ = 0; incRotH_ = 0;
  imu_.setRotationAroundZ(bb::IMU::ROTATE_180);
}

bool RInput::begin() {
  return true;
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
#if defined(LEFT_REMOTE)
  joyRawH = analogRead(P_A_JOY_HOR);
  joyRawV = 4095 - analogRead(P_A_JOY_VER);
#else
  joyRawH = 4095 - analogRead(P_A_JOY_HOR);
  joyRawV = analogRead(P_A_JOY_VER);
#endif

  minJoyRawH = min(minJoyRawH, joyRawH);
  maxJoyRawH = max(maxJoyRawH, joyRawH);
  minJoyRawV = min(minJoyRawV, joyRawV);
  maxJoyRawV = max(maxJoyRawV, joyRawV);

  unsigned int deadbandAbs = rint(4096*(deadbandPercent_/100.0f)/2.0f);
  if(joyRawH < hCalib.center) {
    if(joyRawH < hCalib.center - deadbandAbs)
      joyH = float(map(joyRawH, hCalib.min, hCalib.center-deadbandAbs, 0, 2047)-2047) / 2048.0f;
    else
      joyH = 0;
  } else {
    if(joyRawH > hCalib.center + deadbandAbs)
      joyH = float(map(joyRawH, hCalib.center+deadbandAbs, hCalib.max, 2047, 4095)-2047) / 2048.0f;
    else
      joyH = 0;
  }
  joyH = constrain(joyH, -1.0f, 1.0f);

  if(joyRawV < vCalib.center) {
    if(joyRawV < hCalib.center - deadbandAbs)
      joyV = float(2047-map(joyRawV, vCalib.min, vCalib.center-deadbandAbs, 0, 2047)) / 2048.0f;
    else {
      joyV = 0;
    }
  } else {
    if(joyRawV > hCalib.center + deadbandAbs)
      joyV = float(2047-map(joyRawV, vCalib.center+deadbandAbs, vCalib.max, 2047, 4095)) / 2048.0f;
    else
      joyV = 0;
  }
  if(abs(joyV) < deadbandPercent_/100.0f) joyV = 0.0f;
  joyV = constrain(joyV, -1.0f, 1.0f);
  
  battRaw = analogRead(P_A_BATT_CHECK);
  float battCooked = (battRaw/4095.0)*3.1;
  battRaw = constrain(battRaw, MIN_ANALOG_IN_VDIV, MAX_ANALOG_IN_VDIV);
  battery = float(battRaw-MIN_ANALOG_IN_VDIV)/float(MAX_ANALOG_IN_VDIV-MIN_ANALOG_IN_VDIV);
  
  pot1 = (float)((4095-analogRead(P_A_POT1)) / 4096.0f); // careful - wraps around on the left remote
#if !defined(LEFT_REMOTE)
  pot2 = (float)((4095-analogRead(P_A_POT2)) / 4096.0f);
#else
  pot2 = 0;
#endif

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
      if(incrementalPos_ != BUTTON_NONE && buttons[incrementalPos_] == true) {
#if 0
      Console::console.printfBroadcast("ax:%f,ay:%f,az:%f,p:%f,r:%f,h:%f,gx:%f,gy:%f,gz:%f,ax:%f,ay:%f,az:%f,X:%f,Y:%f,Z:%f\n", 
      ax, ay, az, p, r, h, gravx, gravy, gravz, incAccX_, incAccY_, incAccZ_, incPosX_, incPosY_, incPosZ_);
#else
      //Console::console.printfBroadcast("ax:%f,ay:%f,az:%f,X:%f,Y:%f,Z:%f\n", incAccX_, incAccY_, incAccZ_, incPosX_, incPosY_, incPosZ_);
#endif
      }
    }
    lastIncPosMicros_ = micros();
  } else {
    if(imu_.begin() == true) {
      float dr = imu_.dataRate();
      Console::console.printfBroadcast("Successfully initialized IMU; data rate: %f\n", dr);
      Runloop::runloop.setCycleTimeMicros(1e6/dr);
    } else {
      Console::console.printfBroadcast("IMU not available\n");
    }
  }


#if defined(ARDUINO_ARCH_ESP32) // ESP reads buttons from the MCP expander. Non-ESP does it from the interrupt routine block above.
  if(mcpOK_) {
    for(uint8_t i = 0; i < buttons.size(); i++) {
      Button b = pinToButton(ButtonPin(i));
      if (mcp_.digitalRead(i) == LOW) {
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
    mcpOK_ = initMCP();
  }
#endif // ARDUINO_ARCH_ESP32
  if(buttonsChanged[BUTTON_TOP_LEFT]) {
    if(buttons[BUTTON_TOP_LEFT]) btnTopLeftPressed();
    else btnTopLeftReleased();
  }
  if(buttonsChanged[BUTTON_TOP_RIGHT]) {
    if(buttons[BUTTON_TOP_RIGHT]) btnTopRightPressed();
    else btnTopRightReleased();
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

void RInput::btnTopLeftPressed() {
  tlms_ = millis();
}
void RInput::btnTopLeftReleased() {
  if(millis() - tlms_ < longPressThresh_ || tlLongPressCB_ == nullptr) {
    if(tlShortPressCB_ != nullptr) {
      tlShortPressCB_();
    }
  } else if(tlLongPressCB_ != nullptr) {
    tlLongPressCB_();
  }
}

Result RInput::fillControlPacket(ControlPacket& packet) {
#if defined(LEFT_REMOTE)
  packet.button0 = buttons[BUTTON_PINKY];
  packet.button1 = buttons[BUTTON_INDEX];
  packet.button2 = buttons[BUTTON_RIGHT];
  packet.button3 = buttons[BUTTON_LEFT];
  packet.button4 = buttons[BUTTON_JOY];
  packet.button5 = false;
  packet.button6 = false;
  packet.button7 = false;
#else
  packet.button0 = RInput::input.buttons[BUTTON_PINKY];
  packet.button1 = RInput::input.buttons[BUTTON_INDEX];
  packet.button2 = RInput::input.buttons[BUTTON_LEFT];
  packet.button3 = RInput::input.buttons[BUTTON_RIGHT];
  packet.button4 = RInput::input.buttons[BUTTON_JOY];
  packet.button5 = RInput::input.buttons[BUTTON_TOP_LEFT];
  packet.button6 = RInput::input.buttons[BUTTON_TOP_RIGHT];
  packet.button7 = RInput::input.buttons[BUTTON_CONFIRM];
#endif

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

#if defined(LEFT_REMOTE)
  packet.setAxis(9, 0);
#else
  // FIXME replace by values set in menu system
  packet.setAxis(9, pot2, bb::ControlPacket::UNIT_UNITY);
#endif

  packet.battery = RInput::input.battery * BATTERY_MAX;
  
  return RES_OK;
}

void RInput::btnTopRightPressed() {
  trms_ = millis();
}
void RInput::btnTopRightReleased() {
  if(millis() - trms_ < longPressThresh_ || trLongPressCB_ == nullptr) {
    if(trShortPressCB_ != nullptr) {
      trShortPressCB_();
    }
  } else if(trLongPressCB_ != nullptr) {
    trLongPressCB_();
  }
}

void RInput::btnConfirmPressed() {
  cms_ = millis();
}

void RInput::btnConfirmReleased() {
  if(millis() - cms_ < longPressThresh_ || cLongPressCB_ == nullptr) {
    if(cShortPressCB_ != nullptr) {
      cShortPressCB_();
    } else {
    }
  } else if(cLongPressCB_ != nullptr){
    cLongPressCB_();
  }
}

void RInput::clearCallbacks() {
  tlShortPressCB_ = nullptr;
  tlLongPressCB_ = nullptr;
  trShortPressCB_ = nullptr;
  trLongPressCB_ = nullptr;
  cLongPressCB_ = nullptr;
  cShortPressCB_ = nullptr;
}

void RInput::printOnSerial() {
  Serial.print("L:");
  Serial.print(buttons[BUTTON_LEFT]);
  Serial.print(" R:");
  Serial.print(buttons[BUTTON_RIGHT]);
  Serial.print(" P:");
  Serial.print(buttons[BUTTON_PINKY]);
  Serial.print(" I:");
  Serial.print(buttons[BUTTON_INDEX]);
  Serial.print(" TL:");
  Serial.print(buttons[BUTTON_TOP_LEFT]);
  Serial.print(" TR:");
  Serial.print(buttons[BUTTON_TOP_RIGHT]);
  Serial.print(" Joy:");
  Serial.print(buttons[BUTTON_JOY]);
  Serial.print(" Cf:");
  Serial.print(buttons[BUTTON_CONFIRM]);

  Serial.print(" H:");
  Serial.print(joyH, 3);
  Serial.print(" V:");
  Serial.print(joyV, 3);

  Serial.print(" Batt:");
  Serial.print(battery, 3);
  Serial.print(" Pot1:");
  Serial.print(pot1, 5);
  Serial.print(" Pot2:");
  Serial.println(pot2, 5);
}