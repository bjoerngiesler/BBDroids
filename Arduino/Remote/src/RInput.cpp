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

bool RInput::initMCP() {
  Console::console.printfBroadcast("Initializing MCP\n");

  Console::console.printfBroadcast("Trying address 1\n");
  if(mcp_.begin_I2C(MCP_ADDR1) == 0) {
    Console::console.printfBroadcast("Trying address 2\n");
    if(mcp_.begin_I2C(MCP_ADDR2) == 0) { // HAAAAAACK!!! to fix my stupid right remote with a bad solder joint
      Console::console.printfBroadcast("Couldn't initialize MCP!\n");
      return false;
    }
  }
  Console::console.printfBroadcast("Successfully initialized MCP!\n");
  for (uint8_t i = 0; i < 8; i++) {
    mcp_.pinMode(i, INPUT_PULLUP);
  }
  return true;
}

RInput::RInput(): imu_(IMU_ADDR) {
  for(bool& b: buttons) b = false;
  tlms_ = trms_ = cms_ = 0;
  longPressThresh_ = 500;
  minJoyRawH = 2048;
  maxJoyRawH = 2048;
  minJoyRawV = 2048;
  maxJoyRawV = 2048;
  incrementalAccel_ = BUTTON_NONE;
  incrementalRot_ = BUTTON_NONE;
  incRotR_ = 0; incRotP_ = 0; incRotH_ = 0;
}

bool RInput::begin() {
  return true;
}

void RInput::setIncrementalAccel(ButtonIndex btn) {
  incrementalAccel_ = btn;
}
  
void RInput::setIncrementalRot(ButtonIndex btn) {
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
  joyRawV = 4096 - analogRead(P_A_JOY_VER);
#else
  joyRawH = 4096 - analogRead(P_A_JOY_HOR);
  joyRawV = analogRead(P_A_JOY_VER);
#endif

  minJoyRawH = min(minJoyRawH, joyRawH);
  maxJoyRawH = max(maxJoyRawH, joyRawH);
  minJoyRawV = min(minJoyRawV, joyRawV);
  maxJoyRawV = max(maxJoyRawV, joyRawV);

  joyH = float(map(joyRawH, hCalib_.min, hCalib_.max, 0, 4096)-2048) / 2048.0f;
  if(abs(joyH) < JoystickEpsilon) joyH = 0.0f;
  joyH = constrain(joyH, -1.0f, 1.0f);

  joyV = float(2048-map(joyRawV, vCalib_.min, vCalib_.max, 0, 4096)) / 2048.0f;
  if(abs(joyV) < JoystickEpsilon) joyV = 0.0f;
  joyV = constrain(joyV, -1.0f, 1.0f);
  
  battRaw = analogRead(P_A_BATT_CHECK);
  battRaw = constrain(battRaw, MIN_ANALOG_IN_VDIV, MAX_ANALOG_IN_VDIV);
  battery = float(battRaw-MIN_ANALOG_IN_VDIV)/float(MAX_ANALOG_IN_VDIV-MIN_ANALOG_IN_VDIV);

  pot1 = (float)((4096-analogRead(P_A_POT1)) / 4096.0f); // careful - wraps around on the left remote
#if !defined(LEFT_REMOTE)
  pot2 = (float)((4096-analogRead(P_A_POT2)) / 4096.0f);
#else
  pot2 = 0;
#endif

  if(imu_.available()) {
    imu_.update();
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
      if (mcp_.digitalRead(i) == LOW) {
        if(buttons[i] == false) buttonsChanged[i] = true;
        else buttonsChanged[i] = false;
        buttons[i] = true;
      } else {
        if(buttons[i] == true) buttonsChanged[i] = true;
        else buttonsChanged[i] = false;
        buttons[i] = false;
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

  if(incrementalAccel_ < buttons.size()) {
    if(buttonsChanged[incrementalAccel_]) {

    }
  }
  
  if(incrementalRot_ < buttons.size()) {
    if(buttonsChanged[incrementalRot_]) {
      incRotR_ = 0; incRotP_ = 0; incRotH_ = 0;
      if(buttons[incrementalRot_]) {
        if(imu_.available()) {
          imu_.getFilteredRPH(incRotR_, incRotP_, incRotH_);
        } else {
          Console::console.printfBroadcast("IMU not available!\n");
        }
      }
      Console::console.printfBroadcast("Inc Rot R:%f P:%f H:%f\n", incRotR_, incRotP_, incRotH_);
    }
  }
}

bool RInput::anyButtonPressed() {
  for(bool& b: buttons) if(b == true) return true;
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

bool RInput::isOK() {
  if(mcpOK_ && imu_.available()) return true;
  return false;
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
    float roll, pitch, heading;
    float ax, ay, az;
    imu_.getFilteredRPH(roll, pitch, heading);
    imu_.getAccelMeasurement(ax, ay, az);

    if(incrementalRot_ < buttons.size()) {
      if(buttons[incrementalRot_]) {
        float rollOut, pitchOut, headingOut;
        transformRotation(roll, pitch, heading, incRotR_, incRotP_, incRotH_, rollOut, pitchOut, headingOut, true);
        packet.setAxis(2, rollOut, bb::ControlPacket::UNIT_DEGREES_CENTERED); 
        packet.setAxis(3, pitchOut, bb::ControlPacket::UNIT_DEGREES_CENTERED);
        packet.setAxis(4, headingOut, bb::ControlPacket::UNIT_DEGREES_CENTERED);
      } else {
        packet.setAxis(2, 0, bb::ControlPacket::UNIT_DEGREES_CENTERED); 
        packet.setAxis(3, 0, bb::ControlPacket::UNIT_DEGREES_CENTERED);
        packet.setAxis(4, 0, bb::ControlPacket::UNIT_DEGREES);
      }
    } else {
      packet.setAxis(2, roll, bb::ControlPacket::UNIT_DEGREES_CENTERED); 
      packet.setAxis(3, pitch, bb::ControlPacket::UNIT_DEGREES_CENTERED);
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

  packet.battery = RInput::input.battery * 63;
  
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