#include <LibBB.h>

#include "RInput.h"
#include "Config.h"

using namespace bb;

RInput RInput::input;

#if !defined(ESP32_REMOTE)
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
#endif // ESP32_REMOTE

RInput::RInput() {
  for(bool& b: buttons) b = false;
  tlms_ = trms_ = cms_ = 0;
  longPressThresh_ = 500;
}

bool RInput::begin() {
#if defined(ESP32_REMOTE)
  if(mcp_.begin_I2C(MCP_ADDR) == 0) {
    Console::console.printfBroadcast("Couldn't initialize MCP!\n");
    mcpOK_ = false;
  } else {
    for (uint8_t i = 0; i < 8; i++) {
      mcp_.pinMode(i, INPUT_PULLUP);
    }
    mcpOK_ = true;
  }
#endif

  update();

  return true;
}

void RInput::update() {
  joyRawH = analogRead(P_A_JOY_HOR);
#if defined(LEFT_REMOTE)
  joyRawV = 4096 - analogRead(P_A_JOY_VER);
#else
  joyRawV = analogRead(P_A_JOY_VER);
#endif

  joyH = (float)(joyRawH - hCalib_.center) / 2048.0f;
  if(abs(joyH) < JoystickEpsilon) joyH = 0.0f;
  joyH = constrain(joyH, -1.0f, 1.0f);

  joyV = (float)(vCalib_.center - joyRawV) / 2048.0f;
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

#if defined(ESP32_REMOTE) // ESP reads buttons from the MCP expander. Non-ESP does it from the interrupt routine block above.
  if(mcpOK_) {
#if 0
    for(uint8_t i = 0; i<16; i++) {
      if(mcp_.digitalRead(i) == LOW) Console::console.printfBroadcast("_");
      else Console::console.printfBroadcast("X");
    }
    Console::console.printfBroadcast("\n");
#endif
    for(uint8_t i = 0; i < buttons.size(); i++) {
      if (mcp_.digitalRead(i) == LOW) {
        if(buttons[i] == false) {
          if(i==BUTTON_TOP_LEFT) btnTopLChanged = true;
          else if(i==BUTTON_TOP_RIGHT) btnTopRChanged = true;
          else if(i==BUTTON_CONFIRM) btnConfirmChanged = true;
        }
        buttons[i] = true;
      } else {
        if(buttons[i] == true) {
          if(i==BUTTON_TOP_LEFT) btnTopLChanged = true;
          else if(i==BUTTON_TOP_RIGHT) btnTopRChanged = true;
          else if(i==BUTTON_CONFIRM) btnConfirmChanged = true;
        }
        buttons[i] = false;
      }
    }
  } else {
    //Console::console.printfBroadcast("MCP not OK\n");
  }
#endif // ESP32_REMOTE
  if(btnTopLChanged) {
    if(buttons[BUTTON_TOP_LEFT]) btnTopLeftPressed();
    else btnTopLeftReleased();
    btnTopLChanged = false;
  }
  if(btnTopRChanged) {
    if(buttons[BUTTON_TOP_RIGHT]) btnTopRightPressed();
    else btnTopRightReleased();
    btnTopRChanged = false;
  }
  if(btnConfirmChanged) {
    Console::console.printfBroadcast("Confirm changed\n");
    if(buttons[BUTTON_CONFIRM]) btnConfirmPressed();
    else btnConfirmReleased();
    btnConfirmChanged = false;
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
      Console::console.printfBroadcast("Calling confirm short press cb\n");
      cShortPressCB_();
    } else {
      Console::console.printfBroadcast("No confirm short press cb\n");
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