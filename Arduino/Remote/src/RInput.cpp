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

RInput::RInput(): 
  delegate_(NULL) {
  for(bool& b: buttons) b = false;
}

void RInput::setDelegate(RInput::Delegate* d) {
  delegate_ = d;
}

void RInput::clearDelegate() {
  delegate_ = NULL;
}

bool RInput::begin() {
#if defined(ESP32_REMOTE)
  if(mcp_.begin_I2C(0x27) == 0) {
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
  joyRawV = 4096 - analogRead(P_A_JOY_VER);

  joyH = (float)(hCalib_.center - joyRawH) / 2048.0f;
  if(abs(joyH) < JoystickEpsilon) joyH = 0.0f;
  joyH = constrain(joyH, -1.0f, 1.0f);

  joyV = (float)(vCalib_.center - joyRawV) / 2048.0f;
  if(abs(joyV) < JoystickEpsilon) joyV = 0.0f;
  joyV = constrain(joyV, -1.0f, 1.0f);
  
  battery = (float)(analogRead(P_A_BATT_CHECK) / 4096.0f);
  pot1 = (float)(analogRead(P_A_POT1) / 4096.0f); // careful - wraps around on the left remote
#if !defined(LEFT_REMOTE)
  pot2 = (float)(analogRead(P_A_POT2) / 4096.0f);
#else
  pot2 = 0;
#endif

#if defined(ESP32_REMOTE) // ESP reads buttons from the MCP expander. Non-ESP does it from the interrupt routine block above.
  if(mcpOK_) {
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
  }
#endif // ESP32_REMOTE
  if(delegate_ != NULL) {
    if(btnTopLChanged) {
      if(buttons[BUTTON_TOP_LEFT]) delegate_->buttonTopLeftPressed();
      else delegate_->buttonTopLeftReleased();
      btnTopLChanged = false;
    }
    if(btnTopRChanged) {
      if(buttons[BUTTON_TOP_RIGHT]) delegate_->buttonTopRightPressed();
      else delegate_->buttonTopRightReleased();
      btnTopRChanged = false;
    }
    if(btnConfirmChanged) {
      if(buttons[BUTTON_CONFIRM]) delegate_->buttonConfirmPressed();
      else delegate_->buttonConfirmReleased();
      btnConfirmChanged = false;
    }
  }
}

bool RInput::anyButtonPressed() {
  for(bool& b: buttons) if(b == true) return true;
  return false;
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