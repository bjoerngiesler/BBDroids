#include "RemoteInput.h"
#include "Config.h"

RemoteInput RemoteInput::input;

static void prepareInterruptPin(int pin, void (*isr)(void)) {
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE);
}

void bPinkyISR(void) { RemoteInput::input.btnPinky = !digitalRead(P_D_BTN_PINKY); }
void bIndexISR(void) { RemoteInput::input.btnIndex = !digitalRead(P_D_BTN_INDEX); }
void bJoyISR(void) { RemoteInput::input.btnJoy = !digitalRead(P_D_BTN_JOY); }
void bLISR(void) { RemoteInput::input.btnL = !digitalRead(P_D_BTN_L); }
void bRISR(void) { RemoteInput::input.btnR = !digitalRead(P_D_BTN_R); }
void bConfirmISR(void) { 
  RemoteInput::input.btnConfirm = !digitalRead(P_D_BTN_CONFIRM); 
  RemoteInput::input.btnConfirmChanged = true;
}
void bTopLISR(void) { 
  RemoteInput::input.btnTopL = !digitalRead(P_D_BTN_TOP_L); 
  RemoteInput::input.btnTopLChanged = true;
}
void bTopRISR(void) { 
  RemoteInput::input.btnTopR = !digitalRead(P_D_BTN_TOP_R); 
  RemoteInput::input.btnTopRChanged = true;
}

RemoteInput::RemoteInput(): delegate_(NULL) {
}

void RemoteInput::setDelegate(RemoteInput::Delegate *d) {
  delegate_ = d;
}


bool RemoteInput::begin() {
  prepareInterruptPin(P_D_BTN_PINKY, bPinkyISR);
  prepareInterruptPin(P_D_BTN_INDEX, bIndexISR);
  prepareInterruptPin(P_D_BTN_JOY, bJoyISR);
  prepareInterruptPin(P_D_BTN_L, bLISR);
  prepareInterruptPin(P_D_BTN_R, bRISR);
  prepareInterruptPin(P_D_BTN_CONFIRM, bConfirmISR);
  prepareInterruptPin(P_D_BTN_TOP_L, bTopLISR);
  prepareInterruptPin(P_D_BTN_TOP_R, bTopRISR);

#if 0
#if defined(LEFT_REMOTE)
  pinMode(P_A_ENC, INPUT_PULLUP);
#else
  pinMode(P_A_POT1, INPUT_PULLUP);
  pinMode(P_A_POT2, INPUT_PULLUP);
#endif

  pinMode(P_A_BATT_CHECK, INPUT_PULLUP);
  pinMode(P_A_JOY_HOR, INPUT_PULLUP);
  pinMode(P_A_JOY_VER, INPUT_PULLUP);
#endif

  return true;
}

void RemoteInput::update() {
  joyH = (float)(analogRead(P_A_JOY_HOR) - 512 + CalibBiasJoystickHorizontal) / 512.0f;
  if(abs(joyH) < JoystickEpsilon) joyH = 0.0f;
  else if(joyH > 1.0f) joyH = 1.0f;
  else if(joyH < -1.0f) joyH = -1.0f;
  if(joyH < 0) joyH = -(joyH * joyH);
  else joyH = joyH * joyH;
  
  joyV = (float)(analogRead(P_A_JOY_VER) - 512 + CalibBiasJoystickVertical) / 512.0f;
  if(abs(joyV) < JoystickEpsilon) joyV = 0.0f;
  else if(joyV > 1.0f) joyV = 1.0f;
  else if(joyV < -1.0f) joyV = -1.0f;
  if(joyV < 0) joyV = -(joyV * joyV);
  else joyV = joyV * joyV;

  battery = (float)(analogRead(P_A_BATT_CHECK) / 1024.0f);
#if !defined(LEFT_REMOTE)
  pot1 = (float)(analogRead(P_A_POT1) / 1024.0f);
  pot2 = (float)(analogRead(P_A_POT2) / 1024.0f);
#else
  pot1 = pot2 = 0;
#endif

  if(delegate_ != NULL) {
    if(btnTopLChanged) {
      if(btnTopL) delegate_->buttonTopLeftPressed();
      else delegate_->buttonTopLeftReleased();
      btnTopLChanged = false;
    }
    if(btnTopRChanged) {
      if(btnTopR) delegate_->buttonTopRightPressed();
      else delegate_->buttonTopRightReleased();
      btnTopRChanged = false;
    }
    if(btnConfirmChanged) {
      if(btnTopL) delegate_->buttonConfirmPressed();
      else delegate_->buttonConfirmReleased();
      btnConfirmChanged = false;
    }
  }
}

bool RemoteInput::anyButtonPressed() {
  return btnPinky || btnIndex || btnJoy || btnL || btnR || btnConfirm || btnTopL || btnTopR;
}

void RemoteInput::printOnSerial() {
  Serial.print("L:");
  Serial.print(btnL);
  Serial.print(" R:");
  Serial.print(btnR);
  Serial.print(" P:");
  Serial.print(btnPinky);
  Serial.print(" I:");
  Serial.print(btnIndex);
  Serial.print(" TL:");
  Serial.print(btnTopL);
  Serial.print(" TR:");
  Serial.print(btnTopR);
  Serial.print(" Joy:");
  Serial.print(btnJoy);
  Serial.print(" Cf:");
  Serial.print(btnConfirm);

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