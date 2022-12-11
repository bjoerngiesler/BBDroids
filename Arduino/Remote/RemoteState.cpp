#include "RemoteState.h"
#include "Config.h"

#include <Arduino.h>

static RemoteState *state = NULL;

void preparePin(int pin, void (*isr)(void)) {
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE);
}

static bool button_right_pinky_ = false;
void brpISR(void) { button_right_pinky_ = !digitalRead(RemoteStatePinButtonRightPinky); }

static bool button_right_index_ = false;
void briISR(void) { button_right_index_ = !digitalRead(RemoteStatePinButtonRightIndex); }

static bool button_top_pcb_left_ = false;
void btplISR(void) { button_top_pcb_left_ = !digitalRead(RemoteStatePinButtonTopPCBLeft); }

static bool button_top_pcb_right_ = false;
void btprISR(void) { button_top_pcb_right_ = !digitalRead(RemoteStatePinButtonTopPCBRight); }

static bool button_top_left_ = false;
static bool button_top_right_ = false;
static bool button_joystick_ = false;
static float joystick_horizontal_ = 0;
static float joystick_vertical_ = 0;    

RemoteState* RemoteState::getSharedInstance() {
  if(state == NULL) state = new RemoteState();
  return state;
}

RemoteState::RemoteState() {
  preparePin(RemoteStatePinButtonRightPinky, brpISR);
  preparePin(RemoteStatePinButtonRightIndex, briISR);
  preparePin(RemoteStatePinButtonTopPCBLeft, btplISR);
  preparePin(RemoteStatePinButtonTopPCBRight, btprISR);

#define PREPARE_PIN(p) { pinMode(p, INPUT); digitalWrite(p, HIGH); }
  PREPARE_PIN(RemoteStatePinButtonTopLeft);
  PREPARE_PIN(RemoteStatePinButtonTopRight);
  PREPARE_PIN(RemoteStatePinJoystickHorizontal);
  PREPARE_PIN(RemoteStatePinJoystickVertical);
  PREPARE_PIN(RemoteStatePinButtonTopLeft);
  PREPARE_PIN(RemoteStatePinRotaryEncoder);
}

void RemoteState::update() {
  button_top_left_ = !digitalRead(RemoteStatePinButtonTopLeft);
  button_top_right_ = !digitalRead(RemoteStatePinButtonTopRight);
  button_joystick_ = !digitalRead(RemoteStatePinButtonJoystick);

  joystick_horizontal_ = (float)(512 - analogRead(RemoteStatePinJoystickHorizontal) + CalibBiasJoystickHorizontal) / 512.0f;
  if(abs(joystick_horizontal_) < JoystickEpsilon) joystick_horizontal_ = 0.0f;
  else if(joystick_horizontal_ > 1.0f) joystick_horizontal_ = 1.0f;
  else if(joystick_horizontal_ < -1.0f) joystick_horizontal_ = -1.0f;
  
  joystick_vertical_ = (float)(analogRead(RemoteStatePinJoystickVertical) - 512 + CalibBiasJoystickVertical) / 512.0f;
  if(abs(joystick_vertical_) < JoystickEpsilon) joystick_vertical_ = 0.0f;
  else if(joystick_vertical_ > 1.0f) joystick_vertical_ = 1.0f;
  else if(joystick_vertical_ < -1.0f) joystick_vertical_ = -1.0f;
 }

bool RemoteState::isTopLeftButtonPressed() { return button_top_left_; }
bool RemoteState::isTopRightButtonPressed() { return button_top_right_; }
bool RemoteState::isRightPinkyButtonPressed() { return button_right_pinky_; }
bool RemoteState::isRightIndexButtonPressed() { return button_right_index_; }
bool RemoteState::isTopPCBLeftButtonPressed() { return button_top_pcb_left_; }
bool RemoteState::isTopPCBRightButtonPressed() { return button_top_pcb_right_; }
bool RemoteState::isJoystickButtonPressed() { return button_joystick_; }
float RemoteState::getJoystickHorizontalAxis() { return joystick_horizontal_; }
float RemoteState::getJoystickVerticalAxis() { return joystick_vertical_; }

void RemoteState::printOnSerial() {
  Serial.print("TL:");
  Serial.print(isTopLeftButtonPressed());
  Serial.print(" TR:");
  Serial.print(isTopRightButtonPressed());
  Serial.print(" RP:");
  Serial.print(isRightPinkyButtonPressed());
  Serial.print(" RI:");
  Serial.print(isRightIndexButtonPressed());
  Serial.print(" PCBL:");
  Serial.print(isTopPCBLeftButtonPressed());
  Serial.print(" PCBR:");
  Serial.print(isTopPCBRightButtonPressed());
  Serial.print(" Joy:");
  Serial.print(isJoystickButtonPressed());

  Serial.print(" X:");
  Serial.print(getJoystickHorizontalAxis(), 5);
  Serial.print(" Y:");
  Serial.println(getJoystickVerticalAxis(), 5);
}

void RemoteState::fillStatePacket(StatePacket& packet) {
  #if 0
  packet.button_top_left_ = button_top_left_;
  packet.button_top_right_ = button_top_right_;
  packet.button_right_pinky_ = button_right_pinky_;
  packet.button_right_index_ = button_right_index_;
  packet.button_top_pcb_left_ = button_top_pcb_left_;
  packet.button_top_pcb_right_ = button_top_pcb_right_;

  packet.button_joystick_ = button_joystick_;
  #endif
  packet.joystick_horizontal_ = joystick_horizontal_;
  packet.joystick_vertical_ = joystick_vertical_;
}

