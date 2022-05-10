#include "RemoteState.h"
#include "Config.h"

#include <Arduino.h>

RemoteState::RemoteState() {
#define PREPARE_PIN(p) { pinMode(p, INPUT); digitalWrite(p, HIGH); }
  
  PREPARE_PIN(RemoteStatePinButtonTopLeft);
  PREPARE_PIN(RemoteStatePinButtonTopRight);
  PREPARE_PIN(RemoteStatePinButtonRightPinky);
  PREPARE_PIN(RemoteStatePinButtonRightIndex);
  PREPARE_PIN(RemoteStatePinJoystickHorizontal);
  PREPARE_PIN(RemoteStatePinJoystickVertical);
  PREPARE_PIN(RemoteStatePinJoystickButton);
  PREPARE_PIN(RemoteStatePinButtonTopPCBLeft);
  PREPARE_PIN(RemoteStatePinButtonTopPCBRight);
  PREPARE_PIN(RemoteStatePinRotaryEncoder);

  PREPARE_PIN(RemoteStatePinJoystickHorizontal);
  PREPARE_PIN(RemoteStatePinJoystickVertical);
}

void RemoteState::update() {
  button_top_left_ = !digitalRead(RemoteStatePinButtonTopLeft);
  button_top_right_ = !digitalRead(RemoteStatePinButtonTopRight);
  button_right_pinky_ = !digitalRead(RemoteStatePinButtonRightPinky);
  button_right_index_ = !digitalRead(RemoteStatePinButtonRightIndex);
  button_top_pcb_left_ = !digitalRead(RemoteStatePinButtonTopPCBLeft);
  button_top_pcb_right_ = !digitalRead(RemoteStatePinButtonTopPCBRight);

  button_joystick_ = !digitalRead(RemoteStatePinJoystickButton);

  joystick_horizontal_ = 1024 - analogRead(RemoteStatePinJoystickHorizontal);
  if(CalibBiasJoystickHorizontal < 0) {
    if(joystick_horizontal_ > -CalibBiasJoystickHorizontal) joystick_horizontal_ += CalibBiasJoystickHorizontal;
  } else {
    if(1024 - joystick_horizontal_ > CalibBiasJoystickHorizontal) joystick_horizontal_ += CalibBiasJoystickHorizontal;
  }
  
  joystick_vertical_ = analogRead(RemoteStatePinJoystickVertical);
  if(CalibBiasJoystickVertical < 0) {
    if(joystick_vertical_ > -CalibBiasJoystickVertical) joystick_vertical_ += CalibBiasJoystickVertical;
  } else {
    if(1024 - joystick_vertical_ > CalibBiasJoystickVertical) joystick_vertical_ += CalibBiasJoystickVertical;
  }
}

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
  Serial.print(getJoystickHorizontalAxis());
  Serial.print(" Y:");
  Serial.println(getJoystickVerticalAxis());
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
