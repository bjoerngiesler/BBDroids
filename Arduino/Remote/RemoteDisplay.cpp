#include <Arduino.h>
#include "RemoteDisplay.h"
#include "Config.h"

RemoteDisplay::RemoteDisplay() {
  pinMode(PinLEDNeopixel, OUTPUT);
  digitalWrite(PinLEDNeopixel, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  last_millis_ = millis();
  connected_ = false;
  left_led_state_ = false;
  right_led_state_ = false;
}

void RemoteDisplay::update() {
  // blink left LED as long as we are not connected, otherwise switch it on
  unsigned long current_millis = millis();
  if(current_millis - last_millis_ > 500) {
    if(connected_) {
      digitalWrite(LED_BUILTIN, LOW);
      left_led_state_ = true;
    }
    else {
      Serial.println("Toggling LED");
      digitalWrite(LED_BUILTIN, left_led_state_);
      left_led_state_ = !left_led_state_;
    }
    last_millis_ = current_millis;
  }
}

void RemoteDisplay::setConnected(bool conn) {
  connected_ = conn;
}
