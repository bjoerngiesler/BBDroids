#include "actuator.h"
#include "BB8Servos.h"
#include "BB8Config.h"

BB8Servos BB8Servos::servos;

BB8Servos::BB8Servos() {
  available_ = false;
}

bool BB8Servos::begin() {
  Serial.print("Setting up Servos... ");
  bool retval = true;

  dxl_.setPortProtocolVersion(2.0);
  dxl_.begin(DYNAMIXEL_BPS);
  dxl_.scan();
  for(int i=1; i<=4; i++) {
    if(dxl_.ping(i)) {
      Serial.print("#"); Serial.print(i); Serial.print(": model #"); Serial.print(dxl_.getModelNumber(i)); Serial.print("... ");
      dxl_.setOperatingMode(i, OP_POSITION);
      dxl_.writeControlTableItem(ControlTableItem::SHUTDOWN, i, 0x14); // disable overload shutdown -- potentially dangerous
    } else {
      Serial.print("#"); Serial.print(i); Serial.print(" not found... ");
      retval = false;    
    }
  }

  if(!retval) {
    Serial.println("failed!");
    available_ = false;
    return false;
  }

  Serial.println("success.");
  available_ = true;
  return true;
}

bool BB8Servos::switchTorque(uint8_t servo, bool onoff) {
  if(available_ == false || servo < 1 || servo > 4) return false;
  if(onoff) dxl_.torqueOn(servo);
  else dxl_.torqueOff(servo);
}
  
bool BB8Servos::isTorqueOn(uint8_t servo) {
  if(available_ == false || servo < 1 || servo > 4) return false;
  return dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, servo);
}


void BB8Servos::printStatus() {
  if(!available_) {
    Serial.println("Servo subsystem not started.");
    return;
  }

  for(int i=1; i<=4; i++) {
    Serial.print("Servo #"); Serial.print(i); Serial.print(": ");
    if(dxl_.ping(i)) {
      Serial.print("model #"); Serial.print(dxl_.getModelNumber(i)); Serial.print(", ");
      Serial.print("pos: "); Serial.print(getPresentPosition(i)); Serial.print("°, ");
      Serial.print("hw err: $"); Serial.print(dxl_.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, i), HEX); Serial.print(", ");
      Serial.print("alarm LED field: $"); Serial.print(dxl_.readControlTableItem(ControlTableItem::ALARM_LED, i), HEX); Serial.print(", ");
      Serial.print("alarm shutdown field: $"); Serial.print(dxl_.readControlTableItem(ControlTableItem::SHUTDOWN, i), HEX); Serial.print(", ");
      Serial.print("torque limit: "); Serial.print(dxl_.readControlTableItem(ControlTableItem::TORQUE_LIMIT, i)); Serial.print(", ");
      Serial.print("max torque: "); Serial.print(dxl_.readControlTableItem(ControlTableItem::MAX_TORQUE, i)); Serial.print(", ");
      Serial.print("torque enabled: "); Serial.print(dxl_.readControlTableItem(ControlTableItem::TORQUE_ENABLE, i));
      Serial.println();
    } else {
      Serial.print("#"); Serial.print(i); Serial.print(" not found! ");    
    }
  }
}

bool BB8Servos::moveAllServosToOrigin() {
  if(!available_) return false;

  for(int i=1; i<=4; i++) {
    if(!dxl_.torqueOn(i)) { Serial.print("Couldn't turn on torque on #"); Serial.println(i); }
    if(i!=4) dxl_.setGoalPosition(i, 180.0, UNIT_DEGREE);
  }

  return true;
}

bool BB8Servos::setGoalPosition(uint8_t servo, float goal) {
  if(!available_) return false;

  if(servo<=0 || servo>4) return false;
  if(goal < servolimits[servo-1].min) goal = servolimits[servo-1].min;
  else if(goal > servolimits[servo-1].max) goal = servolimits[servo-1].max;

  dxl_.setGoalPosition(servo, goal, UNIT_DEGREE);
}

float BB8Servos::getPresentPosition(uint8_t servo) {
  if(!available_) return 0.0f;

  if(servo <= 0 || servo > 4) return false;
  return dxl_.getPresentPosition(servo, UNIT_DEGREE);
}