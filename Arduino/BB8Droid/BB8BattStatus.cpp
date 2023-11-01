#include "BB8BattStatus.h"
#include "BB8Config.h"
#include <Adafruit_INA219.h>

BB8BattStatus BB8BattStatus::batt;

static Adafruit_INA219 ina1(BATT1_STATUS_ADDR), ina2(BATT2_STATUS_ADDR);
static Adafruit_INA219* inas[2];

BB8BattStatus::BB8BattStatus() {
  for(int i=BATT_1; i<=BATT_2; i++) {
    status_[i].voltage = 0;
    status_[i].current = 0;
    status_[i].available = false;
  }

  inas[BATT_1] = &ina1;
  inas[BATT_2] = &ina2;
}

bool BB8BattStatus::begin() {
  Wire.beginTransmission(BATT1_STATUS_ADDR);
  if(Wire.endTransmission() == 0) {
    ina1.begin();
    status_[0].available = true;
  }
  Wire.beginTransmission(BATT2_STATUS_ADDR);
  if(Wire.endTransmission() == 0) {
    ina2.begin();
    status_[1].available = true;
  }
  return true;
}
  
bool BB8BattStatus::available(BB8BattStatus::Battery batt) {
  if(batt == BATT_BOTH) return available(BATT_1) && available(BATT_2);
  else return status_[batt].available;
}

bool BB8BattStatus::updateVoltage(BB8BattStatus::Battery batt) {
  if(!available(batt)) return false;
  if(batt == BATT_BOTH) return updateVoltage(BATT_1) && updateVoltage(BATT_2);
  else {
#if defined(BATT_VOLTAGE_PRECISE)
    float voltage = inas[batt]->getShuntVoltage_mV() / 1000 + inas[batt]->getBusVoltage_V();
#else 
    float voltage = inas[batt]->getBusVoltage_V();
#endif
    if(isnan(voltage) || !isfinite(voltage)) {
      status_[batt].available = false;
      status_[batt].voltage = -1;
    } else {
      status_[batt].voltage = voltage;
    }
  }
  return true;
}

bool BB8BattStatus::updateCurrent(BB8BattStatus::Battery batt) {
  if(!available(batt)) return false;
  if(batt == BATT_BOTH) return updateCurrent(BATT_1) && updateCurrent(BATT_2);
  else {
    float current = inas[batt]->getCurrent_mA();
    if(isnan(current) || !isfinite(current)) {
        status_[batt].available = false;
        status_[batt].current = -1;
    } else {
      status_[batt].current = current;
    }
  }
  return true;
}

float BB8BattStatus::current(BB8BattStatus::Battery batt) {
  if(batt == BATT_BOTH) return status_[BATT_1].current + status_[BATT_2].current;
  return status_[batt].current;
}

float BB8BattStatus::voltage(BB8BattStatus::Battery batt) {
  if(batt == BATT_BOTH) return min(status_[BATT_1].voltage, status_[BATT_2].voltage);
  return status_[batt].voltage;
}

bb::BatteryState BB8BattStatus::getBatteryState(BB8BattStatus::Battery batt) {
  bb::BatteryState batteryState;
  if(!available(batt)) {
    batteryState.errorState = bb::ERROR_NOT_PRESENT;
    return batteryState;
  }

  batteryState.errorState = bb::ERROR_OK;
  batteryState.voltage = voltage(batt);
  batteryState.current = current(batt);;
  return batteryState;
}
