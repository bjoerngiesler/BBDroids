#include "BB8ConfigStorage.h"
#include "BB8Config.h"

BB8ConfigStorage BB8ConfigStorage::storage;

FlashStorage(magicStorage, uint32_t);
FlashStorage(wifiParamsStorage, BB8ConfigStorage::WifiParams);
FlashStorage(rollControlParamsStorage, BB8ConfigStorage::RollControlParams);

bool BB8ConfigStorage::begin() {
  Serial.print("Setting up flash storage... ");
  uint32_t magic = magicStorage.read();
  if(magic == 0xbabeface) {
    Serial.print("reading from previous flash... ");
    wifi_ = wifiParamsStorage.read();
    roll_ = rollControlParamsStorage.read();
    Serial.println("success.");
  } else {
    Serial.print("new flash, populating info from defaults... ");
    setWifiParams(WIFI_AP_MODE, WIFI_SSID, WIFI_WPA_KEY);
    setControlParams(ROLL_CONTROLLER, ROLL_CONTROL_KP, ROLL_CONTROL_KI, ROLL_CONTROL_KD, 
                     ROLL_CONTROL_IMAX, ROLL_CONTROL_IABORT, ROLL_CONTROL_DEADBAND);
    Serial.println("success.");
  }
  return true;
}

void BB8ConfigStorage::getWifiParams(bool& ap, String& ssid, String& key) {
  ap = wifi_.ap;
  ssid = wifi_.ssid;
  key = wifi_.key;
}
  
void BB8ConfigStorage::setWifiParams(bool ap, const String& ssid, const String& key) {
  wifi_.ap = ap;

  memset(wifi_.ssid, 0, sizeof(wifi_.ssid));
  if(ssid.length() >= 254) snprintf(wifi_.ssid, 254, ssid.c_str());
  else snprintf(wifi_.ssid, ssid.length()+1, ssid.c_str());
  
  memset(wifi_.key, 0, sizeof(wifi_.key));
  if(key.length() >= 254) snprintf(wifi_.key, 254, key.c_str());
  else snprintf(wifi_.key, key.length()+1, key.c_str());
}

void BB8ConfigStorage::getControlParams(BB8ConfigStorage::Controller c, float& Kp, float& Ki, float& Kd, float& iMax, float& iAbort, float& deadband) {
  if(c == BB8ConfigStorage::ROLL_CONTROLLER) {
    Kp = roll_.Kp;
    Ki = roll_.Ki;
    Kd = roll_.Kd;
    iMax = roll_.iMax;
    iAbort = roll_.iAbort;
    deadband = roll_.deadband;
  } else Serial.println("Unknown controller!");
}
  
void BB8ConfigStorage::setControlParams(BB8ConfigStorage::Controller c, float Kp, float Ki, float Kd, float iMax, float iAbort, float deadband) {
  if(c == BB8ConfigStorage::ROLL_CONTROLLER) {
    roll_.Kp = Kp;
    roll_.Ki = Ki;
    roll_.Kd = Kd;
    roll_.iMax = iMax;
    roll_.iAbort = iAbort;
    roll_.deadband = deadband;
  } else Serial.println("Unknown controller!");
}

bool BB8ConfigStorage::storeFlash() {
  wifiParamsStorage.write(wifi_);
  rollControlParamsStorage.write(roll_);
  magicStorage.write(0xbabeface);
  return true;
}


