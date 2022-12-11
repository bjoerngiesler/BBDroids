#if !defined(BB8CONFIGSTORAGE_H)
#define BB8CONFIGSTORAGE_H

#include <FlashStorage.h>

class BB8ConfigStorage {
public:
  static BB8ConfigStorage storage;

  typedef enum {
    ROLL_CONTROLLER
  } Controller;

  bool begin();

  void getWifiParams(bool& ap, String& ssid, String& key);
  void setWifiParams(bool ap, const String& ssid, const String& key);

  void getControlParams(Controller c, float& Kp, float& Ki, float& Kd, float& iMax, float& iAbort, float& deadband);
  void setControlParams(Controller c, float Kp, float Ki, float Kd, float iMax, float iAbort, float deadband);

  bool storeFlash();

  typedef struct {
    bool ap;
    char ssid[255];
    char key[255];
  } WifiParams;

  typedef struct {
    float Kp, Ki, Kd;
    float iMax, iAbort;
    float deadband;
  } RollControlParams;

protected:
  WifiParams wifi_;
  RollControlParams roll_;
};

#endif // BB8CONFIGSTORAGE_H