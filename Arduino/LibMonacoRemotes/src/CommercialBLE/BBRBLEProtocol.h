#if !defined(BBRBLEPROTOCOL_H)
#define BBRBLEPROTOCOL_H

#include <Arduino.h>

#if !CONFIG_IDF_TARGET_ESP32S2

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "BBRProtocol.h"

namespace bb {
namespace rmt {

class BLEProtocol: public Protocol, public BLEAdvertisedDeviceCallbacks, public BLEClientCallbacks {
public:
    BLEProtocol();

    virtual bool init();

    // BLE callbacks
    virtual bool discoverNodes(float timeout = 5);
    virtual void onResult(BLEAdvertisedDevice advertisedDevice);

    virtual bool isAcceptableForDiscovery(BLEAdvertisedDevice advertisedDevice) = 0;

    BLEClient* getClient() { return pClient_; }

    virtual bool requiresConnection() { return true; }
    virtual bool isConnected() { return connected_; }

    // BLEClientCallbacks methods
    void onConnect(BLEClient *pClient);
    void onDisconnect(BLEClient *pClient);

protected:
    BLEScan* pBLEScan_;
    BLEAdvertisedDevice* myDevice_;
    BLERemoteCharacteristic* pRemoteCharacteristic_;

    BLEClient* pClient_;

    bool connected_;
};

};
}; // bb

#endif // !CONFIG_IDF_TARGET_ESP32S2
#endif // BBRBLEPROTOCOL_H