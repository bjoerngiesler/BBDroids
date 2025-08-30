#if !defined(BBRBLEPROTOCOL_H)
#define BBRBLEPROTOCOL_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "BBRProtocol.h"

namespace bb {
namespace rmt {

class BLEProtocol: public Protocol, public BLEAdvertisedDeviceCallbacks {
public:
    BLEProtocol();

    virtual bool init();

    // BLE callbacks
    virtual Result discoverNodes(float timeout = 5);
    virtual void onResult(BLEAdvertisedDevice advertisedDevice);

    virtual bool isAcceptableForDiscovery(BLEAdvertisedDevice advertisedDevice) = 0;

protected:
    BLEScan* pBLEScan_;
    BLEAdvertisedDevice* myDevice_;
    BLERemoteCharacteristic* pRemoteCharacteristic_;
};

};
}; // bb

#endif // BBRBLEPROTOCOL_H