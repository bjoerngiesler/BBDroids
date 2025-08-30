#if !defined(BBRDROIDDEPOTTRANSMITTER_H)
#define BBRDROIDDEPOTTRANSMITTER_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "BBRTransmitter.h"

namespace bb {
namespace rmt {
class DroidDepotTransmitter: public Transmitter, public BLEClientCallbacks {
public:
    DroidDepotTransmitter();

    virtual uint8_t numAxes();
    virtual const std::string& axisName(uint8_t axis);
    virtual uint8_t bitDepthForAxis(uint8_t axis);

    virtual uint8_t numInputs();
    virtual const std::string& inputName(uint8_t input);

    virtual bool pairWith(const NodeAddr& node);

    virtual void setRawAxisValue(uint8_t axis, uint32_t value);
    virtual uint32_t rawAxisValue(uint8_t axis);
    virtual Result transmit();

    virtual bool receiverSideAxisMapping() { return false; }
    virtual bool syncReceiverSideAxisMapping() { return false; }

    virtual bool requiresConnection() { return true; }
    virtual bool isConnected();
    virtual bool connect(const NodeAddr& addr);

    // BLEClientCallbacks methods
    void onConnect(BLEClient *pClient);
    void onDisconnect(BLEClient *pClient);
    
protected:
    void initialWrites();

    std::vector<uint8_t> axisValues_;
    BLEClient *pClient_;
    BLERemoteCharacteristic *pCharacteristic_;
    bool connected_;
};
}; // rmt
}; // bb

#endif // BBRDROIDDEPOTTRANSMITTER_H