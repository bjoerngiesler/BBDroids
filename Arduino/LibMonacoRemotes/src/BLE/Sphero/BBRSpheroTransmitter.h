#if !defined(BBRSHPEROTRANSMITTER_H)
#define BBRSPHEROTRANSMITTER_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "BBRTransmitter.h"

namespace bb {
namespace rmt {
class SpheroTransmitter: public Transmitter, public BLEClientCallbacks {
public:
    SpheroTransmitter();

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

    struct Command {
        uint8_t byte1, byte2, byte3;
    };

protected:
    bool initialWrites();
    bool sleep(bool onoff);

    void floatToBuf(float f, uint8_t* buf);
    bool transmitCommand(const Command &cmd, uint8_t *payload, uint8_t len);

    std::vector<uint8_t> axisValues_;
    BLEClient *pClient_;
    BLERemoteCharacteristic *pInitialChar_, *pControlChar_;
    bool connected_;
    bool sleeping_;
    uint8_t seqnum_;
};
}; // rmt
}; // bb

#endif // BBRSPHEROTRANSMITTER_H