#if !defined(BBRFACTORY_H)
#define BBRFACTORY_H

#include "BBRProtocol.h"
#include <string>

namespace bb {
namespace rmt {

enum ProtocolType {
    MONACO_XBEE,
    MONACO_ESPNOW,
    MONACO_BLE,
    MONACO_UDP,
    SPHERO_BLE,
    DROIDDEPOT_BLE,
    SPEKTRUM_DSSS
};

class ProtocolFactory {
public:
    static Protocol* loadOrCreateProtocol(ProtocolType type);
    static Protocol* createProtocol(ProtocolType type);
    static bool storeProtocolInfo(Protocol* proto);
    static bool eraseProtocolInfo();
};

}; // rmt
}; // bb

#endif // BBRFACTORY_H