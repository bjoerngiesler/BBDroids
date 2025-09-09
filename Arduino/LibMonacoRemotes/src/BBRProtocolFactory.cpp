#include "BBRProtocolFactory.h"
#include "CommercialBLE/DroidDepot/BBRDroidDepotProtocol.h"
#include "CommercialBLE/Sphero/BBRSpheroProtocol.h"
#include "MCS/ESP/BBRMESPProtocol.h"

using namespace bb;
using namespace rmt;

Protocol* ProtocolFactory::createProtocol(ProtocolType type) {
    switch(type) {
    case MONACO_XBEE:
        Serial.printf("Error creating Protocol: MONACO_XBEE not yet implemented\n");
        return nullptr;
        break;
    
    case MONACO_ESPNOW:
        return new MESPProtocol;
        break;

    case MONACO_UDP:
        Serial.printf("Error creating Protocol: MONACO_UDP not yet implemented\n");
        return nullptr;
        break;

    case MONACO_BLE:
#if CONFIG_IDF_TARGET_ESP32S2
        Serial.printf("Error creating Protocol: ESP32S2 target does not support BLE\n");
        return nullptr;
#else
        Serial.printf("Error creating Protocol: MONACO_UDP not yet implemented\n");
        return nullptr;
#endif
        break;
        
    case SPEKTRUM_DSSS:
        Serial.printf("Error creating Protocol: SPEKTRUM_DSSS protocol not yet implemented\n");
        return nullptr;
        break;

    case SPHERO_BLE:
#if CONFIG_IDF_TARGET_ESP32S2
        Serial.printf("Error creating Protocol: ESP32S2 target does not support BLE\n");
        return nullptr;
#else
        return new SpheroProtocol;
#endif
        break;

    case DROIDDEPOT_BLE:
#if CONFIG_IDF_TARGET_ESP32S2
        Serial.printf("Error creating Protocol: ESP32S2 target does not support BLE\n");
        return nullptr;
#else
        return new DroidDepotProtocol;
#endif
        break;

    default:
        Serial.printf("Error creating Protocol: Unknown protocol %d\n", type);
    }

    return nullptr;
}

bool ProtocolFactory::storeProtocolInfo(Protocol* protocol) {
    return true;
}

bool ProtocolFactory::eraseProtocolInfo() {
    return true;
}
