#include <Arduino.h>

#if !CONFIG_IDF_TARGET_ESP32S2

#include "BBRSpheroProtocol.h"
#include "BBRSpheroTransmitter.h"

using namespace bb;
using namespace bb::rmt;

static BLEUUID INITIAL_SVC_UUID("00020001-574f-4f20-5370-6865726f2121");
static BLEUUID INITIAL_CHR_UUID("00020005-574f-4f20-5370-6865726f2121");
static BLEUUID CONTROL_SVC_UUID("00010001-574f-4f20-5370-6865726f2121");
static BLEUUID CONTROL_CHR_UUID("00010002-574f-4f20-5370-6865726f2121");

static const uint8_t ESC = 0xAB;
static const uint8_t SOP = 0x8D;
static const uint8_t EOP = 0xD8;
static const uint8_t ESC_ESC = 0x23;
static const uint8_t ESC_SOP = 0x05;
static const uint8_t ESC_EOP = 0x50;

SpheroProtocol::SpheroProtocol() {
    seqnum_ = 0;
    pInitialChar_ = nullptr;
    pControlChar_ = nullptr;
}

uint8_t SpheroProtocol::numTransmitterTypes() { return 1; }

Transmitter* SpheroProtocol::createTransmitter(uint8_t transmitterType) {
    if(transmitter_ == nullptr) transmitter_ = new SpheroTransmitter(this);
    return transmitter_;
}

bool SpheroProtocol::isAcceptableForDiscovery(BLEAdvertisedDevice advertisedDevice) {
    if(advertisedDevice.getName().rfind("D2-", 0) == 0) {
        Serial.printf("Found device with name \"%s\"\n", advertisedDevice.getName().c_str());
        return true;
    }
    return false;
}

bool SpheroProtocol::pairWith(const NodeDescription& descr) {
    if(isPaired(descr.addr)) {
        Serial.printf("Already paired with %s\n", descr.addr.toString().c_str());
        return false;
    }

    if(pClient_ == nullptr) {
        pClient_ = BLEDevice::createClient();
        Serial.printf(" - Created client\n");
        pClient_->setClientCallbacks(this);
    }

    if(connect(descr.addr) == false) {
        Serial.printf("Could not connect\n");
        return false;
    }
    
    return Protocol::pairWith(descr);
}

bool SpheroProtocol::connect() {
    if(pairedNodes_.size() == 0) {
        Serial.printf("No paired receivers -- cannot connect!\n");
        return false;
    }
    return connect(pairedNodes_[0].addr);
}

bool SpheroProtocol::connect(const NodeAddr& addr) {
    if(pClient_ == nullptr) {
        pClient_ = BLEDevice::createClient();
        Serial.printf(" - Created client\n");
        pClient_->setClientCallbacks(this);
    }
    
    // Connect to the remove BLE Server.
    Serial.printf("Connecting to %s\n", addr.toString().c_str());
    pClient_->connect(BLEAddress(addr.toString()), esp_ble_addr_type_t(1));
    Serial.printf(" - Connected to server\n");
    pClient_->setMTU(46); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient_->getService(INITIAL_SVC_UUID);
    if (pRemoteService == nullptr) {
      Serial.printf("Failed to find initial service UUID: %s\n", INITIAL_SVC_UUID.toString().c_str());
      pClient_->disconnect();
      return false;
    }
    Serial.printf(" - Found initial service\n");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pInitialChar_ = pRemoteService->getCharacteristic(INITIAL_CHR_UUID);
    if (pInitialChar_ == nullptr) {
      Serial.printf("Failed to find initial writes UUID: %s\n", INITIAL_CHR_UUID.toString().c_str());
      pClient_->disconnect();
      return false;
    }
    Serial.printf(" - Found initial characteristic\n");

    pRemoteService = pClient_->getService(CONTROL_SVC_UUID);
    if (pRemoteService == nullptr) {
      Serial.printf("Failed to find control service UUID: %s\n", CONTROL_SVC_UUID.toString().c_str());
      pClient_->disconnect();
      return false;
    }
    Serial.printf(" - Found control service\n");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pControlChar_ = pRemoteService->getCharacteristic(CONTROL_CHR_UUID);
    if (pControlChar_ == nullptr) {
      Serial.printf("Failed to find control UUID: %s\n", CONTROL_CHR_UUID.toString().c_str());
      pClient_->disconnect();
      return false;
    }
    Serial.printf(" - Found control characteristic\n");

    Serial.printf(" - Doing initial writes... ");
    initialWrites();
    Serial.printf("done.\n");

    Serial.printf(" - Waking up from sleep... ");
    sleep(false);
    Serial.printf("done.\n");

    return true;
}

bool SpheroProtocol::initialWrites() {
    if(pInitialChar_ == nullptr) {
        Serial.printf("Initial characteristic is NULL\n");
        return false;
    }

    const char *initial = "usetheforce...band";
    pInitialChar_->writeValue(initial, strlen(initial));
    return true;
}

bool SpheroProtocol::transmitCommand(const Command& cmd, uint8_t *payload, uint8_t len) {
    if(pControlChar_ == nullptr) return false;
    uint8_t buf[255], escapedBuf[255];
    uint8_t i=0, j=0, checksum=0;

    // fill buf
    buf[i++] = cmd.byte1;
    buf[i++] = cmd.byte2;
    buf[i++] = cmd.byte3;
    buf[i++] = seqnum_++;
    for(int ii=0; ii<len; ii++) buf[i++] = payload[ii];

    // Serial.printf("%d bytes in original buffer\n", i);

    // escape buf
    escapedBuf[j++] = SOP;
    for(int ii=0; ii<i; ii++) {
        switch(buf[ii]) {
        case ESC:
            escapedBuf[j++] = ESC;
            checksum += ESC;
            escapedBuf[j++] = ESC_ESC;
            checksum += ESC_ESC;
            break;
        case SOP:
            escapedBuf[j++] = ESC;
            checksum += ESC;
            escapedBuf[j++] = ESC_SOP;
            checksum += ESC_SOP;
            break;
        case EOP:
            escapedBuf[j++] = ESC;
            checksum += ESC;
            escapedBuf[j++] = ESC_EOP;
            checksum += ESC_EOP;
            break;
        default:
            escapedBuf[j++] = buf[ii];
            checksum += buf[ii];
            break;
        }
    }
    escapedBuf[j++] = checksum ^ 0xff;
    escapedBuf[j++] = EOP;

    // Serial.printf("Sending %d bytes: ", j);
    // for(uint8_t ii=0; ii<j; ii++) Serial.printf("0x%02x ", escapedBuf[ii]);
    // Serial.printf("\n");

    pControlChar_->writeValue(escapedBuf, j);
    return true;
}

#endif // !CONFIG_IDF_TARGET_ESP32S2