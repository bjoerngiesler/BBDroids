#include "BBRDroidDepotTransmitter.h"
#include <string>
#include <vector>

using namespace bb;
using namespace bb::rmt;

static BLEUUID SERVICE_UUID("09b600A0-3e42-41fc-b474-e9c0c8f0c801");
static BLEUUID NOTIFICATION_UUID("09b600b0-3e42-41fc-b474-e9c0c8f0c801");
static BLEUUID WRITE_UUID("09b600b1-3e42-41fc-b474-e9c0c8f0c801");

#define BITDEPTH 8
#define MAXVAL ((1<<BITDEPTH)-1)

static std::vector<std::string> axisNames = {"CH0", "CH1", "CH2", "CH3", "CH4"};
static std::vector<std::string> inputNames = {"speed", "turn", "dome", "sound", "accessory"};
enum Inputs {
    INPUT_SPEED     = 0,
    INPUT_TURN      = 1,
    INPUT_DOME      = 2,
    INPUT_SOUND     = 3,
    INPUT_ACCESSORY = 4
};

static std::string invalidAxisName = "INVALID";

DroidDepotTransmitter::DroidDepotTransmitter() {
    axisValues_ = {127, 127, 127, 127, 127};
    pClient_ = nullptr;
    pCharacteristic_ = nullptr;

    addAxisInputMapping(AxisInputMapping(INPUT_SPEED, 0));
    addAxisInputMapping(AxisInputMapping(INPUT_TURN, 1));
}

void DroidDepotTransmitter::setRawAxisValue(uint8_t axis, uint32_t value) {
    if(axis >= axisValues_.size()) return;
    value = constrain(value, 0, MAXVAL);
    axisValues_[axis] = uint8_t(value);
}

uint32_t DroidDepotTransmitter::rawAxisValue(uint8_t axis) {
    if(axis >= axisValues_.size()) return 0;
    return axisValues_[axis];
}

void DroidDepotTransmitter::onConnect(BLEClient *pClient) {
    Serial.printf("Connected\n");
    connected_ = true;
}

void DroidDepotTransmitter::onDisconnect(BLEClient *pClient) {
    Serial.printf("Disconnected\n");
    connected_ = false;
}

uint8_t DroidDepotTransmitter::numAxes() { 
    return axisNames.size(); 
}

const std::string& DroidDepotTransmitter::axisName(uint8_t axis) { 
    if(axis >= axisNames.size()) return invalidAxisName;
    return axisNames[axis];
}

uint8_t DroidDepotTransmitter::bitDepthForAxis(uint8_t axis) { 
    return 8; 
}

uint8_t DroidDepotTransmitter::numInputs() {
    return inputNames.size();
}

const std::string& DroidDepotTransmitter::inputName(uint8_t input) {
    return inputNames[input];
}

bool DroidDepotTransmitter::pairWith(const NodeAddr& addr) {
    if(isPaired(addr)) {
        Serial.printf("Already paired with %s\n", addr.toString().c_str());
        return false;
    }

    if(pClient_ == nullptr) {
        pClient_ = BLEDevice::createClient();
        Serial.printf(" - Created client\n");

        pClient_->setClientCallbacks(this);
    }

    if(connect(addr) == false) {
        Serial.printf("Could not connect\n");
        return false;
    }
    
    pairedNodes_.push_back(addr);

    return true;
}

Result DroidDepotTransmitter::transmit() {
    return RES_OK;
}

bool DroidDepotTransmitter::isConnected() {
    return connected_;
}

bool DroidDepotTransmitter::connect(const NodeAddr& addr) {
    // Connect to the remote BLE Server.
    Serial.printf("Connecting to %s\n", addr.toString().c_str());
    pClient_->connect(BLEAddress(addr.toString()), esp_ble_addr_type_t(1));
    Serial.printf(" - Connected to server\n");
    pClient_->setMTU(46); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient_->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) {
      Serial.printf("Failed to find our service UUID: %s\n", SERVICE_UUID.toString().c_str());
      pClient_->disconnect();
      return false;
    }
    Serial.printf(" - Found our service\n");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pCharacteristic_ = pRemoteService->getCharacteristic(WRITE_UUID);
    if (pCharacteristic_ == nullptr) {
      Serial.printf("Failed to find our characteristic UUID: %s\n", WRITE_UUID.toString().c_str());
      pClient_->disconnect();
      return false;
    }
    Serial.printf(" - Found our characteristic\n");

    initialWrites();

    return true;
}

void DroidDepotTransmitter::initialWrites() {
    if(pCharacteristic_ == nullptr) {
        Serial.printf("Characteristic is NULL\n");
        return;
    }

    Serial.printf("Doing initial writes\n");
    for(int i=0; i<4; i++) {
        pCharacteristic_->writeValue({0x22, 0x20, 0x01}, 3);
        delay(500);
    }
    for(int i=0; i<2; i++) {
        pCharacteristic_->writeValue({0x27, 0x42, 0x0f, 0x44, 0x44, 0x00, 0x1f, 0x00}, 8);
        delay(500);
        pCharacteristic_->writeValue({0x27, 0x42, 0x0f, 0x44, 0x44, 0x00, 0x18, 0x02}, 8);
        delay(500);
    }
    Serial.printf("Initial writes done\n");
}
