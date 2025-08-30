#include "BBRSpheroTransmitter.h"
#include <string>
#include <vector>
#include <map>

using namespace bb;
using namespace bb::rmt;

static BLEUUID INITIAL_SVC_UUID("00020001-574f-4f20-5370-6865726f2121");
static BLEUUID INITIAL_CHR_UUID("00020005-574f-4f20-5370-6865726f2121");
static BLEUUID CONTROL_SVC_UUID("00010001-574f-4f20-5370-6865726f2121");
static BLEUUID CONTROL_CHR_UUID("00010002-574f-4f20-5370-6865726f2121");

static const SpheroTransmitter::Command CMD_WAKEUP = {0x0a, 0x13, 0x0d}; // no args
static const SpheroTransmitter::Command CMD_SLEEP = {0x0a, 0x13, 0x01};  // no args
static const SpheroTransmitter::Command CMD_MOVE = {0x0a, 0x16, 0x07};   // 6 bytes: uint8_t speed, float32 degree, 0x00
static const SpheroTransmitter::Command CMD_ANIM = {0x0a, 0x17, 0x05};   // 2 bytes: 0x00, num
static const SpheroTransmitter::Command CMD_232 = {0x0a, 0x17, 0x0d};    // 0x01 -- 3-leg mode, 0x02 -- 2-leg mode
static const SpheroTransmitter::Command CMD_DOME = {0x0a, 0x17, 0x0f};   // 4 bytes: float32 degree
static const SpheroTransmitter::Command CMD_HOLO = {0x0a, 0x1a, 0x0e};   // 3 bytes: 0x00, 0x80, intensity

static const uint8_t ESC = 0xAB;
static const uint8_t SOP = 0x8D;
static const uint8_t EOP = 0xD8;
static const uint8_t ESC_ESC = 0x23;
static const uint8_t ESC_SOP = 0x05;
static const uint8_t ESC_EOP = 0x50;

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

SpheroTransmitter::SpheroTransmitter() {
    axisValues_ = {127, 127, 127, 127, 127};
    pClient_ = nullptr;
    pInitialChar_ = nullptr;
    pControlChar_ = nullptr;
    seqnum_ = 0;

    sleeping_ = true;
    connected_ = false;

    addAxisInputMapping(AxisInputMapping(INPUT_SPEED, 0));
    addAxisInputMapping(AxisInputMapping(INPUT_TURN, 1));
    addAxisInputMapping(AxisInputMapping(INPUT_DOME, 2, AxisInputMapping::INTERP_LIN_CENTERED_INV));
}

void SpheroTransmitter::onConnect(BLEClient *pClient) {
    Serial.printf("Connected\n");
    connected_ = true;
}

void SpheroTransmitter::onDisconnect(BLEClient *pClient) {
    Serial.printf("Disconnected\n");
    connected_ = false;
}

uint8_t SpheroTransmitter::numAxes() { 
    return axisNames.size(); 
}

const std::string& SpheroTransmitter::axisName(uint8_t axis) { 
    if(axis >= axisNames.size()) return invalidAxisName;
    return axisNames[axis];
}

uint8_t SpheroTransmitter::bitDepthForAxis(uint8_t axis) { 
    return 8; 
}

uint8_t SpheroTransmitter::numInputs() {
    return inputNames.size();
}

const std::string& SpheroTransmitter::inputName(uint8_t input) {
    return inputNames[input];
}

bool SpheroTransmitter::pairWith(const NodeAddr& addr) {
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

void SpheroTransmitter::setRawAxisValue(uint8_t axis, uint32_t value) {
    if(axis >= axisValues_.size()) return;
    value = constrain(value, 0, MAXVAL);
    axisValues_[axis] = uint8_t(value);
}

uint32_t SpheroTransmitter::rawAxisValue(uint8_t axis) {
    if(axis >= axisValues_.size()) return 0;
    return axisValues_[axis];
}

Result SpheroTransmitter::transmit() {
    float dome = computeInputValue(INPUT_DOME) * 180.0f;
    uint8_t buf[4];
    floatToBuf(dome, buf);
    transmitCommand(CMD_DOME, buf, 4);
    return RES_OK;
}

bool SpheroTransmitter::isConnected() {
    return connected_;
}

bool SpheroTransmitter::connect(const NodeAddr& addr) {
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

bool SpheroTransmitter::sleep(bool onoff) {
    if(pControlChar_ == nullptr) {
        Serial.printf("Control characteristic is NULL\n");
        return false;
    }
    if(onoff) {
        transmitCommand(CMD_SLEEP, nullptr, 0);
        sleeping_ = true;
    } else {
        transmitCommand(CMD_WAKEUP, nullptr, 0);
        sleeping_ = false;
    }
    
    return true;
}

bool SpheroTransmitter::initialWrites() {
    if(pInitialChar_ == nullptr) {
        Serial.printf("Initial characteristic is NULL\n");
        return false;
    }

    const char *initial = "usetheforce...band";
    pInitialChar_->writeValue(initial, strlen(initial));
    return true;
}

void SpheroTransmitter::floatToBuf(float f, uint8_t *buf) {
    buf[0] = ((uint8_t*)&f)[3];
    buf[1] = ((uint8_t*)&f)[2];
    buf[2] = ((uint8_t*)&f)[1];
    buf[3] = ((uint8_t*)&f)[0];
}

bool SpheroTransmitter::transmitCommand(const Command& cmd, uint8_t *payload, uint8_t len) {
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
