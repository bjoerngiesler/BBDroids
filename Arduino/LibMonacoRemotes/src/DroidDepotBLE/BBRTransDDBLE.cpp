#include "BBRTransDDBLE.h"
#include <string>
#include <vector>

using namespace bb;
using namespace bb::rmt;

static BLEUUID SERVICE_UUID("09b600A0-3e42-41fc-b474-e9c0c8f0c801");
static BLEUUID NOTIFICATION_UUID("09b600b0-3e42-41fc-b474-e9c0c8f0c801");
static BLEUUID WRITE_UUID("09b600b1-3e42-41fc-b474-e9c0c8f0c801");


static std::vector<std::string> channelNames = {"CH0", "CH1", "CH2", "CH3", "CH4"};
static std::string invalidChannelName = "INVALID";

DroidDepotBLETransmitter::DroidDepotBLETransmitter() {

}

uint8_t DroidDepotBLETransmitter::numChannels() { 
    return channelNames.size(); 
}

const std::string& DroidDepotBLETransmitter::channelName(uint8_t channel) { 
    if(channel >= channelNames.size()) return invalidChannelName;
    return channelNames[channel];
}

uint8_t DroidDepotBLETransmitter::bitDepthForChannel(uint8_t channel) { 
    return 8; 
}

void DroidDepotBLETransmitter::setChannelValue(uint8_t channel, float value, Unit unit) {

}

Result DroidDepotBLETransmitter::transmit() {
    return RES_OK;
}

