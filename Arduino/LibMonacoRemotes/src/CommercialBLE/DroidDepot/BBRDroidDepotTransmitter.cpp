#include <Arduino.h>

#if !CONFIG_IDF_TARGET_ESP32S2

#include "BBRDroidDepotTransmitter.h"
#include <string>
#include <vector>

using namespace bb;
using namespace bb::rmt;

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

DroidDepotTransmitter::DroidDepotTransmitter(DroidDepotProtocol *proto): TransmitterBase<DroidDepotProtocol>(proto) {
    axisValues_ = {127, 127, 127, 127, 127};
}

uint8_t DroidDepotTransmitter::numInputs() {
    return inputNames.size();
}

const std::string& DroidDepotTransmitter::inputName(uint8_t input) {
    return inputNames[input];
}

#endif // !CONFIG_IDF_TARGET_ESP32S2