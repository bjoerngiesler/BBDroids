#include "BBRAxisInputManager.h"
#include "BBRTypes.h"

using namespace bb;
using namespace bb::rmt;

static AxisMix InvalidMix;

uint8_t AxisInputManager::inputWithName(const std::string& name) {
    for(int i=0; i<numInputs(); i++) {
        if(inputName(i) == name) return i;
    }
    return INPUT_INVALID;
}

uint8_t AxisInputManager::numMixes() { 
    return mixes_.size(); 
}

const AxisMix& AxisInputManager::mixForInput(uint8_t input) { 
    for(auto& mix: mixes_) {
        if(mix.first == input) return mix.second;
    }
    return InvalidMix;
}

bool AxisInputManager::addMix(uint8_t input, const AxisMix& mix) { 
    if(hasMixForInput(input) || input == INPUT_INVALID) return false;
    mixes_[input] = mix;
    return true;
}

void AxisInputManager::clearMixes() { 
    mixes_.clear(); 
}

bool AxisInputManager::hasMixForInput(uint8_t input) {
    for(auto& mix: mixes_) {
        if(mix.first == input) return true;
    }
    return false;
}

