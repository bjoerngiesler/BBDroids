#include "BBRAxisInputManager.h"
#include "BBRTypes.h"

using namespace bb;
using namespace bb::rmt;

static AxisInputMapping InvalidMapping;

uint8_t AxisInputManager::inputWithName(const std::string& name) {
    for(int i=0; i<numInputs(); i++) {
        if(inputName(i) == name) return i;
    }
    return INPUT_INVALID;
}

uint8_t AxisInputManager::numMappings() { 
    return axisInputMappings_.size(); 
}

const AxisInputMapping& AxisInputManager::mappingForInput(uint8_t input) { 
    for(auto& mapping: axisInputMappings_) {
        if(mapping.input == input) return mapping;
    }
    return InvalidMapping;
}

bool AxisInputManager::addMapping(const AxisInputMapping& mapping) { 
    if(hasMappingForInput(mapping.input) || mapping.input == INPUT_INVALID) return false;
    axisInputMappings_.push_back(mapping);
    return true;
}

void AxisInputManager::clearMappings() { 
    axisInputMappings_.clear(); 
}

bool AxisInputManager::hasMappingForInput(uint8_t input) {
    for(auto& mapping: axisInputMappings_) {
        if(mapping.input == input) return true;
    }
    return false;
}

