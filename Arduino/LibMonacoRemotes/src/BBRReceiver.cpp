#include "BBRReceiver.h"

using namespace bb;
using namespace bb::rmt;

static const std::string EMPTY = "";

uint8_t Receiver::numInputs(){
    return inputs_.size();
}

uint8_t Receiver::addInput(const std::string& name, std::function<void(float)> callback) {
    if(inputWithName(name) != INPUT_INVALID) return false;

    Input input;
    input.name = name;
    input.callback = callback;
    inputs_.push_back(input);
    return inputs_.size()-1;
}

uint8_t Receiver::addInput(const std::string& name, float& var) {
    return addInput(name, [&var](float v) { var = v; });
}

const std::string& Receiver::inputName(uint8_t input)
{
    if(input >= inputs_.size()) return EMPTY;
    return inputs_[input].name;
}

uint8_t Receiver::inputWithName(const std::string& name) {
    for(uint8_t i=0; i<inputs_.size(); i++) {
        if(inputs_[i].name == name) return i;
    }
    return INPUT_INVALID;
}
