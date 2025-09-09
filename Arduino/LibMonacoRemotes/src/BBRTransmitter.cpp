#include "BBRTransmitter.h"

using namespace bb;
using namespace bb::rmt;

static std::string InvalidString = "INVALID";

uint8_t Transmitter::numAxes() {
    return axes_.size();
}

const std::string& Transmitter::axisName(uint8_t axis) {
    if(axis >= axes_.size()) return InvalidString;
    return axes_[axis].name;
}

bool Transmitter::setAxisName(uint8_t axis, const std::string& name) {
    if(axis >= axes_.size()) return false;
    axes_[axis].name = name;
    return true;
}

uint8_t Transmitter::axisWithName(const std::string& name, bool add, uint8_t bitDepth) {
    for(int i=0; i<numAxes(); i++) {
        if(axisName(i) == name) return i;
    }

    if(add == false || canAddAxes() == false) {
        return AXIS_INVALID;
    }

    return addAxis(name, bitDepth);
}

uint8_t Transmitter::bitDepthForAxis(uint8_t axis) {
    if(axis >= axes_.size()) return 0;
    return axes_[axis].bitDepth; 
}

uint8_t Transmitter::addAxis(const std::string& name, uint8_t bitDepth) {
    if(canAddAxes() == false) return AXIS_INVALID;
    Axis axis = {name, bitDepth, 0};
    axes_.push_back(axis);
    return axes_.size()-1;
}

bool Transmitter::setAxisValue(uint8_t axis, float value, Unit unit) {
    if(axis >= axes_.size()) return false;
    uint32_t maxval = (1<<bitDepthForAxis(axis))-1;
    switch(unit) {
    case UNIT_DEGREES:
        value = (value/360.0f) * maxval;
        break;
    case UNIT_DEGREES_CENTERED:
        value = ((value+180.0f)/360.0f) * maxval;
        break;
    case UNIT_UNITY_CENTERED:
        value = ((value + 1.0f)/2.0f) * maxval;
        break;
    case UNIT_UNITY:
        value = value * maxval;
        break;
    case UNIT_RAW:
        break;
    default:
        break;
    }

    value = constrain(value, 0.0f, maxval);
    //Serial.printf("setAxisValue: %f\n", value);
    setRawAxisValue(axis, value);
    return true;
}

float Transmitter::axisValue(uint8_t axis, Unit unit) {
    if(axis >= axes_.size()) return 0;
    uint32_t raw = rawAxisValue(axis);
    if(unit == UNIT_RAW) return raw;

    uint32_t maxval = (1<<bitDepthForAxis(axis))-1;
    float normed = float(double(raw)/double(maxval)); // double here because of potentially big numbers.
    switch(unit) {
    case UNIT_DEGREES:
        return normed * 360.0f;
        break;
    case UNIT_DEGREES_CENTERED:
        return (normed * 360.0f)-180.0f;
        break;
    case UNIT_UNITY_CENTERED:
        return (normed-0.5)*2.0f;
        break;
    case UNIT_UNITY:
        return normed;
        break;
    default:
        return 0.0f;
        break;
    }
}

bool Transmitter::setRawAxisValue(uint8_t axis, uint32_t value) {
    if(axis >= axes_.size()) return false;
    uint32_t maxval = (1 << axes_[axis].bitDepth)-1;
    value = constrain(value, 0, maxval);
    axes_[axis].value = value;
    return true;
}

uint32_t Transmitter::rawAxisValue(uint8_t axis) {
    if(axis >= axes_.size()) return 0;
    return axes_[axis].value;
}

float Transmitter::computeInputValue(uint8_t input) {
    //Serial.printf("Computing input value for input %d\n", input);
    float value = 0;
    for(auto& aim: axisInputMappings_) {
        if(aim.input == input) {
            //Serial.printf("Found input mapping: axes %d and %d, mix mode %d\n", aim.axis1, aim.axis2, aim.mixType);
            float v1 = axisValue(aim.axis1, UNIT_UNITY);
            float v2 = axisValue(aim.axis2, UNIT_UNITY);
            //Serial.printf("Mixing %.2f and %.2f with mix mode %d\n", v1, v2, aim.mixType);
            return aim.computeMix(v1, 0.0f, 1.0f, v2, 0.0f, 1.0f);
        }
    }

    return 0.0f;
}

