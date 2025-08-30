#include "BBRTransmitter.h"

using namespace bb;
using namespace bb::rmt;

void Transmitter::setAxisValue(uint8_t axis, float value, Unit unit) {
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
}

float Transmitter::axisValue(uint8_t axis, Unit unit) {
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
