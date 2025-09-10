#if 0

#include "BBRemote.h"


uint8_t bb::Remote::Sender::numSenderAxes() { 
    return axes_.size(); 
}

const char* bb::Remote::Sender::senderAxisName(uint8_t id) {
    if(id >= axes_.size()) return nullptr;
    return axes_[id].name;
}

uint8_t bb::Remote::Sender::findSenderAxisByName(const char* name) {
    for(uint8_t i=0; i<axes_.size(); i++) {
        if(strcmp(name, axes_[i].name) == 0) return i;
    }
    return 255;
}

bool bb::Remote::Sender::setSenderAxisValue(uint8_t id, float value, Unit unit) {
    if(id >= axes_.size()) return false;
    axes_[id].value = value;
    axes_[id].unit = unit;
    return true;
}

bool bb::Remote::Sender::setSenderAxisValue(const char* name, float value, Unit unit) {
    uint8_t id = findSenderAxisByName(name);
    if(id >= axes_.size()) return false;
    axes_[id].value = value;
    axes_[id].unit = unit;
    return true;
}

uint8_t bb::Remote::Sender::numSenderTriggers() {
    return triggers_.size();
}

const char* bb::Remote::Sender::senderTriggerName(uint8_t id) {
    if(id >= triggers_.size()) return nullptr;
    return triggers_[id].name;    
}

uint8_t bb::Remote::Sender::findSenderTriggerByName(const char* name) {
    for(uint8_t i=0; i<triggers_.size(); i++) {
        if(strcmp(name, triggers_[i].name) == 0) return i;
    }
    return 255;
}

bool bb::Remote::Sender::setSenderTriggerValue(uint8_t id, bool value) {
    if(id >= triggers_.size()) return false;
    triggers_[id].value = value;
    return true;
}

bool bb::Remote::Sender::setSenderTriggerValue(const char* name, bool value) {
    uint8_t id = findSenderTriggerByName(name);
    if(id >= triggers_.size()) return false;
    triggers_[id].value = value;
    return true;
}
#endif