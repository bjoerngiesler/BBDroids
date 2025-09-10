#include "BBRProtocol.h"

using namespace bb;
using namespace rmt;

bool Protocol::init(const std::string& nodeName) {
    nodeName_ = nodeName;
    Serial.printf("Initialized %s\n", nodeName_.c_str());
    return true;
}

Transmitter* Protocol::createTransmitter(uint8_t transmitterType) { 
    return nullptr; 
}

Receiver* Protocol::createReceiver() { 
    return nullptr; 
}

Configurator* Protocol::createConfigurator() { 
    return nullptr; 
}

unsigned int Protocol::numDiscoveredNodes() { 
    return discoveredNodes_.size(); 
}

const NodeDescription& Protocol::discoveredNode(unsigned int index) {
    static NodeDescription nil;
    if(index >= discoveredNodes_.size()) return nil;
    return discoveredNodes_[index];
}

bool Protocol::isPaired() { 
    return pairedNodes_.size() != 0;
}

bool Protocol::isPaired(const NodeAddr& addr) { 
    for(auto& d: pairedNodes_) {
        if(d.addr == addr) return true;
    }

    return false;
}

bool Protocol::pairWith(const NodeDescription& node) {
    if(isPaired(node.addr)) return true; // already paired
    pairedNodes_.push_back(node);
    return true;
}
    
bool Protocol::step() {
    bool retval = true;
    if(transmitter_ != nullptr) {
        if(transmitter_->transmit() == false) retval = false;
    }
    return retval;
}
