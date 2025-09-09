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
    return pairedConfigurators_.size() != 0 || pairedReceivers_.size() != 0 || pairedTransmitters_.size() != 0; 
}

bool Protocol::isPaired(const NodeAddr& addr) { 
    bool paired = false;
    if(std::find(pairedTransmitters_.begin(), pairedTransmitters_.end(), addr) != pairedTransmitters_.end()) {
        paired = true;
    }
    if(std::find(pairedReceivers_.begin(), pairedReceivers_.end(), addr) != pairedReceivers_.end()) {
        paired = true;
    }
    if(std::find(pairedConfigurators_.begin(), pairedConfigurators_.end(), addr) != pairedConfigurators_.end()) {
        paired = true;
    }

    return paired;
}

bool Protocol::pairWith(const NodeDescription& node) {
    if(isPaired(node.addr)) return true; // already paired
    if(node.isTransmitter) pairedTransmitters_.push_back(node.addr);
    if(node.isReceiver) pairedReceivers_.push_back(node.addr);
    if(node.isConfigurator) pairedConfigurators_.push_back(node.addr);
    return true;
}
    
bool Protocol::step() {
    bool retval = true;
    if(transmitter_ != nullptr) {
        if(transmitter_->transmit() == false) retval = false;
    }
    return retval;
}
