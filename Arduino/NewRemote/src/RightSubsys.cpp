#include "RightSubsys.h"
#include "RemoteSubsys.h"
#include "Config.h"

#include <MCS/BBRMProtocol.h>


RightSubsys RightSubsys::inst;

Result RightSubsys::initialize() {
    transmitter_ = nullptr;
    currentProto_ = nullptr;
    return Subsystem::initialize("right", "Right Subsystem", "help");
}

Result RightSubsys::start() {
    return Subsystem::start();
}

Result RightSubsys::stop() { 
    return Subsystem::stop();
}

Result RightSubsys::step() {
    if(currentProto_ == nullptr) {
        currentProto_ = RemoteSubsys::inst.currentProtocol();
        if(currentProto_ == nullptr) {
            bb::printf("currentProtocol is nullptr!");
            return RES_SUBSYS_HW_DEPENDENCY_MISSING;
        }
        currentProto_->addDestroyCB([this](Protocol *p){protocolDestroyed(p);});

        ProtocolType type = currentProto_->protocolType();
        switch(type) {
        case MONACO_XBEE:
        case MONACO_UDP:
        case MONACO_ESPNOW:
        case MONACO_BLE:
            ((MProtocol*)currentProto_)->setPacketSource(MPacket::PACKET_SOURCE_RIGHT_REMOTE);
            ((MProtocol*)currentProto_)->setTransmittersArePrimary(false);
            break;
        default:
            break;
        }

        transmitter_ = currentProto_->createTransmitter();
        Input::inst.setupAxesFromTransmitter(transmitter_);
        bb::printf("Right remote now using new protocol of tye %c\n", currentProto_->protocolType());
    }

    Input::inst.update();

    if(transmitter_ == nullptr) return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    Input::inst.updateTransmitter(transmitter_);

    return RES_OK;
}

Result RightSubsys::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
    return Subsystem::handleConsoleCommand(words, stream);
}

void RightSubsys::protocolDestroyed(Protocol* p) {
    if(p == currentProto_) {
        transmitter_ = nullptr;
        currentProto_ = nullptr;
    }
}