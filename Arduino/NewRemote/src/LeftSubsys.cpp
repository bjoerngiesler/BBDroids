#include "LeftSubsys.h"
#include "RemoteSubsys.h"
#include "UI/UI.h"
#include "Config.h"

#include <MCS/BBRMTransmitter.h>
#include <MCS/BBRMProtocol.h>

LeftSubsys LeftSubsys::inst;

Result LeftSubsys::initialize() {
    receiver_ = nullptr;
    currentProto_ = nullptr;
    interSetupComplete_ = false;
    return Subsystem::initialize("left", "Left Subsystem", "help");
}

Result LeftSubsys::start() {
    UI::ui.start();
    RemoteSubsys::inst.setCurrentChangedCB([this](Protocol *p){setupCurrent(p);});
    setupCurrent(RemoteSubsys::inst.interremoteProtocol());
    if(RemoteSubsys::inst.currentProtocol() != RemoteSubsys::inst.interremoteProtocol())
        setupCurrent(RemoteSubsys::inst.currentProtocol());
    return Subsystem::start();
}

Result LeftSubsys::stop() { 
    return Subsystem::stop();
}

Result LeftSubsys::step() {
    Input::inst.update();

    if(Runloop::runloop.getSequenceNumber() % 4 == 0) {
        static uint8_t seqnum;
        UI::ui.leftRemoteVis()->visualizeFromInput();
        UI::ui.updateLeftSeqnum(seqnum++);
        UI::ui.drawGUI();
    }

    if(transmitter_ == nullptr) return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    //bb::printf("Updating transmitter in left\n");
    Input::inst.updateTransmitter(transmitter_);
    //bb::printf("Updating transmitter in left done\n");

    return RES_OK;
}

Result LeftSubsys::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
    return Subsystem::handleConsoleCommand(words, stream);
}

void LeftSubsys::setupCurrent(Protocol* p) {
    if(p == nullptr) {
        bb::printf("p is nullptr!");
        return;
    }

    ProtocolType type = p->protocolType();
    switch(type) {
    case MONACO_XBEE:
    case MONACO_UDP:
    case MONACO_ESPNOW:
    case MONACO_BLE:
        ((MProtocol*)p)->setPacketSource(MPacket::PACKET_SOURCE_LEFT_REMOTE);
        ((MProtocol*)p)->setTransmittersArePrimary(true);
        p->rePairWithConnected(true, false, false);
        break;
    default:
        break;
    }

    RemoteSubsys::inst.interremoteProtocol()->setTelemetryReceivedCB(nullptr);
    p->setTelemetryReceivedCB([](Protocol* p, const NodeAddr& a, uint8_t s, const Telemetry& t) {UI::ui.visualizeFromTelemetry(p, a, s, t);});

    transmitter_ = p->createTransmitter();
    if(transmitter_ != nullptr) Input::inst.setupAxesFromTransmitter(transmitter_);

    if(p == RemoteSubsys::inst.interremoteProtocol()) {
        receiver_ = p->createReceiver();
        if(receiver_ != nullptr) {
            receiver_->setDataReceivedCallback([this](const NodeAddr& addr, uint8_t seqnum, const void* data, uint8_t len){dataReceivedCB(addr, seqnum, data, len);});
        }
        interSetupComplete_ = true;
    }

    if(p->requiresConnection() && p->isConnected() == false) {
        bb::printf("Connecting\n");
        p->connect();
    }

    currentProto_ = p;
    bb::printf("Left remote now using new protocol of type %c\n", currentProto_->protocolType());
    bb::printf("Receiver: 0x%x, transmitter: 0x%x\n", receiver_, transmitter_);
}

void LeftSubsys::dataReceivedCB(const NodeAddr& addr, uint8_t seqnum, const void* data, uint8_t len) {
    static unsigned long lastDataReceivedMS = 0;

    lastDataReceivedMS = millis();
    if(transmitter_ == nullptr) return;
    if(RemoteSubsys::inst.interremoteProtocol() == nullptr) return;
    ProtocolType type = RemoteSubsys::inst.interremoteProtocol()->protocolType();
    if(type != MONACO_XBEE) {
        bb::printf("Can't forward or visualize packets from protocol %c!\n", type);
        return;
    }

    if(len != sizeof(MControlPacket)) {
        bb::printf("Wrong packet size %d, MControlPacket is %d\n", len, sizeof(MControlPacket));
        return;
    }

    MControlPacket* p = (MControlPacket*)data;

    UI::ui.rightRemoteVis()->visualizeFromControlPacket(*p);
    UI::ui.updateRightSeqnum(seqnum);
    ((MTransmitter*)transmitter_)->transmitRawControlPacket(*p);
} 