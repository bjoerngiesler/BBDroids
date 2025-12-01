#include "MonacoForward.h"
#include "Config.h"
#include <LibBBRemotes.h>

MonacoForward MonacoForward::monaco;

Result MonacoForward::initialize() {
    bb::printf("Initializing MonacoForward\n");

    name_ = "monaco";
    description_ = "Monaco Port Forwarder";
    protocol_.init("MonacoFwd");
    receiver_ = protocol_.createReceiver();
    if(receiver_ == nullptr) {
        bb::printf("Could not create receiver\n");
        return RES_SUBSYS_HW_DEPENDENCY_MISSING;
    }

    protocol_.setPacketReceivedCB([this](const NodeAddr& a, const MPacket& p) { packetReceivedCB(a, p); });

    Serial1.begin(230400, SERIAL_8N1, RX_PIN, TX_PIN);

    lastPacketMS_ = millis();

    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);

    return Subsystem::initialize();
}

Result MonacoForward::start(ConsoleStream *stream) {
    bb::printf("Starting Monaco Forwarder.\n");
    return Subsystem::start(stream);
}

Result MonacoForward::stop(ConsoleStream *stream) {
    return Subsystem::stop(stream);
}

Result MonacoForward::step() {
    protocol_.step();

    if(WRAPPEDDIFF(millis(), lastPacketMS_, ULONG_MAX) > 1000) {
        digitalWrite(BUILTIN_LED, HIGH);
    }

    return RES_OK;
}

Result MonacoForward::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
    if(words.size() == 0) return RES_CMD_INVALID_ARGUMENT;

    return Subsystem::handleConsoleCommand(words, stream);
}

void MonacoForward::packetReceivedCB(const NodeAddr& addr, const MPacket& packet) {
    bb::printf("Packet received (type %d, primary %d), forwarding\n",
                packet.type, packet.type == PACKET_TYPE_CONTROL ? packet.payload.control.primary : 0);
    digitalWrite(BUILTIN_LED, LOW);
    lastPacketMS_ = millis();
    std::string ser = serializePacket(packet);
    Serial1.print(ser.c_str());
}
