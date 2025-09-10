#include "BBRMReceiver.h"

using namespace bb;
using namespace bb::rmt;

bool MReceiver::incomingControlPacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MControlPacket& packet) {
    if(dataReceivedCB_ != nullptr) dataReceivedCB_(addr, seqnum);
    for(uint8_t i = 0; i<inputs_.size(); i++) {
        if(!hasMixForInput(i)) continue;
        Input& inp = inputs_[i];

        const AxisMix& mix = mixForInput(i);

        float val1 = packet.getAxis(mix.axis1, UNIT_UNITY);
        float val2 = packet.getAxis(mix.axis2, UNIT_UNITY);

        float out = mix.compute(val1, 0, 1, val2, 0, 1);

        inp.callback(out);
    }
    if(dataFinishedCB_ != nullptr) dataFinishedCB_(addr, seqnum);
    return true;
}

bool MReceiver::incomingStatePacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MStatePacket& packet) {
    Serial.printf("Incoming state packet from %s, source %d, seqnum %d!\n",
                  addr.toString().c_str(), source, seqnum);
    return true;
}