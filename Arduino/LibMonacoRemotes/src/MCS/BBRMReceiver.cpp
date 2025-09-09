#include "BBRMReceiver.h"

using namespace bb;
using namespace bb::rmt;

bool MReceiver::incomingControlPacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MControlPacket& packet) {
    if(dataReceivedCB_ != nullptr) dataReceivedCB_(addr, seqnum);
    for(uint8_t i = 0; i<inputs_.size(); i++) {
        if(!hasMappingForInput(i)) continue;
        Input& inp = inputs_[i];

        const AxisInputMapping& mapping = mappingForInput(i);

        float val1 = packet.getAxis(mapping.axis1, UNIT_UNITY);
        float val2 = packet.getAxis(mapping.axis2, UNIT_UNITY);

        float out = mapping.computeMix(val1, 0, 1, val2, 0, 1);

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