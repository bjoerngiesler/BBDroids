#if !defined(BBRMRECEIVER_H)
#define BBRMRECEIVER_H

#include "../BBRReceiver.h"
#include "BBRMPacket.h"

namespace bb {
namespace rmt {

class MReceiver: public Receiver {
public:
	virtual bool incomingControlPacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MControlPacket& packet);
	virtual bool incomingStatePacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MStatePacket& packet);
};

}; // rmt
}; // bb

#endif // BBRMRECEIVER_H