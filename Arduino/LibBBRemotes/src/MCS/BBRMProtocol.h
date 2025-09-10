#if !defined(BBRMPROTOCOL_H)
#define BBRMPROTOCOL_H

#include "../BBRProtocol.h"
#include "BBRMPacket.h"

namespace bb {
namespace rmt {

class MProtocol: public Protocol {
public:
    enum TransmitterTypes {
        LEFT_TX = 0,
        RIGHT_TX = 1
    };

    MProtocol();

    virtual bool init(const std::string& nodeName);

    virtual Transmitter* createTransmitter(uint8_t transmitterType=0);
    virtual Receiver* createReceiver();
    virtual Configurator* createConfigurator();

    virtual uint8_t numTransmitterTypes() { return 2; }
    virtual uint8_t numChannels(uint8_t transmitterType) { return 19; }

    virtual bool discoverNodes(float timeout = 5);
    bool pairWith(const NodeDescription& descr);

    virtual bool step();
    virtual bool sendPacket(const NodeAddr& addr, const MPacket& packet, bool bumpSeqnum=true) = 0;
    virtual bool sendBroadcastPacket(const MPacket& packet, bool bumpSeqnum=true) = 0;
    virtual void bumpSeqnum();

    void setPacketSource(MPacket::PacketSource src) { source_ = src; }
	MPacket::PacketSource packetSource() { return source_; }
	void setBuilderId(uint8_t builderId) { builderId_ = builderId; }
	uint8_t builderId() { return builderId_; }
	void setStationId(uint8_t stationId) { stationId_ = stationId; }
	uint8_t stationId() { return stationId_; }
	void setStationDetail(uint8_t stationDetail) { stationDetail_ = stationDetail; }
	uint8_t stationDetail() { return stationDetail_; }

    void setPairingSecret(uint32_t secret) { pairingSecret_ = secret; }

    virtual bool incomingPacket(const NodeAddr& addr, const MPacket& packet);
	virtual bool incomingConfigPacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MConfigPacket& packet);
	virtual bool incomingPairingPacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MPairingPacket& packet);
    virtual bool waitForPacket(std::function<bool(const MPacket&, const NodeAddr& )> fn, 
                               NodeAddr& addr, MPacket& packet, 
                               bool handleOthers, float timeout) = 0;

protected:
	uint8_t builderId_, stationId_, stationDetail_;
    uint32_t pairingSecret_;
	MPacket::PacketSource source_;
    uint8_t seqnum_;
};

}; // rmt
}; // bb

#endif // BBRMPROTOCOL_H