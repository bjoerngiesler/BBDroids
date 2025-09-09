#if !defined(BBRMESPPROTOCOL_H)
#define BBRMESPPROTOCOL_H

#include "../BBRMProtocol.h"
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <mutex>
#include <deque>

namespace bb {
namespace rmt {

class MESPProtocol: public MProtocol {
public:
    MESPProtocol();
    virtual bool init(const std::string& nodeName);

    virtual Result discoverNodes(float timeout = 5);

    static void onDataSent(const unsigned char *buf, esp_now_send_status_t status);
    static void onDataReceived(const uint8_t *mac, const uint8_t *data, int len);

    virtual bool acceptsPairingRequests();

    virtual bool step();

    virtual bool sendPacket(const NodeAddr& addr, const MPacket& packet, bool bumpSeqnum=true);
    virtual bool sendBroadcastPacket(const MPacket& packet, bool bumpSeqnum=true);

    virtual bool incomingPairingPacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MPairingPacket& packet);

    virtual void enqueuePacket(const NodeAddr& addr, const MPacket& packet);
    virtual bool waitForPacket(std::function<bool(const MPacket&, const NodeAddr&)> fn, 
                               NodeAddr& addr, MPacket& packet, 
                               bool handleOthers, float timeout);


protected:
    void enterPairingModeIfNecessary();
    void addBroadcastAddress();
    void removeBroadcastAddress();

    void addTempPeer(const NodeAddr& addr);
    void cleanupTempPeers();

    unsigned long keepTempPeerMS_;
    struct TempPeer {
        NodeAddr addr;
        unsigned long msAdded;
    };
    std::vector<TempPeer> tempPeers_; 
    bool broadcastAdded_;

    struct AddrAndPacket {
        NodeAddr addr;
        MPacket packet;
    };
    std::deque<AddrAndPacket> packetQueue_;
    std::mutex packetQueueMutex_;
}; 
}; // rmt
}; // bb

#endif // BBRMESPPROTOCOL_H