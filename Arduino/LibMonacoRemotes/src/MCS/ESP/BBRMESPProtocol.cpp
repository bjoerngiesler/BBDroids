#include <Arduino.h>

#include "BBRMESPProtocol.h"
#include "../../BBRTypes.h"

using namespace bb;
using namespace bb::rmt;

#if !defined(WRAPPEDDIFF)
#define WRAPPEDDIFF(a, b, max) ((a>=b) ? a-b : (max-b)+a)
#endif // WRAPPEDDIFF

static MESPProtocol *proto = nullptr;

static const NodeAddr broadcastAddr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00};
static esp_now_peer_info broadcastPeer = {};

MESPProtocol::MESPProtocol() {
    memcpy(broadcastPeer.peer_addr, &broadcastAddr, 6);
    broadcastPeer.channel = 0;  
    broadcastPeer.encrypt = false;
    proto = this;
    keepTempPeerMS_ = 10000;
    broadcastAdded_ = false;
}

bool MESPProtocol::init(const std::string& nodeName) {
    Serial.printf("Initializing ESP-NOW... ");

    WiFi.mode(WIFI_STA);
    if(esp_now_init() != ESP_OK) {
        Serial.printf("failure.\n");
        return false;
    }
    esp_now_register_send_cb(esp_now_send_cb_t(onDataSent));
    esp_now_register_recv_cb(esp_now_recv_cb_t(onDataReceived));

    Serial.printf("success.\n");

    seqnum_ = 0;

    return MProtocol::init(nodeName);
}

bool MESPProtocol::discoverNodes(float timeout) {
    if(!broadcastAdded_) addBroadcastAddress();
    bool res = MProtocol::discoverNodes(timeout);
    removeBroadcastAddress();
    return res;
}

void MESPProtocol::onDataSent(const unsigned char *buf, esp_now_send_status_t status) {
    if(status != ESP_OK) Serial.printf("onDataSent() received error status %d\n", status);
}

void MESPProtocol::onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if(len != sizeof(MPacket)) {
        Serial.printf("onDataReceived(%02x:%02x:%02x:%02x:%02x:%02x, 0x%x, %d) - invalid size (should be %d)\n", 
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], data, len), sizeof(MPacket);
        return;
    }

    MPacket* packet = (MPacket*)data;
    if(packet->calculateCRC() != packet->crc) {
        Serial.printf("Packet received, but CRC invalid (0x%x, should be 0x%x)\n", packet->crc, packet->calculateCRC());
        return;
    }

    if(proto == nullptr) {
        Serial.printf("Packet received, but proto is NULL\n");
        return;
    }

    NodeAddr addr;
    addr.fromMACAddress(mac);
    proto->enqueuePacket(addr, *packet);
}

bool MESPProtocol::step() {
    enterPairingModeIfNecessary();
    cleanupTempPeers();

    while(packetQueue_.size()) {
        packetQueueMutex_.lock();
        AddrAndPacket ap = packetQueue_.front();
        packetQueue_.pop_front();
        packetQueueMutex_.unlock();
        incomingPacket(ap.addr, ap.packet);
    }

    return MProtocol::step();
}

bool MESPProtocol::sendPacket(const NodeAddr& addr, const MPacket& packet, bool bumpS) {
    packet.seqnum = seqnum_;
    packet.crc = packet.calculateCRC();

    bool retval = (esp_now_send(addr.byte, (uint8_t*)&packet, sizeof(packet)) == ESP_OK);
    if(retval == true && bumpS) bumpSeqnum();
    return retval;
}

bool MESPProtocol::sendBroadcastPacket(const MPacket& packet, bool bumpS) {
    if(sendPacket(broadcastAddr, packet, bumpS) == true) {
        return true;
    } else {
        Serial.printf("Failed to send broadcast packet\n");
        return false;
    }
}

bool MESPProtocol::acceptsPairingRequests() {
    bool isConfigurator = (configurator_ == nullptr);
    for(auto& n: pairedNodes_) {
        if(n.isConfigurator) isConfigurator = true;
    }
    return isConfigurator;
}

void MESPProtocol::enterPairingModeIfNecessary() {
    if(acceptsPairingRequests()) addBroadcastAddress();
    else removeBroadcastAddress();
}

bool MESPProtocol::incomingPairingPacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MPairingPacket& packet) {
    if((packet.type == packet.PAIRING_DISCOVERY_BROADCAST || 
        packet.type == packet.PAIRING_DISCOVERY_REPLY) &&
        acceptsPairingRequests()) {       
        //Serial.printf("Received discovery broadcast. Temporarily adding %s as a peer.\n", addr.toString().c_str());
        addTempPeer(addr);
    }

    return MProtocol::incomingPairingPacket(addr, source, seqnum, packet);
}

void MESPProtocol::addBroadcastAddress() {
    if(broadcastAdded_ == true) return;
    if(esp_now_add_peer(&broadcastPeer) != ESP_OK) {
        Serial.printf("Failed to add peer\n");
    } else {
        broadcastAdded_ = true;
    }
}

void MESPProtocol::removeBroadcastAddress() {
    if(broadcastAdded_ == false) return;
    if(esp_now_del_peer(broadcastAddr.byte) != ESP_OK) {
        Serial.printf("Failed to remove peer\n");
    } else {
        broadcastAdded_ = false;
    }
}

void MESPProtocol::addTempPeer(const NodeAddr& addr) {
    for(auto& t: tempPeers_) {
        if(t.addr == addr) {
            t.msAdded = millis();
            return;
        }
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, addr.byte, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    esp_now_add_peer(&peerInfo);

    tempPeers_.push_back({addr, millis()});
}

void MESPProtocol::cleanupTempPeers() {
    std::vector<TempPeer> tempTempPeers;
    for(auto& p: tempPeers_) {
        if(WRAPPEDDIFF(millis(), p.msAdded, ULONG_MAX) > keepTempPeerMS_) {
            if(isPaired(p.addr)) {
                Serial.printf("Removing %s from temp peer list but not from ESP-NOW, since we are paired with it.\n", p.addr.toString().c_str());
            } else {
                Serial.printf("Removing %s from temp peer list\n", p.addr.toString().c_str());
                esp_now_del_peer(p.addr.byte);
            }
        } else {
            tempTempPeers.push_back(p);
        }
    }
    tempPeers_ = tempTempPeers;
}

void MESPProtocol::enqueuePacket(const NodeAddr& addr, const MPacket& packet) {
    packetQueueMutex_.lock();
    packetQueue_.push_back({addr, packet});
    packetQueueMutex_.unlock();
}

bool MESPProtocol::waitForPacket(std::function<bool(const MPacket&, const NodeAddr&)> fn, 
                                 NodeAddr& addr, MPacket& packet, 
                                 bool handleOthers, float timeout) {
    bool retval = false;

    while(true) {
        packetQueueMutex_.lock();
        while(packetQueue_.size()) {
            AddrAndPacket ap = packetQueue_.front();
            packetQueue_.pop_front();

            if(fn(ap.packet, ap.addr) == true) {
                addr = ap.addr;
                packet = ap.packet;
                retval = true;
            } else if(handleOthers == true) {
                incomingPacket(ap.addr, ap.packet);
            }
        }
        packetQueueMutex_.unlock();
        if(retval == true) return true;
        timeout -= .01;
        if(timeout < 0) break;
        delay(10);
    }
    return false;
}
