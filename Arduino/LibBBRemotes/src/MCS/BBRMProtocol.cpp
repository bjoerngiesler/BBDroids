#include "BBRMProtocol.h"
#include "BBRMReceiver.h"
#include "BBRMTransmitter.h"

using namespace bb;
using namespace bb::rmt;

#if !defined(WRAPPEDDIFF)
#define WRAPPEDDIFF(a, b, max) ((a>=b) ? a-b : (max-b)+a)
#endif // WRAPPEDDIFF

static const uint8_t crc7Table[256] = {
	0x00, 0x12, 0x24, 0x36, 0x48, 0x5a, 0x6c, 0x7e,
	0x90, 0x82, 0xb4, 0xa6, 0xd8, 0xca, 0xfc, 0xee,
	0x32, 0x20, 0x16, 0x04, 0x7a, 0x68, 0x5e, 0x4c,
	0xa2, 0xb0, 0x86, 0x94, 0xea, 0xf8, 0xce, 0xdc,
	0x64, 0x76, 0x40, 0x52, 0x2c, 0x3e, 0x08, 0x1a,
	0xf4, 0xe6, 0xd0, 0xc2, 0xbc, 0xae, 0x98, 0x8a,
	0x56, 0x44, 0x72, 0x60, 0x1e, 0x0c, 0x3a, 0x28,
	0xc6, 0xd4, 0xe2, 0xf0, 0x8e, 0x9c, 0xaa, 0xb8,
	0xc8, 0xda, 0xec, 0xfe, 0x80, 0x92, 0xa4, 0xb6,
	0x58, 0x4a, 0x7c, 0x6e, 0x10, 0x02, 0x34, 0x26,
	0xfa, 0xe8, 0xde, 0xcc, 0xb2, 0xa0, 0x96, 0x84,
	0x6a, 0x78, 0x4e, 0x5c, 0x22, 0x30, 0x06, 0x14,
	0xac, 0xbe, 0x88, 0x9a, 0xe4, 0xf6, 0xc0, 0xd2,
	0x3c, 0x2e, 0x18, 0x0a, 0x74, 0x66, 0x50, 0x42,
	0x9e, 0x8c, 0xba, 0xa8, 0xd6, 0xc4, 0xf2, 0xe0,
	0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x70,
	0x82, 0x90, 0xa6, 0xb4, 0xca, 0xd8, 0xee, 0xfc,
	0x12, 0x00, 0x36, 0x24, 0x5a, 0x48, 0x7e, 0x6c,
	0xb0, 0xa2, 0x94, 0x86, 0xf8, 0xea, 0xdc, 0xce,
	0x20, 0x32, 0x04, 0x16, 0x68, 0x7a, 0x4c, 0x5e,
	0xe6, 0xf4, 0xc2, 0xd0, 0xae, 0xbc, 0x8a, 0x98,
	0x76, 0x64, 0x52, 0x40, 0x3e, 0x2c, 0x1a, 0x08,
	0xd4, 0xc6, 0xf0, 0xe2, 0x9c, 0x8e, 0xb8, 0xaa,
	0x44, 0x56, 0x60, 0x72, 0x0c, 0x1e, 0x28, 0x3a,
	0x4a, 0x58, 0x6e, 0x7c, 0x02, 0x10, 0x26, 0x34,
	0xda, 0xc8, 0xfe, 0xec, 0x92, 0x80, 0xb6, 0xa4,
	0x78, 0x6a, 0x5c, 0x4e, 0x30, 0x22, 0x14, 0x06,
	0xe8, 0xfa, 0xcc, 0xde, 0xa0, 0xb2, 0x84, 0x96,
	0x2e, 0x3c, 0x0a, 0x18, 0x66, 0x74, 0x42, 0x50,
	0xbe, 0xac, 0x9a, 0x88, 0xf6, 0xe4, 0xd2, 0xc0,
	0x1c, 0x0e, 0x38, 0x2a, 0x54, 0x46, 0x70, 0x62,
	0x8c, 0x9e, 0xa8, 0xba, 0xc4, 0xd6, 0xe0, 0xf2
};

static uint8_t calcCRC7(const uint8_t *buffer, size_t len) {
	uint8_t crc = 0;
	while(len--) {
		crc = crc7Table[crc ^ *buffer++];
	}
	return crc;
}

uint8_t MPacket::calculateCRC() const {
	return calcCRC7((const uint8_t*)this, sizeof(MPacket)-1);
}

MProtocol::MProtocol() {
	pairingSecret_ = 0xbabeface;
    seqnum_ = 0;
}

bool MProtocol::init(const std::string& nodeName) {
    return Protocol::init(nodeName);
}

Transmitter* MProtocol::createTransmitter(uint8_t transmitterType) {
    if(transmitter_ == nullptr) {
        transmitter_ = new MTransmitter(this);
    }
    return transmitter_;
}

Receiver* MProtocol::createReceiver() {
	if(receiver_ == nullptr) {
		receiver_ = new MReceiver;
	}
    return receiver_;
}

Configurator* MProtocol::createConfigurator() {
    return nullptr;
}

bool MProtocol::incomingPacket(const NodeAddr& addr, const MPacket& packet) {
	bool res;
	MConfigPacket::ConfigReplyType reply = packet.payload.config.reply;
	MPacket packet2 = packet;

	switch(packet.type) {
	case MPacket::PACKET_TYPE_CONTROL:
		if(receiver_ == nullptr) {
			Serial.printf("Got control packet from %s but we are not a receiver.\n", addr.toString().c_str());
			return false;
		}
		return ((MReceiver*)receiver_)->incomingControlPacket(addr, packet.source, packet.seqnum, packet.payload.control);
		break;

	case MPacket::PACKET_TYPE_STATE:
		if(receiver_ == nullptr) {
			Serial.printf("Got state packet from %s but we are not a receiver.\n", addr.toString().c_str());
			return false;
		}
		return ((MReceiver*)receiver_)->incomingStatePacket(addr, packet.source, packet.seqnum, packet.payload.state);
		break;

	case MPacket::PACKET_TYPE_CONFIG:
		Serial.printf("Config packet from %s\n", addr.toString().c_str());
		if(reply == MConfigPacket::CONFIG_REPLY_ERROR || reply == MConfigPacket::CONFIG_REPLY_OK) {
			Serial.printf("This is a Reply packet! Discarding.\n");
			return false;
		}
		res = incomingConfigPacket(addr, packet.source, packet.seqnum, packet.payload.config);
		if(res == true) {
			Serial.printf("Sending reply with OK flag set\n");
			packet2.payload.config.reply = MConfigPacket::CONFIG_REPLY_OK;
		}
		else {
			Serial.printf("Sending reply with ERROR flag set\n");
			packet2.payload.config.reply = MConfigPacket::CONFIG_REPLY_ERROR;
		}

		return sendPacket(addr, packet2);
		break;

	case MPacket::PACKET_TYPE_PAIRING:
		incomingPairingPacket(addr, packet.source, packet.seqnum, packet.payload.pairing);
		break;

	default:
		Serial.printf("Error: Unknown packet type %d\n", packet.type);
		return false;
	}

    return true;
}

bool MProtocol::incomingConfigPacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MConfigPacket& packet) {
	(void)addr;
	(void)source;
	(void)packet;
	Serial.printf("Incoming config packet default impl\n");
	return true;
}

bool MProtocol::incomingPairingPacket(const NodeAddr& addr, MPacket::PacketSource source, uint8_t seqnum, const MPairingPacket& packet) {
	if(packet.type == packet.PAIRING_DISCOVERY_BROADCAST) {
		if(!acceptsPairingRequests()) {
			return false;
		}

		// FIXME filter for builder ID

		Serial.printf("Replying to %s (\"%s\") with broadcast pairing packet\n", 
			          addr.toString().c_str(), packet.pairingPayload.discovery.name);
		MPacket reply;
		reply.source = source_;
		reply.type = MPacket::PACKET_TYPE_PAIRING;
		MPairingPacket& p = reply.payload.pairing;
		p.type = p.PAIRING_DISCOVERY_REPLY;
		p.pairingPayload.discovery.builderId = builderId_;
		p.pairingPayload.discovery.stationId = stationId_;
		p.pairingPayload.discovery.stationDetail = stationDetail_;
		p.pairingPayload.discovery.isTransmitter = (transmitter_ != nullptr);
		p.pairingPayload.discovery.isReceiver = (receiver_ != nullptr);
		p.pairingPayload.discovery.isConfigurator = (configurator_ != nullptr);
		snprintf(p.pairingPayload.discovery.name, NAME_MAXLEN, nodeName_.c_str());

		reply.seqnum = seqnum_;
		seqnum_ = (seqnum_ + 1) % MAX_SEQUENCE_NUMBER;
		sendBroadcastPacket(reply);
		return true;
	}

	if(packet.type == packet.PAIRING_DISCOVERY_REPLY) {
		for(auto& n: discoveredNodes_) {
			if(n.addr == addr) {
				return true;
			}
		}

		NodeDescription descr;
		descr.addr = addr;
		descr.isConfigurator = packet.pairingPayload.discovery.isConfigurator;
		descr.isTransmitter = packet.pairingPayload.discovery.isTransmitter;
		descr.isReceiver = packet.pairingPayload.discovery.isReceiver;
		descr.setName(packet.pairingPayload.discovery.name);

		if(!descr.isConfigurator && !descr.isTransmitter && !descr.isReceiver) {
			Serial.printf("Node \"%s\" at %s is neither configurator nor receiver nor transmitter. Ignoring.\n",
			              descr.getName().c_str(), addr.toString().c_str());
			return false;
		}

		discoveredNodes_.push_back(descr);
		return true;
	}

	if(packet.type == packet.PAIRING_REQUEST) {
		if(!acceptsPairingRequests()) {
			Serial.printf("Received pairing request but not in pairing mode. Ignoring.\n");
			return false;
		}

		const MPairingPacket::PairingRequest& r = packet.pairingPayload.request;
		
		MPacket reply;
		reply.source = source_;
		reply.type = MPacket::PACKET_TYPE_PAIRING;
		reply.payload.pairing.type = MPairingPacket::PAIRING_REPLY;
		
		// Secret invalid? ==> error
		if(r.pairingSecret != pairingSecret_) {
			Serial.printf("Invalid secret.\n");
			reply.payload.pairing.pairingPayload.reply.res = MPairingPacket::PAIRING_REPLY_INVALID_SECRET;
			sendPacket(addr, reply);
			return true;
		}

		// Pairing request nonsensical? ==> error
		if(!r.pairAsConfigurator && !r.pairAsReceiver && !r.pairAsTransmitter) {
			Serial.printf("Received pairing request but as neither configurator nor receiver nor transmitter.\n");
			reply.payload.pairing.pairingPayload.reply.res = MPairingPacket::PAIRING_REPLY_INVALID_ARGUMENT;
			sendPacket(addr, reply);
			return true;
		}

		// Already have this node? ==> error
		for(auto& n: pairedNodes_) {
			if(n.addr == addr) {
				reply.payload.pairing.pairingPayload.reply.res = MPairingPacket::PAIRING_REPLY_ALREADY_PAIRED;
				sendPacket(addr, reply);
				return true;
			}
		}

		// Do we know this node from the discovery? Add it and reply OK
		for(auto& n: discoveredNodes_) {
			if(n.addr == addr) {
				pairedNodes_.push_back(n);
				reply.payload.pairing.pairingPayload.reply.res = MPairingPacket::PAIRING_REPLY_OK;
				sendPacket(addr, reply);
				return true;
			}
		}

		// Don't know this node from the discovery? ==> error
		reply.payload.pairing.pairingPayload.reply.res = MPairingPacket::PAIRING_REPLY_UNKNOWN_NODE;
		sendPacket(addr, reply);
	}

	Serial.printf("Unknown pairing packet type %d\n", packet.type);
	return false;
}


bool MProtocol::discoverNodes(float timeout) {
	discoveredNodes_.clear();

	MPacket packet;
	packet.source = source_;
	packet.type = MPacket::PACKET_TYPE_PAIRING;
	MPairingPacket& p = packet.payload.pairing;

	p.type = p.PAIRING_DISCOVERY_BROADCAST;
	p.pairingPayload.discovery.builderId = builderId_;
	p.pairingPayload.discovery.stationId = stationId_;
	p.pairingPayload.discovery.stationDetail = stationDetail_;
	p.pairingPayload.discovery.isTransmitter = (transmitter_ != nullptr);
	p.pairingPayload.discovery.isReceiver = (receiver_ != nullptr);
	p.pairingPayload.discovery.isConfigurator = (configurator_ != nullptr);
	snprintf(p.pairingPayload.discovery.name, NAME_MAXLEN, nodeName_.c_str());
	sendBroadcastPacket(packet);

	unsigned long msSinceDiscoveryPacket = millis();
	msSinceDiscoveryPacket = millis();

	while(timeout > 0) {
		step();

		if(WRAPPEDDIFF(millis(), msSinceDiscoveryPacket, ULONG_MAX) > 1000) {
			sendBroadcastPacket(packet);
			msSinceDiscoveryPacket = millis();
		}
		delay(10);
		timeout -= .01;
	}

	return true;
}

bool MProtocol::pairWith(const NodeDescription& descr) {
	MPacket packet;
	packet.source = source_;
	packet.type = MPacket::PACKET_TYPE_PAIRING;
	MPairingPacket& p = packet.payload.pairing;

	p.type = p.PAIRING_REQUEST;
	p.pairingPayload.request.pairingSecret = pairingSecret_;
	p.pairingPayload.request.pairAsConfigurator = true;
	p.pairingPayload.request.pairAsReceiver = (receiver_ != nullptr);
	p.pairingPayload.request.pairAsTransmitter = (transmitter_ != nullptr);

	sendPacket(descr.addr, packet);

	MPacket pairingReplyPacket;
	NodeAddr pairingReplyAddr;
	NodeAddr addr = descr.addr;
	std::function<bool(const MPacket&, const NodeAddr&)> fn = [addr](const MPacket& p, const NodeAddr& a) {
			return p.type == p.PACKET_TYPE_PAIRING &&
					p.payload.pairing.type == MPairingPacket::PAIRING_REPLY &&
					a == addr; 
	};
	if(waitForPacket(fn, pairingReplyAddr, pairingReplyPacket, true, 0.5) == false) {
		Serial.printf("Timed out waiting for pairing reply.\n");
		return false;
	}

	//Serial.printf("Received pairing reply.\n");
	const MPairingPacket::PairingReply& r = pairingReplyPacket.payload.pairing.pairingPayload.reply;

	if(r.res == MPairingPacket::PAIRING_REPLY_INVALID_SECRET) {
		Serial.printf("Pairing reply: Invalid secret.\n");
		return true;
	} else if(r.res == MPairingPacket::PAIRING_REPLY_OTHER_ERROR) {
		Serial.printf("Pairing reply: Other error.\n");
		return true;
	}

	for(auto& n: discoveredNodes_) {
		if(n.addr == addr) {
			if(!n.isConfigurator && !n.isReceiver && !n.isTransmitter) {
				Serial.printf("Huh. This node is neither configurator nor receiver nor transmitter. Ignoring.\n");
				return false;
			}

			return Protocol::pairWith(descr);
		}
	}

	Serial.printf("Node sent a pairing reply but is not in our discovery list. Ignoring.\n");
	return false;
}

bool MProtocol::step() {
    return Protocol::step();
}

void MProtocol::bumpSeqnum() {
	seqnum_ = (seqnum_ + 1) % MAX_SEQUENCE_NUMBER;
}