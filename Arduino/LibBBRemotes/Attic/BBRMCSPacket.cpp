#if 0
#include "BBRMCSPacket.h"
#include "BBXBee.h"


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

uint8_t bb::Packet::calculateCRC() const {
	return calcCRC7((const uint8_t*)this, sizeof(Packet)-1);
}

bb::Result bb::PacketReceiver::incomingPacket(const NodeAddr& station, uint8_t rssi, Packet& packet) {
	Result res;
	ConfigPacket::ConfigReplyType reply = packet.payload.config.reply;
	Packet packet2 = packet;
	switch(packet.type) {
	case bb::PACKET_TYPE_CONTROL:
		return incomingControlPacket(station, packet.source, rssi, packet.seqnum, packet.payload.control);
		break;
	case bb::PACKET_TYPE_STATE:
		return incomingStatePacket(station, packet.source, rssi, packet.seqnum, packet.payload.state);
		break;
	case bb::PACKET_TYPE_CONFIG:
		Console::console.printfBroadcast("Config packet from 0x%lx:%lx\n", station.addrHi(), station.addrLo());
		if(reply == ConfigPacket::CONFIG_REPLY_ERROR || reply == ConfigPacket::CONFIG_REPLY_OK) {
			Console::console.printfBroadcast("This is a Reply packet! Discarding.\n");
			return RES_SUBSYS_COMM_ERROR;
		}
		res = incomingConfigPacket(station, packet.source, rssi, packet.seqnum, packet.payload.config);
		if(reply == ConfigPacket::CONFIG_TRANSMIT_NOREPLY) {
			Console::console.printfBroadcast("Config packet but wants no reply\n"); 
			return res;
		}
		if(res == RES_OK) {
			Console::console.printfBroadcast("Sending reply with OK flag set\n");
			packet2.payload.config.reply = ConfigPacket::CONFIG_REPLY_OK;
		}
		else {
			Console::console.printfBroadcast("Sending reply with ERROR flag set\n");
			packet2.payload.config.reply = ConfigPacket::CONFIG_REPLY_ERROR;
		}

		return XBee::xbee.sendTo(station, packet2, false);
		break;
	case bb::PACKET_TYPE_PAIRING:
		bb::printf("Pairing packet from 0x%lx:%lx\n", station.addrHi(), station.addrLo());
		switch(packet.payload.pairing.type) {
		// we're handling info packets here. Everything else will be passed on.
		case bb::PairingPacket::PAIRING_INFO_REQ:
			bb::printf("Pairing packet type PAIRING_INFO_REQ, answering with {%d,%d,%d,%d}\n", packetSource(), builderId(), stationId(), stationDetail());
			packet.payload.pairing.pairingPayload.info = { packetSource(), builderId(), stationId(), stationDetail() };
			packet.payload.pairing.reply = PairingPacket::PAIRING_OK;
			res = XBee::xbee.sendTo(station, packet, false);
			bb::printf("Returning packet to 0x%lx:%lx, result: %s\n", station.addrHi(), station.addrLo(), errorMessage(res));
			return res;
			break;		

		default:
		bb::printf("Pairing packet type %d, passing on\n", packet.payload.pairing.type);
		res = incomingPairingPacket(station, packet.source, rssi, packet.seqnum, packet.payload.pairing);
			if(res == RES_OK) {
				bb::printf("Sending reply with OK flag set\n");
				packet.payload.pairing.reply = PairingPacket::PAIRING_OK;
			} else {
				bb::printf("Sending reply with ERROR flag set\n");
				packet.payload.pairing.reply = PairingPacket::PAIRING_ERROR;
			}
			return XBee::xbee.sendTo(station, packet, false);
			break;
		}
	default:
		return incomingPairingPacket(station, packet.source, rssi, packet.seqnum, packet.payload.pairing);
	}
}

bb::Result bb::PacketReceiver::incomingControlPacket(const NodeAddr& station, bb::PacketSource source, uint8_t rssi, uint8_t seqnum, const bb::ControlPacket& packet) {
	(void)station;
	(void)source;
	(void)rssi;
	(void)packet;
	Console::console.printfBroadcast("Incoming control packet default impl\n");
	return RES_OK;
}

bb::Result bb::PacketReceiver::incomingStatePacket(const NodeAddr& station, bb::PacketSource source, uint8_t rssi, uint8_t seqnum, const bb::StatePacket& packet) {
	(void)station;
	(void)source;
	(void)rssi;
	(void)packet;
	Console::console.printfBroadcast("Incoming state packet default impl\n");
	return RES_OK;
}

bb::Result bb::PacketReceiver::incomingConfigPacket(const NodeAddr& station, bb::PacketSource source, uint8_t rssi, uint8_t seqnum, bb::ConfigPacket& packet) {
	(void)station;
	(void)source;
	(void)rssi;
	(void)packet;
	Console::console.printfBroadcast("Incoming config packet default impl\n");
	return RES_OK;
}

bb::Result bb::PacketReceiver::incomingPairingPacket(const NodeAddr& station, bb::PacketSource source, uint8_t rssi, uint8_t seqnum, bb::PairingPacket& packet) {
	(void)station;
	(void)source;
	(void)rssi;
	(void)packet;
	Console::console.printfBroadcast("Incoming pairing packet default impl\n");
	return RES_OK;
}
#endif