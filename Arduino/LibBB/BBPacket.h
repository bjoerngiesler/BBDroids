#if !defined(BBPACKETRECEIVER_H)
#define BBPACKETRECEIVER_H

#include <BBError.h>

namespace bb {

// Command packet struct
typedef struct {
	// byte 0
	bool button0    : 1;
	bool button1    : 1;
	bool button2    : 1;
	bool button3    : 1;
	bool button4    : 1;
	bool sign_axis0 : 1;
	bool sign_axis1 : 1;
	uint8_t         : 0; // next byte

	// byte 1: Axis signs
	bool sign_axis2 : 1;
	bool sign_axis3 : 1;
	bool sign_axis4 : 1;
	uint8_t unused  : 4;
	uint8_t         : 0; // next byte

	// byte 2-7: Axes
	uint8_t axis0   : 7;
	uint8_t         : 0; // next byte
	uint8_t axis1   : 7;
	uint8_t         : 0; // next byte
	uint8_t axis2   : 7;
	uint8_t         : 0; // next byte
	uint8_t axis3   : 7;
	uint8_t         : 0; // next byte
	uint8_t axis4   : 7;
	uint8_t         : 0; // next byte

	void setAxis(uint8_t num, int8_t value) {
		bool sign = value < 0 ? true : false;
		if(sign) value = -value;
		switch(num) {
		case 0: sign_axis0 = sign; axis0 = value; break;
		case 1: sign_axis1 = sign; axis1 = value; break;
		case 2: sign_axis2 = sign; axis2 = value; break;
		case 3: sign_axis3 = sign; axis3 = value; break;
		case 4: sign_axis4 = sign; axis4 = value; break;
		default: break;
		}
	}

	int8_t getAxis(uint8_t num) const {
		switch(num) {
		case 0: if(sign_axis0) return -axis0; else return axis0; break;
		case 1: if(sign_axis1) return -axis1; else return axis1; break;
		case 2: if(sign_axis2) return -axis2; else return axis2; break;
		case 3: if(sign_axis3) return -axis3; else return axis3; break;
		case 4: if(sign_axis4) return -axis4; else return axis4; break;
		default: break;
		}
		return 0;
	}
} CommandPacket;     // 9 bytes long, maximum should be <=10

typedef struct {
	uint8_t dummy;
} StatePacket;

typedef enum {
	PACKET_TYPE_COMMAND = 0,
	PACKET_TYPE_STATUS  = 1,
	PACKET_TYPE_UNUSED1 = 2,
	PACKET_TYPE_UNUSED2 = 3
} PacketType;

typedef enum {
	PACKET_SOURCE_LEFT_REMOTE  = 0,
	PACKET_SOURCE_RIGHT_REMOTE = 1,
	PACKET_SOURCE_DROID        = 2,
	PACKET_SOURCE_TEST_ONLY    = 3
} PacketSource;

typedef struct {
	PacketType type     : 2;
	PacketSource source : 2;
	uint8_t seqnum      : 3; // automatically set by Runloop
	uint8_t             : 0; // next byte

	union {
		CommandPacket cmd;
		StatePacket state;
	} payload;
} Packet;

static const uint8_t MAX_SEQUENCE_NUMBER = 8;

typedef struct {
	Packet packet;
	uint8_t crc  : 7;
	bool highbit : 1;
} PacketFrame;

uint8_t calculateCRC(const Packet& packet);

class PacketReceiver { 
public:
	virtual Result incomingPacket(const Packet& packet) { (void)packet; return RES_OK; } // remote --> droid
};

};

#endif // BBPACKETRECEIVER_H