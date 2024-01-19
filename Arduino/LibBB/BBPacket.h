#if !defined(BBPACKETRECEIVER_H)
#define BBPACKETRECEIVER_H

#include <BBError.h>

#define AXIS_MIN -255
#define AXIS_MAX 255

namespace bb {

/*
 * REALTIME PROTOCOL
 *
 * This is designed to be sent via real-time remote control links.
 */

struct __attribute__ ((packed)) ControlPacket {
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
	bool    axish0  : 1;
	bool    axish1  : 1;
	bool    axish2  : 1;
	bool    axish3  : 1;
	bool    axish4  : 1;
	bool    axish5  : 1;
	bool    axish6  : 1;
	uint8_t         : 0; // next byte

	void setAxis(uint8_t num, int16_t value) {
		value = constrain(value, AXIS_MIN, AXIS_MAX);
		bool sign = value < 0 ? true : false;
		if(sign) value = -value;
		switch(num) {
		case 0: sign_axis0 = sign; axis0 = (value&0x7f); axish0 = (value>=128); break;
		case 1: sign_axis1 = sign; axis1 = (value&0x7f); axish1 = (value>=128); break;
		case 2: sign_axis2 = sign; axis2 = (value&0x7f); axish2 = (value>=128); break;
		case 3: sign_axis3 = sign; axis3 = (value&0x7f); axish3 = (value>=128); break;
		case 4: sign_axis4 = sign; axis4 = (value&0x7f); axish4 = (value>=128); break;
		default: break;
		}
	}

	int16_t getAxis(uint8_t num) const {
		int sign = 1;
		switch(num) {
		case 0: if(sign_axis0) sign=-1; return sign*(axis0 | (axish0?0x80:0)); break;
		case 1: if(sign_axis1) sign=-1; return sign*(axis1 | (axish1?0x80:0)); break;
		case 2: if(sign_axis2) sign=-1; return sign*(axis2 | (axish2?0x80:0)); break;
		case 3: if(sign_axis3) sign=-1; return sign*(axis3 | (axish3?0x80:0)); break;
		case 4: if(sign_axis4) sign=-1; return sign*(axis4 | (axish4?0x80:0)); break;
		default: break;
		}
		return 0;
	}
};     // 8 bytes long, maximum should be <=10

struct __attribute__ ((packed)) StatePacket {
	uint8_t dummy;
};

enum ConfigType {
	CONFIG_SET_DESTINATION_ID = 0
};

struct __attribute__ ((packed)) ConfigPacket {
	ConfigType type         : 7;
	uint8_t					: 0; // next byte
	uint8_t bits0to6        : 7;
	uint8_t 				: 0; // next byte	
	uint8_t bits7to13       : 7;
	uint8_t 				: 0; // next byte	
	uint8_t bits14to20      : 7;
	uint8_t 				: 0; // next byte	
	uint8_t bits21to27      : 7;
	uint8_t 				: 0; // next byte	
	uint8_t bits28to34      : 7;
	uint8_t 				: 0; // next byte	
	uint8_t bits35to41      : 7;
	uint8_t 				: 0; // next byte	
	uint8_t bits42to48      : 7;
	uint8_t 				: 0; // next byte	
};

struct __attribute__ ((packed)) PairPacket {
	uint8_t dummy;
};

enum PacketType {
	PACKET_TYPE_CONTROL  = 0,
	PACKET_TYPE_STATUS   = 1,
	PACKET_TYPE_CONFIG   = 2,
	PACKET_TYPE_PAIR     = 3
};

enum PacketSource {
	PACKET_SOURCE_LEFT_REMOTE  = 0,
	PACKET_SOURCE_RIGHT_REMOTE = 1,
	PACKET_SOURCE_DROID        = 2,
	PACKET_SOURCE_TEST_ONLY    = 3
};

struct Packet {
	PacketType type     : 2;
	PacketSource source : 2;
	uint8_t seqnum      : 3; // automatically set by Runloop
	uint8_t             : 0; // next byte

	union {
		ControlPacket control;
		StatePacket state;
		ConfigPacket config;
		PairPacket pair;
	} payload;
};

static const uint8_t MAX_SEQUENCE_NUMBER = 8;

struct PacketFrame {
	Packet packet;
	uint8_t crc  : 7;
	bool highbit : 1;
};

uint8_t calculateCRC(const Packet& packet);

class PacketReceiver { 
public:
	virtual Result incomingPacket(const Packet& packet) { (void)packet; return RES_OK; } // remote --> droid
};

/*
 * STATUS / DIAGNOSTICS PROTOCOL
 *
 * This is designed to be sent via a checksummed highspeed protocol, like UDP, where packets can be up to 64kb long,
 * and not much is lost if a packet is dropped. Do not use over realtime links used for remote control, for example.
 */

struct __attribute__ ((packed)) DriveControlState {
    ErrorState errorState;
    uint8_t controlMode;
    float presentPWM, presentSpeed, presentPos;
    float goal, err, errI, errD, control; 
};

struct __attribute__ ((packed)) IMUState {
	ErrorState errorState;
	float r, p, h;    // Integrated / filtered orientation in space (as coming from a Madgwick or Ext Kalman filter)
	float dr, dp, dh; // Rotation rates (as coming directly from the gyro)
	float ax, ay, az; // Acceleration values (as coming directly from the accelerometer)
};

struct __attribute__ ((packed)) ServoState {
	ErrorState errorState;
	float goal, present, load;
};

struct __attribute__ ((packed)) BatteryState {
	ErrorState errorState;
	float voltage, current;
};

enum DroidType {
	DROID_R2    = 0, // differential driven, 360° rotating dome
	DROID_BB8   = 1, // longitudinal/roll angle driven, 360° rotating dome
	DROID_DO    = 2, // differential driven, servoed redundant dome
	DROID_OTHER = 3
};

struct __attribute__ ((packed)) LargeStatusPacket {
	float timestamp;
	uint8_t droidType;
	char droidName[16];

	DriveControlState drive[3];
	IMUState imu[3];
	ControlPacket lastControl[2];
	ServoState servo[10];
	BatteryState battery[3];
};

};

#endif // BBPACKETRECEIVER_H