#if !defined(BBPACKETRECEIVER_H)
#define BBPACKETRECEIVER_H

#include <BBError.h>
#include <BBRunloop.h>
#include <BBConsole.h>

namespace bb {

//
// REALTIME PROTOCOL
//
// See https://github.com/bjoerngiesler/Droids/wiki/10-Remote-Control for documentation
//

struct __attribute__ ((packed)) ControlPacket {

#define AXIS_MAX1 1023
#define AXIS_MAX2   255
#define BATTERY_MAX 

	uint16_t axis0 : 10; // bit 0..9
	uint16_t axis1 : 10; // bit 10..19
	uint16_t axis2 : 10; // bit 20..29
	uint16_t axis3 : 10; // bit 30..39
	uint16_t axis4 : 10; // bit 40..49
	uint8_t axis5;       // bit 50..57
	uint8_t axis6;       // bit 58..65
	uint8_t axis7;       // bit 66..73
	uint8_t axis8;       // bit 74..81
	uint8_t axis9;       // bit 82..89
	bool button0    : 1; // bit 90
	bool button1    : 1; // bit 91
	bool button2    : 1; // bit 92
	bool button3    : 1; // bit 93
	bool button4    : 1; // bit 94
	bool button5    : 1; // bit 95
	bool button6    : 1; // bit 96
	bool button7    : 1; // bit 97
	uint8_t battery : 6; // bit 98..103

	enum Unit {
		UNIT_DEGREES,   
		UNIT_DEGREES_CENTERED,
		UNIT_UNITY,
		UNIT_UNITY_CENTERED,
		UNIT_RAW
	};

	void setAxis(uint8_t num, float value, Unit unit = UNIT_UNITY_CENTERED) {
		uint16_t multiplier = (num < 5) ? AXIS_MAX1 : AXIS_MAX2;

		switch(unit) {
		case UNIT_DEGREES:
			value = constrain(value, 0, 360.0);
			value = (value / 360.0) * multiplier;
			break;
		case UNIT_DEGREES_CENTERED:
			value = constrain(value, -180.0, 180.0);
			value = ((value + 180.0)/360.0) * multiplier;
			break;
		case UNIT_UNITY:
			value = constrain(value, 0.0, 1.0);
			value *= multiplier;
			break;
		case UNIT_UNITY_CENTERED:
			value = constrain(value, -1.0, 1.0);
			value = ((value + 1.0)/2.0) * multiplier;
			break;
		case UNIT_RAW:
		default:
			value = constrain(value, 0, multiplier);
			break;
		}

		switch(num) {
		case 0: axis0 = value; break;
		case 1: axis1 = value; break;
		case 2: axis2 = value; break;
		case 3: axis3 = value; break;
		case 4: axis4 = value; break;
		case 5: axis5 = value; break;
		case 6: axis6 = value; break;
		case 7: axis7 = value; break;
		case 8: axis8 = value; break;
		case 9: 
		default:
			axis9 = value; break;
		}
	}

	float getAxis(uint8_t num, Unit unit = UNIT_UNITY_CENTERED) const {
		float multiplier = (num < 5) ? AXIS_MAX1 : AXIS_MAX2;
		float value;
		switch(num) {
		case 0: value = axis0; break;
		case 1: value = axis1; break;
		case 2: value = axis2; break;
		case 3: value = axis3; break;
		case 4: value = axis4; break;
		case 5: value = axis5; break;
		case 6: value = axis6; break;
		case 7: value = axis7; break;
		case 8: value = axis8; break;
		case 9: 
		default:
			value = axis9; break;
		}

		switch(unit) {
		case UNIT_DEGREES:
			value = (value / multiplier) * 360.0;
			break;
		case UNIT_DEGREES_CENTERED:
			value = ((value / multiplier) * 360.0) - 180.0;
			break;
		case UNIT_UNITY:
			value = value / multiplier;
			break;
		case UNIT_UNITY_CENTERED:
			value = ((value / multiplier) * 2.0) - 1.0;
			break;
		case UNIT_RAW:
		default:
			break;
		}

		return value;
	}

	void print() const {
		Console::console.printfBroadcast("%d %d %d %d %d %d %d %d %d %d %s%s%s%s%s%s%s%s\n",
			axis0, axis1, axis2, axis3, axis4, axis5, axis6, axis7, axis8, axis9, 
			button0?"X":"_", button1?"X":"_", button2?"X":"_", button3?"X":"_", button4?"X":"_", button5?"X":"_", button6?"X":"_", button7?"X":"_");
	}

};     // 13 bytes long

struct __attribute__ ((packed)) StatePacket {
	enum StatusType {
		STATUS_OK		= 0,
		STATUS_DEGRADED	= 1,
		STATUS_ERROR	= 2,
		STATUS_CRITICAL	= 3
	};

	enum ControlType {
		CONTROL_OFF		 	 = 0,
		CONTROL_RC			 = 1,
		CONTROL_RC_FREE_ANIM = 2,
		CONTROL_AUTOMATIC	 = 3
	};

	StatusType battery 	: 2;
	StatusType drive 	: 2;
	StatusType servos   : 2;
	StatusType comm		: 2;

	ControlType driveControl : 2;
	ControlType domeControl	 : 2;
	ControlType armsControl	 : 2;
	ControlType soundControl : 2;
};

struct __attribute__ ((packed)) ConfigPacket {
	static const uint64_t MAGIC = 0xbadeaffebabeface;
#define CONFIG_TYPE_BITS 6

	enum ConfigType {
		CONFIG_SET_LEFT_REMOTE_ID       = 0,  // L->R - parameter: ID of left remote
		CONFIG_SET_DROID_ID             = 1,  // L->R - parameter: ID of right remote
		CONFIG_CALIBRATE                = 2,  // L->R - parameter: MAGIC
		CONFIG_SET_DRIVE_CONTROL_MODE   = 2,  // L->D - parameter: control mode
		CONFIG_SET_DOME_CONTROL_MODE    = 3,  // L->D - parameter: control mode
		CONFIG_SET_ARMS_CONTROL_MODE    = 4,  // L->D - parameter: control mode
		CONFIG_SET_SOUND_CONTROL_MODE   = 5,  // L->D - parameter: control mode
		CONFIG_SET_PRIMARY_REMOTE       = 6,  // L->D - parameter: 0 for secondary remote, 1 to become primary
		CONFIG_FACTORY_RESET            = (1<<CONFIG_TYPE_BITS-1) // L->R - parameter: MAGIC
	};

	enum ConfigReplyType {
		CONFIG_TRANSMIT       = 0, // transmission
		CONFIG_REPLY_OK       = 1, // OK reply
		CONFIG_REPLY_ERROR    = 2, // something went wrong
		CONFIG_REPLY_RESERVED = 3
	};

	ConfigType type  : CONFIG_TYPE_BITS;
	bool       reply : 8-CONFIG_TYPE_BITS;
	uint64_t parameter;
};

struct __attribute__ ((packed)) PairingPacket {
	uint8_t dummy;
};

enum PacketType {
	PACKET_TYPE_CONTROL  = 0,
	PACKET_TYPE_STATE    = 1,
	PACKET_TYPE_CONFIG   = 2,
	PACKET_TYPE_PAIRING  = 3
};

enum PacketSource {
	PACKET_SOURCE_LEFT_REMOTE    = 0,
	PACKET_SOURCE_RIGHT_REMOTE   = 1,
	PACKET_SOURCE_DROID          = 2,
	PACKET_SOURCE_PRIMARY_REMOTE = 3
};

struct __attribute__ ((packed)) Packet {
	PacketType type     : 2;
	PacketSource source : 2;
	uint8_t seqnum      : 3; // automatically set by Runloop
	uint8_t reserved    : 1; 

	union {
		ControlPacket control;
		StatePacket state;
		ConfigPacket config;
		PairingPacket pairing;
	} payload;

	Packet(PacketType t, PacketSource s, unsigned long seq) {
		type = t;
		source = s;
		seqnum = seq%8;
		reserved = 0;
	}
	Packet() { }
	uint8_t calculateCRC();
};

static const uint8_t MAX_SEQUENCE_NUMBER = 8;

struct PacketFrame {
	Packet packet;
	uint8_t crc;
};

class PacketReceiver { 
public:
	virtual Result incomingPacket(uint64_t srcAddr, uint8_t rssi, const Packet& packet);
	virtual Result incomingControlPacket(uint64_t srcAddr, PacketSource source, uint8_t rssi, const ControlPacket& packet);
	virtual Result incomingStatePacket(uint64_t srcAddr, PacketSource source, uint8_t rssi, const StatePacket& packet);
	virtual Result incomingConfigPacket(uint64_t srcAddr, PacketSource source, uint8_t rssi, const ConfigPacket& packet);
	virtual Result incomingPairingPacket(uint64_t srcAddr, PacketSource source, uint8_t rssi, const PairingPacket& packet);
};

/*
 * STATE / DIAGNOSTICS PROTOCOL
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

struct __attribute__ ((packed)) LargeStatePacket {
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