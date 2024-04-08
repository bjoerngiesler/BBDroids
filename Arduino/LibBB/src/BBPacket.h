#if !defined(BBPACKETRECEIVER_H)
#define BBPACKETRECEIVER_H

#include <BBError.h>
#include <BBRunloop.h>

namespace bb {

//
// REALTIME PROTOCOL
//
// See https://github.com/bjoerngiesler/Droids/wiki/10-Remote-Control for documentation
//

struct __attribute__ ((packed)) ControlPacket {

#define AXIS_MAX 511

	int16_t axis0  : 10; // bit 0..9
	int16_t axis1  : 10; // bit 10..19
	int16_t axis2  : 10; // bit 20..29
	int16_t axis3  : 10; // bit 30..39
	int16_t axis4  : 10; // bit 40..49
	bool button0    : 1;  // bit 50
	bool button1    : 1;  // bit 51
	bool button2    : 1;  // bit 52
	bool button3    : 1;  // bit 53
	bool button4    : 1;  // bit 54
	bool event      : 1;  // bit 55

	void setAxis(uint8_t num, float value) {
		value = constrain(value, -1.0, 1.0);
		switch(num) {
		case 0: axis0 = value*AXIS_MAX; 
		case 1: axis1 = value*AXIS_MAX; 
		case 2: axis2 = value*AXIS_MAX; 
		case 3: axis3 = value*AXIS_MAX; 
		case 4: axis4 = value*AXIS_MAX;
		default: break;
		}
	}

	float getAxis(uint8_t num) const {
		switch(num) {
		case 0: return ((float)axis0)/AXIS_MAX;
		case 1: return ((float)axis1)/AXIS_MAX;
		case 2: return ((float)axis2)/AXIS_MAX;
		case 3: return ((float)axis3)/AXIS_MAX;
		case 4: return ((float)axis4)/AXIS_MAX;
		default: break;
		}
		return 0.0;
	}
};     // 8 bytes long, maximum should be <=10

struct __attribute__ ((packed)) ControlMode {
	enum ControlType {
		CONTROL_OFF		 	 = 0,
		CONTROL_RC			 = 1,
		CONTROL_RC_FREE_ANIM = 2,
		CONTROL_AUTOMATIC	 = 3
	};

	ControlType driveControl : 2;
	ControlType domeControl	 : 2;
	ControlType armsControl	 : 2;
	ControlType soundControl : 2;
};

struct __attribute__ ((packed)) SubsysStatus {
	enum StatusType {
		STATUS_OK		= 0,
		STATUS_DEGRADED	= 1,
		STATUS_ERROR	= 2,
		STATUS_CRITICAL	= 3
	};
	StatusType battery 	: 2;
	StatusType drive 	: 2;
	StatusType servos   : 2;
	StatusType comm		: 2;
};

struct __attribute__ ((packed)) StatePacket {
	ControlMode controlMode;
	SubsysStatus subsysStatus;
};

struct __attribute__ ((packed)) ConfigPacket {
	enum ConfigType {
		CONFIG_SET_LEFT_REMOTE_ID = 0,
		CONFIG_SET_DROID_ID       = 1,
		CONFIG_SET_CONTROL_MODE   = 2
	};

	ConfigType type;
	union {
		uint16_t id;
		ControlMode controlMode;
	} parameter;
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
	PACKET_SOURCE_LEFT_REMOTE  = 0,
	PACKET_SOURCE_RIGHT_REMOTE = 1,
	PACKET_SOURCE_DROID        = 2,
	PACKET_SOURCE_TEST_ONLY    = 3
};

struct Packet {
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

	Packet(PacketType t, PacketSource s) {
		type = t;
		source = s;
		seqnum = bb::Runloop::runloop.getSequenceNumber()%8;
	}
	Packet() {}
	uint8_t calculateCRC();
};

static const uint8_t MAX_SEQUENCE_NUMBER = 8;

struct PacketFrame {
	Packet packet;
	uint8_t crc;
};

class PacketReceiver { 
public:
	virtual Result incomingPacket(uint16_t station, uint8_t rssi, const Packet& packet);
	virtual Result incomingControlPacket(uint16_t station, PacketSource source, uint8_t rssi, const ControlPacket& packet);
	virtual Result incomingStatePacket(uint16_t station, PacketSource source, uint8_t rssi, const StatePacket& packet);
	virtual Result incomingConfigPacket(uint16_t station, PacketSource source, uint8_t rssi, const ConfigPacket& packet);
	virtual Result incomingPairingPacket(uint16_t station, PacketSource source, uint8_t rssi, const PairingPacket& packet);
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