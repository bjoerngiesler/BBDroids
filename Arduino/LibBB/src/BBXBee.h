#if !defined(BB8XBEE_H)
#define BB8XBEE_H

#include <Arduino.h>
#include <vector>
#include "BBSubsystem.h"
#include "BBConfigStorage.h"
#include "BBPacket.h"

#define DEFAULT_CHAN    0x19   // no overlap with Wifi according to XBee documentation
#define DEFAULT_PAN     0x3332

#define DEFAULT_BPS     9600
	
namespace bb {

class XBee: public Subsystem {
public:
	static XBee xbee;

	enum StationType {
		STATION_DROID      = 0,
		STATION_REMOTE     = 1,
		STATION_DIAGNOSTIC = 2,
		STATION_PAIRING    = 3
	};

	enum DroidType {
		DROID_STATIC        = 0,
		DROID_DIFF_STABLE   = 1,
		DROID_DIFF_UNSTABLE = 2,
		DROID_DRIVE_TILT    = 3,
		DROID_HOLONOMOUS    = 4,
		DROID_4LEG_WALKING  = 5,
		DROID_2LEG_WALKING  = 6,
		DROID_RESERVED      = 7
	};

	enum RemoteType {
		REMOTE_BAVARIAN_L = 0,
		REMOTE_BAVARIAN_R = 1,
		REMOTE_RESERVED1  = 2,
		REMOTE_RESERVED2  = 3,
		REMOTE_RESERVED3  = 4,
		REMOTE_RESERVED4  = 5,
		REMOTE_RESERVED5  = 6,
		REMOTE_RESERVED6  = 7
	};

	HWAddress hwAddress() { return hwAddress_; }

	virtual Result start(ConsoleStream *stream = NULL);
	virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
	virtual Result parameterValue(const String& name, String& value);
	virtual Result setParameterValue(const String& name, const String& value);
	virtual Result initialize(uint8_t chan, uint16_t pan, uint32_t bps, HardwareSerial *uart=&Serial1);
	virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

	Result addPacketReceiver(PacketReceiver *receiver);
	Result removePacketReceiver(PacketReceiver *receiver);

	void setChannel(uint8_t chan) { params_.chan = chan; }
	void setPAN(uint16_t pan) { params_.pan = pan; }
	void setBPS(uint32_t bps) { params_.bps = bps; }
	void setName(const char* name) { memset(params_.name, 0, 20); snprintf(params_.name, 19, name); }

	Result enterATModeIfNecessary(ConsoleStream *stream = NULL);
	Result leaveATMode(ConsoleStream *stream = NULL); // incurs a mandatory delay of 1000ms!
	bool isInATMode();
	Result changeBPSTo(uint32_t bps, ConsoleStream *stream=NULL, bool stayInAT=false);
	int getCurrentBPS() { return currentBPS_; }
	Result setConnectionInfo(uint8_t chan, uint16_t pan, bool stayInAT=false);
	Result getConnectionInfo(uint8_t& chan, uint16_t& pan, bool stayInAT=false);

	Result setAPIMode(bool onoff);
	Result sendAPIModeATCommand(uint8_t frameID, const char* cmd, uint32_t& argument, bool request=false);

	struct Node {
		HWAddress address;
		uint8_t rssi;
		char name[20];
	};
	Result discoverNodes(std::vector<Node>& nodes);

	Result send(const String& str);
	Result send(const uint8_t *bytes, size_t size);
	Result send(const Packet& packet);
	Result sendTo(const HWAddress& dest, const Packet& packet, bool ack) { return sendToXBee3(dest, packet, ack); }
	Result sendToXBee3(const HWAddress& dest, const Packet& packet, bool ack);
	Result sendToXBee(const HWAddress& dest, const Packet& packet, bool ack);
	Result sendConfigPacket(const HWAddress& dest, bb::PacketSource src, const ConfigPacket& packet, ConfigPacket::ConfigReplyType& replyType,
	                        uint8_t seqnum, bool waitForReply = true);
	Result sendPairingPacket(const HWAddress& dest, bb::PacketSource src, PairingPacket& packet, uint8_t seqnum);
	int numFailedACKs();
	
	bool available();
	String receive();
	Result receiveAPIMode(HWAddress& src, uint8_t& rssi, Packet& packet);

	typedef enum {
		DEBUG_SILENT = 0,
		DEBUG_PROTOCOL   = 0x01,
		DEBUG_XBEE_COMM  = 0x02
	} DebugFlags;

	void setDebugFlags(DebugFlags);

protected:
	XBee();
	virtual ~XBee();

	DebugFlags debug_;
	int timeout_;
	unsigned long atmode_millis_, atmode_timeout_;
	bool atmode_, stayInAT_;
	HardwareSerial* uart_;
	int currentBPS_;
	bool apiMode_;

	typedef struct {
		int chan;
		int pan;
		int bps;
		char name[20];
	} XBeeParams;
	XBeeParams params_;
	HWAddress hwAddress_;
	ConfigStorage::HANDLE paramsHandle_;
	std::vector<PacketReceiver*> receivers_;

	uint8_t packetBuf_[255];
	size_t packetBufPos_;

	class APIFrame {
	public:
		APIFrame();
		APIFrame(const uint8_t *data, uint16_t dataLength);
		APIFrame(uint16_t length);
		APIFrame(const APIFrame& frame);
		~APIFrame();

		APIFrame& operator=(const APIFrame& frame);
		
		virtual uint8_t *data() const { return data_; }
		virtual uint16_t length() const { return length_; }
		virtual uint8_t checksum() const { return checksum_; }

		void calcChecksum();

		static APIFrame atRequest(uint8_t frameID, uint16_t command);
		bool isATRequest();
		bool isATResponse();
		bool is16BitRXPacket();
		bool is64BitRXPacket();
		bool isRXPacket() { return is16BitRXPacket() || is64BitRXPacket(); }

		enum Type {
			TRANSMITLEGACY  = 0x00,
			ATREQUEST 		= 0x08,
			TRANSMITREQUEST = 0x10,
			RECEIVE64BIT    = 0x80,
			RECEIVE16BIT	= 0x81,
			ATRESPONSE		= 0x88
		};



		Result unpackATResponse(uint8_t &frameID, uint16_t &command, uint8_t &status, uint8_t** data, uint16_t &length);

		class __attribute__ ((packed)) EndianInt16 {
			uint16_t value;
		public:
			inline operator uint16_t() const {
				return ((value & 0xff) << 8) | (value >> 8);
			}
		};
		class __attribute__ ((packed)) EndianInt32 {
			uint32_t value;
		public:
			inline operator uint32_t() const {
				return ((value & 0x000000ff) << 24) | ((value & 0x0000ff00) << 8) | ((value & 0x00ff0000) >> 8) | ((value & 0xff000000) >> 24);
			}
		};
		class __attribute__ ((packed)) EndianInt64 {
			uint64_t value;
		public:
			inline operator uint64_t() const {
				return ((value & 0x00000000000000ff) << 56) | ((value & 0x000000000000ff00) << 40) | ((value & 0x0000000000ff0000) << 24) | ((value & 0x00000000ff000000) <<  8) | 
					   ((value & 0x000000ff00000000) >>  8) | ((value & 0x0000ff0000000000) >> 24) | ((value & 0x00ff000000000000) >> 40) | ((value & 0xff00000000000000) >> 56);
			}
		};

		static const uint8_t ATResponseNDMinLength = 11;
		struct __attribute__ ((packed)) ATResponseND {
			EndianInt16 my;
			EndianInt32 addrHi, addrLo;
			uint8_t rssi;
			char name[20];
		};

	protected:
		uint8_t *data_;
		uint16_t length_;
		uint8_t checksum_;
	};

	String sendStringAndWaitForResponse(const String& str, int predelay=0, bool cr=true);
	bool sendStringAndWaitForOK(const String& str, int predelay=0, bool cr=true);
	bool readString(String& str, unsigned char terminator='\r');

	Result send(const APIFrame& frame);
	Result receive(APIFrame& frame);
};

};

#endif // BB8XBEE_H