#if !defined(BB8XBEE_H)
#define BB8XBEE_H

#include <Arduino.h>
#include "BBSubsystem.h"
#include "BBConfigStorage.h"
#include "BBPacket.h"

#define DEFAULT_CHAN    0xC
#define DEFAULT_PAN     0x3332

#define DEFAULT_STATION_DROID 0x1
#define DEFAULT_STATION_LEFT_REMOTE 0x2
#define DEFAULT_STATION_RIGHT_REMOTE 0x3
#define DEFAULT_BPS     9600

namespace bb {

class XBee: public Subsystem {
public:
	static XBee xbee;

	virtual Result start(ConsoleStream *stream = NULL);
	virtual Result stop(ConsoleStream *stream = NULL);
	virtual Result step();
	virtual Result parameterValue(const String& name, String& value);
	virtual Result setParameterValue(const String& name, const String& value);
	virtual Result initialize() { 
		return initialize(DEFAULT_CHAN, DEFAULT_PAN, DEFAULT_STATION_DROID, DEFAULT_STATION_LEFT_REMOTE, DEFAULT_BPS, &Serial1); 
	}
	virtual Result initialize(uint8_t chan, uint16_t pan, uint16_t station, 
		uint16_t partner, uint32_t bps, HardwareSerial *uart=&Serial1);
	virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

	void setPacketMode(bool onoff);
	Result addPacketReceiver(PacketReceiver *receiver);
	Result removePacketReceiver(PacketReceiver *receiver);

	void setChannel(uint8_t chan) { params_.chan = chan; }
	void setPAN(uint16_t pan) { params_.pan = pan; }
	void setStation(uint16_t station) { params_.station = station; }
	void setPartner(uint16_t partner) { params_.partner = partner; }
	void setBPS(uint32_t bps) { params_.bps = bps; }

	Result enterATModeIfNecessary(ConsoleStream *stream = NULL);
	Result leaveATMode(ConsoleStream *stream = NULL); // incurs a mandatory delay of 1000ms!
	bool isInATMode();
	Result changeBPSTo(uint32_t bps, ConsoleStream *stream=NULL, bool stayInAT=false);
	int getCurrentBPS() { return currentBPS_; }
	Result setConnectionInfo(uint8_t chan, uint16_t pan, uint16_t station, uint16_t partner, bool stayInAT=false);
	Result getConnectionInfo(uint8_t& chan, uint16_t& pan, uint16_t& station, uint16_t& partner, bool stayInAT=false);

	Result discoverNetworks();

	Result send(const String& str);
	Result send(const uint8_t *bytes, size_t size);
	Result send(const Packet& packet);
	bool available();
	String receive();
	Result receiveAndHandlePacket();

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
	unsigned int currentBPS_;

	int chan_, pan_, station_, partner_, bps_;

	typedef struct {
		uint8_t chan;
		uint16_t pan, station, partner;
		uint32_t bps;
	} XBeeParams;
	XBeeParams params_;
	ConfigStorage::HANDLE paramsHandle_;
	std::vector<PacketReceiver*> receivers_;

	bool sendContinuous_;
	int continuous_;
	bool packetMode_; // if false, string mode
	uint8_t packetBuf_[255];
	size_t packetBufPos_;


	String sendStringAndWaitForResponse(const String& str, int predelay=0, bool cr=true);
	bool sendStringAndWaitForOK(const String& str, int predelay=0, bool cr=true);
	bool readString(String& str, unsigned char terminator='\r');
};

};

#endif // BB8XBEE_H