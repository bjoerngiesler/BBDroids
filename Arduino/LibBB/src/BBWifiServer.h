#if !defined(BBWIFISERVER_H)
#define BBWIFISERVER_H

#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <Wifi.h>
#else
#include <WiFiNINA.h>
#endif
#include "BBSubsystem.h"
#include "BBConfigStorage.h"
#include "BBConsole.h"

#define DEFAULT_SSID      "BB8WifiServer-$MAC"
#define DEFAULT_WPAKEY    "BB8WifiKey"
#define MAX_STRLEN        31
#define DEFAULT_APMODE    true
#define DEFAULT_UDP_PORT  3000
#define DEFAULT_TCP_PORT  23

namespace bb {

class WifiConsoleStream: public ConsoleStream {
public:
	WifiConsoleStream();
	void setClient(const WiFiClient& client);
	virtual bool available();
	virtual bool readStringUntil(unsigned char c, String& str);
	virtual void printfFinal(const char* str);
protected:
	WiFiClient client_;
};

class WifiServer: public Subsystem {
public:
	static WifiServer server;
	
	virtual Result initialize() { return initialize(DEFAULT_SSID, DEFAULT_WPAKEY, DEFAULT_APMODE, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT); }
	virtual Result initialize(const String& ssid, const String& wpakey, bool apmode, uint16_t udpPort, uint16_t tcpPort);
	virtual Result setOTANameAndPassword(const String& name, const String& password);
	virtual Result start(ConsoleStream* stream = NULL);
	virtual Result stop(ConsoleStream* stream = NULL);
	virtual Result step();
	virtual Result operationStatus();

	virtual void printStatus(ConsoleStream *stream);

	virtual Result setParameterValue(const String& name, const String& value);

	bool tryToStartAP(const String& ssid, const String& key);
	bool isAPStarted();
	bool tryToConnect(const String& ssid, const String& key);
	bool isConnected();

	bool broadcastUDPPacket(const uint8_t* packet, size_t len);
	bool sendUDPPacket(const IPAddress& addr, const uint8_t* packet, size_t len);

protected:
	WifiServer();

	unsigned int readDataIfAvailable(uint8_t* buf, unsigned int maxsize, IPAddress& remoteIP);

	WiFiUDP udp_;
	WiFiServer tcp_;
	WiFiClient client_;

	WifiConsoleStream consoleStream_;

	String macStr_, ssid_, wpaKey_;
	String otaName_, otaPassword_;

	typedef struct {
		char ssid[MAX_STRLEN+1], wpaKey[MAX_STRLEN+1];
		bool ap;
		int udpPort, tcpPort;
	} WifiServerParams;
	WifiServerParams params_;
	ConfigStorage::HANDLE paramsHandle_;
};

};

#endif // BBTCPSERVER_H