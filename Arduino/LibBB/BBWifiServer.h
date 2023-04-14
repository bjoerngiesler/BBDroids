#if !defined(BBWIFISERVER_H)
#define BBWIFISERVER_H

#include <Arduino.h>
#include <WiFiNINA.h>
#include "BBSubsystem.h"
#include "BBConfigStorage.h"
#include "BBConsole.h"

#define DEFAULT_SSID      "BB8WifiServer-$MAC"
#define DEFAULT_WPAKEY    "BB8WifiKey"
#define MAX_STRLEN        31
#define DEFAULT_APMODE    true
#define DEFAULT_UDP_PORT  3000
#define DEFAULT_TCP_PORT  3000

namespace bb {

class WiFiConsoleStream: public ConsoleStream {
public:
	WiFiConsoleStream();
	void setClient(const WiFiClient& client);
	virtual bool available();
	virtual bool readStringUntil(unsigned char c, String& str);
	virtual void print(size_t val);
	virtual void print(int val);
	virtual void print(float val);
	virtual void print(const String& val);
	virtual void println(int val);
	virtual void println(float val);
	virtual void println(const String& val);
	virtual void println();
protected:
	WiFiClient client_;
};

class WifiServer: public Subsystem {
public:
	static WifiServer server;
	
	virtual Result initialize() { return initialize(DEFAULT_SSID, DEFAULT_WPAKEY, DEFAULT_APMODE, DEFAULT_UDP_PORT, DEFAULT_TCP_PORT); }
	virtual Result initialize(const char *ssid, const char *wpakey, bool apmode, uint16_t udpPort, uint16_t tcpPort);
	virtual Result setOTANameAndPassword(const String& name, const String& password);
	virtual Result start(ConsoleStream* stream = NULL);
	virtual Result stop(ConsoleStream* stream = NULL);
	virtual Result step();
	virtual Result operationStatus();

	virtual const String& description() {
		updateDescription();
		return description_;
	}

	virtual Result parameterValue(const String& name, String& value);
	virtual Result setParameterValue(const String& name, const String& value);

	bool tryToStartAP(const String& ssid, const String& key);
	bool isAPStarted();
	bool tryToConnect(const String& ssid, const String& key);
	bool isConnected();
	bool startUDPServer(ConsoleStream *stream = NULL);
	bool startTCPServer(ConsoleStream *stream = NULL);

	bool sendCommandReply(const IPAddress& remoteIP, const uint8_t* buf, uint8_t len);

protected:
	WifiServer();
	~WifiServer();

	unsigned int readDataIfAvailable(uint8_t* buf, unsigned int maxsize, IPAddress& remoteIP);
	void updateDescription();

	WiFiUDP udp_;
	bool udpStarted_;
	WiFiServer *tcp_;
	WiFiClient client_;
	WiFiConsoleStream consoleStream_;
	String macStr_, ssid_, wpaKey_;
	String otaName_, otaPassword_;

	typedef struct {
		char ssid[MAX_STRLEN+1], wpaKey[MAX_STRLEN+1];
		bool ap;
		uint16_t udpPort, tcpPort;
	} WifiServerParams;
	WifiServerParams params_;
	ConfigStorage::HANDLE paramsHandle_;
};

};

#endif // BBTCPSERVER_H