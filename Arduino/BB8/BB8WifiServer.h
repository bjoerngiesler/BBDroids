#include <WiFiUdp.h>
#include "BB8Packet.h"

class BB8WifiServer {
  public:
  static BB8WifiServer server;

  BB8WifiServer();
  ~BB8WifiServer();

  bool begin();
  void shutdown();
  void printStatus();

  bool tryToStartAP(const String& ssid, const String& key);
  bool isAPStarted();
  bool tryToConnect(const String& ssid, const String& key);
  bool isConnected();
  void startUDPServer();
  bool isUDPServerStarted() { return udp_ != NULL; }

  bool readCommandPacketIfAvailable(BB8CommandPacket& cmd, IPAddress& remoteIP);
  bool sendStatePacket(BB8StatePacket& state);
  bool sendCommandReply(const IPAddress& remoteIP, const uint8_t* buf, uint8_t len);

  protected:
  unsigned int readDataIfAvailable(uint8_t* buf, unsigned int maxsize, IPAddress& remoteIP);
  WiFiUDP *udp_;
};
