#include <WiFiUdp.h>

class BB8WifiServer {
  public:
  BB8WifiServer();
  bool tryToStartAP();
  bool isAPStarted();
  void startUDPServer();
  bool isUDPServerStarted() { return udp_server_started_; }

  unsigned int readDataIfAvailable(uint8_t* buf, unsigned int maxsize);

  protected:
  bool udp_server_started_;
  WiFiUDP udp_;
};
