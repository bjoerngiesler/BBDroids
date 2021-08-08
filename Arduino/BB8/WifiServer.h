#include <WiFiUdp.h>

class WifiServer {
  public:
  WifiServer();
  bool tryToStartAP();
  bool isAPStarted();
  void startUDPServer();
  bool isUDPServerStarted() { return udp_server_started_; }

  unsigned int readDataIfAvailable(uint8_t* buf, unsigned int maxsize);

  protected:
  bool udp_server_started_;
  WiFiUDP udp_;
};
