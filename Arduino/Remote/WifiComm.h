#include <WiFiNINA.h>
#include <WiFiUdp.h>

class WifiComm {
  public:
    WifiComm();
    bool tryToConnect();
    bool isConnected();

    bool broadcastUDPPacket(uint8_t* packet, unsigned int packetsize);
  protected:
    WiFiUDP udp_;
};
