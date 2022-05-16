#if !defined(BB8SERIALTX_H)
#define BB8SERIALTX_H

#include <Arduino.h>
#include <wiring_private.h>

class BB8SerialTX {
public:

  BB8SerialTX();
  bool begin(Uart* uart);

  bool available();
  void read();

private:
  Uart* uart;
};

#endif //BB8SERIALTX_H
