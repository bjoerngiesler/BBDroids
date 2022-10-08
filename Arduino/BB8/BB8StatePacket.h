#if !defined(BB8STATE_PACKET_H)
#define BB8STATE_PACKET_H

typedef struct {
  uint8_t sequence_num_;
  int16_t joystick_horizontal_, joystick_vertical_;  
} BB8StatePacket;

#endif // !defined(STATE_PACKET_H)
