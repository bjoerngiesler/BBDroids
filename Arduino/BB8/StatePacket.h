#if !defined(STATE_PACKET_H)
#define STATE_PACKET_H

typedef struct {
  uint8_t sequence_num_;
  short joystick_horizontal_, joystick_vertical_;  
} StatePacket;

#endif // !defined(STATE_PACKET_H)
