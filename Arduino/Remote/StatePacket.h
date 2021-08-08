#if !defined(STATE_PACKET_H)
#define STATE_PACKET_H

#include <Arduino.h>

typedef struct {
  uint8_t sequence_num_;

  #if 0
  bool button_top_left_;
  bool button_top_right_;
  bool button_right_pinky_;
  bool button_right_index_;
  bool button_top_pcb_left_;
  bool button_top_pcb_right_;

  bool button_joystick_;
  #endif
  short joystick_horizontal_, joystick_vertical_;  
} StatePacket;

#endif // !defined(STATE_PACKET_H)
