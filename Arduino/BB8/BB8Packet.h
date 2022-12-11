#if !defined(BB8STATE_PACKET_H)
#define BB8STATE_PACKET_H

#include <sys/types.h>

typedef enum {
  // _NR commands do not prompt a reply on REPLY_UDP_PORT since the value they are changing is included in the
  // running status message. 
  CMD_SET_SERVO_NR = 0,             // IndexedFloat argument, index 1..4, param range between 0.0f and 360.0f degrees
  CMD_SET_DRIVE_MOTOR_SPEED_NR = 1, // Float argument, param range FIXME
  CMD_SET_TURN_MOTOR_SPEED_NR = 2,  // Float argument, param range FIXME
  CMD_SET_ALL_MOTORS_FLAGS_NR = 3,  // FlagFloatListArg, turn on flags to specify which motors to move

  // These commands *do* prompt a reply on REPLY_UDP_PORT.
  CMD_GET_DROID_NAME = 32,           // Argument is ignored. The reply has the droid name (or "BB8" if none set) in the uint8BufArg argument.
  CMD_SET_DROID_NAME = 33,           // uint8BufArg argument. Prompts a REPLY_ERROR or REPLY_OK reply.
  CMD_GET_SOUND_LIST = 34,           // Argument is ignored. This one is special, as it will trigger multiple replies, each with a CMD_GET_SOUND_LIST and uint8BufArg argument.

  // These are reply only; if you send them to the droid they are ignored.
  REPLY_ERROR = 62,                  // uint8BufArg containing an error string.
  REPLY_OK = 63                      // Argument undefined.
} Command;

// Flags and indexes for CMD_SET_ALL_MOTORS_FLAGS_NR
const uint8_t SERVO_1_FLAG     = 0x01;
const uint8_t SERVO_2_FLAG     = 0x02;
const uint8_t SERVO_3_FLAG     = 0x04;
const uint8_t SERVO_4_FLAG     = 0x08;
const uint8_t DRIVE_MOTOR_FLAG = 0x10;
const uint8_t TURN_MOTOR_FLAG  = 0x20;
const uint8_t UNUSED_1_FLAG    = 0x40;
const uint8_t UNUSED_2_FLAG    = 0x80;

const uint8_t SERVO_1_INDEX     = 0;
const uint8_t SERVO_2_INDEX     = 1;
const uint8_t SERVO_3_INDEX     = 2;
const uint8_t SERVO_4_INDEX     = 3;
const uint8_t DRIVE_MOTOR_INDEX = 4;
const uint8_t TURN_MOTOR_INDEX  = 5;
const uint8_t UNUSED_1_INDEX    = 6;
const uint8_t UNUSED_2_INDEX    = 7;

struct __attribute__((packed)) FloatArg {
  float param;
};

struct __attribute__((packed)) IndexedFloatArg {
  uint8_t index;
  float param;
};

struct __attribute__((packed)) FlagFloatListArg {
  uint8_t flags;
  float param[8];
};

struct __attribute__((packed)) UInt8BufArg {
  uint8_t buf[255];
};

// BB8CommandPacket
// This goes from the remote to BB8
typedef struct __attribute__((packed)) {
  uint8_t seqnum;
  Command cmd;
  union {
    struct FloatArg floatArg;
    struct IndexedFloatArg indexedFloatArg;
    struct FlagFloatListArg flagFloatListArg;
    struct UInt8BufArg uint8BufArg;
  } arg;
} BB8CommandPacket;

// BB8StatePacket
// This goes from BB8 to the remote
typedef struct {
  uint8_t seqnum;
  bool imusOK, motorsOK, servosOK;
  int16_t domeimuX, domeimuY, domeimuZ;
  int16_t bodyimuX, bodyimuY, bodyimuZ;
  float servo[4];
} BB8StatePacket;

extern BB8StatePacket state;

#endif // !defined(STATE_PACKET_H)
