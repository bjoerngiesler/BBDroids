#if !defined(BBRTYPES_H)
#define BBRTYPES_H

#include <sys/types.h>
#include <string>

namespace bb {
namespace rmt {

enum Unit {
    UNIT_DEGREES          = 0, // [0..360)
    UNIT_DEGREES_CENTERED = 1, // (-180..180)
    UNIT_UNITY            = 2, // [0..1]
    UNIT_UNITY_CENTERED   = 3, // [-1..1]
    UNIT_RAW              = 4  // dependent on wire interface
};

enum NodeType {
    NODE_TRANSMITTER = 0,
    NODE_RECEIVER    = 1,
    NODE_OTHER       = 2,
    NODE_UNKNOWN     = 3
};

struct __attribute__ ((packed)) NodeAddr {
    uint8_t byte[8];

    uint32_t addrHi() const { 
       return byte[4] | (byte[5]>>8) | (byte[6]>>16) | (byte[7])>>24; 
    }

    uint32_t addrLo() const { 
        return byte[0] | (byte[1]>>8) | (byte[2]>>16) | (byte[3])>>24; 
    }

    bool isZero() const { 
        for(int i=0; i<8; i++) if(byte[i] != 0) return false;
        return true;
    }
    bool operator==(const NodeAddr& other) const { 
        for(int i=0; i<8; i++) if(byte[i] != other.byte[i]) return false;
        return true;
    }
    bool operator!=(const NodeAddr& other) const { 
        for(int i=0; i<8; i++) if(byte[i] != other.byte[i]) return true;
        return false;
    }

    void fromXBeeAddress(uint32_t addrHi, uint32_t addrLo) {
        for(int i=0; i<4; i++) byte[i] = (addrLo<<(8*i)) & 0xff;
        for(int i=4; i<8; i++) byte[i] = (addrHi<<(8*(i-4))) & 0xff;
    }

    void fromMACAddress(uint8_t addr[6]) {
        for(int i=0; i<6; i++) byte[i] = addr[i];
        byte[6] = byte[7] = 0;
    }

    void fromString(const std::string& str) {
        if(str.length() == 17 && 
           str[2] == str[5] == str[8] == str[11] == str[14] == ':') { // MAC address
            uint8_t m[6];
            sscanf(str.c_str(), "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]);
            fromMACAddress(m);
        } else if(str.length() == 17 && str[8] == ':' && str[2] != ':') {
            uint32_t hi, lo;
            sscanf(str.c_str(), "%lx:%lx", &hi, &lo);
            fromXBeeAddress(hi, lo);
        } else {
            for(int i=0; i<8; i++) byte[i] = 0;
        }
    }

    std::string toString() const {
        char buf[20] = "";
        if(byte[6] == 0 && byte[7] == 0) {
            sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", byte[0], byte[1], byte[2], byte[3], byte[4], byte[5]);
        } else {
            sprintf(buf, "%0lx:%0lx", addrHi(), addrLo());
        }
        return buf;
    }
};

struct NodeDescription {
    NodeAddr addr;
    NodeType type;
    std::string name;
    void* protocolSpecific;
};

enum MixType {
    MIX_MULT = 0, // input = map(p1*a1*a2)
    MIX_ADD  = 1, // input = map(p1*a1 + p2*a2)
    MIX_NONE = 2  // 
};

static const uint8_t INPUT_INVALID = 255;
static const uint8_t AXIS_INVALID = 255;

struct __attribute__ ((packed)) AxisInputMapping {
public:
    AxisInputMapping(uint8_t inp, uint8_t ax) { 
        input = inp; axis1 = ax; 
    }
    uint8_t input = INPUT_INVALID;
    int8_t interp1[5] = {0, 25, 50, 57, 100};
    uint8_t axis1 = AXIS_INVALID;
    int8_t interp2[5] = {0, 25, 50, 57, 100};
    uint8_t axis2 = AXIS_INVALID;
    MixType mixType = MIX_NONE;
};

static uint8_t AXIS_NONE = 0xff;


}; // rmt
}; // bb


#endif // BBRTYPES_H