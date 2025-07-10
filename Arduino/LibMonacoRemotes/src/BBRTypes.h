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
        byte[7] = byte[8] = 0;
    }

    void fromString(const std::string& str) {
    }
    std::string toString() {
        return "";
    }
};

struct NodeDescription {
    NodeAddr addr;
    std::string name;
};


}; // rmt
}; // bb


#endif // BBRTYPES_H