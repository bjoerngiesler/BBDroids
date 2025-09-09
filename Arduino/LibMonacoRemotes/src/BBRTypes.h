#if !defined(BBRTYPES_H)
#define BBRTYPES_H

#include <sys/types.h>
#include <string>
#include <Arduino.h>

namespace bb {
namespace rmt {

static const uint8_t NAME_MAXLEN = 10;

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

    void fromMACAddress(const uint8_t addr[6]) {
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

// NodeDescription - 20 bytes
struct __attribute__ ((packed)) NodeDescription {
    NodeAddr addr;               // byte 0..7
    char name_[NAME_MAXLEN];     // byte 8..17
    bool isTransmitter     : 1;  // byte 18 bit 0
    bool isReceiver        : 1;  // byte 18 bit 1
    bool isConfigurator    : 1;  // byte 18 bit 2
    uint16_t protoSpecific : 13; // byte 18 bit 3..7, byte 19

    void setName(const std::string& n) {
        memset(name_, 0, 10);
        for(uint8_t i=0; i<n.length() && i<NAME_MAXLEN; i++) name_[i] = n[i];
    }
    void setName(const char* buf) {
        memset(name_, 0, 10);
        for(uint8_t i=0; i<NAME_MAXLEN; i++) {
            if(buf[i] == '\0') break;
            name_[i] = buf[i];
        } 
    }
    std::string getName() const {
        char buf[NAME_MAXLEN+1];
        memset(buf, 0, NAME_MAXLEN+1);
        memcpy(buf, name_, NAME_MAXLEN);
        return std::string(buf);
    }
};

// Some standard input names. Of course you can define your own but these help to programmatically decide what to map to what.
// Note that these are truncated to 10 chars in MCP packets, so don't make them too long.
static const std::string INPUT_SPEED        = "Speed";     // body speed over ground -- v_x, forward positive
static const std::string INPUT_TURN_RATE    = "TurnRate";  // body turn rate         -- v_alpha, left positive
static const std::string INPUT_DOME_RATE    = "DomeRate";  // dome turn rate         -- left positive
static const std::string INPUT_DOME_ANGLE   = "DomeAngle"; // dome absolute angle    -- left positive
static const std::string INPUT_TURN_ANGLE   = "TurnAngle"; // body absolute angle    -- alpha, left positive
static const std::string INPUT_HEAD_ROLL    = "HeadRoll";
static const std::string INPUT_HEAD_PITCH   = "HeadPitch";
static const std::string INPUT_HEAD_HEADING = "HeadHeadng"; // calling this "heading" because "yaw" is abbrev'd to "y" which is confusing
static const std::string INPUT_V_Y          = "v_y";        // for holonomous droids (e.g. B2EMO) - l/r motion, left positive
static const std::string INPUT_EMOTE_0      = "Emote0";     // sound or other emotion -- specific one from the 0 group if there are several
static const std::string INPUT_EMOTE_0_RND  = "Emote0Rnd";  // random sound from the 0 group
static const std::string INPUT_EMOTE_0_INC  = "Emote0Inc";  // next sound from the 0 group
static const std::string INPUT_EMOTE_1      = "Emote1";     // sound or other emotion
static const std::string INPUT_EMOTE_1_RND  = "Emote1Rnd";  // random sound from the 1 group
static const std::string INPUT_EMOTE_1_INC  = "Emote1Inc";  // next sound from the 1 group
static const std::string INPUT_EMOTE_2      = "Emote2";     // sound or other emotion
static const std::string INPUT_EMOTE_2_RND  = "Emote2Rnd";  // random sound from the 2 group
static const std::string INPUT_EMOTE_2_INC  = "Emote2Inc";  // next sound from the 2 group
static const std::string INPUT_EMOTE_3      = "Emote3";     // sound or other emotion
static const std::string INPUT_EMOTE_3_RND  = "Emote3Rnd";  // random sound from the 3 group
static const std::string INPUT_EMOTE_3_INC  = "Emote3Inc";  // next sound from the 3 group
static const std::string INPUT_EMOTE_4      = "Emote4";     // sound or other emotion
static const std::string INPUT_EMOTE_4_RND  = "Emote4Rnd";  // random sound from the 4 group
static const std::string INPUT_EMOTE_4_INC  = "Emote4Inc";  // next sound from the 4 group

struct Axis {
    std::string name;
    uint8_t bitDepth;
    uint32_t value;
};

enum MixType {
    MIX_NONE = 0, // don't mix -- input = a1
    MIX_ADD  = 1, // input = map(interp(a1) + interp(a2))
    MIX_MULT = 2, // input = map(p1*a1*a2)
};

static const uint8_t INPUT_INVALID = 255;
static const uint8_t AXIS_INVALID = 127;

struct Interpolator {
    int8_t i0, i25, i50, i75, i100;
};
static constexpr Interpolator INTERP_ZERO = {0, 0, 0, 0, 0};
static constexpr Interpolator INTERP_LIN_POSITIVE = {0, 25, 50, 75, 100};
static constexpr Interpolator INTERP_LIN_POSITIVE_INV = {100, 75, 50, 25, 0};
static constexpr Interpolator INTERP_LIN_CENTERED = {-100, -50, 0, 50, 100};
static constexpr Interpolator INTERP_LIN_CENTERED_INV = {100, 50, 0, -50, -100};

// FIXME bit of a problem - AxisInputMapping is 13 bytes (input, 2x 5-byte interpolator, 2x 7-bit axis, 2 bits mix type). 
// We really don't want to enlarge the config packet format, it's at 12 bytes max now, so we have to save 1 byte?
// Options --
// 1. don't care, just make it bigger.
// 2. steal bits from Interpolator structure by going to half resolution (2 instead of 1) - would save 10 bits, but at the expense of accuracy?
// 3. reduce number of possible inputs and axes - could maybe save 2 bits here? not enough.
struct __attribute__ ((packed)) AxisInputMapping {
public:
    uint8_t input = INPUT_INVALID;
    Interpolator interp1 = INTERP_LIN_POSITIVE;
    Interpolator interp2 = INTERP_LIN_POSITIVE;
    uint8_t axis1   : 7;
    uint8_t axis2   : 7;
    MixType mixType : 2;

    AxisInputMapping() { 
        input = INPUT_INVALID; 
        interp1 = INTERP_ZERO;
        interp2 = INTERP_ZERO;
        axis1 = AXIS_INVALID; 
        axis2 = AXIS_INVALID;
        mixType = MIX_NONE;
    }

    AxisInputMapping(uint8_t inp, uint8_t ax1, Interpolator ip1 = INTERP_LIN_CENTERED, 
                     uint8_t ax2=AXIS_INVALID, Interpolator ip2 = INTERP_ZERO,
                     MixType mix=MIX_NONE) { 
        input = inp; 
        axis1 = ax1; interp1 = ip1;
        axis2 = ax2; interp2 = ip2;
        mixType = mix;
    }

    float computeMix(float a1, float min1, float max1, float a2, float min2, float max2) const {
        float i0=0, i1=0, frac=0, b1=0, b2=0;
        
        if(axis1 == AXIS_INVALID && axis2 == AXIS_INVALID) return 0;

        frac = (a1-min1)/(max1-min1);
        if(frac >= 0 && frac <= 0.25) {
            i0 = float(interp1.i0)/100.0f;
            i1 = float(interp1.i25)/100.0f;
            frac = frac * 4;
        } else if(frac > 0.25 && frac <= 0.5) {
            i0 = float(interp1.i25)/100.0f;
            i1 = float(interp1.i50)/100.0f;
            frac = (frac-0.25)*4;
        } else if(frac > 0.5 && frac <= 0.75) {
            i0 = float(interp1.i50)/100.0f;
            i1 = float(interp1.i75)/100.0f;
            frac = (frac-0.5)*4;
        } else if(frac > 0.75 && frac <= 1) {
            i0 = float(interp1.i75)/100.0f;
            i1 = float(interp1.i100)/100.0f;
            frac = (frac-0.75)*4;
        }
        b1 = i0 + (i1-i0)*frac;

        frac = (a2-min2)/(max2-min2);
        if(frac >= 0 && frac <= 0.25) {
            i0 = float(interp2.i0)/100.0f;
            i1 = float(interp2.i25)/100.0f;
            frac = frac * 4;
        } else if(frac > 0.25 && frac <= 0.5) {
            i0 = float(interp2.i25)/100.0f;
            i1 = float(interp2.i50)/100.0f;
            frac = (frac-0.25)*4;
        } else if(frac > 0.5 && frac <= 0.75) {
            i0 = float(interp2.i50)/100.0f;
            i1 = float(interp2.i75)/100.0f;
            frac = (frac-0.5)*4;
        } else if(frac > 0.75 && frac <= 1) {
            i0 = float(interp2.i75)/100.0f;
            i1 = float(interp2.i100)/100.0f;
            frac = (frac-0.75)*4;
        }
        b2 = i0 + (i1-i0)*frac;

        // we capture the case of both invalid at the beginning of the function
        if(axis1 == AXIS_INVALID) return b2;
        if(axis2 == AXIS_INVALID) return b1;

        switch(mixType) {
        case MIX_ADD:
            return b1+b2;
            break;
        case MIX_MULT:
            return b1*b2;
            break;
        case MIX_NONE:
        default:
            return b1;
            break;
        }
    }
};

//! normed goes from 0 to 1, and gets transformed into [0..360], [-180..180], [-1..1] depending on Unit value.
static float normedToUnit(float normed, Unit unit) {
    normed = constrain(normed, 0.0f, 1.0f);
    switch(unit) {
    case UNIT_DEGREES:
        return normed * 360.0f;
    case UNIT_DEGREES_CENTERED:
        return (normed-0.5)*360.0f;
    case UNIT_UNITY_CENTERED:
        return (normed-0.5)*2.0f;
    default:
        break;
    }
    return normed;
}

//! val goes from [0..360], [-180..180], [-1..1] depending on Unit value. Returns value in range [0..1].
static float unitToNormed(float val, Unit unit) {
    switch(unit) {
    case UNIT_DEGREES:
        val = constrain(val, 0.0f, 360.0f);
        return val / 360.0f;
    case UNIT_DEGREES_CENTERED:
        val = constrain(val, -180.0f, 180.0f);
        return (val / 360.0f) + 0.5;
    case UNIT_UNITY_CENTERED:
        val = constrain(val, -1.0f, 1.0f);
        return (val / 2.0f) + 0.5;
    case UNIT_UNITY:
        return constrain(val, 0.0f, 1.0f);
        break;
    default:
        break;
    }
    return val;
}

}; // rmt
}; // bb


#endif // BBRTYPES_H