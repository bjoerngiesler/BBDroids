#if 0
#if !defined(BBMCSXBEEBACKEND_H)
#define BBMCSXBEEBACKEND_H
#include "BBRemote.h"
#include "BBXBee.h"

namespace bb {
namespace Remote {
class Monaco: public Remote::Sender {
public:
    enum AxisID {
        JOY_H  = 0,
        JOY_V  = 1,
        IMU_P  = 2,
        IMU_R  = 3,
        IMU_H  = 4,
        IMU_AX = 5,
        IMU_AY = 6,
        IMU_AZ = 7,
        POT1   = 8,
        POT2   = 9,
        BATT   = 10
    };
    constexpr static const char *JOY_H_NAME  = "joy_h";
    constexpr static const char *JOY_V_NAME  = "joy_v";
    constexpr static const char *IMU_P_NAME  = "imu_p";
    constexpr static const char *IMU_R_NAME  = "imu_r";
    constexpr static const char *IMU_H_NAME  = "imu_h";
    constexpr static const char *IMU_AX_NAME = "imu_ax";
    constexpr static const char *IMU_AY_NAME = "imu_ax";
    constexpr static const char *IMU_AZ_NAME = "imu_ax";
    constexpr static const char *POT1_NAME   = "pot1";
    constexpr static const char *POT2_NAME   = "pot2";
    constexpr static const char *BATT_NAME   = "batt";
    
    enum TriggerID {
        TRIG_0       = 0,
        TRIG_1       = 1,
        TRIG_2       = 2,
        TRIG_3       = 3,
        TRIG_JOY     = 4,
        TRIG_TL      = 5,
        TRIG_TR      = 6,
        TRIG_CONFIRM = 7
    };
    constexpr static const char *TRIG_0_NAME        = "trig_0";
    constexpr static const char *TRIG_1_NAME        = "trig_1";
    constexpr static const char *TRIG_2_NAME        = "trig_2";
    constexpr static const char *TRIG_3_NAME        = "trig_3";
    constexpr static const char *TRIG_JOY_NAME      = "trig_joy";
    constexpr static const char *TRIG_TL_NAME       = "trig_tl";
    constexpr static const char *TRIG_TR_NAME       = "trig_tr";
    constexpr static const char *TRIG_CONFIRM_NAME  = "trig_confirm";

    Monaco();
    virtual bool isPartOfDualSystem() { return true; }

    virtual bool isPrimarySender();
    virtual void setPrimarySender(bool yn);

    virtual bool isLeftSender();
    virtual void setLeftSender(bool yn);

    virtual bb::Packet convertToPacket(unsigned long seqnum);
protected:
    bool primary_, left_;
};

class MonacoXBee: public Monaco {
public:
    MonacoXBee();
    virtual Result sendTo(UUID uuid, unsigned long seq);
};

#if 0
class MonacoBluetooth: publich Monaco {
public:
    Result sendTo(UUID uuid);
};

class MonacoESPNow: publich Monaco {
public:
    Result sendTo(UUID uuid);
};
#endif
};
};
#endif
#endif