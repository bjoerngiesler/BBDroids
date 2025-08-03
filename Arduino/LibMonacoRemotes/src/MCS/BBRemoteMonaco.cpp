#if 0
#include "BBRemoteMonaco.h"
#include "BBPacket.h"

bb::Remote::Monaco::Monaco() {
    primary_ = true;
    left_ = true;
    axes_ = {{JOY_H_NAME, 0, UNIT_UNITY_CENTERED},
             {JOY_V_NAME, 0, UNIT_UNITY_CENTERED},
             {IMU_P_NAME, 0, UNIT_DEGREES_CENTERED},
             {IMU_R_NAME, 0, UNIT_DEGREES_CENTERED},
             {IMU_H_NAME, 0, UNIT_DEGREES_CENTERED},
             {IMU_AX_NAME, 0, UNIT_UNITY_CENTERED},
             {IMU_AY_NAME, 0, UNIT_UNITY_CENTERED},
             {IMU_AZ_NAME, 0, UNIT_UNITY_CENTERED},
             {POT1_NAME, 0, UNIT_UNITY},
             {POT2_NAME, 0, UNIT_UNITY}};
    triggers_ = {{TRIG_0_NAME, false},
                 {TRIG_1_NAME, false},
                 {TRIG_2_NAME, false},
                 {TRIG_3_NAME, false},
                 {TRIG_JOY_NAME, false},
                 {TRIG_TL_NAME, false},
                 {TRIG_TR_NAME, false},
                 {TRIG_CONFIRM_NAME, false}};
}

bool bb::Remote::Monaco::isPrimarySender() {
    return primary_;
}

void bb::Remote::Monaco::setPrimarySender(bool yn) {
    primary_ = yn;
}

bool bb::Remote::Monaco::isLeftSender() {
    return left_;
}

void bb::Remote::Monaco::setLeftSender(bool yn) {
    left_ = yn;
}

bb::Packet bb::Remote::Monaco::convertToPacket(unsigned long seqnum) {
    bb::Packet packet(bb::PACKET_TYPE_CONTROL, 
                      left_ ? bb::PACKET_SOURCE_LEFT_REMOTE : bb::PACKET_SOURCE_RIGHT_REMOTE,
                      seqnum);

    packet.payload.control.primary = isPrimarySender();

    packet.payload.control.setAxis(0, axes_[JOY_H].value, axes_[JOY_H].unit);
    packet.payload.control.setAxis(1, axes_[JOY_V].value, axes_[JOY_V].unit);
    packet.payload.control.setAxis(2, axes_[IMU_P].value, axes_[IMU_P].unit);
    packet.payload.control.setAxis(3, axes_[IMU_R].value, axes_[IMU_R].unit);
    packet.payload.control.setAxis(4, axes_[IMU_H].value, axes_[IMU_H].unit);
    packet.payload.control.setAxis(5, axes_[IMU_AX].value, axes_[IMU_AX].unit);
    packet.payload.control.setAxis(6, axes_[IMU_AY].value, axes_[IMU_AY].unit);
    packet.payload.control.setAxis(7, axes_[IMU_AZ].value, axes_[IMU_AZ].unit);
    packet.payload.control.setAxis(8, axes_[POT1].value, axes_[POT1].unit);
    packet.payload.control.setAxis(9, axes_[POT2].value, axes_[POT2].unit);
    packet.payload.control.setAxis(10, axes_[BATT].value, axes_[BATT].unit);

    packet.payload.control.button0 = triggers_[TRIG_0].value;
    packet.payload.control.button1 = triggers_[TRIG_1].value;
    packet.payload.control.button2 = triggers_[TRIG_2].value;
    packet.payload.control.button3 = triggers_[TRIG_3].value;
    packet.payload.control.button4 = triggers_[TRIG_JOY].value;
    packet.payload.control.button5 = triggers_[TRIG_TL].value;
    packet.payload.control.button6 = triggers_[TRIG_TR].value;
    packet.payload.control.button7 = triggers_[TRIG_CONFIRM].value;

    return packet;
}


Result bb::Remote::MonacoXBee::sendTo(UUID uuid, unsigned long seqnum) {
    bb::Packet packet = convertToPacket(seqnum);
    return XBee::xbee.sendTo(uuid, packet, false);
}

#endif