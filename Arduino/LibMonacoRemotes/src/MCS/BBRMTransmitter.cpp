#include "BBRMTransmitter.h"

using namespace bb;
using namespace bb::rmt;

static const std::string EMPTY = "";

MTransmitter::MTransmitter(MProtocol *proto): 
    TransmitterBase<MProtocol>(proto) {
    axes_.push_back({ "Axis 0", MControlPacket::BITDEPTH1, 1<<(MControlPacket::BITDEPTH1)-1 });
    axes_.push_back({ "Axis 1", MControlPacket::BITDEPTH1, 1<<(MControlPacket::BITDEPTH1)-1 });
    axes_.push_back({ "Axis 2", MControlPacket::BITDEPTH1, 1<<(MControlPacket::BITDEPTH1)-1 });
    axes_.push_back({ "Axis 3", MControlPacket::BITDEPTH1, 1<<(MControlPacket::BITDEPTH1)-1 });
    axes_.push_back({ "Axis 4", MControlPacket::BITDEPTH1, 1<<(MControlPacket::BITDEPTH1)-1 });
    axes_.push_back({ "Axis 5", MControlPacket::BITDEPTH2, 1<<(MControlPacket::BITDEPTH2)-1 });
    axes_.push_back({ "Axis 6", MControlPacket::BITDEPTH2, 1<<(MControlPacket::BITDEPTH2)-1 });
    axes_.push_back({ "Axis 7", MControlPacket::BITDEPTH2, 1<<(MControlPacket::BITDEPTH2)-1 });
    axes_.push_back({ "Axis 8", MControlPacket::BITDEPTH2, 1<<(MControlPacket::BITDEPTH2)-1 });
    axes_.push_back({ "Axis 9", MControlPacket::BITDEPTH2, 1<<(MControlPacket::BITDEPTH2)-1 });
    axes_.push_back({ "Axis 10", MControlPacket::BITDEPTH3, 1<<(MControlPacket::BITDEPTH3)-1 });
    axes_.push_back({ "Axis 11", MControlPacket::BITDEPTH3, 1<<(MControlPacket::BITDEPTH3)-1 });
    axes_.push_back({ "Axis 12", MControlPacket::BITDEPTH3, 1<<(MControlPacket::BITDEPTH3)-1 });
    axes_.push_back({ "Axis 13", MControlPacket::BITDEPTH3, 1<<(MControlPacket::BITDEPTH3)-1 });
    axes_.push_back({ "Axis 14", MControlPacket::BITDEPTH3, 1<<(MControlPacket::BITDEPTH3)-1 });
    axes_.push_back({ "Axis 15", MControlPacket::BITDEPTH3, 1<<(MControlPacket::BITDEPTH3)-1 });
    axes_.push_back({ "Axis 16", MControlPacket::BITDEPTH3, 1<<(MControlPacket::BITDEPTH3)-1 });
    axes_.push_back({ "Axis 17", MControlPacket::BITDEPTH3, 1<<(MControlPacket::BITDEPTH3)-1 });
    axes_.push_back({ "Axis 18", MControlPacket::BITDEPTH4, 1<<(MControlPacket::BITDEPTH4)-1 });
}

uint8_t MTransmitter::numInputs() {
    return 0;
}

const std::string& MTransmitter::inputName(uint8_t input) {
    return EMPTY;
}

Result MTransmitter::transmit(){
    MPacket packet;

    packet.type = MPacket::PACKET_TYPE_CONTROL;
    MControlPacket& p = packet.payload.control;
    for(uint8_t i=0; i<axes_.size(); i++)
        p.setAxis(i, axes_[i].value, UNIT_RAW);

    protocol_->sendPacket(protocol_->pairedReceivers(), packet);

    return RES_OK;
}

bool MTransmitter::syncReceiverSideMapping() {
    return false;
}