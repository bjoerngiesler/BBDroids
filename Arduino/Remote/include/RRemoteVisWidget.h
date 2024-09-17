#if !defined(RREMOTEVISWIDGET_H)
#define RREMOTEVISWIDGET_H

#include "RMultiWidget.h"
#include "RDisplay.h"
#include "RCrosshairWidget.h"
#include "RIMUWidget.h"
#include "RLabelWidget.h"
#include "RRoundScaleWidget.h"

class RRemoteVisWidget: public RMultiWidget {
public:
    RRemoteVisWidget();

    void setRepresentsLeftRemote(bool left);
    virtual void setSize(uint8_t w, uint8_t h) {}

    virtual void setPosition(int x, int y);
    virtual Result draw(ConsoleStream* stream);
    virtual Result visualizeFromPacket(const bb::ControlPacket& packet);


protected:
    void moveWidgetsAround();

    RCrosshairWidget crosshair_;
    RIMUWidget imu_;
    RLabelWidget mainBtns_[4];
    RLabelWidget topButtons_[3];
    RLabelWidget batteryState_[4];
    bool left_;
    uint8_t bodyWidth_, bodyHeight_;
    RRoundScaleWidget pot1_, pot2_;
};

#endif // RREMOTEVISWIDGET_H