#if !defined(REMOTEVISWIDGET_H)
#define REMOTEVISWIDGET_H

#include <LibBBRemotes.h>
#include "UI/MultiWidget.h"
#include "UI/Display.h"
#include "UI/CrosshairWidget.h"
#include "UI/IMUWidget.h"
#include "UI/Label.h"
#include "UI/RoundScaleWidget.h"

class RemoteVisWidget: public MultiWidget {
public:
    RemoteVisWidget();

    void setRepresentsLeftRemote(bool left);
    virtual void setSize(uint8_t w, uint8_t h) {}

    virtual void setPosition(int x, int y);
    virtual Result draw();
    virtual Result visualizeFromControlPacket(const MControlPacket& packet);
    virtual Result visualizeFromInput();
    virtual void takeInputFocus();
    void encTurn(float enc);

    std::shared_ptr<CrosshairWidget>& crosshair() { return crosshair_; }

    void selectPot1();
    void selectPot2();

protected:
    void moveWidgetsAround();

    shared_ptr<CrosshairWidget> crosshair_;
    shared_ptr<IMUWidget> imu_;
    shared_ptr<Label> mainBtns_[4];
    shared_ptr<Label> topButtons_[3];
    shared_ptr<Label> batteryState_[4];
    bool left_;
    uint8_t bodyWidth_, bodyHeight_;
    shared_ptr<RoundScaleWidget> pot1_, pot2_;
    enum PotSelected {
        POT1_SELECTED,
        POT2_SELECTED
    };
    PotSelected potSelected_;
};

#endif // RREMOTEVISWIDGET_H