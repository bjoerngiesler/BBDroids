#if !defined(UI_H)
#define UI_H

#include <LibBBRemotes.h>
#include "UI/DroidVisWidget.h"
#include "UI/RWidget.h"
#include "UI/RMenuWidget.h"
#include "UI/RMessageWidget.h"
#include "UI/RGraphsWidget.h"
#include "UI/RCrosshairWidget.h"
#include "UI/RLabelWidget.h"
#include "UI/RIMUWidget.h"
#include "UI/RRemoteVisWidget.h"
#include "UI/RRotaWidget.h"
#include "UI/RSeqnumWidget.h"
#include "UI/RDialogWidget.h"

using namespace bb;
using namespace bb::rmt;

class UI {
public:
    static UI ui;

    void start();

    void setMainWidget(RWidget* widget);

    void showPairDroidMenu();
    void showPairRemoteMenu();
    void showMenu(RMenuWidget* menu);
    void showMain();

    void setNeedsMenuRebuild(bool yesno = true) { needsMenuRebuild_ = yesno; }
    void populateMenus();
    void drawGUI();
    void drawScreensaver();

    void setTopTitle(const String& title);

    // Other callbacks
    void setIncrRotButtonCB(Input::Button button, bool left);

    void showMessage(const String& str, unsigned int delayms=0, uint8_t color=Display::WHITE);
    void showDialog();
    void hideDialog();
    
    void showLEDBrightnessDialog();
    void showJoyDeadbandDialog();
    void showSendRepeatsDialog();
    void showCalibration(bb::PacketSource source);
    void hideCalibration(bb::PacketSource source);

    void setSeqnumState(bb::PacketSource source, bool active);
    void setNoComm(bb::PacketSource source, bool nocomm);

    RRemoteVisWidget& leftRemoteVis() { return remoteVisL_; }
    RRemoteVisWidget& rightRemoteVis() { return remoteVisR_; }

    void updateLeftSeqnum(uint8_t seqnum);
    void updateRightSeqnum(uint8_t seqnum);
    void visualizeFromTelemetry(Protocol* proto, const NodeAddr& addr, uint8_t seqnum, const Telemetry& telem);

    void toggleLockFaceButtons();

protected:
    UI();

    RMenuWidget mainMenu_, pairMenu_, pairDroidMenu_, pairRemoteMenu_;
    RMenuWidget leftRemoteMenu_, rightRemoteMenu_, bothRemotesMenu_, droidMenu_;
    RMenuWidget lRIncrRotMenu_, rRIncrRotMenu_;
    RMessageWidget message_;
    RDialogWidget dialog_;
    bool dialogActive_;
    RLabelWidget topLabel_, bottomLabel_, lockedLabel_;
    RSeqnumWidget leftSeqnum_, rightSeqnum_, droidSeqnum_;
    RRotaWidget mainVis_;
    RRemoteVisWidget remoteVisL_, remoteVisR_;
    DroidVisWidget droidVis_;
    RWidget titleWidget;

    RWidget* mainWidget_;
    RLabelWidget *ledBrightnessLabel_, *deadbandPercentLabel_, *sendRepeatsLabel_;

    bool needsMenuRebuild_, needsScreensaverRedraw_;

    std::vector<XBee::Node> discoveredNodes_;

    uint8_t lastRightSeqnum_, lastDroidSeqnum_;
};

#endif // UI_H