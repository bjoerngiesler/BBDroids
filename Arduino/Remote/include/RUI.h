#if !defined(RUI_H)
#define RUI_H

#include "RWidget.h"
#include "RMenuWidget.h"
#include "RMessageWidget.h"
#include "RGraphsWidget.h"
#include "RCrosshairWidget.h"
#include "RLabelWidget.h"
#include "RIMUWidget.h"
#include "RRemoteVisWidget.h"
#include "RDroidVisWidget.h"
#include "RRotaWidget.h"
#include "RSeqnumWidget.h"
#include "RDialogWidget.h"

using namespace bb;

class RUI {
public:
    static RUI ui;

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
    void setIncrRotButtonCB(RInput::Button button, bool left);

    void showMessage(const String& str, unsigned int delayms=0, uint8_t color=RDisplay::WHITE);
    void showDialog();
    void hideDialog();
    
    void showLEDBrightnessDialog();
    void showJoyDeadbandDialog();
    void showSendRepeatsDialog();
    void showCalibration(bb::PacketSource source);
    void hideCalibration(bb::PacketSource source);

    void setSeqnumState(bb::PacketSource source, bool active);
    void setSeqnumNoComm(bb::PacketSource source, bool nocomm);

    void visualizeFromControlPacket(bb::PacketSource source, uint8_t seqnum, const bb::ControlPacket& packet);
    void visualizeFromStatePacket(bb::PacketSource source, uint8_t seqnum, const bb::StatePacket& packet);

    void toggleLockFaceButtons();

protected:
    RUI();

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
    RDroidVisWidget droidVis_;
    RWidget titleWidget;

    RWidget* mainWidget_;
    RLabelWidget *ledBrightnessLabel_, *deadbandPercentLabel_, *sendRepeatsLabel_;

    bool needsMenuRebuild_, needsScreensaverRedraw_;

    std::vector<XBee::Node> discoveredNodes_;

    uint8_t lastRightSeqnum_, lastDroidSeqnum_;
};

#endif // RUI_H