#if !defined(UI_H)
#define UI_H

#include <LibBBRemotes.h>

#include "Menu.h"

#include "UI/DroidVisWidget.h"
#include "UI/Widget.h"
#include "UI/Menu.h"
#include "UI/MessageWidget.h"
#include "UI/GraphsWidget.h"
#include "UI/CrosshairWidget.h"
#include "UI/Label.h"
#include "UI/IMUWidget.h"
#include "UI/RemoteVisWidget.h"
#include "UI/RotaWidget.h"
#include "UI/SeqnumWidget.h"
#include "UI/Dialog.h"
#include "UI/Button.h"
#include "UI/MixCurveDialog.h"

using namespace bb;
using namespace bb::rmt;
using namespace std;

class UI {
public:
    static UI ui;

    void start();

    void setMainWidget(Widget* widget);

    void populateConfigMenu(Menu* menu);
    void populatePairDroidMenu(Menu* menu);
    void populatePairRemoteMenu(Menu* menu);
    void populateMappingMenu(Menu* menu);
    void populateMappingMenu(Menu* menu, const NodeDescription& n);
    void populateMappingMenu(Menu* menu, const NodeDescription& n, Transmitter* tx, MixManager* mgr, InputID inp);

    void showMenu(const shared_ptr<Menu>& menu);
    void showMain();

    void setNeedsMenuRebuild(bool yesno = true) { needsMenuRebuild_ = yesno; }
    void populateMenus();
    void drawGUI();
    void drawScreensaver();

    void setTopTitle(const String& title);

    // Other callbacks
    void setIncrRotButtonCB(Input::Button button, bool left);
    void saveCurrentConfigAsCB(Dialog* dialog);

    void showMessage(const String& str, unsigned int delayms=0, uint8_t color=Display::WHITE);
    void showDialog(Widget* dialog);
    void hideDialog();
    
    void showLEDBrightnessDialog();
    void showJoyDeadbandDialog();
    void showSendRepeatsDialog();
    void showCalibration(bb::PacketSource source);
    void hideCalibration(bb::PacketSource source);

    void setSeqnumState(bb::PacketSource source, bool active);
    void setNoComm(bb::PacketSource source, bool nocomm);

    shared_ptr<RemoteVisWidget> leftRemoteVis() { return remoteVisL_; }
    shared_ptr<RemoteVisWidget> rightRemoteVis() { return remoteVisR_; }

    void updateLeftSeqnum(uint8_t seqnum);
    void updateRightSeqnum(uint8_t seqnum);
    void visualizeFromTelemetry(Protocol* proto, const NodeAddr& addr, uint8_t seqnum, const Telemetry& telem);

    void toggleLockFaceButtons();

protected:
    UI();

    shared_ptr<Menu> main_;
    //MenuItem config_;

    shared_ptr<Button> testBtn_;
    shared_ptr<Menu> mainMenu_, configMenu_, pairMenu_, pairDroidMenu_, pairRemoteMenu_;
    shared_ptr<Menu>leftRemoteMenu_, rightRemoteMenu_, bothRemotesMenu_, droidMenu_;
    shared_ptr<Menu>lRIncrRotMenu_, rRIncrRotMenu_;
    shared_ptr<MessageWidget> message_;
    shared_ptr<Dialog> valueDialog_;
    shared_ptr<MixCurveDialog> mixCurveDialog_;
    
    bool dialogActive_;
    Widget *dialog_;

    shared_ptr<Label> topLabel_, bottomLabel_, lockedLabel_;
    shared_ptr<SeqnumWidget> leftSeqnum_, rightSeqnum_, droidSeqnum_;
    shared_ptr<RotaWidget> mainVis_;
    shared_ptr<RemoteVisWidget> remoteVisL_, remoteVisR_;
    shared_ptr<DroidVisWidget> droidVis_;
    shared_ptr<Widget> titleWidget;

    Widget* mainWidget_;
    shared_ptr<Label> ledBrightnessLabel_, deadbandPercentLabel_, sendRepeatsLabel_;

    bool needsMenuRebuild_, needsScreensaverRedraw_;

    uint8_t lastRightSeqnum_, lastDroidSeqnum_;
};

#endif // UI_H