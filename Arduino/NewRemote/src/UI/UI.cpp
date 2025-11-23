#include "UI/UI.h"
#include "Todo/RRemote.h"

UI UI::ui;

UI::UI() {
    message_.setTitle("?");

    remoteVisL_.setRepresentsLeftRemote(true);
    remoteVisL_.setName("Left Remote");
    remoteVisR_.setRepresentsLeftRemote(false);
    remoteVisR_.setName("Right Remote");
    droidVis_.setName("Droid");
    mainVis_.addWidget(&remoteVisL_);
    mainVis_.addWidget(&remoteVisR_);
    mainVis_.addWidget(&droidVis_);
  
    topLabel_.setSize(Display::DISPLAY_WIDTH, Display::CHAR_HEIGHT);
    topLabel_.setPosition(0, 0);
    topLabel_.setFrameType(RLabelWidget::FRAME_BOTTOM);
    topLabel_.setTitle("Top Label");

    lockedLabel_.setSize(Display::DISPLAY_WIDTH, Display::CHAR_HEIGHT);
    lockedLabel_.setPosition(0, 0);
    lockedLabel_.setFrameType(RLabelWidget::FRAME_BOTTOM);
    lockedLabel_.setBackgroundColor(Display::DARKBLUE);
    lockedLabel_.setTitle("LOCKED");
  
    bottomLabel_.setSize(Display::DISPLAY_WIDTH, Display::CHAR_HEIGHT+4);
    bottomLabel_.setPosition(0, Display::DISPLAY_HEIGHT-Display::CHAR_HEIGHT-3);
    bottomLabel_.setJustification(RLabelWidget::LEFT_JUSTIFIED, RLabelWidget::BOTTOM_JUSTIFIED);
    bottomLabel_.setFrameType(RLabelWidget::FRAME_TOP);
    bottomLabel_.setTitle("");
  
    leftSeqnum_.setSize(23, 8);
    leftSeqnum_.setPosition(1, Display::DISPLAY_HEIGHT-Display::CHAR_HEIGHT);
    leftSeqnum_.setFrameColor(Display::LIGHTBLUE2);
    leftSeqnum_.setChar('L');
    rightSeqnum_.setSize(23, 8);
    rightSeqnum_.setPosition(leftSeqnum_.x()+leftSeqnum_.width()+4, leftSeqnum_.y());
    rightSeqnum_.setChar('R');
    droidSeqnum_.setSize(23, 8);
    droidSeqnum_.setPosition(rightSeqnum_.x()+rightSeqnum_.width()+4, leftSeqnum_.y());
    droidSeqnum_.setChar('D');
  
    dialog_.setTitle("Hi! :-)");
    dialog_.setValue(5);
    dialog_.setRange(0, 10);  
}

void UI::start() {
    populateMenus();
    showMain();
    drawGUI();
    delay(10);
    mainWidget_->setNeedsFullRedraw();
    leftSeqnum_.setNeedsFullRedraw();
    droidSeqnum_.setNeedsFullRedraw();
    rightSeqnum_.setNeedsFullRedraw();  
}

void UI::setMainWidget(RWidget* widget) {
    widget->setPosition(Display::MAIN_X, Display::MAIN_Y);
    widget->setSize(Display::MAIN_WIDTH, Display::MAIN_HEIGHT);
    Input::inst.clearCallbacks();
    widget->takeInputFocus();
    widget->setNeedsFullRedraw();
    widget->setNeedsContentsRedraw();
    setTopTitle(widget->name());
    mainWidget_ = widget;
}
  
void UI::showPairDroidMenu() {
    showMessage("Please wait");
    
    pairDroidMenu_.clear();
  
    Console::console.printfBroadcast("Discovering nodes...\n");
    Result res = XBee::xbee.discoverNodes(discoveredNodes_);
    if(res != RES_OK) {
      Console::console.printfBroadcast("%s\n", errorMessage(res));
      return;
    }
    Console::console.printfBroadcast("%d nodes discovered.\n", discoveredNodes_.size());
  
    pairDroidMenu_.clear();
    int num = 0;
    for(auto& n: discoveredNodes_) {
      bb::printf("Found node at 0x%lx:%lx, sending PAIRING_INFO_REQ\n", n.address.addrHi, n.address.addrLo);
      bb::PairingPacket pairing;
      pairing.type = PairingPacket::PAIRING_INFO_REQ;
      Result res = RES_COMM_TIMEOUT;
      for(int i=0; i<5 && res == RES_COMM_TIMEOUT; i++) {
        res = XBee::xbee.sendPairingPacket(n.address, PACKET_SOURCE_LEFT_REMOTE, pairing, Runloop::runloop.sequenceNumber());
      }
      
      if(res != RES_OK) {
        bb::printf("Couldn't send PAIRING_INFO_REQ: %s\n", errorMessage(res));
        continue;
      } else if(pairing.type != PairingPacket::PAIRING_INFO_REQ) {
        bb::printf("PAIRING_INFO_REQ returned %d packet instead of %d\n", pairing.type, PairingPacket::PAIRING_INFO_REQ);
        continue;
      } else if(pairing.pairingPayload.info.packetSource != PACKET_SOURCE_DROID) {
        bb::printf("This is not a droid but a %d\n", pairing.pairingPayload.info.packetSource);
        continue;
      }
      bb::printf("This is a droid, adding to the menu.\n");
      pairDroidMenu_.addEntry(n.name, [=]() { RRemote::remote.selectDroid(n.address); showMain(); });
      num++;
    }
    pairDroidMenu_.setName(String(num) + " Droids");
    pairDroidMenu_.addEntry("<--", [=]() { UI::ui.showMenu(&pairMenu_); });
  
    Console::console.printfBroadcast("Showing droidsMenu\n");
    showMenu(&pairDroidMenu_);
}
  
void UI::showPairRemoteMenu() {
    showMessage("Please wait");
  
    pairRemoteMenu_.clear();
  
    Result res = XBee::xbee.discoverNodes(discoveredNodes_);
    if(res != RES_OK) {
      Console::console.printfBroadcast("%s\n", errorMessage(res));
      showMenu(&pairMenu_);
      return;
    }
  
    int num = 0;
    for(auto& n: discoveredNodes_) {
      bb::PairingPacket pairing;
      pairing.type = PairingPacket::PAIRING_INFO_REQ;
      Result res = RES_COMM_TIMEOUT;
      for(int i=0; i<5 && res == RES_COMM_TIMEOUT; i++) {
        res = XBee::xbee.sendPairingPacket(n.address, PACKET_SOURCE_LEFT_REMOTE, pairing, Runloop::runloop.sequenceNumber());
      }
      if(res != RES_OK || pairing.type != PairingPacket::PAIRING_INFO_REQ || pairing.pairingPayload.info.packetSource != PACKET_SOURCE_RIGHT_REMOTE) 
        continue;

      pairRemoteMenu_.addEntry(n.name, [=]() { RRemote::remote.selectRightRemote(n.address); showMain(); });
      num++;
    }
    pairRemoteMenu_.addEntry("<--", [=]() { UI::ui.showMenu(&pairMenu_); });
    pairRemoteMenu_.setName(String(num) + " Remotes");
  
    showMenu(&pairRemoteMenu_);
}
  
void UI::showMenu(RMenuWidget* menu) {
    setMainWidget(menu);
    menu->resetCursor();
}
  
void UI::showMain() {
    setMainWidget(&mainVis_);
    Input::inst.setConfirmShortPressCallback([=]{UI::ui.showMenu(&mainMenu_);});
    Input::inst.setConfirmLongPressCallback([=]{UI::ui.toggleLockFaceButtons();});
}

void UI::populateMenus() {
    mainMenu_.setName("Main Menu");



    pairMenu_.setName("Pair");
    leftRemoteMenu_.setName("Left Remote");
    rightRemoteMenu_.setName("Right Remote");
    bothRemotesMenu_.setName("Both Remotes");
    droidMenu_.setName("Droid");
    
    mainMenu_.clear();
    if(!RRemote::remote.droidAddress().isZero()) mainMenu_.addEntry("Droid...", [=]() { showMenu(&droidMenu_); });
    mainMenu_.addEntry("Left Remote...", [=]() { showMenu(&leftRemoteMenu_); });
    if(!RRemote::remote.otherRemoteAddress().isZero()) mainMenu_.addEntry("Right Remote...", [=]() { showMenu(&rightRemoteMenu_); });
    mainMenu_.addEntry("Both Remotes...", [=](){showMenu(&bothRemotesMenu_);});
    mainMenu_.addEntry("Pair...", [=]() { showMenu(&pairMenu_); });
    mainMenu_.addEntry("<--", [=]() { showMain(); });
    
    pairMenu_.clear();
    pairMenu_.addEntry("Right Remote...", [=]() { showPairRemoteMenu(); });
    pairMenu_.addEntry("Droid...", [=]() { showPairDroidMenu(); });
    pairMenu_.addEntry("<--", [=]() { showMenu(&mainMenu_); });

    droidMenu_.clear();
    droidMenu_.addEntry("<--", [=]() {showMenu(&mainMenu_);});

    lRIncrRotMenu_.clear();
    lRIncrRotMenu_.setName("Incr Rotation");
    lRIncrRotMenu_.addEntry("Disable", [=]{setIncrRotButtonCB(Input::BUTTON_NONE, true);showMain();}, Input::BUTTON_NONE);
    lRIncrRotMenu_.addEntry("Button 1", [=]{setIncrRotButtonCB(Input::BUTTON_1, true);showMain();}, Input::BUTTON_1);
    lRIncrRotMenu_.addEntry("Button 2", [=]{setIncrRotButtonCB(Input::BUTTON_2, true);showMain();}, Input::BUTTON_2);
    lRIncrRotMenu_.addEntry("Button 3", [=]{setIncrRotButtonCB(Input::BUTTON_3, true);showMain();}, Input::BUTTON_3);
    lRIncrRotMenu_.addEntry("Button 4", [=]{setIncrRotButtonCB(Input::BUTTON_4, true);showMain();}, Input::BUTTON_4);
    lRIncrRotMenu_.addEntry("<--", [=]{showMenu(&leftRemoteMenu_);});
    lRIncrRotMenu_.highlightWidgetsWithTag(RRemote::remote.incrRotButton(PACKET_SOURCE_LEFT_REMOTE));

    rRIncrRotMenu_.clear();
    rRIncrRotMenu_.setName("Incr Rotation");
    rRIncrRotMenu_.addEntry("Disable", [=]{setIncrRotButtonCB(Input::BUTTON_NONE, false);showMain();}, Input::BUTTON_NONE);
    rRIncrRotMenu_.addEntry("Button 1", [=]{setIncrRotButtonCB(Input::BUTTON_1, false);showMain();}, Input::BUTTON_1);
    rRIncrRotMenu_.addEntry("Button 2", [=]{setIncrRotButtonCB(Input::BUTTON_2, false);showMain();}, Input::BUTTON_2);
    rRIncrRotMenu_.addEntry("Button 3", [=]{setIncrRotButtonCB(Input::BUTTON_3, false);showMain();}, Input::BUTTON_3);
    rRIncrRotMenu_.addEntry("Button 4", [=]{setIncrRotButtonCB(Input::BUTTON_4, false);showMain();}, Input::BUTTON_4);
    rRIncrRotMenu_.addEntry("<--", [=]{showMenu(&rightRemoteMenu_);});
    rRIncrRotMenu_.highlightWidgetsWithTag(RRemote::remote.incrRotButton(PACKET_SOURCE_RIGHT_REMOTE));

    leftRemoteMenu_.clear();
    leftRemoteMenu_.addEntry("Incr Rot...", [=]{showMenu(&lRIncrRotMenu_);});
    leftRemoteMenu_.addEntry("Calib Joystick", [=]{RRemote::remote.startCalibration();});
    leftRemoteMenu_.addEntry("Set Primary", [=]{RRemote::remote.setLeftIsPrimary(true);showMain();}, 
                             (RRemote::remote.primary() == PACKET_SOURCE_LEFT_REMOTE)?1:0);
    leftRemoteMenu_.addEntry("Factory Reset", [=]{RRemote::remote.factoryReset();});
    leftRemoteMenu_.addEntry("<--", [=]() {showMenu(&mainMenu_);});
    leftRemoteMenu_.highlightWidgetsWithTag(1);

    rightRemoteMenu_.clear();
    rightRemoteMenu_.addEntry("Incr Rot...", [=]{showMenu(&rRIncrRotMenu_);});
    rightRemoteMenu_.addEntry("Calib Joystick", [=]{RRemote::remote.sendStartCalibration();});
    rightRemoteMenu_.addEntry("Set Primary", [=]{RRemote::remote.setLeftIsPrimary(false);showMain();}, 
                              (RRemote::remote.primary() == PACKET_SOURCE_LEFT_REMOTE)?0:1);
    rightRemoteMenu_.addEntry("Factory Reset", [=]() {RRemote::remote.sendFactoryReset();});
    rightRemoteMenu_.addEntry("<--", [=]() {showMenu(&mainMenu_);});
    rightRemoteMenu_.highlightWidgetsWithTag(1);

    bothRemotesMenu_.clear();
    bothRemotesMenu_.addEntry("LED Level", [=]{showLEDBrightnessDialog();});
    bothRemotesMenu_.addEntry("Joy Deadband", [=]{showJoyDeadbandDialog();});
    bothRemotesMenu_.addEntry("Send Repeats", [=]{showSendRepeatsDialog();});
    bothRemotesMenu_.addEntry("<--", [=]() {showMenu(&mainMenu_);});
}

void UI::drawGUI() {
    if(Input::inst.faceButtonsLocked()) {
        lockedLabel_.draw();
    } else {
        topLabel_.draw();
    }
    bottomLabel_.draw();
    leftSeqnum_.draw();
    rightSeqnum_.draw();
    droidSeqnum_.draw();
    if(dialogActive_) dialog_.draw();
    else if(mainWidget_ != NULL) mainWidget_->draw();
    if(needsMenuRebuild_) {
      populateMenus();
      needsMenuRebuild_ = false;
    }
    needsScreensaverRedraw_ = true;
}

void UI::drawScreensaver() {
    if(needsScreensaverRedraw_) {
        Display::display.rect(0, 0, Display::DISPLAY_WIDTH, Display::DISPLAY_HEIGHT, Display::BLACK, true);
        topLabel_.setNeedsFullRedraw();
        bottomLabel_.setNeedsFullRedraw();
        leftSeqnum_.setNeedsFullRedraw();
        rightSeqnum_.setNeedsFullRedraw();
        droidSeqnum_.setNeedsFullRedraw();
        if(dialogActive_) dialog_.setNeedsFullRedraw();
        if(mainWidget_ != nullptr) mainWidget_->setNeedsFullRedraw();
        needsScreensaverRedraw_ = false;
    }
}

void UI::setTopTitle(const String& title) {
    topLabel_.setTitle(title);
}  

void UI::setIncrRotButtonCB(Input::Button button, bool left) {
    if(left) {
        if(RRemote::remote.incrRotButton(PACKET_SOURCE_LEFT_REMOTE) == button) return;
        Input::inst.setIncrementalRot(button);
        RRemote::remote.setIncrRotButton(PACKET_SOURCE_LEFT_REMOTE, button);
        lRIncrRotMenu_.highlightWidgetsWithTag(button);
        RRemote::remote.storeParams();
    } else {
        Input::Button tempBtn = RRemote::remote.incrRotButton(PACKET_SOURCE_RIGHT_REMOTE);
        if(tempBtn == button) return;
        RRemote::remote.setIncrRotButton(PACKET_SOURCE_RIGHT_REMOTE, button);
        rRIncrRotMenu_.highlightWidgetsWithTag(button);
        if(RRemote::remote.sendConfigToRightRemote() != RES_OK) {
            // couldn't send to right remote -- roll back to old setting
            RRemote::remote.setIncrRotButton(PACKET_SOURCE_RIGHT_REMOTE, tempBtn);
        } else {
            RRemote::remote.storeParams();
        }
    }
    needsMenuRebuild_ = true;
}
       
    
void UI::showMessage(const String& msg, unsigned int delayms, uint8_t color) {
    message_.setTitle(msg);
    message_.setForegroundColor(color);
    message_.setFrameColor(color);
    message_.draw();
    if(delayms != 0) delay(delayms);
}
  
void UI::showDialog() {
    dialogActive_ = true;
    dialog_.setNeedsFullRedraw();
    dialog_.takeInputFocus();
}
  
void UI::hideDialog() {
    dialogActive_ = false;
    mainWidget_->setNeedsFullRedraw();
    mainWidget_->takeInputFocus();
}
  
void UI::showLEDBrightnessDialog() {
    dialog_.setTitle("LED Level");
    dialog_.setValue(RRemote::remote.ledBrightness());
    dialog_.setSuffix("");
    dialog_.setRange(0, 7);
    dialog_.setOKCallback([=](int brt){RRemote::remote.setLEDBrightness(brt);});
    showDialog();
}

void UI::showJoyDeadbandDialog() {
    dialog_.setTitle("Joy Deadb.");
    dialog_.setValue(RRemote::remote.joyDeadband());
    dialog_.setSuffix("%");
    dialog_.setRange(0, 15);
    dialog_.setOKCallback([=](int db){RRemote::remote.setJoyDeadband(db);});
    showDialog();
}
  
void UI::showSendRepeatsDialog() {
    dialog_.setTitle("Send Reps");
    dialog_.setValue(RRemote::remote.sendRepeats());
    dialog_.setSuffix("");
    dialog_.setRange(0, 7);
    dialog_.setOKCallback([=](int sr){RRemote::remote.setSendRepeats(sr);});
    showDialog();
}

void UI::showCalibration(PacketSource source) {
    switch(source) {
    case PACKET_SOURCE_LEFT_REMOTE:
        mainVis_.showIndex(0);
        remoteVisL_.crosshair().setMinMax(1024, 4096-1024, 1024, 4096-1024);
        remoteVisL_.crosshair().showMinMaxRect();
        remoteVisL_.crosshair().setMinMaxRectColor(Display::RED);
        setTopTitle("Calib L");
        showMain();
        break;
    case PACKET_SOURCE_RIGHT_REMOTE:
        mainVis_.showIndex(1);
        remoteVisR_.crosshair().setMinMax(1024, 4096-1024, 1024, 4096-1024);
        remoteVisR_.crosshair().showMinMaxRect();
        remoteVisR_.crosshair().setMinMaxRectColor(Display::RED);
        setTopTitle("Calib R");
        showMain();
        break;
    default:
        bb::printf("Error: showCalibration() called with argument %d\n", source);
    }
}

void UI::hideCalibration(PacketSource source) {
    switch(source) {
    case PACKET_SOURCE_LEFT_REMOTE:
        remoteVisL_.crosshair().showMinMaxRect(false);
        break;
    case PACKET_SOURCE_RIGHT_REMOTE:
        remoteVisR_.crosshair().showMinMaxRect(false);
        break;
    default:
        bb::printf("Error: showCalibration() called with argument %d\n", source);        
    }
}

void UI::setSeqnumState(bb::PacketSource source, bool active) {
    switch(source) {
    case PACKET_SOURCE_DROID:
        if(active) droidSeqnum_.setFrameColor(Display::LIGHTBLUE2);
        else droidSeqnum_.setFrameColor(Display::LIGHTGREY);
        break;
    case PACKET_SOURCE_LEFT_REMOTE:
        if(active) leftSeqnum_.setFrameColor(Display::LIGHTBLUE2);
        else leftSeqnum_.setFrameColor(Display::LIGHTGREY);
        break;
    case PACKET_SOURCE_RIGHT_REMOTE:
    default:
        if(active) rightSeqnum_.setFrameColor(Display::LIGHTBLUE2);
        else rightSeqnum_.setFrameColor(Display::LIGHTGREY);
        break;
    }
}
  
void UI::setNoComm(bb::PacketSource source, bool nocomm) {
    switch(source) {
        case PACKET_SOURCE_DROID:
            droidSeqnum_.setNoComm(nocomm);
            droidVis_.reset();
            break;
        case PACKET_SOURCE_LEFT_REMOTE:
            leftSeqnum_.setNoComm(nocomm);
            break;
        case PACKET_SOURCE_RIGHT_REMOTE:
        default:
            rightSeqnum_.setNoComm(nocomm);
            break;
        }    
}

void UI::updateLeftSeqnum(uint8_t seqnum) {
    leftSeqnum_.setSquareColor(seqnum%8, Display::GREEN);
    leftSeqnum_.setSquareColor((seqnum+1)%8, leftSeqnum_.backgroundColor());
    leftSeqnum_.setNoComm(false);
}

void UI::updateRightSeqnum(uint8_t seqnum) {
    rightSeqnum_.setSquareColor(seqnum%8, Display::GREEN);
    rightSeqnum_.setSquareColor((seqnum+1)%8, leftSeqnum_.backgroundColor());
    rightSeqnum_.setNoComm(false);

    unsigned long diff = WRAPPEDDIFF(seqnum, lastRightSeqnum_, 8);
    if(diff>1) {
        Console::console.printfBroadcast("Seqnum expected: %d, received: %d, missed %d\n", lastRightSeqnum_+1, seqnum, diff-1);
        for(int i=1; i<diff; i++) rightSeqnum_.setSquareColor(lastRightSeqnum_+i, Display::RED);
    }
    lastRightSeqnum_ = seqnum;
}


void UI::visualizeFromTelemetry(Protocol* proto, const NodeAddr& addr, uint8_t seqnum, const Telemetry& telem) {
    uint8_t expected;

    droidSeqnum_.setSquareColor(seqnum%8, Display::GREEN);
    droidSeqnum_.setSquareColor((seqnum+1)%8, rightSeqnum_.backgroundColor());
    droidSeqnum_.setNoComm(false);

    expected = (lastDroidSeqnum_+1)%8;
    if(seqnum != expected) {
        int missed;
        if(expected < seqnum) missed = seqnum - expected;
        else missed = 8 + (seqnum - expected);
        Console::console.printfBroadcast("Seqnum expected: %d, received: %d, missed %d\n", expected, seqnum, missed);
        for(int i=1; i<missed; i++) droidSeqnum_.setSquareColor(lastRightSeqnum_+i, Display::RED);
        }
    
    lastDroidSeqnum_ = seqnum;

    droidVis_.visualizeFromTelemetry(telem);
}

void UI::toggleLockFaceButtons() {
    bb::printf("toggle lock face buttons\n");
    if(Input::inst.faceButtonsLocked()) {
        bb::printf("Unlocking!\n");
        Input::inst.setFaceButtonsLocked(false);
        topLabel_.setNeedsFullRedraw();
    } else {
        bb::printf("Locking!\n");
        Input::inst.setFaceButtonsLocked(true);
        lockedLabel_.setNeedsFullRedraw();
    }
}