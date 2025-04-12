#include "RUI.h"
#include "RRemote.h"

RUI RUI::ui;

RUI::RUI() {
    message_.setTitle("?");

    remoteVisL_.setRepresentsLeftRemote(true);
    remoteVisL_.setName("Left Remote");
    remoteVisR_.setRepresentsLeftRemote(false);
    remoteVisR_.setName("Right Remote");
    mainVis_.addWidget(&remoteVisL_);
    mainVis_.addWidget(&remoteVisR_);
  
    topLabel_.setSize(RDisplay::DISPLAY_WIDTH, RDisplay::CHAR_HEIGHT);
    topLabel_.setPosition(0, 0);
    topLabel_.setFrameType(RLabelWidget::FRAME_BOTTOM);
    topLabel_.setTitle("Top Label");

    lockedLabel_.setSize(RDisplay::DISPLAY_WIDTH, RDisplay::CHAR_HEIGHT);
    lockedLabel_.setPosition(0, 0);
    lockedLabel_.setFrameType(RLabelWidget::FRAME_BOTTOM);
    lockedLabel_.setBackgroundColor(RDisplay::DARKBLUE);
    lockedLabel_.setTitle("LOCKED");
  
    bottomLabel_.setSize(RDisplay::DISPLAY_WIDTH, RDisplay::CHAR_HEIGHT+4);
    bottomLabel_.setPosition(0, RDisplay::DISPLAY_HEIGHT-RDisplay::CHAR_HEIGHT-3);
    bottomLabel_.setJustification(RLabelWidget::LEFT_JUSTIFIED, RLabelWidget::BOTTOM_JUSTIFIED);
    bottomLabel_.setFrameType(RLabelWidget::FRAME_TOP);
    bottomLabel_.setTitle("");
  
    leftSeqnum_.setSize(23, 8);
    leftSeqnum_.setPosition(1, RDisplay::DISPLAY_HEIGHT-RDisplay::CHAR_HEIGHT);
    leftSeqnum_.setFrameColor(RDisplay::LIGHTBLUE2);
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

void RUI::start() {
    populateMenus();
    showMain();
    drawGUI();
    delay(10);
    mainWidget_->setNeedsFullRedraw();
    leftSeqnum_.setNeedsFullRedraw();
    droidSeqnum_.setNeedsFullRedraw();
    rightSeqnum_.setNeedsFullRedraw();  
}

void RUI::setMainWidget(RWidget* widget) {
    widget->setPosition(RDisplay::MAIN_X, RDisplay::MAIN_Y);
    widget->setSize(RDisplay::MAIN_WIDTH, RDisplay::MAIN_HEIGHT);
    RInput::input.clearCallbacks();
    widget->takeInputFocus();
    widget->setNeedsFullRedraw();
    widget->setNeedsContentsRedraw();
    setTopTitle(widget->name());
    mainWidget_ = widget;
}
  
void RUI::showPairDroidMenu() {
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
    pairDroidMenu_.addEntry("<--", [=]() { RUI::ui.showMenu(&pairMenu_); });
  
    Console::console.printfBroadcast("Showing droidsMenu\n");
    showMenu(&pairDroidMenu_);
}
  
void RUI::showPairRemoteMenu() {
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
    pairRemoteMenu_.addEntry("<--", [=]() { RUI::ui.showMenu(&pairMenu_); });
    pairRemoteMenu_.setName(String(num) + " Remotes");
  
    showMenu(&pairRemoteMenu_);
}
  
void RUI::showMenu(RMenuWidget* menu) {
    setMainWidget(menu);
    menu->resetCursor();
}
  
void RUI::showMain() {
    setMainWidget(&mainVis_);
    RInput::input.setConfirmShortPressCallback([=]{RUI::ui.showMenu(&mainMenu_);});
    RInput::input.setConfirmLongPressCallback([=]{RUI::ui.toggleLockFaceButtons();});
}

void RUI::populateMenus() {
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
    lRIncrRotMenu_.addEntry("Disable", [=]{setIncrRotButtonCB(RInput::BUTTON_NONE, true);showMain();}, RInput::BUTTON_NONE);
    lRIncrRotMenu_.addEntry("Button 1", [=]{setIncrRotButtonCB(RInput::BUTTON_1, true);showMain();}, RInput::BUTTON_1);
    lRIncrRotMenu_.addEntry("Button 2", [=]{setIncrRotButtonCB(RInput::BUTTON_2, true);showMain();}, RInput::BUTTON_2);
    lRIncrRotMenu_.addEntry("Button 3", [=]{setIncrRotButtonCB(RInput::BUTTON_3, true);showMain();}, RInput::BUTTON_3);
    lRIncrRotMenu_.addEntry("Button 4", [=]{setIncrRotButtonCB(RInput::BUTTON_4, true);showMain();}, RInput::BUTTON_4);
    lRIncrRotMenu_.addEntry("<--", [=]{showMenu(&leftRemoteMenu_);});
    lRIncrRotMenu_.highlightWidgetsWithTag(RRemote::remote.incrRotButton(PACKET_SOURCE_LEFT_REMOTE));

    rRIncrRotMenu_.clear();
    rRIncrRotMenu_.setName("Incr Rotation");
    rRIncrRotMenu_.addEntry("Disable", [=]{setIncrRotButtonCB(RInput::BUTTON_NONE, false);showMain();}, RInput::BUTTON_NONE);
    rRIncrRotMenu_.addEntry("Button 1", [=]{setIncrRotButtonCB(RInput::BUTTON_1, false);showMain();}, RInput::BUTTON_1);
    rRIncrRotMenu_.addEntry("Button 2", [=]{setIncrRotButtonCB(RInput::BUTTON_2, false);showMain();}, RInput::BUTTON_2);
    rRIncrRotMenu_.addEntry("Button 3", [=]{setIncrRotButtonCB(RInput::BUTTON_3, false);showMain();}, RInput::BUTTON_3);
    rRIncrRotMenu_.addEntry("Button 4", [=]{setIncrRotButtonCB(RInput::BUTTON_4, false);showMain();}, RInput::BUTTON_4);
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

void RUI::drawGUI() {
    if(RInput::input.faceButtonsLocked()) {
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

void RUI::drawScreensaver() {
    if(needsScreensaverRedraw_) {
        RDisplay::display.rect(0, 0, RDisplay::DISPLAY_WIDTH, RDisplay::DISPLAY_HEIGHT, RDisplay::BLACK, true);
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

void RUI::setTopTitle(const String& title) {
    topLabel_.setTitle(title);
}  

void RUI::setIncrRotButtonCB(RInput::Button button, bool left) {
    if(left) {
        if(RRemote::remote.incrRotButton(PACKET_SOURCE_LEFT_REMOTE) == button) return;
        RInput::input.setIncrementalRot(button);
        RRemote::remote.setIncrRotButton(PACKET_SOURCE_LEFT_REMOTE, button);
        lRIncrRotMenu_.highlightWidgetsWithTag(button);
        RRemote::remote.storeParams();
    } else {
        RInput::Button tempBtn = RRemote::remote.incrRotButton(PACKET_SOURCE_RIGHT_REMOTE);
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
       
    
void RUI::showMessage(const String& msg, unsigned int delayms, uint8_t color) {
    message_.setTitle(msg);
    message_.setForegroundColor(color);
    message_.setFrameColor(color);
    message_.draw();
    if(delayms != 0) delay(delayms);
}
  
void RUI::showDialog() {
    dialogActive_ = true;
    dialog_.setNeedsFullRedraw();
    dialog_.takeInputFocus();
}
  
void RUI::hideDialog() {
    dialogActive_ = false;
    mainWidget_->setNeedsFullRedraw();
    mainWidget_->takeInputFocus();
}
  
void RUI::showLEDBrightnessDialog() {
    dialog_.setTitle("LED Level");
    dialog_.setValue(RRemote::remote.ledBrightness());
    dialog_.setSuffix("");
    dialog_.setRange(0, 7);
    dialog_.setOKCallback([=](int brt){RRemote::remote.setLEDBrightness(brt);});
    showDialog();
}

void RUI::showJoyDeadbandDialog() {
    dialog_.setTitle("Joy Deadb.");
    dialog_.setValue(RRemote::remote.joyDeadband());
    dialog_.setSuffix("%");
    dialog_.setRange(0, 15);
    dialog_.setOKCallback([=](int db){RRemote::remote.setJoyDeadband(db);});
    showDialog();
}
  
void RUI::showSendRepeatsDialog() {
    dialog_.setTitle("Send Reps");
    dialog_.setValue(RRemote::remote.sendRepeats());
    dialog_.setSuffix("");
    dialog_.setRange(0, 7);
    dialog_.setOKCallback([=](int sr){RRemote::remote.setSendRepeats(sr);});
    showDialog();
}

void RUI::showCalibration(PacketSource source) {
    switch(source) {
    case PACKET_SOURCE_LEFT_REMOTE:
        mainVis_.showIndex(0);
        remoteVisL_.crosshair().setMinMax(1024, 4096-1024, 1024, 4096-1024);
        remoteVisL_.crosshair().showMinMaxRect();
        remoteVisL_.crosshair().setMinMaxRectColor(RDisplay::RED);
        setTopTitle("Calib L");
        showMain();
        break;
    case PACKET_SOURCE_RIGHT_REMOTE:
        mainVis_.showIndex(1);
        remoteVisR_.crosshair().setMinMax(1024, 4096-1024, 1024, 4096-1024);
        remoteVisR_.crosshair().showMinMaxRect();
        remoteVisR_.crosshair().setMinMaxRectColor(RDisplay::RED);
        setTopTitle("Calib R");
        showMain();
        break;
    default:
        bb::printf("Error: showCalibration() called with argument %d\n", source);
    }
}

void RUI::hideCalibration(PacketSource source) {
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

void RUI::setSeqnumState(bb::PacketSource source, bool active) {
    switch(source) {
    case PACKET_SOURCE_DROID:
        if(active) droidSeqnum_.setFrameColor(RDisplay::LIGHTBLUE2);
        else droidSeqnum_.setFrameColor(RDisplay::LIGHTGREY);
        break;
    case PACKET_SOURCE_LEFT_REMOTE:
        if(active) leftSeqnum_.setFrameColor(RDisplay::LIGHTBLUE2);
        else leftSeqnum_.setFrameColor(RDisplay::LIGHTGREY);
        break;
    case PACKET_SOURCE_RIGHT_REMOTE:
    default:
        if(active) rightSeqnum_.setFrameColor(RDisplay::LIGHTBLUE2);
        else rightSeqnum_.setFrameColor(RDisplay::LIGHTGREY);
        break;
    }
}
  
void RUI::setSeqnumNoComm(bb::PacketSource source, bool nocomm) {
    switch(source) {
        case PACKET_SOURCE_DROID:
            droidSeqnum_.setNoComm(nocomm);
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

void RUI::visualizeFromControlPacket(bb::PacketSource source, uint8_t seqnum, const bb::ControlPacket& packet) {
    uint8_t diff;

    switch(source) {
    case PACKET_SOURCE_LEFT_REMOTE:
        remoteVisL_.visualizeFromControlPacket(packet);
        leftSeqnum_.setSquareColor(seqnum%8, RDisplay::GREEN);
        leftSeqnum_.setSquareColor((seqnum+1)%8, leftSeqnum_.backgroundColor());
        leftSeqnum_.setNoComm(false);

        break;
    
    case PACKET_SOURCE_RIGHT_REMOTE:
        remoteVisR_.visualizeFromControlPacket(packet);
        rightSeqnum_.setSquareColor(seqnum%8, RDisplay::GREEN);
        rightSeqnum_.setSquareColor((seqnum+1)%8, rightSeqnum_.backgroundColor());
        rightSeqnum_.setNoComm(false);

        diff = WRAPPEDDIFF(seqnum, lastRightSeqnum_, 8);
        if(diff>1) {
          Console::console.printfBroadcast("Seqnum expected: %d, received: %d, missed %d\n", lastRightSeqnum_+1, seqnum, diff-1);
          for(int i=1; i<diff; i++) rightSeqnum_.setSquareColor(lastRightSeqnum_+i, RDisplay::RED);
        }
        lastRightSeqnum_ = seqnum;
  
        break;
    
    default:
        break;
    }
}

void RUI::visualizeFromStatePacket(bb::PacketSource source, uint8_t seqnum, const bb::StatePacket& packet) {
    uint8_t expected;

    switch(source) {
    case PACKET_SOURCE_DROID:
        droidSeqnum_.setSquareColor(seqnum%8, RDisplay::GREEN);
        droidSeqnum_.setSquareColor((seqnum+1)%8, rightSeqnum_.backgroundColor());
        droidSeqnum_.setNoComm(false);

        expected = (lastDroidSeqnum_+1)%8;
        if(seqnum != expected) {
            int missed;
            if(expected < seqnum) missed = seqnum - expected;
            else missed = 8 + (seqnum - expected);
            Console::console.printfBroadcast("Seqnum expected: %d, received: %d, missed %d\n", expected, seqnum, missed);
            for(int i=1; i<missed; i++) droidSeqnum_.setSquareColor(lastRightSeqnum_+i, RDisplay::RED);
          }
        
        lastDroidSeqnum_ = seqnum;
    
        break;
    default:
          break;
    }
}

void RUI::toggleLockFaceButtons() {
    bb::printf("toggle lock face buttons\n");
    if(RInput::input.faceButtonsLocked()) {
        bb::printf("Unlocking!\n");
        RInput::input.setFaceButtonsLocked(false);
        topLabel_.setNeedsFullRedraw();
    } else {
        bb::printf("Locking!\n");
        RInput::input.setFaceButtonsLocked(true);
        lockedLabel_.setNeedsFullRedraw();
    }
}