#include "UI/UI.h"
#include "Todo/RRemote.h"
#include "RemoteSubsys.h"

UI UI::ui;

UI::UI() {
    message_ = make_shared<MessageWidget>();
    message_->setTitle("?");

    remoteVisL_ = make_shared<RemoteVisWidget>();
    remoteVisL_->setRepresentsLeftRemote(true);
    remoteVisL_->setName("Left Remote");
    remoteVisR_ = make_shared<RemoteVisWidget>();
    remoteVisR_->setRepresentsLeftRemote(false);
    remoteVisR_->setName("Right Remote");

    droidVis_ = make_shared<DroidVisWidget>();
    droidVis_->setName("Droid");

    mainVis_ = make_shared<RotaWidget>();
    mainVis_->addWidget(remoteVisL_);
    mainVis_->addWidget(remoteVisR_);
    mainVis_->addWidget(droidVis_);

    topLabel_ = make_shared<Label>();
    topLabel_->setSize(Display::DISPLAY_WIDTH, Display::CHAR_HEIGHT);
    topLabel_->setPosition(0, 0);
    topLabel_->setFrameType(Label::FRAME_BOTTOM);
    topLabel_->setTitle("Top Label");

    lockedLabel_ = make_shared<Label>();
    lockedLabel_->setSize(Display::DISPLAY_WIDTH, Display::CHAR_HEIGHT);
    lockedLabel_->setPosition(0, 0);
    lockedLabel_->setFrameType(Label::FRAME_BOTTOM);
    lockedLabel_->setBackgroundColor(Display::DARKBLUE);
    lockedLabel_->setTitle("LOCKED");
  
    bottomLabel_ = make_shared<Label>();
    bottomLabel_->setSize(Display::DISPLAY_WIDTH, Display::CHAR_HEIGHT+4);
    bottomLabel_->setPosition(0, Display::DISPLAY_HEIGHT-Display::CHAR_HEIGHT-3);
    bottomLabel_->setJustification(Label::LEFT_JUSTIFIED, Label::BOTTOM_JUSTIFIED);
    bottomLabel_->setFrameType(Label::FRAME_TOP);
    bottomLabel_->setTitle("");
  
    leftSeqnum_ = make_shared<SeqnumWidget>();
    leftSeqnum_->setSize(23, 8);
    leftSeqnum_->setPosition(1, Display::DISPLAY_HEIGHT-Display::CHAR_HEIGHT);
    leftSeqnum_->setFrameColor(Display::LIGHTBLUE2);
    leftSeqnum_->setChar('L');
    rightSeqnum_ = make_shared<SeqnumWidget>();
    rightSeqnum_->setSize(23, 8);
    rightSeqnum_->setPosition(leftSeqnum_->x()+leftSeqnum_->width()+4, leftSeqnum_->y());
    rightSeqnum_->setChar('R');
    droidSeqnum_ = make_shared<SeqnumWidget>();
    droidSeqnum_->setSize(23, 8);
    droidSeqnum_->setPosition(rightSeqnum_->x()+rightSeqnum_->width()+4, leftSeqnum_->y());
    droidSeqnum_->setChar('D');
  
    dialog_ = make_shared<Dialog>();
    dialog_->setTitle("Hi! :-)");
    dialog_->setValue(5);
    dialog_->setRange(0, 10);  
}

void UI::start() {
    populateMenus();
    mainMenu_->enter();
    //showMain();
    drawGUI();
    delay(10);
    if(mainWidget_ != nullptr)
        mainWidget_->setNeedsFullRedraw();
    leftSeqnum_->setNeedsFullRedraw();
    droidSeqnum_->setNeedsFullRedraw();
    rightSeqnum_->setNeedsFullRedraw();  
}

void UI::setMainWidget(Widget* widget) {
    mainWidget_ = widget;
    if(widget == nullptr) {
        setTopTitle("No main");
        return;
    }

    mainWidget_->setPosition(Display::MAIN_X, Display::MAIN_Y);
    mainWidget_->setSize(Display::MAIN_WIDTH, Display::MAIN_HEIGHT);
    Input::inst.clearCallbacks();
    mainWidget_->takeInputFocus();
    mainWidget_->setNeedsFullRedraw();
    mainWidget_->setNeedsContentsRedraw();
    setTopTitle(mainWidget_->name());
}

void UI::populateConfigMenu(Menu* menu) {
    menu->clear();
    Protocol* current = RemoteSubsys::inst.currentProtocol();
    if(current == nullptr) {
        menu->setName(String("Cur:None ") + char(current->protocolType()));
    } else if(current->storageName().empty()) {
        menu->setName(String("Cur:- ") + char(current->protocolType()));
    } else {
        menu->setName(String("Cur:") + String(current->storageName().c_str()) + " " + char(current->protocolType()));
    }

    shared_ptr<Menu> pairRemoteMenu = menu->addSubmenu("Pair remote...");
    pairRemoteMenu->setOnEnterCallback([=](Menu* m){ UI::ui.populatePairRemoteMenu(m); });
    shared_ptr<Menu> pairDroidMenu = menu->addSubmenu("Pair droid...");
    pairDroidMenu->setOnEnterCallback([=](Menu* m){ UI::ui.populatePairDroidMenu(m); });

    menu->addEntry("Save current", [=](Widget*){bb::printf("Save config");
        dialog_->setTitle("Save as...");
        dialog_->setValue(current->storageName().c_str());
        dialog_->setMaxLen(10);
        dialog_->setCancelCallback([=](Dialog* d){UI::ui.hideDialog();});
        dialog_->setOKCallback([=](Dialog* d){UI::ui.saveCurrentConfigAsCB(d);});
        UI::ui.showDialog();
    });

    shared_ptr<Menu> newMenu = menu->addSubmenu("New...");
    newMenu->addEntry("Monaco/XBee", [=](Widget* w){
        bb::printf("New Monaco/XBee config\n"); 
        Protocol *p = RemoteSubsys::inst.createProtocol(ProtocolType::MONACO_XBEE);
        if(p != nullptr) {
            UI::ui.showMessage("Created Monaco/XBee protocol", 2000, Display::LIGHTGREEN1);
            menu->setName(String("Cur:") + String(p->storageName().c_str()) + " " + char(p->protocolType()));
        } else {
            UI::ui.showMessage("Failed to create Monaco/XBee protocol", 2000, Display::LIGHTRED1);
        }
        newMenu->back(w);
    });
    newMenu->addEntry("Monaco/ESPnow", [=](Widget* w){
        bb::printf("New Monaco/ESPnow config\n");
        Protocol *p = RemoteSubsys::inst.createProtocol(ProtocolType::MONACO_ESPNOW);
        if(p != nullptr) {
            UI::ui.showMessage("Created Monaco/ESPNow protocol", 2000, Display::LIGHTGREEN1);
            menu->setName(String("Cur:") + String(p->storageName().c_str()) + " " + char(p->protocolType()));
        } else {
            UI::ui.showMessage("Failed to create Monaco/ESPNow protocol", 2000, Display::LIGHTRED1);
        }
        newMenu->back(w);
    });
    newMenu->addEntry("DroidDepot", [=](Widget* w){bb::printf("New DroidDepot config\n");
        Protocol *p = RemoteSubsys::inst.createProtocol(ProtocolType::DROIDDEPOT_BLE);
        if(p != nullptr) {
            UI::ui.showMessage("Created DroidDepot protocol", 2000, Display::LIGHTGREEN1);
            menu->setName(String("Cur:") + String(p->storageName().c_str()) + " " + char(p->protocolType()));
        } else {
            UI::ui.showMessage("Failed to create DroidDepot protocol", 2000, Display::LIGHTRED1);
        }
        newMenu->back(w);
    });
    newMenu->addEntry("Sphero", [=](Widget* w){bb::printf("New Sphero config\n");
        Protocol *p = RemoteSubsys::inst.createProtocol(ProtocolType::SPHERO_BLE);
        if(p != nullptr) {
            UI::ui.showMessage("Created Sphero protocol", 2000, Display::LIGHTGREEN1);
            menu->setName(String("Cur:") + String(p->storageName().c_str()) + " " + char(p->protocolType()));
        } else {
            UI::ui.showMessage("Failed to create Sphero protocol", 2000, Display::LIGHTRED1);
        }
        newMenu->back(w);
    });

    std::vector<std::string> names = ProtocolFactory::storedProtocolNames();
    shared_ptr<Menu> loadMenu = menu->addSubmenu("Load...");
    for(auto& n: names) {
        if(n != current->storageName()) // can't load current
            loadMenu->addEntry(String(n.c_str()), [=](Widget*){
                bb::printf("Load config %s", n.c_str());
                UI::ui.showMessage(String("Loading ") + n.c_str() + "...");
                if(RemoteSubsys::inst.loadCurrent(n) == false) {
                    UI::showMessage("Loading failed", 2000, Display::LIGHTRED1);
                    return;
                }
                menu->setName(String("Cur:") + String(n.c_str()) + " "+ char(current->protocolType()));
                loadMenu->back(nullptr);
            });
    }

    shared_ptr<Menu> deleteMenu = menu->addSubmenu("Delete...");
    for(auto& n: names) {
        if(n != current->storageName() && n != RemoteSubsys::INTERREMOTE_PROTOCOL_NAME) // can't delete current or "default"
            deleteMenu->addEntry(String(n.c_str()), [=](Widget*){
                bb::printf("Delete config %s", n.c_str());
                if(ProtocolFactory::eraseProtocol(n.c_str()) == false) {
                    UI::showMessage("Delete failed", 2000, Display::LIGHTRED1);
                    return;
                }
                UI::showMessage("Deleted", 2000, Display::LIGHTGREEN1);
                deleteMenu->back(nullptr);
            });
    }
}

  
void UI::populatePairDroidMenu(Menu* menu) {
    showMessage("Please wait");
    
    menu->clear();
    Protocol* current = RemoteSubsys::inst.currentProtocol();
    if(current == nullptr) {
        menu->setName("Cur:None");
        return;
    }
  
    if(current->discoverNodes() == false) {
      return;
    }

    int num;
    for(num=0; num<current->numDiscoveredNodes(); num++) {
        const NodeDescription& n = current->discoveredNode(num);
        if(!n.isReceiver) continue;
        if(current->isPaired(n.addr)) {
            menu->addEntry(String("*")+n.name, [=](Widget* w){ UI::ui.showMessage("Already paired", 2000); });
        } else {
            menu->addEntry(String(n.name), [=](Widget* w) { 
                UI::showMessage("Pairing..."); 
                if(current->pairWith(n) == true) {
                    UI::showMessage("Paired", 2000, Display::LIGHTGREEN1); 
                }
                else UI::showMessage("Pairing failed", 2000, Display::LIGHTRED1);
                menu->back(w);
            });
        }
    }
    menu->setName(String(num) + " Droids");
}
  
void UI::populatePairRemoteMenu(Menu* menu) {
    menu->clear();
    Protocol* inter = RemoteSubsys::inst.interremoteProtocol();
    if(inter == nullptr) {
        menu->setName("NO INTERREMOTE");
        return;
    }
  
    if(inter->discoverNodes() == false) {
      return;
    }

    int num;
    for(num=0; num<inter->numDiscoveredNodes(); num++) {
        const NodeDescription& n = inter->discoveredNode(num);
        if(n.isTransmitter == false) continue;
        if(inter->isPaired(n.addr)) {
            menu->addEntry(String("*")+n.name, [=](Widget* w){ UI::ui.showMessage("Already paired", 2000); });
        } else {
            menu->addEntry(String(n.name), [=](Widget* w) { 
                UI::showMessage("Pairing..."); 
                if(inter->pairWith(n) == true) {
                    UI::showMessage("Paired", 2000, Display::LIGHTGREEN1); 
                    if(RemoteSubsys::inst.storeInterremote() == false ||
                        ProtocolFactory::commit() == false) {
                        UI::showMessage("Storing failed", 2000, Display::LIGHTRED1);
                    }
                }
                else UI::showMessage("Pairing failed", 2000, Display::LIGHTRED1);
                menu->back(w);
            });
        }
    }
    menu->setName(String(num) + " Remotes");
}
  
void UI::showMenu(const shared_ptr<Menu>& menu) {
    setMainWidget(menu.get());
    menu->resetCursor();
}
  
void UI::showMain() {
    setMainWidget(mainVis_.get());
    Input::inst.setConfirmShortPressCallback([=]{UI::ui.showMenu(mainMenu_);});
    Input::inst.setConfirmLongPressCallback([=]{UI::ui.toggleLockFaceButtons();});
}

void UI::populateMenus() {
    mainMenu_ = make_shared<Menu>();
    mainMenu_->setName("Main Menu");

    shared_ptr<Menu> m = mainMenu_->addSubmenu("Config...");
    m->setOnEnterCallback([this](Menu* menu){ populateConfigMenu(menu); });

    //shared_ptr<Menu> configMenu_ = mainMenu_->addSubmenu("Config");
#if 0
    mainMenu_.setName("Main Menu");

    pairMenu_.setName("Pair");
    leftRemoteMenu_.setName("Left Remote");
    rightRemoteMenu_.setName("Right Remote");
    bothRemotesMenu_.setName("Both Remotes");
    droidMenu_.setName("Droid");
    
    mainMenu_.clear();
    mainMenu_.addEntry("Config...", [=]() {showMenu(&configMenu_);});
#if 0
    if(!RRemote::remote.droidAddress().isZero()) mainMenu_.addEntry("Droid...", [=]() { showMenu(&droidMenu_); });
    mainMenu_.addEntry("Left Remote...", [=]() { showMenu(&leftRemoteMenu_); });
    if(!RRemote::remote.otherRemoteAddress().isZero()) mainMenu_.addEntry("Right Remote...", [=]() { showMenu(&rightRemoteMenu_); });
    mainMenu_.addEntry("Both Remotes...", [=](){showMenu(&bothRemotesMenu_);});
    mainMenu_.addEntry("Pair...", [=]() { showMenu(&pairMenu_); });
#endif
    mainMenu_.addEntry("<--", [=]() { showMain(); });

    configMenu_.clear();
    configMenu_.setName("Config");
    for(auto& n: ProtocolFactory::storedProtocolNames()) {
        if(n=="inter") 
            configMenu_.addEntry("D-O", [=]() {RemoteSubsys::inst.loadCurrent(n);});
        else
            configMenu_.addEntry(String(n.c_str()), [=]() {RemoteSubsys::inst.loadCurrent(n);});
    }
    configMenu_.addEntry("<--", [=]() { showMenu(&mainMenu_); });

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
#endif
}

void UI::drawGUI() {
    if(Input::inst.faceButtonsLocked()) {
        lockedLabel_->draw();
    } else {
        topLabel_->draw();
    }
    bottomLabel_->draw();
    leftSeqnum_->draw();
    rightSeqnum_->draw();
    droidSeqnum_->draw();
    if(dialogActive_) dialog_->draw();
    else if(mainWidget_ != NULL) mainWidget_->draw();
#if 0
    if(needsMenuRebuild_) {
      populateMenus();
      needsMenuRebuild_ = false;
    }
#endif
    needsScreensaverRedraw_ = true;
}

void UI::drawScreensaver() {
    if(needsScreensaverRedraw_) {
        Display::display.rect(0, 0, Display::DISPLAY_WIDTH, Display::DISPLAY_HEIGHT, Display::BLACK, true);
        topLabel_->setNeedsFullRedraw();
        bottomLabel_->setNeedsFullRedraw();
        leftSeqnum_->setNeedsFullRedraw();
        rightSeqnum_->setNeedsFullRedraw();
        droidSeqnum_->setNeedsFullRedraw();
        if(dialogActive_) dialog_->setNeedsFullRedraw();
        if(mainWidget_ != nullptr) mainWidget_->setNeedsFullRedraw();
        needsScreensaverRedraw_ = false;
    }
}

void UI::setTopTitle(const String& title) {
    topLabel_->setTitle(title);
}  

void UI::saveCurrentConfigAsCB(Dialog* dialog) {
    String name = dialog->valueString();
    name.trim();
    if(name.length() == 0) {
        showMessage("Invalid name", 2000, Display::LIGHTRED1);
        return;
    }
    bb::printf("Saving current config as \"%s\"\n", name.c_str());
    RemoteSubsys::inst.storeCurrent(name.c_str());
    if(ProtocolFactory::commit() == false) {
        showMessage("Storing failed", 2000, Display::LIGHTRED1);
    } else {
        showMessage("Config saved", 2000, Display::LIGHTGREEN1);
    }
}


void UI::setIncrRotButtonCB(Input::Button button, bool left) {
    if(left) {
        if(RRemote::remote.incrRotButton(PACKET_SOURCE_LEFT_REMOTE) == button) return;
        Input::inst.setIncrementalRot(button);
        RRemote::remote.setIncrRotButton(PACKET_SOURCE_LEFT_REMOTE, button);
        lRIncrRotMenu_->highlightWidgetsWithTag(button);
        RRemote::remote.storeParams();
    } else {
        Input::Button tempBtn = RRemote::remote.incrRotButton(PACKET_SOURCE_RIGHT_REMOTE);
        if(tempBtn == button) return;
        RRemote::remote.setIncrRotButton(PACKET_SOURCE_RIGHT_REMOTE, button);
        rRIncrRotMenu_->highlightWidgetsWithTag(button);
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
    message_->setTitle(msg);
    message_->setForegroundColor(color);
    message_->setFrameColor(color);
    message_->draw();
    if(delayms != 0) delay(delayms);
    if(mainWidget_ != nullptr) {
        mainWidget_->setNeedsFullRedraw();
    }
}
  
void UI::showDialog() {
    dialogActive_ = true;
    dialog_->setNeedsFullRedraw();
    dialog_->takeInputFocus();
}
  
void UI::hideDialog() {
    dialogActive_ = false;
    if(mainWidget_ != nullptr) {
        mainWidget_->setNeedsFullRedraw();
        mainWidget_->takeInputFocus();
    }
}
  
void UI::showLEDBrightnessDialog() {
    dialog_->setTitle("LED Level");
    dialog_->setValue(RRemote::remote.ledBrightness());
    dialog_->setSuffix("");
    dialog_->setRange(0, 7);
    dialog_->setOKCallback([=](Dialog* d){RRemote::remote.setLEDBrightness(d->value());});
    showDialog();
}

void UI::showJoyDeadbandDialog() {
    dialog_->setTitle("Joy Deadb.");
    dialog_->setValue(RRemote::remote.joyDeadband());
    dialog_->setSuffix("%");
    dialog_->setRange(0, 15);
    dialog_->setOKCallback([=](Dialog* d){RRemote::remote.setJoyDeadband(d->value());});
    showDialog();
}
  
void UI::showSendRepeatsDialog() {
    dialog_->setTitle("Send Reps");
    dialog_->setValue(RRemote::remote.sendRepeats());
    dialog_->setSuffix("");
    dialog_->setRange(0, 7);
    dialog_->setOKCallback([=](Dialog* d){RRemote::remote.setSendRepeats(d->value());});
    showDialog();
}

void UI::showCalibration(PacketSource source) {
    switch(source) {
    case PACKET_SOURCE_LEFT_REMOTE:
        mainVis_->showIndex(0);
        remoteVisL_->crosshair()->setMinMax(1024, 4096-1024, 1024, 4096-1024);
        remoteVisL_->crosshair()->showMinMaxRect();
        remoteVisL_->crosshair()->setMinMaxRectColor(Display::RED);
        setTopTitle("Calib L");
        showMain();
        break;
    case PACKET_SOURCE_RIGHT_REMOTE:
        mainVis_->showIndex(1);
        remoteVisR_->crosshair()->setMinMax(1024, 4096-1024, 1024, 4096-1024);
        remoteVisR_->crosshair()->showMinMaxRect();
        remoteVisR_->crosshair()->setMinMaxRectColor(Display::RED);
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
        remoteVisL_->crosshair()->showMinMaxRect(false);
        break;
    case PACKET_SOURCE_RIGHT_REMOTE:
        remoteVisR_->crosshair()->showMinMaxRect(false);
        break;
    default:
        bb::printf("Error: showCalibration() called with argument %d\n", source);        
    }
}

void UI::setSeqnumState(bb::PacketSource source, bool active) {
    switch(source) {
    case PACKET_SOURCE_DROID:
        if(active) droidSeqnum_->setFrameColor(Display::LIGHTBLUE2);
        else droidSeqnum_->setFrameColor(Display::LIGHTGREY);
        break;
    case PACKET_SOURCE_LEFT_REMOTE:
        if(active) leftSeqnum_->setFrameColor(Display::LIGHTBLUE2);
        else leftSeqnum_->setFrameColor(Display::LIGHTGREY);
        break;
    case PACKET_SOURCE_RIGHT_REMOTE:
    default:
        if(active) rightSeqnum_->setFrameColor(Display::LIGHTBLUE2);
        else rightSeqnum_->setFrameColor(Display::LIGHTGREY);
        break;
    }
}
  
void UI::setNoComm(bb::PacketSource source, bool nocomm) {
    switch(source) {
        case PACKET_SOURCE_DROID:
            droidSeqnum_->setNoComm(nocomm);
            droidVis_->reset();
            break;
        case PACKET_SOURCE_LEFT_REMOTE:
            leftSeqnum_->setNoComm(nocomm);
            break;
        case PACKET_SOURCE_RIGHT_REMOTE:
        default:
            rightSeqnum_->setNoComm(nocomm);
            break;
        }    
}

void UI::updateLeftSeqnum(uint8_t seqnum) {
    leftSeqnum_->setSquareColor(seqnum%8, Display::GREEN);
    leftSeqnum_->setSquareColor((seqnum+1)%8, leftSeqnum_->backgroundColor());
    leftSeqnum_->setNoComm(false);
}

void UI::updateRightSeqnum(uint8_t seqnum) {
    rightSeqnum_->setSquareColor(seqnum%8, Display::GREEN);
    rightSeqnum_->setSquareColor((seqnum+1)%8, leftSeqnum_->backgroundColor());
    rightSeqnum_->setNoComm(false);

    unsigned long diff = WRAPPEDDIFF(seqnum, lastRightSeqnum_, 8);
    if(diff>1) {
        Console::console.printfBroadcast("Seqnum expected: %d, received: %d, missed %d\n", lastRightSeqnum_+1, seqnum, diff-1);
        for(int i=1; i<diff; i++) rightSeqnum_->setSquareColor(lastRightSeqnum_+i, Display::RED);
    }
    lastRightSeqnum_ = seqnum;
}


void UI::visualizeFromTelemetry(Protocol* proto, const NodeAddr& addr, uint8_t seqnum, const Telemetry& telem) {
    uint8_t expected;

    bb::printf("Got telemetry from %s\n", addr.toString().c_str());

    droidSeqnum_->setSquareColor(seqnum%8, Display::GREEN);
    droidSeqnum_->setSquareColor((seqnum+1)%8, rightSeqnum_->backgroundColor());
    droidSeqnum_->setNoComm(false);

    expected = (lastDroidSeqnum_+1)%8;
    if(seqnum != expected) {
        int missed;
        if(expected < seqnum) missed = seqnum - expected;
        else missed = 8 + (seqnum - expected);
        Console::console.printfBroadcast("Seqnum expected: %d, received: %d, missed %d\n", expected, seqnum, missed);
        for(int i=1; i<missed; i++) droidSeqnum_->setSquareColor(lastRightSeqnum_+i, Display::RED);
        }
    
    lastDroidSeqnum_ = seqnum;

    droidVis_->visualizeFromTelemetry(telem);
}

void UI::toggleLockFaceButtons() {
    bb::printf("toggle lock face buttons\n");
    if(Input::inst.faceButtonsLocked()) {
        bb::printf("Unlocking!\n");
        Input::inst.setFaceButtonsLocked(false);
        topLabel_->setNeedsFullRedraw();
    } else {
        bb::printf("Locking!\n");
        Input::inst.setFaceButtonsLocked(true);
        lockedLabel_->setNeedsFullRedraw();
    }
}