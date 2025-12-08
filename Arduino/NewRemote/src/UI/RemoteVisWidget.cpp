#include "UI/RemoteVisWidget.h"

static const uint8_t w = 80;     // width of widget
static const uint8_t h = 130;    // height of widget
static const uint8_t bw = 55;    // width of remote outline
static const uint8_t bh = 120;   // height of remote outline
static const uint8_t bxl = 0;    // x of remote outline for left remote
static const uint8_t bxr = w-bw-1; // x of remote outline for right remote
static const uint8_t by = h-bh;  // y of remote outline

static const uint8_t joyW = 47;         // width/height of crosshair and IMU widget
static const uint8_t joyXL = 5;         // x position of crosshair and IMU widget
static const uint8_t joyXR = joyXL+bxr; // x position of crosshair and IMU widget
static const uint8_t xhairY = by+5;     // y position of crosshair
static const uint8_t imuY = by+bh-joyW-5;         // y position of IMU

static const uint8_t btnW = 5;   // width of button label
static const uint8_t btnH = 10;  // height of button label

static const uint8_t btn0Y = 110; 
static const uint8_t btn1Y = 40;
static const uint8_t btnSideXL = 55;
static const uint8_t btnSideXR = btnSideXL+bxr-bw-6;
static const uint8_t btn2XL = 35;         // #2 is the right one
static const uint8_t btn3XL = 10;         // #3 is the left one
static const uint8_t btn2XR = btn3XL+bxr; // #2 is the left one
static const uint8_t btn3XR = btn2XL+bxr; // #3 is the right one
static const uint8_t btnTopY = 5;

static const uint8_t potW = 19;

static const uint8_t potXL = 50;
static const uint8_t potXR = bxr-15;
static const uint8_t pot1Y = 61;
static const uint8_t pot2Y = pot1Y+potW+3;

static const uint8_t battStateW = 8;
static const uint8_t battStateY = by + joyW + 9;
static const uint8_t battStateXL = bxl + 7;
static const uint8_t battStateXR = bxr + 7;

using namespace bb::rmt;

RemoteVisWidget::RemoteVisWidget() {
    for(auto& b: mainBtns_) {
        b = make_shared<Label>();

        b->setDrawsFrame();
        b->setFillsBackground();
        b->setTitle("");
        b->setBackgroundColor(Display::DARKBLUE);
        b->setFrameColor(Display::LIGHTBLUE2);

    }

    mainBtns_[0]->setSize(btnW, btnH);
    mainBtns_[1]->setSize(btnW, btnH);
    mainBtns_[2]->setSize(btnH, btnW);
    mainBtns_[3]->setSize(btnH, btnW);

    for(auto& b: batteryState_) {
        b = make_shared<Label>();

        b->setDrawsFrame();
        b->setFillsBackground();
        b->setTitle("");
        b->setBackgroundColor(Display::LIGHTGREEN1);
        b->setSize(battStateW, battStateW);
    }

    crosshair_ = make_shared<CrosshairWidget>();
    crosshair_->setSize(joyW, joyW);
    imu_ = make_shared<IMUWidget>();
    imu_->setSize(joyW, joyW);

    pot1_ = make_shared<RoundScaleWidget>();
    pot1_->setSize(potW, potW);
    pot2_ = make_shared<RoundScaleWidget>();
    pot2_->setSize(potW, potW);

    bgCol_ = Display::LIGHTGREY;
    fgCol_ = Display::GREY;

    setRepresentsLeftRemote(false);

    moveWidgetsAround();

    potSelected_ = POT1_SELECTED;
}

void RemoteVisWidget::setRepresentsLeftRemote(bool left) {
    left_ = left;

    clearWidgets();
    for(auto& b: mainBtns_) addWidget(b);
    for(auto& b: batteryState_) addWidget(b);
    addWidget(crosshair_);
    addWidget(imu_);
    addWidget(pot1_);
    addWidget(pot2_);

    if(left) pot1_->setFrameColor(Display::GREEN);

    moveWidgetsAround();
}

void RemoteVisWidget::setPosition(int x, int y) {
    Widget::setPosition(x, y);
    moveWidgetsAround();
}

void RemoteVisWidget::moveWidgetsAround() {
    if(left_) {
        crosshair_->setPosition(joyXL, y_+xhairY);
        imu_->setPosition(joyXL, y_+imuY);
        mainBtns_[0]->setPosition(btnSideXL, y_+btn0Y);
        mainBtns_[1]->setPosition(btnSideXL, y_+btn1Y);
        mainBtns_[2]->setPosition(btn2XL, y_+btnTopY);
        mainBtns_[3]->setPosition(btn3XL, y_+btnTopY);
        for(unsigned int i=0; i<4; i++) {
            batteryState_[i]->setPosition(battStateXL + i*(battStateW+1), y_+battStateY);
        }
        pot1_->setPosition(potXL, y_+pot1Y);
        pot2_->setPosition(potXL, y_+pot2Y);
    } else {
        crosshair_->setPosition(joyXR, y_+xhairY);
        imu_->setPosition(joyXR, y_+imuY);
        mainBtns_[0]->setPosition(btnSideXR, y_+btn0Y);
        mainBtns_[1]->setPosition(btnSideXR, y_+btn1Y);
        mainBtns_[2]->setPosition(btn2XR, y_+btnTopY);
        mainBtns_[3]->setPosition(btn3XR, y_+btnTopY);
        for(unsigned int i=0; i<4; i++) {
            batteryState_[i]->setPosition(battStateXR + i*(battStateW+1), y_+battStateY);
        }
        pot1_->setPosition(potXR, y_+pot1Y);
        pot2_->setPosition(potXR, y_+pot2Y);
    }

    setNeedsFullRedraw();
}

Result RemoteVisWidget::draw() {
    if(needsFullRedraw_) {
        Display::display.rect(0, y_, w, h+y_, Display::BLACK, true);
        if(left_) {
            Display::display.rect(bxl, y_+by, bxl+bw, y_+by+bh, bgCol_, true);
            Display::display.rect(bxl, y_+by, bxl+bw, y_+by+bh, fgCol_, false);
        } else {
            Display::display.rect(bxr, y_+by, bxr+bw, y_+by+bh, bgCol_, true);
            Display::display.rect(bxr, y_+by, bxr+bw, y_+by+bh, fgCol_, false);
        }
        needsFullRedraw_ = false;
    }

    return MultiWidget::draw();
}

void RemoteVisWidget::takeInputFocus() {
    if(left_) {
        Input::inst.setEncTurnCallback([this](float enc) { this->encTurn(enc); });
        Input::inst.setLeftPressCallback([this]{ this->selectPot2();});
        Input::inst.setLeftReleaseCallback([this]{ this->selectPot1();});
    } else {
        Input::inst.setEncTurnCallback(nullptr);
    }
}

void RemoteVisWidget::encTurn(float enc) {
    if(left_) {
        if(potSelected_ == POT1_SELECTED) {
            Input::inst.pot1 += enc/360;
            if(Input::inst.pot1 > 1.0) Input::inst.pot1 = 1.0;
            if(Input::inst.pot1 < 0) Input::inst.pot1 = 0;
        } else {
            Input::inst.pot2 += enc/360;
            if(Input::inst.pot2 > 1.0) Input::inst.pot2 = 1.0;
            if(Input::inst.pot2 < 0) Input::inst.pot2 = 0;
        }
    }
}

void RemoteVisWidget::selectPot1() {
    pot2_->setFrameColor(pot1_->frameColor());
    pot1_->setFrameColor(Display::GREEN);
    potSelected_ = POT1_SELECTED;
}

void RemoteVisWidget::selectPot2() {
    pot1_->setFrameColor(pot2_->frameColor());
    pot2_->setFrameColor(Display::GREEN);
    potSelected_ = POT2_SELECTED;
}

Result RemoteVisWidget::visualizeFromInput() {
    crosshair_->setHorVer(Input::inst.joyH, Input::inst.joyV);
    if(EPSILON(Input::inst.joyH) && EPSILON(Input::inst.joyV)) {
        crosshair_->setCursorColor(Display::WHITE);
    } else {
        crosshair_->setCursorColor(Display::LIGHTGREEN1);
    }
    
    imu_->setRPH(Input::inst.rotR, Input::inst.rotP, Input::inst.rotH);
    imu_->setAccel(Input::inst.aX, Input::inst.aY, Input::inst.aZ);
    pot1_->setAngle(Input::inst.pot1*360.0);
    pot2_->setAngle(Input::inst.pot2*360.0);

    float batt = 6*Input::inst.battery;
    for(int i=1; i<=batt&&i<=4; i++) batteryState_[i-1]->setBackgroundColor(Display::LIGHTGREEN1);
    for(int i=batt; i<=4; i++) batteryState_[i-1]->setBackgroundColor(Display::LIGHTRED1);
    
    for(int i=0; i<4; i++) {
        if(Input::inst.buttons[(Input::Button)i] == true) {
            mainBtns_[i]->setBackgroundColor(Display::LIGHTRED1);
            mainBtns_[i]->setFrameColor(Display::RED);
        } else {
            mainBtns_[i]->setBackgroundColor(Display::DARKBLUE);
            mainBtns_[i]->setFrameColor(Display::LIGHTBLUE2);
        }
    }

    if(Input::inst.buttons[Input::BUTTON_JOY] == true) {
        crosshair_->setBackgroundColor(Display::RED);
    } else {
        crosshair_->setBackgroundColor(Display::BLACK);
    }

    return RES_OK;
}

Result RemoteVisWidget::visualizeFromControlPacket(const bb::rmt::MControlPacket& packet) {
    crosshair_->setHorVer(packet.getAxis(0, UNIT_UNITY_CENTERED), 
                         packet.getAxis(1, UNIT_UNITY_CENTERED));
    if(EPSILON(packet.getAxis(0)) && EPSILON(packet.getAxis(1))) {
        crosshair_->setCursorColor(Display::WHITE);
    } else {
        crosshair_->setCursorColor(Display::LIGHTGREEN1);
    }

    imu_->setRPH(packet.getAxis(2, UNIT_DEGREES_CENTERED), 
                 packet.getAxis(3, UNIT_DEGREES_CENTERED),
                 packet.getAxis(4, UNIT_DEGREES));
    imu_->setAccel(packet.getAxis(5, UNIT_UNITY_CENTERED),
                   packet.getAxis(6, UNIT_UNITY_CENTERED),
                   packet.getAxis(7, UNIT_UNITY_CENTERED));

    pot1_->setAngle(packet.getAxis(8, UNIT_DEGREES));
    pot2_->setAngle(packet.getAxis(9, UNIT_DEGREES));

    float batt = 6*(packet.getAxis(10, UNIT_UNITY));
    for(int i=1; i<=batt&&i<=4; i++) batteryState_[i-1]->setBackgroundColor(Display::LIGHTGREEN1);
    for(int i=batt; i<=4; i++) batteryState_[i-1]->setBackgroundColor(Display::LIGHTRED1);

    if(packet.getAxis(11, UNIT_UNITY)>0) {
        mainBtns_[0]->setBackgroundColor(Display::LIGHTRED1);
        mainBtns_[0]->setFrameColor(Display::RED);
    } else {
        mainBtns_[0]->setBackgroundColor(Display::DARKBLUE);
        mainBtns_[0]->setFrameColor(Display::LIGHTBLUE2);
    }
    if(packet.getAxis(12, UNIT_UNITY)>0) {
        mainBtns_[1]->setBackgroundColor(Display::LIGHTRED1);
        mainBtns_[1]->setFrameColor(Display::RED);
    } else {
        mainBtns_[1]->setBackgroundColor(Display::DARKBLUE);
        mainBtns_[1]->setFrameColor(Display::LIGHTBLUE2);
    }
    if(packet.getAxis(13, UNIT_UNITY)>0) {
        mainBtns_[2]->setBackgroundColor(Display::LIGHTRED1);
        mainBtns_[2]->setFrameColor(Display::RED);
    } else {
        mainBtns_[2]->setBackgroundColor(Display::DARKBLUE);
        mainBtns_[2]->setFrameColor(Display::LIGHTBLUE2);
    }
    if(packet.getAxis(14, UNIT_UNITY)>0) {
        mainBtns_[3]->setBackgroundColor(Display::LIGHTRED1);
        mainBtns_[3]->setFrameColor(Display::RED);
    } else {
        mainBtns_[3]->setBackgroundColor(Display::DARKBLUE);
        mainBtns_[3]->setFrameColor(Display::LIGHTBLUE2);
    }
    if(packet.getAxis(15, UNIT_UNITY)>0) {
        crosshair_->setBackgroundColor(Display::RED);
    } else {
        crosshair_->setBackgroundColor(Display::BLACK);
    }

    return RES_OK;
}
