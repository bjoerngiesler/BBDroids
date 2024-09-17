#include "RRemoteVisWidget.h"

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
static const uint8_t btn2XL = 35;
static const uint8_t btn3XL = 10;
static const uint8_t btn2XR = btn2XL+bxr;
static const uint8_t btn3XR = btn3XL+bxr;
static const uint8_t btnTopY = 5;

static const uint8_t potW = 19;

static const uint8_t potXL = 50;
static const uint8_t potXR = bxr - (bw-6);
static const uint8_t pot1Y = 61;
static const uint8_t pot2Y = pot1Y+potW+3;

static const uint8_t battStateW = 8;
static const uint8_t battStateY = by + joyW + 9;
static const uint8_t battStateXL = bxl + 7;
static const uint8_t battStateXR = bxr + 7;

RRemoteVisWidget::RRemoteVisWidget() {
    for(auto& b: mainBtns_) {
        b.setDrawsFrame();
        b.setFillsBackground();
        b.setTitle("");
        b.setBackgroundColor(RDisplay::BLUE);
    }

    mainBtns_[0].setSize(btnW, btnH);
    mainBtns_[1].setSize(btnW, btnH);
    mainBtns_[2].setSize(btnH, btnW);
    mainBtns_[3].setSize(btnH, btnW);

    for(auto& b: batteryState_) {
        b.setDrawsFrame();
        b.setFillsBackground();
        b.setTitle("");
        b.setBackgroundColor(RDisplay::RED);
        b.setSize(battStateW, battStateW);
    }

    crosshair_.setSize(joyW, joyW);
    imu_.setSize(joyW, joyW);

    pot1_.setSize(potW, potW);
    pot2_.setSize(potW, potW);

    bgCol_ = RDisplay::LIGHTGREY;
    fgCol_ = RDisplay::GREY;

    setRepresentsLeftRemote(false);

    moveWidgetsAround();
}

void RRemoteVisWidget::setRepresentsLeftRemote(bool left) {
    left_ = left;

    clearWidgets();
    for(auto& b: mainBtns_) addWidget(&b);
    for(auto& b: batteryState_) addWidget(&b);
    addWidget(&crosshair_);
    addWidget(&imu_);
    addWidget(&pot1_);
    if(left_ == false) addWidget(&pot2_);

    moveWidgetsAround();
}

void RRemoteVisWidget::setPosition(int x, int y) {
    RWidget::setPosition(x, y);
    moveWidgetsAround();
}


void RRemoteVisWidget::moveWidgetsAround() {
    if(left_) {
        crosshair_.setPosition(joyXL, y_+xhairY);
        imu_.setPosition(joyXL, y_+imuY);
        mainBtns_[0].setPosition(btnSideXL, y_+btn0Y);
        mainBtns_[1].setPosition(btnSideXL, y_+btn1Y);
        mainBtns_[2].setPosition(btn2XL, y_+btnTopY);
        mainBtns_[3].setPosition(btn3XL, y_+btnTopY);
        for(unsigned int i=0; i<4; i++) {
            batteryState_[i].setPosition(battStateXL + i*(battStateW+1), y_+battStateY);
        }
        pot1_.setPosition(potXL, y_+pot1Y);
        pot2_.setPosition(potXL, y_+pot2Y);
    } else {
        crosshair_.setPosition(joyXR, y_+xhairY);
        imu_.setPosition(joyXR, y_+imuY);
        mainBtns_[0].setPosition(btnSideXR, y_+btn0Y);
        mainBtns_[1].setPosition(btnSideXR, y_+btn1Y);
        mainBtns_[2].setPosition(btn2XR, y_+btnTopY);
        mainBtns_[3].setPosition(btn3XR, y_+btnTopY);
        for(unsigned int i=0; i<4; i++) {
            batteryState_[i].setPosition(battStateXR + i*(battStateW+1), y_+battStateY);
        }
        pot1_.setPosition(potXR, y_+pot1Y);
        pot2_.setPosition(potXR, y_+pot2Y);
    }

    setNeedsFullRedraw();
}

Result RRemoteVisWidget::draw(ConsoleStream* stream) {
    int bodyWidth_ = 56;
    int bodyHeight_ = 120;

    if(needsFullRedraw_) {
        RDisplay::display.rect(0, y_, w, h+y_, RDisplay::BLACK, true);
        if(left_) {
            RDisplay::display.rect(bxl, y_+by, bxl+bw, y_+by+bh, bgCol_, true);
            RDisplay::display.rect(bxl, y_+by, bxl+bw, y_+by+bh, fgCol_, false);
        } else {
            RDisplay::display.rect(bxr, y_+by, bxr+bw, y_+by+bh, bgCol_, true);
            RDisplay::display.rect(bxr, y_+by, bxr+bw, y_+by+bh, fgCol_, false);
        }
        needsFullRedraw_ = false;
    }

    return RMultiWidget::draw(stream);

#if 0
    crosshair_.draw(stream);
    imu_.draw(stream);
    for(auto& b: mainBtns_) b.draw(stream);
    for(auto& b: batteryState_) b.draw(stream);
    pot1_.draw(stream);
    pot2_.draw(stream);
    if(!left_) for(auto btn: topButtons_) btn.draw(stream);
#endif

    return RES_OK;
}

Result RRemoteVisWidget::visualizeFromPacket(const bb::ControlPacket& packet) {
    crosshair_.setHorVer(packet.getAxis(0, bb::ControlPacket::UNIT_UNITY_CENTERED), 
                         packet.getAxis(1, bb::ControlPacket::UNIT_UNITY_CENTERED));
    imu_.setRPH(packet.getAxis(2, bb::ControlPacket::UNIT_DEGREES_CENTERED), 
                packet.getAxis(3, bb::ControlPacket::UNIT_DEGREES_CENTERED),
                packet.getAxis(4, bb::ControlPacket::UNIT_DEGREES));
    imu_.setAccel(packet.getAxis(5, bb::ControlPacket::UNIT_UNITY_CENTERED),
                  packet.getAxis(6, bb::ControlPacket::UNIT_UNITY_CENTERED),
                  packet.getAxis(7, bb::ControlPacket::UNIT_UNITY_CENTERED));

    if(packet.button0) {
        mainBtns_[0].setBackgroundColor(RDisplay::LIGHTRED1);
        mainBtns_[0].setBorderColor(RDisplay::RED);
    } else {
        mainBtns_[0].setBackgroundColor(RDisplay::DARKBLUE);
        mainBtns_[0].setBorderColor(RDisplay::DIMBLUE);
    }
    if(packet.button1) {
        mainBtns_[1].setBackgroundColor(RDisplay::LIGHTRED1);
        mainBtns_[1].setBorderColor(RDisplay::RED);
    } else {
        mainBtns_[1].setBackgroundColor(RDisplay::DARKBLUE);
        mainBtns_[1].setBorderColor(RDisplay::DIMBLUE);
    }
    if(packet.button2) {
        mainBtns_[2].setBackgroundColor(RDisplay::LIGHTRED1);
        mainBtns_[2].setBorderColor(RDisplay::RED);
    } else {
        mainBtns_[2].setBackgroundColor(RDisplay::DARKBLUE);
        mainBtns_[2].setBorderColor(RDisplay::DIMBLUE);
    }
    if(packet.button3) {
        mainBtns_[3].setBackgroundColor(RDisplay::LIGHTRED1);
        mainBtns_[3].setBorderColor(RDisplay::RED);
    } else {
        mainBtns_[3].setBackgroundColor(RDisplay::DARKBLUE);
        mainBtns_[3].setBorderColor(RDisplay::DIMBLUE);
    }
    if(packet.button4) {
        crosshair_.setBackgroundColor(RDisplay::RED);
    } else {
        crosshair_.setBackgroundColor(RDisplay::BLACK);
    }

    pot1_.setAngle(packet.getAxis(8, bb::ControlPacket::UNIT_DEGREES));
    //pot2_.setAngle(packet.getAxis(9, bb::ControlPacket::UNIT_DEGREES));

    int batt = (packet.battery / 16) % 5;
    for(int i=1; i<=batt; i++) batteryState_[i-1].setBackgroundColor(RDisplay::LIGHTGREEN1);
    for(int i=batt+1; i<=4; i++) batteryState_[i-1].setBackgroundColor(RDisplay::LIGHTRED1);

    return RES_OK;
}
