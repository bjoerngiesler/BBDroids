#include "RDroidVisWidget.h"

RDroidVisWidget::RDroidVisWidget() {
    width_ = RDisplay::MAIN_WIDTH;
    height_ = RDisplay::MAIN_HEIGHT;
    droidStatusLabel_.setTitle("Droid");
    batt1Label_.setTitle("Batt1");
    batt2Label_.setTitle("Batt2");
    driveStatusLabel_.setTitle("Drive");
    servoStatusLabel_.setTitle("Servos");
    driveModeLabel_.setTitle("Mode");
    
    unsigned int w = 5 * RDisplay::CHAR_WIDTH;
    unsigned int h = RDisplay::CHAR_HEIGHT;

    droidStatus_.setTitle("?");
    droidStatus_.setSize(w, h);
    droidStatus_.setDrawsFrame();
    droidStatus_.setFillsBackground(true);
    droidStatus_.setBackgroundColor(RDisplay::BLACK);

    batt1Status_.setTitle("?");
    batt1Status_.setSize(w, h);
    batt1Status_.setDrawsFrame();
    batt1Status_.setFillsBackground(true);
    batt1Status_.setBackgroundColor(RDisplay::BLACK);

    batt2Status_.setTitle("?");
    batt2Status_.setSize(w, h);
    batt2Status_.setDrawsFrame();
    batt2Status_.setFillsBackground(true);
    batt2Status_.setBackgroundColor(RDisplay::BLACK);

    driveStatus_.setTitle("?");
    driveStatus_.setSize(w, h);
    driveStatus_.setDrawsFrame();
    driveStatus_.setFillsBackground(true);
    driveStatus_.setBackgroundColor(RDisplay::BLACK);

    servoStatus_.setTitle("?");
    servoStatus_.setSize(w, h);
    servoStatus_.setDrawsFrame();
    servoStatus_.setFillsBackground(true);
    servoStatus_.setBackgroundColor(RDisplay::BLACK);

    driveMode_.setTitle("?");
    driveMode_.setSize(w, h);
    driveMode_.setDrawsFrame();
    driveMode_.setFillsBackground(true);
    driveMode_.setBackgroundColor(RDisplay::BLACK);
    
    addWidget(&droidStatusLabel_);
    addWidget(&batt1Label_);
    addWidget(&batt2Label_);
    addWidget(&driveStatusLabel_);
    addWidget(&servoStatusLabel_);
    addWidget(&driveModeLabel_);
    
    addWidget(&droidStatus_);
    addWidget(&batt1Status_);
    addWidget(&batt2Status_);
    addWidget(&driveStatus_);
    addWidget(&servoStatus_);
    addWidget(&driveMode_);

    setFillsBackground();

    moveWidgetsAround();
}

void RDroidVisWidget::moveWidgetsAround() {
    unsigned int row = 0;
    unsigned int inset = 7;
    unsigned int rowstep = RDisplay::CHAR_HEIGHT+3;

    droidStatusLabel_.setPosition(x(), y() + row*rowstep);
    droidStatus_.setPosition(x()+inset*RDisplay::CHAR_WIDTH, y() + row*rowstep);
    row++;

    batt1Label_.setPosition(x(), y() + row*rowstep);
    batt1Status_.setPosition(x()+inset*RDisplay::CHAR_WIDTH, y() + row*rowstep);
    row++;
    
    batt2Label_.setPosition(x(), y() + row*rowstep);
    batt2Status_.setPosition(x()+inset*RDisplay::CHAR_WIDTH, y() + row*rowstep);
    row++;

    driveStatusLabel_.setPosition(x(), y() + row*rowstep);
    driveStatus_.setPosition(x()+inset*RDisplay::CHAR_WIDTH, y() + row*rowstep);
    row++;

    servoStatusLabel_.setPosition(x(), y() + row*rowstep);
    servoStatus_.setPosition(x()+inset*RDisplay::CHAR_WIDTH, y() + row*rowstep);
    row++;

    driveModeLabel_.setPosition(x(), y() + row*rowstep);
    driveMode_.setPosition(x()+inset*RDisplay::CHAR_WIDTH, y() + row*rowstep);
    row++;
}

void RDroidVisWidget::visualizeFromStatePacket(const bb::StatePacket& packet) {
    configureLabel(droidStatus_, packet.droidStatus, true);
    
    if(packet.batt1Status == StatePacket::STATUS_OK) {
        configureLabel(batt1Status_, packet.batt1Status, false);
        batt1Status_.setTitle(String(packet.batt1Voltage));
    } else {
        configureLabel(batt1Status_, packet.batt1Status, true);
    }
    if(packet.batt2Status == StatePacket::STATUS_OK) {
        configureLabel(batt2Status_, packet.batt2Status, false);
        batt2Status_.setTitle(String(packet.batt2Voltage));
    } else {
        configureLabel(batt2Status_, packet.batt2Status, true);
    }
    configureLabel(driveStatus_, packet.driveStatus, true);
    configureLabel(servoStatus_, packet.servoStatus, true);

    switch(packet.driveMode) {
    case StatePacket::DRIVE_OFF:
        driveMode_.setTitle("Off");
        driveMode_.setBackgroundColor(RDisplay::BLACK);
        driveMode_.setForegroundColor(RDisplay::WHITE);
        break;
    case StatePacket::DRIVE_VEL:
        driveMode_.setTitle("Vel");
        driveMode_.setBackgroundColor(RDisplay::DARKGREEN);
        driveMode_.setForegroundColor(RDisplay::WHITE);
        break;
    case StatePacket::DRIVE_AUTO_POS:
        driveMode_.setTitle("APos");
        driveMode_.setBackgroundColor(RDisplay::DARKBLUE);
        driveMode_.setForegroundColor(RDisplay::WHITE);
        break;
    case StatePacket::DRIVE_POS:
        driveMode_.setTitle("Pos");
        driveMode_.setBackgroundColor(RDisplay::DARKYELLOW);
        driveMode_.setForegroundColor(RDisplay::WHITE);
        break;
    case StatePacket::DRIVE_AUTONOMOUS:
        driveMode_.setTitle("Auto");
        driveMode_.setBackgroundColor(RDisplay::WHITE);
        driveMode_.setForegroundColor(RDisplay::BLACK);
        break;
    }
}

void RDroidVisWidget::configureLabel(RLabelWidget& widget, StatePacket::StatusType type, bool setTitle) {
    switch(type) {
    case StatePacket::STATUS_OK:
        if(setTitle) widget.setTitle("OK");
        widget.setForegroundColor(RDisplay::WHITE);
        widget.setBackgroundColor(RDisplay::DARKGREEN);
        break;
    case StatePacket::STATUS_ERROR:
        if(setTitle) widget.setTitle("Err");
        widget.setForegroundColor(RDisplay::WHITE);
        widget.setBackgroundColor(RDisplay::DARKRED);
        break;
    case StatePacket::STATUS_DEGRADED:
        if(setTitle) widget.setTitle("Degr");
        widget.setForegroundColor(RDisplay::BLACK);
        widget.setBackgroundColor(RDisplay::YELLOW);
        break;
    case StatePacket::STATUS_NA:
    default:
        if(setTitle) widget.setTitle("N/A");
        widget.setForegroundColor(RDisplay::BLACK);
        widget.setBackgroundColor(RDisplay::GREY);
        break;
    }
}
