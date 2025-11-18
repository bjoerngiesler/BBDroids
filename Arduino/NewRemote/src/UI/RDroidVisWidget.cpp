#include "UI/RDroidVisWidget.h"

RDroidVisWidget::RDroidVisWidget() {
    width_ = RDisplay::MAIN_WIDTH;
    height_ = RDisplay::MAIN_HEIGHT;
    droidStatusLabel_.setTitle("Droid");
    battLabel_.setTitle("Batt");
    driveStatusLabel_.setTitle("Drive");
    servoStatusLabel_.setTitle("Servos");
    driveModeLabel_.setTitle("Mode");
    
    unsigned int w = 6 * RDisplay::CHAR_WIDTH;
    unsigned int h = RDisplay::CHAR_HEIGHT;

    droidStatus_.setSize(w, h);
    droidStatus_.setDrawsFrame();
    droidStatus_.setFillsBackground(true);

    driveStatus_.setSize(w, h);
    driveStatus_.setDrawsFrame();
    driveStatus_.setFillsBackground(true);

    servoStatus_.setSize(w, h);
    servoStatus_.setDrawsFrame();
    servoStatus_.setFillsBackground(true);

    driveMode_.setSize(w, h);
    driveMode_.setDrawsFrame();
    driveMode_.setFillsBackground(true);

    battStatus_.setSize(w, 2*h);
    battStatus_.setLinebreak(true);
    battStatus_.setDrawsFrame();
    battStatus_.setFillsBackground(true);

    velScale_.setStartEndAngle(100, 360+80);
    velScale_.setMinMaxValue(0, 4.0);
    
    addWidget(&droidStatusLabel_);
    addWidget(&battLabel_);
    addWidget(&driveStatusLabel_);
    addWidget(&servoStatusLabel_);
    addWidget(&driveModeLabel_);
    
    addWidget(&droidStatus_);
    addWidget(&battStatus_);
    addWidget(&driveStatus_);
    addWidget(&servoStatus_);
    addWidget(&driveMode_);

    addWidget(&velScale_);
    addWidget(&velLabel_);
    addWidget(&imuDisplay_);

    setFillsBackground();

    moveWidgetsAround();
}

void RDroidVisWidget::reset() {
    droidStatus_.setTitle("?");
    droidStatus_.setBackgroundColor(RDisplay::GREY);
    droidStatus_.setForegroundColor(RDisplay::WHITE);

    battStatus_.setTitle("?");
    battStatus_.setBackgroundColor(RDisplay::GREY);
    battStatus_.setForegroundColor(RDisplay::WHITE);

    driveStatus_.setTitle("?");
    driveStatus_.setBackgroundColor(RDisplay::GREY);
    driveStatus_.setForegroundColor(RDisplay::WHITE);

    servoStatus_.setTitle("?");
    servoStatus_.setBackgroundColor(RDisplay::GREY);
    servoStatus_.setForegroundColor(RDisplay::WHITE);

    driveMode_.setTitle("?");
    driveMode_.setBackgroundColor(RDisplay::GREY);
    driveMode_.setForegroundColor(RDisplay::WHITE);

    velScale_.setValue(0);
    imuDisplay_.setRPH(0, 0, 0);
    velLabel_.setTitle("?");
}

void RDroidVisWidget::moveWidgetsAround() {
    unsigned int ypos = 5, xpos = 2;
    unsigned int inset = 7;
    unsigned int rowstep = RDisplay::CHAR_HEIGHT+3;

    droidStatusLabel_.setPosition(xpos, y() + ypos);
    droidStatus_.setPosition(xpos+inset*RDisplay::CHAR_WIDTH, y() + ypos);
    ypos += rowstep;

    driveStatusLabel_.setPosition(xpos, y() + ypos);
    driveStatus_.setPosition(xpos+inset*RDisplay::CHAR_WIDTH, y() + ypos);
    ypos += rowstep;

    servoStatusLabel_.setPosition(xpos, y() + ypos);
    servoStatus_.setPosition(xpos+inset*RDisplay::CHAR_WIDTH, y() + ypos);
    ypos += rowstep;

    driveModeLabel_.setPosition(xpos, y() + ypos);
    driveMode_.setPosition(xpos+inset*RDisplay::CHAR_WIDTH, y() + ypos);
    ypos += rowstep;

    battLabel_.setPosition(xpos, y() + ypos);
    battStatus_.setPosition(xpos+inset*RDisplay::CHAR_WIDTH, y() + ypos);
    ypos += 2*rowstep;
    
    ypos += 2;
    velScale_.setSize(34, 34);
    velScale_.setPosition(xpos+3, y()+ypos);
    
    imuDisplay_.setSize(34, 34);
    imuDisplay_.setPosition(velScale_.x()+velScale_.width()+6, y()+ypos);
    ypos += velScale_.height()+1;
    
    velLabel_.setSize(velScale_.width(), RDisplay::CHAR_HEIGHT);
    velLabel_.setPosition(velScale_.x(), y()+ypos);
}

void RDroidVisWidget::visualizeFromStatePacket(const bb::StatePacket& packet) {
    char buf[16];

    configureLabel(droidStatus_, packet.droidStatus, true);
    
    if(packet.battStatus == StatePacket::STATUS_OK) {
        configureLabel(battStatus_, packet.battStatus, false);
        sprintf(buf, "%.1fV\n%.1fA", 3.0f+0.1f*packet.battVoltage, 0.1*packet.battCurrent);
        battStatus_.setTitle(buf);
    } else {
        configureLabel(battStatus_, packet.battStatus, true);
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

    float angleFactor = 360.0/1024.0;
    imuDisplay_.setRPH(packet.roll*angleFactor, packet.pitch*angleFactor, packet.heading*angleFactor);
    float velKph = (3.6f * packet.speed)/1000.0f;
    velScale_.setValue(velKph);
    sprintf(buf, "%.1fkph", velKph);
    velLabel_.setTitle(buf);
    
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
