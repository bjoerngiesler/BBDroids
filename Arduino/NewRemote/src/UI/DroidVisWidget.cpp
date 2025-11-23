#include "UI/DroidVisWidget.h"

DroidVisWidget::DroidVisWidget() {
    width_ = Display::MAIN_WIDTH;
    height_ = Display::MAIN_HEIGHT;
    droidStatusLabel_.setTitle("Droid");
    battLabel_.setTitle("Batt");
    driveStatusLabel_.setTitle("Drive");
    servoStatusLabel_.setTitle("Servos");
    driveModeLabel_.setTitle("Mode");
    
    unsigned int w = 6 * Display::CHAR_WIDTH;
    unsigned int h = Display::CHAR_HEIGHT;

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

void DroidVisWidget::reset() {
    droidStatus_.setTitle("?");
    droidStatus_.setBackgroundColor(Display::GREY);
    droidStatus_.setForegroundColor(Display::WHITE);

    battStatus_.setTitle("?");
    battStatus_.setBackgroundColor(Display::GREY);
    battStatus_.setForegroundColor(Display::WHITE);

    driveStatus_.setTitle("?");
    driveStatus_.setBackgroundColor(Display::GREY);
    driveStatus_.setForegroundColor(Display::WHITE);

    servoStatus_.setTitle("?");
    servoStatus_.setBackgroundColor(Display::GREY);
    servoStatus_.setForegroundColor(Display::WHITE);

    driveMode_.setTitle("?");
    driveMode_.setBackgroundColor(Display::GREY);
    driveMode_.setForegroundColor(Display::WHITE);

    velScale_.setValue(0);
    imuDisplay_.setRPH(0, 0, 0);
    velLabel_.setTitle("?");
}

void DroidVisWidget::moveWidgetsAround() {
    unsigned int ypos = 5, xpos = 2;
    unsigned int inset = 7;
    unsigned int rowstep = Display::CHAR_HEIGHT+3;

    droidStatusLabel_.setPosition(xpos, y() + ypos);
    droidStatus_.setPosition(xpos+inset*Display::CHAR_WIDTH, y() + ypos);
    ypos += rowstep;

    driveStatusLabel_.setPosition(xpos, y() + ypos);
    driveStatus_.setPosition(xpos+inset*Display::CHAR_WIDTH, y() + ypos);
    ypos += rowstep;

    servoStatusLabel_.setPosition(xpos, y() + ypos);
    servoStatus_.setPosition(xpos+inset*Display::CHAR_WIDTH, y() + ypos);
    ypos += rowstep;

    driveModeLabel_.setPosition(xpos, y() + ypos);
    driveMode_.setPosition(xpos+inset*Display::CHAR_WIDTH, y() + ypos);
    ypos += rowstep;

    battLabel_.setPosition(xpos, y() + ypos);
    battStatus_.setPosition(xpos+inset*Display::CHAR_WIDTH, y() + ypos);
    ypos += 2*rowstep;
    
    ypos += 2;
    velScale_.setSize(34, 34);
    velScale_.setPosition(xpos+3, y()+ypos);
    
    imuDisplay_.setSize(34, 34);
    imuDisplay_.setPosition(velScale_.x()+velScale_.width()+6, y()+ypos);
    ypos += velScale_.height()+1;
    
    velLabel_.setSize(velScale_.width(), Display::CHAR_HEIGHT);
    velLabel_.setPosition(velScale_.x(), y()+ypos);
}

void DroidVisWidget::visualizeFromTelemetry(const bb::rmt::Telemetry& t) {
    char buf[16];

    configureLabel(droidStatus_, t.overallStatus, true);
    
    if(t.batteryStatus == Telemetry::STATUS_OK) {
        configureLabel(battStatus_, t.batteryStatus, false);
        sprintf(buf, "%.1fV\n%.1fA", t.batteryVoltage, t.batteryCurrent);
        battStatus_.setTitle(buf);
    } else {
        configureLabel(battStatus_, t.batteryStatus, true);
    }
    configureLabel(driveStatus_, t.driveStatus, true);
    configureLabel(servoStatus_, t.servoStatus, true);

    switch(t.driveMode) {
    case Telemetry::DRIVE_OFF:
        driveMode_.setTitle("Off");
        driveMode_.setBackgroundColor(Display::BLACK);
        driveMode_.setForegroundColor(Display::WHITE);
        break;
    case Telemetry::DRIVE_VEL:
        driveMode_.setTitle("Vel");
        driveMode_.setBackgroundColor(Display::DARKGREEN);
        driveMode_.setForegroundColor(Display::WHITE);
        break;
    case Telemetry::DRIVE_AUTO_POS:
        driveMode_.setTitle("APos");
        driveMode_.setBackgroundColor(Display::DARKBLUE);
        driveMode_.setForegroundColor(Display::WHITE);
        break;
    case Telemetry::DRIVE_POS:
        driveMode_.setTitle("Pos");
        driveMode_.setBackgroundColor(Display::DARKYELLOW);
        driveMode_.setForegroundColor(Display::WHITE);
        break;
    case Telemetry::DRIVE_AUTONOMOUS:
        driveMode_.setTitle("Auto");
        driveMode_.setBackgroundColor(Display::WHITE);
        driveMode_.setForegroundColor(Display::BLACK);
        break;
    case Telemetry::DRIVE_CUSTOM1:
        driveMode_.setTitle("Cus1");
        driveMode_.setBackgroundColor(Display::WHITE);
        driveMode_.setForegroundColor(Display::BLACK);
        break;
    case Telemetry::DRIVE_CUSTOM2:
        driveMode_.setTitle("Cus2");
        driveMode_.setBackgroundColor(Display::WHITE);
        driveMode_.setForegroundColor(Display::BLACK);
        break;
    case Telemetry::DRIVE_CUSTOM3:
    default:
        driveMode_.setTitle("Cus3");
        driveMode_.setBackgroundColor(Display::WHITE);
        driveMode_.setForegroundColor(Display::BLACK);
        break;
    }

    imuDisplay_.setRPH(t.imuPitch, t.imuRoll, t.imuHeading);
    float velKph = 3.6f * t.speed;
    velScale_.setValue(velKph);
    sprintf(buf, "%.1fkph", velKph);
    velLabel_.setTitle(buf);
}

void DroidVisWidget::configureLabel(RLabelWidget& widget, Telemetry::SubsysStatus type, bool setTitle) {
    switch(type) {
    case Telemetry::STATUS_OK:
        if(setTitle) widget.setTitle("OK");
        widget.setForegroundColor(Display::WHITE);
        widget.setBackgroundColor(Display::DARKGREEN);
        break;
    case Telemetry::STATUS_ERROR:
        if(setTitle) widget.setTitle("Err");
        widget.setForegroundColor(Display::WHITE);
        widget.setBackgroundColor(Display::DARKRED);
        break;
    case Telemetry::STATUS_DEGRADED:
        if(setTitle) widget.setTitle("Degr");
        widget.setForegroundColor(Display::BLACK);
        widget.setBackgroundColor(Display::YELLOW);
        break;
    case Telemetry::STATUS_NA:
    default:
        if(setTitle) widget.setTitle("N/A");
        widget.setForegroundColor(Display::BLACK);
        widget.setBackgroundColor(Display::GREY);
        break;
    }
}
