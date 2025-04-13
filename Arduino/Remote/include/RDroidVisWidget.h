#if !defined(RDROIDVISWIDGET_H)
#define RDROIDVISWIDGET_H

#include "RMultiWidget.h"
#include "RLabelWidget.h"

class RDroidVisWidget: public RMultiWidget {
public:
    RDroidVisWidget();

    virtual void setSize(uint8_t w, uint8_t h) {} // we can't be resized

    virtual void visualizeFromStatePacket(const bb::StatePacket& packet);

    void moveWidgetsAround();
protected:
    void configureLabel(RLabelWidget& widget, StatePacket::StatusType type, bool setTitle);

    RLabelWidget droidStatusLabel_;
    RLabelWidget batt1Label_;
    RLabelWidget batt2Label_;
    RLabelWidget driveStatusLabel_;
    RLabelWidget servoStatusLabel_;
    RLabelWidget driveModeLabel_;
    
    RLabelWidget droidStatus_;
    RLabelWidget batt1Status_;
    RLabelWidget batt2Status_;
    RLabelWidget driveStatus_;
    RLabelWidget servoStatus_;
    RLabelWidget driveMode_;
};

#endif