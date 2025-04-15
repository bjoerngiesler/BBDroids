#if !defined(RDROIDVISWIDGET_H)
#define RDROIDVISWIDGET_H

#include "RMultiWidget.h"
#include "RLabelWidget.h"
#include "RRoundScaleWidget.h"
#include "RIMUWidget.h"

class RDroidVisWidget: public RMultiWidget {
public:
    RDroidVisWidget();

    virtual void setSize(uint8_t w, uint8_t h) {} // we can't be resized

    virtual void visualizeFromStatePacket(const bb::StatePacket& packet);
    virtual void reset();

    void moveWidgetsAround();
protected:
    void configureLabel(RLabelWidget& widget, StatePacket::StatusType type, bool setTitle);

    RLabelWidget droidStatusLabel_;
    RLabelWidget battLabel_;
    RLabelWidget driveStatusLabel_;
    RLabelWidget servoStatusLabel_;
    RLabelWidget driveModeLabel_;
    
    RLabelWidget droidStatus_;
    RLabelWidget battStatus_;
    RLabelWidget driveStatus_;
    RLabelWidget servoStatus_;
    RLabelWidget driveMode_;

    RRoundScaleWidget velScale_;
    RLabelWidget velLabel_;
    RIMUWidget imuDisplay_;
};

#endif