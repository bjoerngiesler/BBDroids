#if !defined(DROIDVISWIDGET_H)
#define DROIDVISWIDGET_H

#include "UI/RMultiWidget.h"
#include "UI/RLabelWidget.h"
#include "UI/RRoundScaleWidget.h"
#include "UI/RIMUWidget.h"

#include <LibBBRemotes.h>

class DroidVisWidget: public RMultiWidget {
public:
    DroidVisWidget();

    virtual void setSize(uint8_t w, uint8_t h) {} // we can't be resized

    virtual void visualizeFromTelemetry(const bb::rmt::Telemetry& telem);
    virtual void reset();

    void moveWidgetsAround();
protected:
    void configureLabel(RLabelWidget& widget, Telemetry::SubsysStatus type, bool setTitle);

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