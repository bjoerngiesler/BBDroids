#if !defined(DROIDVISWIDGET_H)
#define DROIDVISWIDGET_H

#include "UI/MultiWidget.h"
#include "UI/Label.h"
#include "UI/RoundScaleWidget.h"
#include "UI/IMUWidget.h"

#include <LibBBRemotes.h>

class DroidVisWidget: public MultiWidget {
public:
    DroidVisWidget();

    virtual void setSize(uint8_t w, uint8_t h) {} // we can't be resized

    virtual void visualizeFromTelemetry(const bb::rmt::Telemetry& telem);
    virtual void reset();

    void moveWidgetsAround();
protected:
    void configureLabel(shared_ptr<Label>& widget, Telemetry::SubsysStatus type, bool setTitle);

    shared_ptr<Label> droidStatusLabel_;
    shared_ptr<Label> battLabel_;
    shared_ptr<Label> driveStatusLabel_;
    shared_ptr<Label> servoStatusLabel_;
    shared_ptr<Label> driveModeLabel_;
    
    shared_ptr<Label> droidStatus_;
    shared_ptr<Label> battStatus_;
    shared_ptr<Label> driveStatus_;
    shared_ptr<Label> servoStatus_;
    shared_ptr<Label> driveMode_;

    shared_ptr<RoundScaleWidget> velScale_;
    shared_ptr<Label> velLabel_;
    shared_ptr<IMUWidget> imuDisplay_;
};

#endif