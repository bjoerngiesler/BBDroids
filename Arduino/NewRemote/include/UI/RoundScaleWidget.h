#if !defined(ROUNDSCALEWIDGET_H)
#define ROUNDSCALEWIDGET_H

#include "UI/Widget.h"
#include "Input.h"

class RoundScaleWidget: public Widget {
public:
    RoundScaleWidget();

    virtual Result draw();

    virtual void setAngle(float angle, bool radians = false);
    virtual void setStartEndAngle(float start, float end);
    virtual void setValue(float value);
    virtual void setMinMaxValue(float min, float max);
    virtual void setInfinite();

protected:
    float angle_, oldAngle_;
    bool infinite_;
    float start_, end_;
    float min_, max_;
};

#endif // ROUNDSCALEWIDGET_H