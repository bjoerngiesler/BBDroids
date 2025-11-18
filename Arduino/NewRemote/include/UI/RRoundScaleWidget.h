#if !defined(RROUNDSCALEWIDGET_H)
#define RROUNDSCALEWIDGET_H

#include "UI/RWidget.h"
#include "Todo/RInput.h"

class RRoundScaleWidget: public RWidget {
public:
    RRoundScaleWidget();

    virtual Result draw(ConsoleStream* stream);

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

#endif // RROUNDSCALEWIDGET_H