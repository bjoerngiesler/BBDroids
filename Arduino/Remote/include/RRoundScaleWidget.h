#if !defined(RROUNDSCALEWIDGET_H)
#define RROUNDSCALEWIDGET_H

#include "RWidget.h"

class RRoundScaleWidget: public RWidget {
public:
    RRoundScaleWidget();

    virtual Result draw(ConsoleStream* stream);

    virtual void setAngle(float angle);
    virtual void setStartEndAngle(float start, float end);
    virtual void setInfinite();

protected:
    float angle_, oldAngle_;
    bool infinite_;
    float start_, end_;
};

#endif // RROUNDSCALEWIDGET_H