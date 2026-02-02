#if !defined(MIXCURVEWIDGET_H)
#define MIXCURVEWIDGET_H

#include "Widget.h"
#include <BBRTypes.h>

class MixCurveWidget: public Widget {
public:
    void setInterpolator(const Interpolator& inter);
    Interpolator interpolator() { return inter_; }

    virtual Result draw();

    virtual void setShowsCursor(bool shows=true);
    virtual bool showsCursor() { return showsCursor_; }
    virtual void setCursor(uint8_t cursor);
    virtual uint8_t cursor() { return cursor_; }

protected:
    void drawNode(unsigned int num, int8_t position);
    void drawLineFrom(unsigned int num, int8_t pos1, int8_t pos2);
    void drawCalibLines();

    Interpolator inter_ = INTERP_ZERO;
    uint8_t cursor_;
    bool showsCursor_;
};

#endif // MIXCURVEWIDGET_H