#if !defined(MIXCURVEWIDGET_H)
#define MIXCURVEWIDGET_H

#include "Widget.h"
#include <BBRTypes.h>

//! Shows an Interpolator (5 points at 0/25/50/75/100% of the axis range, each -125..125%) as a curve.
//! Optionally shows a cursor on one of the points, drawn differently while that point is being edited.
class MixCurveWidget: public Widget {
public:
    static const uint8_t NUM_POINTS = 5;
    static const int8_t MIN_VALUE = -125, MAX_VALUE = 125;

    void setInterpolator(const Interpolator& inter);
    Interpolator interpolator() { return inter_; }

    //! Value of point num (0..4).
    int8_t pointValue(uint8_t num);
    //! Set point num (0..4) to value, clamped to MIN_VALUE..MAX_VALUE.
    void setPointValue(uint8_t num, int value);

    virtual Result draw();

    virtual void setShowsCursor(bool shows=true);
    virtual bool showsCursor() { return showsCursor_; }
    virtual void setCursor(uint8_t cursor);
    virtual uint8_t cursor() { return cursor_; }
    virtual void setEditing(bool editing=true);
    virtual bool editing() { return editing_; }
    //! Draw the node boxes. Off for small thumbnails.
    virtual void setShowsNodes(bool shows=true) { showsNodes_ = shows; setNeedsFullRedraw(); }

protected:
    int pointX(unsigned int num);
    int pointY(int value);
    void drawNode(unsigned int num, int8_t position);
    void drawCalibLines();

    Interpolator inter_ = INTERP_ZERO;
    uint8_t cursor_ = 0;
    bool showsCursor_ = false;
    bool editing_ = false;
    bool showsNodes_ = true;
};

#endif // MIXCURVEWIDGET_H
