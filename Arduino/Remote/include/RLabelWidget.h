#if !defined(RLABELWIDGET_H)
#define RLABELWIDGET_H

#include "RWidget.h"

class RLabelWidget: virtual public RWidget {
public:
    RLabelWidget();

    enum HorizontalJustification {
        LEFT_JUSTIFIED,
        HOR_CENTERED,
        RIGHT_JUSTIFIED
    };
    enum VerticalJustification {
        TOP_JUSTIFIED,
        VER_CENTERED,
        BOTTOM_JUSTIFIED
    };

    virtual Result draw(ConsoleStream *stream);

    void setTitle(const String& title);
    void setJustification(HorizontalJustification hor, VerticalJustification ver);
    void setAutoscroll(bool autoscroll = true);
    void setAutoscrollTiming(float leftwait, float pixelwait, float rightwait);

protected:
    HorizontalJustification hor_;
    VerticalJustification ver_;
    String title_;
    bool autoscroll_;
    float leftwait_, pixelwait_, rightwait_;
    int8_t xdelta_;
};

#endif