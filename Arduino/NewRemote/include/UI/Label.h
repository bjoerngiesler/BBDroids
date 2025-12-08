#if !defined(LABEL_H)
#define LABEL_H

#include "UI/Widget.h"

class Label: public Widget {
public:
    Label();

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

    enum FrameType {
        FRAME_NONE = 0x0,
        FRAME_BOTTOM = 0x1,
        FRAME_LEFT = 0x2,
        FRAME_RIGHT = 0x4,
        FRAME_TOP = 0x8,
        FRAME_ALL = 0xf
    };

    virtual Result draw();

    virtual void setTitle(const String& title);
    virtual const String& title() { return title_; }
    void setJustification(HorizontalJustification hor, VerticalJustification ver);
    void setAutoscroll(bool autoscroll = true);
    void setAutoscrollTiming(float leftwait, float pixelwait, float rightwait);
    void setLinebreak(bool lb=true) { linebreak_ = lb; }
    void setAutosize(bool as=true) { autosize_ = as; }

    virtual void setDrawsFrame(bool yesno = true) { setFrameType(FRAME_ALL); }
    virtual void setFrameType(FrameType type) { frameType_ = type; setNeedsFullRedraw(); }

    //! Split str into lines, with a maximum line width of maxwidth (in characters)
    static std::vector<String> splitLines(const String& str, unsigned int maxwidth);

protected:
    HorizontalJustification hor_;
    VerticalJustification ver_;
    String title_;
    std::vector<String> lines_;
    bool autoscroll_;
    bool linebreak_;
    bool autosize_;
    float leftwait_, pixelwait_, rightwait_;
    int8_t xdelta_;
    FrameType frameType_;
};

#endif