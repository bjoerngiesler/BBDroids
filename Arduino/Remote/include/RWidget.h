#if !defined(RWIDGET_H)
#define RWIDGET_H

#include "RDisplay.h"

class RWidget {
public:
    RWidget();

    virtual Result draw(ConsoleStream* stream = NULL);

    virtual void setPosition(int x, int y);
    virtual void setSize(uint8_t w, uint8_t h);

    virtual uint8_t width() { return width_; }
    virtual uint8_t height() { return height_; }
    virtual int x() { return x_; }
    virtual int y() { return y_; }

    // Whether or not the widget should fill its background. False by default for most non-graphing widgets.
    virtual void setFillsBackground(bool fills = true) { fillsBg_ = fills; needsFullRedraw_ = true; }
    virtual void setDrawsFrame(bool draws = true) { drawsFrame_ = draws; needsFullRedraw_ = true;}

    virtual void setNeedsFullRedraw(bool needs = true);
    virtual void setNeedsContentsRedraw(bool needs = true);

    virtual void setBackgroundColor(uint8_t background);
    virtual void setForegroundColor(uint8_t foreground);
    virtual void setCursorColor(uint8_t cursor);
    virtual void setMarkingColor(uint8_t marking);
    virtual void setBorderColor(uint8_t border);

    virtual void takeInputFocus();

    virtual void setAction(std::function<void(void)> cb);
    virtual std::function<void(void)> action();
    virtual void triggerAction();

    enum CursorHint {
        CURSOR_NONE  = 0,
        CURSOR_LEFT  = 0x01,
        CURSOR_RIGHT = 0x02
    };

    virtual void setName(const String& name);
    virtual const String& name() { return name_; }

    virtual CursorHint cursorHint() { return CURSOR_NONE; }
protected:
    bool needsFullRedraw_, needsContentsRedraw_;
    int x_, y_;
    uint8_t width_, height_;
    uint8_t bgCol_, fgCol_, borderCol_, cursorCol_, markingCol_;
    bool fillsBg_, drawsFrame_;
    String name_;
    std::function<void(void)> action_;
};

#endif // RWIDGET_H