#if !defined(WIDGET_H)
#define WIDGET_H

#include "UI/Display.h"

class Widget {
public:
    Widget();
    virtual ~Widget();

    virtual Result draw();

    virtual void setPosition(int x, int y);
    virtual void setSize(uint8_t w, uint8_t h);
    virtual void centerOnDisplay();
    virtual void centerOnMain();

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
    uint8_t backgroundColor() { return bgCol_; }
    virtual void setForegroundColor(uint8_t foreground);
    uint8_t foregroundColor() { return fgCol_; }
    virtual void setCursorColor(uint8_t cursor);
    uint8_t cursorColor() { return cursorCol_; }
    virtual void setMarkingColor(uint8_t marking);
    uint8_t markingColor() { return markingCol_; }
    virtual void setFrameColor(uint8_t frame);
    uint8_t frameColor() { return frameCol_; }
    virtual void setHighlightColor(uint8_t highlight);
    uint8_t highlightColor() { return hlCol_; }

    virtual void takeInputFocus();

    virtual void setAction(std::function<void(Widget*)> cb);
    virtual std::function<void(Widget*)> action();
    virtual void triggerAction();

    enum CursorHint {
        CURSOR_NONE  = 0,
        CURSOR_LEFT  = 0x01,
        CURSOR_RIGHT = 0x02
    };

    virtual void setName(const String& name);
    virtual const String& name() { return name_; }

    virtual CursorHint cursorHint() { return CURSOR_NONE; }

    //! Highlighted widgets draw their fg in hlCol_ instead of fgCol_
    virtual void setHighlighted(bool yesno=true); 
    virtual bool isHighlighted() { return highlighted_; }

    void setTag(int tag) { tag_ = tag; }
    int tag() { return tag_; }

protected:
    bool needsFullRedraw_, needsContentsRedraw_;
    int x_, y_;
    uint8_t width_, height_;
    uint8_t bgCol_, fgCol_, frameCol_, cursorCol_, markingCol_, hlCol_;
    bool fillsBg_, drawsFrame_;
    String name_;
    std::function<void(Widget*)> action_;
    bool highlighted_;
    int tag_;
};

#endif // RWIDGET_H