#if !defined(RWIDGET_H)
#define RWIDGET_H

#include "RDisplay.h"

class RWidget {
public:
    RWidget();

    virtual Result draw(ConsoleStream* stream = NULL);

    virtual void setPosition(uint8_t x, uint8_t y);
    virtual void setSize(uint8_t w, uint8_t h);

    virtual uint8_t width() { return width_; }
    virtual uint8_t height() { return height_; }

    // Whether or not the widget should fill its background. False by default for most non-graphing widgets.
    virtual void setFillsBackground(bool fills = true) { fillsBg_ = fills; needsContentsRedraw_ = true; }
    virtual void setDrawsFrame(bool draws = true) { drawsFrame_ = draws; needsFullRedraw_ = true;}

    virtual void setNeedsCls(bool needs = true);
    virtual void setNeedsFullRedraw(bool needs = true);
    virtual void setNeedsContentsRedraw(bool needs = true);

    void setBackgroundColor(uint16_t background);
    void setForegroundColor(uint16_t foreground);
    void setBorderColor(uint16_t border);
protected:
  bool needsCls_, needsFullRedraw_, needsContentsRedraw_;
  uint8_t x_, y_;
  uint8_t width_, height_;
  uint16_t bgCol_, fgCol_, borderCol_;
  bool fillsBg_, drawsFrame_;
};

#endif // RWIDGET_H