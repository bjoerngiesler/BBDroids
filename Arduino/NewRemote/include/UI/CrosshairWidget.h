#if !defined(CROSSHAIRWIDGET_H)
#define CROSSHAIRWIDGET_H

#include <LibBB.h>

#include "UI/Display.h"
#include "UI/Widget.h"
#include "Input.h"

using namespace bb;

// Crosshair Visualization Drawable
class CrosshairWidget: public Widget {
public:
  CrosshairWidget();
  virtual Result draw();

  void setHorVer(uint16_t h, uint16_t v);
  void setHorVer(float h, float v);
  void horVerToScreen(uint16_t hor, uint16_t ver, uint8_t& x, uint8_t& y);
  void showMinMaxRect(bool show = true);
  bool doesShowMinMaxRect();
  void setMinMaxRectColor(uint16_t col) { minMaxRectCol_ = col; setNeedsContentsRedraw(); }
  void getMinMax(uint16_t &minHor, uint16_t &maxHor, uint16_t &minVer, uint16_t &maxVer) {
    minHor = minHor_; maxHor = maxHor_; minVer = minVer_; maxVer = maxVer_;
  }
  void setMinMax(uint16_t minHor, uint16_t maxHor, uint16_t minVer, uint16_t maxVer) {
    minHor_ = minHor; maxHor_ = maxHor; minVer_ = minVer; maxVer_ = maxVer;
    setNeedsContentsRedraw();
  }

protected:
    uint16_t hor_, ver_, oldHor_, oldVer_;
    uint16_t minHor_, maxHor_, minVer_, maxVer_;
    uint16_t oldMinHor_, oldMaxHor_, oldMinVer_, oldMaxVer_;
    bool showsMinMaxRect_;
    uint16_t minMaxRectCol_;
};
#endif