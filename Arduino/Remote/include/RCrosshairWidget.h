#if !defined(RCROSSHAIRWIDGET_H)
#define RCROSSHAIRWIDGET_H

#include <LibBB.h>

#include "RDisplay.h"
#include "RInput.h"
#include "RWidget.h"

using namespace bb;

// Crosshair Visualization Drawable
class RCrosshairWidget: public RWidget {
public:
  RCrosshairWidget();
  virtual Result draw(ConsoleStream* stream = NULL);

  void setHorVer(uint16_t h, uint16_t v);
  void setHorVer(float h, float v);
  void horVerToScreen(uint16_t hor, uint16_t ver, uint8_t& x, uint8_t& y);

protected:
    uint16_t hor_, ver_, oldHor_, oldVer_;
};
#endif