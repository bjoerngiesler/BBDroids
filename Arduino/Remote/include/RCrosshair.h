#if !defined(RJOYVIS_H)
#define RJOYVIS_H

#include <LibBB.h>

#include "RDisplay.h"
#include "RInput.h"

using namespace bb;

// Joystick Visualization Drawable
class RJoyVis: public RDrawable {
public:
  RJoyVis();
  virtual Result draw(ConsoleStream* stream = NULL);
  void setHorVer(uint16_t h, uint16_t v);
  void horVerToScreen(uint16_t hor, uint16_t ver, uint8_t& x, uint8_t& y);

protected:
    uint16_t hor_, ver_, oldHor_, oldVer_;
};
#endif