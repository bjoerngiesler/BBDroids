#include "RMessage.h"

Result RMessage::draw(ConsoleStream* stream) {
  if(needsCls_) RDisplay::display.cls();

  unsigned int x = (RDisplay::DISPLAY_WIDTH-strlen(title_)*RDisplay::CHAR_WIDTH)/2;
  unsigned int y = (RDisplay::DISPLAY_HEIGHT-RDisplay::CHAR_HEIGHT)/2;
  RDisplay::display.text(x, y, RDisplay::WHITE, title_);

  return RES_OK;
}