#include "RMessageWidget.h"

RMessageWidget::RMessageWidget() {
  title_ = "(null)";
}

void RMessageWidget::setTitle(const char* title) {
  if(title == NULL) title_ = "(null)";
  else title_ = title;
}

Result RMessageWidget::draw(ConsoleStream* stream) {
  unsigned int x = (RDisplay::DISPLAY_WIDTH-strlen(title_)*RDisplay::CHAR_WIDTH)/2;
  unsigned int y = (RDisplay::DISPLAY_HEIGHT-RDisplay::CHAR_HEIGHT)/2;
  RDisplay::display.text(x, y, RDisplay::WHITE, title_);

  return RWidget::draw(stream);
}