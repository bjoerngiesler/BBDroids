#include "RMessageWidget.h"

void RMessageWidget::setTitle(const String& title) {
  title_ = title;
  setSize(title_.length()*RDisplay::CHAR_WIDTH+4, RDisplay::CHAR_HEIGHT+4);
  setPosition(RDisplay::MAIN_X + (RDisplay::MAIN_WIDTH-width())/2, 
              RDisplay::MAIN_Y + (RDisplay::MAIN_HEIGHT-height())/2);
  setFillsBackground();
  setDrawsFrame(true);
  setNeedsFullRedraw();
}