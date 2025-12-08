#include "UI/MessageWidget.h"

void MessageWidget::setTitle(const String& title) {
  setLinebreak();
  setAutosize();
  setFillsBackground();
  setDrawsFrame();
  Label::setTitle(title);
  centerOnMain();
}