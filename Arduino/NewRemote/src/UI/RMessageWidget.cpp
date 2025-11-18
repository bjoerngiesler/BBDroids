#include "UI/RMessageWidget.h"

void RMessageWidget::setTitle(const String& title) {
  setLinebreak();
  setAutosize();
  setFillsBackground();
  setDrawsFrame();
  RLabelWidget::setTitle(title);
  centerOnMain();
}