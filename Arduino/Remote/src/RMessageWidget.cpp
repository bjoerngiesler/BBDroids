#include "RMessageWidget.h"

void RMessageWidget::setTitle(const String& title) {
  setLinebreak();
  setAutosize();
  setFillsBackground();
  setDrawsFrame();
  RLabelWidget::setTitle(title);
  centerOnMain();
  Console::console.printfBroadcast("%d lines\n", lines_.size());
  for(unsigned int i=0; i<lines_.size(); i++) {
    Console::console.printfBroadcast("%d: '%s'\n", i, lines_[i].c_str());
  }
}