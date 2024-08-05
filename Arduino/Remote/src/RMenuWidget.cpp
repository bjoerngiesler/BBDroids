#include "RMenuWidget.h"

RMenuWidget::RMenuWidget() {
  title_ = "(null)";
  cursor_ = 0;
  needsCls_ = true;
}

void RMenuWidget::setTitle(const char *title) {
  if(title == NULL)
    title_ = "(null)";
  else
    title_ = title;
}

void RMenuWidget::addEntry(const char* title, std::function<void(void)> callback) {
  Entry e = {title, callback};
  if(title == NULL) e = {"(null)", callback};
  entries_.push_back(e);
  needsCls_ = true;
}

void RMenuWidget::clear() {
  entries_ = std::vector<Entry>();
  cursor_ = 0;
}

void RMenuWidget::buttonTopLeftPressed() {
  if(cursor_ == 0) return;
  cursor_--;
  draw();
}

void RMenuWidget::buttonTopRightPressed() {
  if(cursor_ >= entries_.size()-1) return;
  cursor_++;
  draw();
}
  
void RMenuWidget::buttonConfirmPressed() {
  entries_[cursor_].callback();
}
  
Result RMenuWidget::draw(ConsoleStream *stream) {
  if(needsCls_) {
    RDisplay::display.cls();
    needsCls_ = false;
  }

  RDisplay::display.text(0, 0, RDisplay::WHITE, title_);

  RDisplay::display.hline(0, 12, 80, RDisplay::WHITE);
  for(uint8_t i=0; i<entries_.size(); i++) {
    if(cursor_ == i) {
      RDisplay::display.text(0, 10*(i+2), RDisplay::YELLOW, entries_[i].title);
    } else {
      RDisplay::display.text(0, 10*(i+2), RDisplay::WHITE, entries_[i].title);
    }
  }
  RDisplay::display.hline(0, 140, 80, RDisplay::WHITE);
  return RWidget::draw(stream);
}
