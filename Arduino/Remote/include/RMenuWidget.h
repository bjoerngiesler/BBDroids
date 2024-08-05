#if !defined(RMENUWIDGET_H)
#define RMENUWIDGET_H

#include <vector>
#include <functional>
#include <LibBB.h>

#include "Config.h"
#include "RDisplay.h"
#include "RWidget.h"
#include "RInput.h"

using namespace bb;

class RMenuWidget: public RWidget, public RInput::Delegate {
public:
  RMenuWidget();

  void setTitle(const char* title);
  void addEntry(const char* title, std::function<void(void)> callback);
  void clear();

  Result draw(ConsoleStream* stream = NULL);
  void buttonTopLeftPressed();
  void buttonTopRightPressed();
  void buttonConfirmPressed();

  void resetCursor() { cursor_ = 0; }
protected:
  struct Entry {
    const char* title;
    std::function<void(void)> callback;
  };

  const char* title_;
  std::vector<Entry> entries_;
  uint8_t numEntries_;
  uint8_t cursor_;
};


#endif