#if !defined(RMESSAGEWIDGET_H)
#define RMESSAGEWIDGET_H

#include <LibBB.h>

#include "Config.h"
#include "RDisplay.h"
#include "RWidget.h"

class RMessageWidget: public RWidget {
public:
  RMessageWidget();

  void setTitle(const char* title);
  Result draw(ConsoleStream* stream = NULL);

protected:
  const char* title_;
};

#endif // RMESSAGE_H