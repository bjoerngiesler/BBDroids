#if !defined(RMESSAGE_H)
#define RMESSAGE_H

#include <LibBB.h>

#include "Config.h"
#include "RDisplay.h"

class RMessage: public RDrawable {
public:
  RMessage(const char* title) { title_ = title; }

  Result draw(ConsoleStream* stream = NULL);

protected:
  const char* title_;
};

#endif // RMESSAGE_H