#if !defined(RIGHTSUBSYS_H)
#define RIGHTSUBSYS_H

#include <LibBB.h>
#include <LibBBRemotes.h>
#include "Input.h"
#include "LRBase.h"

using namespace bb;
using namespace bb::rmt;

class RightSubsys: public LRBase {
public:
    static RightSubsys inst;

    virtual Result initialize();
    virtual Result start();
    virtual Result stop();
    virtual Result step();

    void protocolDestroyed(Protocol *protocol);

protected:
};

#endif // RCONTROLLERSUBSYS_H