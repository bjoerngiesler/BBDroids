#if !defined(RIGHTSUBSYS_H)
#define RIGHTSUBSYS_H

#include <LibBB.h>
#include <LibBBRemotes.h>
#include "Input.h"

using namespace bb;
using namespace bb::rmt;

class RightSubsys: public Subsystem {
public:
    static RightSubsys inst;

    virtual Result initialize();
    virtual Result start();
    virtual Result stop();
    virtual Result step();

    virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

    void protocolDestroyed(Protocol *protocol);

protected:
    Transmitter* transmitter_;
    Protocol *currentProto_;
};

#endif // RCONTROLLERSUBSYS_H