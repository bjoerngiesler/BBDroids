#if !defined(LEFTSUBSYS_H)
#define LEFTSUBSYS_H

#include <LibBB.h>
#include <LibBBRemotes.h>
#include "Input.h"

using namespace bb;
using namespace bb::rmt;

class LeftSubsys: public Subsystem {
public:
    static LeftSubsys inst;

    virtual Result initialize();
    virtual Result start();
    virtual Result stop();
    virtual Result step();

    virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

    void setupCurrent(Protocol *protocol);
    void dataReceivedCB(const NodeAddr& addr, uint8_t seqnum, const void* data, uint8_t len);

protected:
    Receiver* receiver_;
    Transmitter* transmitter_;
    Protocol *currentProto_;
    bool interSetupComplete_;
};

#endif // RCONTROLLERSUBSYS_H