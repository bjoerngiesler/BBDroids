#if !defined(LEFTSUBSYS_H)
#define LEFTSUBSYS_H

#include <LibBB.h>
#include <LibBBRemotes.h>
#include "Input.h"
#include "LRBase.h"

using namespace bb;
using namespace bb::rmt;

class LeftSubsys: public LRBase {
public:
    static LeftSubsys inst;

    virtual Result initialize();
    virtual Result start();
    virtual Result stop();
    virtual Result step();

    void setupCurrent(Protocol *protocol);
    void dataReceivedCB(const NodeAddr& addr, uint8_t seqnum, const void* data, uint8_t len);

protected:
    Receiver* receiver_;
    bool interSetupComplete_;
};

#endif // RCONTROLLERSUBSYS_H