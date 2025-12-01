#if !defined(MONACOFORWARD_H)
#define MONACOFORWARD_H

#include <LibBB.h>
#include <LibBBRemotes.h>

using namespace bb;
using namespace bb::rmt;

class MonacoForward: public Subsystem {
public:
    static MonacoForward monaco;

    virtual Result initialize();
    virtual Result start(ConsoleStream *stream = NULL);
    virtual Result stop(ConsoleStream *stream = NULL);
    virtual Result step();
    virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

    virtual void packetReceivedCB(const NodeAddr& addr, const MPacket& packet);

protected:
    MESPProtocol protocol_;
    Receiver *receiver_;
    unsigned long lastPacketMS_;
};

#endif // MOUSE_H