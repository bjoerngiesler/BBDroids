#if !defined(REMOTESUBSYS_H)
#define REMOTESUBSYS_H

#include <LibBB.h>
#include <LibBBRemotes.h>

using namespace bb;
using namespace bb::rmt;

class RemoteSubsys: public Subsystem {
public:
    static RemoteSubsys inst;
    virtual Result initialize();
    virtual Result start();
    virtual Result stop();
    virtual Result step();

    virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
protected:
    Protocol *current, *interremote;
    static bool memoryRead(ProtocolStorage& storage);
    static bool memoryWrite(const ProtocolStorage& storage);
};

#endif // REMOTESUBSYS_H