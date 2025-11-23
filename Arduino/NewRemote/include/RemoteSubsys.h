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

    Protocol *currentProtocol() { return current_; }
    Protocol *interremoteProtocol() { return interremote_; }

    bool storeCurrent(const MaxlenString& name);
    bool storeInterremote();

    void setCurrentChangedCB(std::function<void(Protocol*)> currentChangedCB);

protected:
    Protocol *current_, *interremote_;
    MaxlenString currentName_;
    static bool memoryRead(ProtocolStorage& storage);
    static bool memoryWrite(const ProtocolStorage& storage);

    bool initialized_ = false;
    std::function<void(Protocol*)> currentChangedCB_ = nullptr;
};

#endif // REMOTESUBSYS_H