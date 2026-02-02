#if !defined(REMOTESUBSYS_H)
#define REMOTESUBSYS_H

#include <LibBB.h>
#include <LibBBRemotes.h>

using namespace bb;
using namespace bb::rmt;

class RemoteSubsys: public Subsystem {
public:
    static const std::string INTERREMOTE_PROTOCOL_NAME;
    
    static RemoteSubsys inst;

    virtual Result initialize();
    virtual Result start();
    virtual Result stop();
    virtual Result step();

    virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

    Protocol *currentProtocol() { return current_; }
    Protocol *interremoteProtocol() { return interremote_; }

    Protocol *createProtocol(ProtocolType type);

    std::string nextProtocolName();

    bool storeCurrent(const std::string& name);
    bool storeInterremote();
    bool store();

    void setCurrentChangedCB(std::function<void(Protocol*)> currentChangedCB);

    bool loadCurrent(const std::string& name);

    void protocolPairedCB(Protocol* proto, const NodeDescription& node);

protected:
    Protocol *current_, *interremote_;
    MaxlenString currentName_;
    static bool memoryRead(ProtocolStorage& storage);
    static bool memoryWrite(const ProtocolStorage& storage);

    bool initialized_ = false;
    std::function<void(Protocol*)> currentChangedCB_ = nullptr;
};

#endif // REMOTESUBSYS_H