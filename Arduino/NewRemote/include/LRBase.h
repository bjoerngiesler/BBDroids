#if !defined LRBASE_H
#define LRBASE_H

#include <LibBB.h>
#include <LibBBRemotes.h>
#include <string>

using namespace bb;
using namespace bb::rmt;

class LRBase: public Subsystem {
public:
    LRBase();

    virtual Result step();

    virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);
    virtual String statusLine();
    virtual void printExtendedStatus(ConsoleStream* stream);
    virtual void printRunningStatus();
protected:
    std::string help_;
    bool runningStatus_;
    Transmitter* transmitter_;
    Protocol *currentProto_;
};

#endif // LRBASE_H