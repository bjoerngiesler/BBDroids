#if !defined(MOUSE_H)
#define MOUSE_H

#include <LibBB.h>
#include <DFPlayerMini_Fast.h>

using namespace bb;

class Mouse: public Subsystem {
public:
    static Mouse mouse;
    virtual Result initialize();
    virtual Result start(ConsoleStream *stream = NULL);
    virtual Result stop(ConsoleStream *stream = NULL);
    virtual Result step();
    virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

protected:
    DFPlayerMini_Fast dfp;
    bool playing_, soundOK_;
};

#endif // MOUSE_H