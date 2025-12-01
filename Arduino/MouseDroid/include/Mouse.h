#if !defined(MOUSE_H)
#define MOUSE_H

#include <LibBB.h>
#include <DFPlayerMini_Fast.h>
#include <LibBBRemotes.h>
#include <pwmWrite.h>

using namespace bb;
using namespace bb::rmt;

class Mouse: public Subsystem {
public:
    static Mouse mouse;

    virtual Result initialize();
    virtual Result start(ConsoleStream *stream = NULL);
    virtual Result stop(ConsoleStream *stream = NULL);
    virtual Result step();
    virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

    virtual void playSoundCB(float val, uint8_t snd);
    virtual void commTimeoutCB(Protocol* p, float s);
    virtual void dataFinishedCB(const NodeAddr& addr, uint8_t seqnum);

    Result sendTelemetry();

protected:
    DFPlayerMini_Fast dfp;
    MESPProtocol protocol_;
    Receiver *receiver_;
    bool playing_, soundOK_;

    Pwm pwm;

    float remTurn_=0, remVel_=0, remDome_=0;
    bool pressed[4];

};

#endif // MOUSE_H