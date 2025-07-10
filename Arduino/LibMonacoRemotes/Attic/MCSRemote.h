#if 0
#if !defined(BBREMOTE_H)
#define BBREMOTE_H

namespace bb {
namespace mcs {
/*
    \brief Virtual superclass for remote senders

    A remote sender has a fixed number of axes (0-254) and a number of triggers (0-254). Axes and triggers can and should be named uniquely, which allows you to look them up by name, but that is mostly for debug and documentation
    reasons. All of that stuff is defined at compile time and can be queried at runtime but not set (e.g. you can't
    change the number of axes on the fly).

    The runtime API is defined by setAxisValue(), setTriggerValue(), and send(). At every cycle, a Remote should set all axis values and all trigger values, and when all is done, call send().

    This API does not handle sender/receiver pairing; refer to RemotePair for that.

    Subclasses should implement all the pure virtual methods, and (in the constructor) fill the axes_ and triggers_
    vectors.
*/
class Sender {
public:
    //! Return true if this sender is a part of a dual sender system (e.g. Monaco Controls).
    virtual bool isPartOfDualSystem() = 0; 
    //! Return true if this sender is the primary sender of a dual sender system, or not part of a dual sender system.
    virtual bool isPrimarySender() = 0;
    //! Return true if this sender is the left sender of a dual sender system, false otherwise.
    virtual bool isLeftSender() = 0;

    //! Return the number of sender axes this sender supports.
    virtual uint8_t numSenderAxes();
    //! Return the axis name for the given ID, or nullptr if not found.
    virtual const char* senderAxisName(uint8_t id);
    //! Return the ID for the given axis name, or 255 if not found.
    virtual uint8_t findSenderAxisByName(const char* name);
    //! Set the value for the given ID. Returns true if successful, false if not found.
    virtual bool setSenderAxisValue(uint8_t id, float value, Unit unit);
    //! Set the value for the given name. Returns true if successful, false if not found.
    virtual bool setSenderAxisValue(const char* name, float value, Unit unit);

    //! Return the number of sender triggers this sender supports.
    virtual uint8_t numSenderTriggers();
    //! Return the name for the given trigger, or nullptr if not found.
    virtual const char* senderTriggerName(uint8_t id);
    //! Return the ID for the given trigger name, or 255 if not found.
    virtual uint8_t findSenderTriggerByName(const char* name);
    //! Set the value for the given ID. Returns true if successful, false if not found.
    virtual bool setSenderTriggerValue(uint8_t id, bool value);
    //! Set the value for the given ID. Returns true if successful, false if not found.
    virtual bool setSenderTriggerValue(const char* name, bool value);

    //! Send out a packet with the given values (if applicable) to the given UUID.
    virtual Result sendTo(UUID uuid, unsigned long seq) = 0;

    protected:
    struct Axis {
        const char* name;
        float value;
        Unit unit;
    };
    struct Trigger {
        const char* name;
        bool value;
    };
    std::vector<Axis> axes_;
    std::vector<Trigger> triggers_;
};
};
};

#endif // BBREMOTE_H
#endif