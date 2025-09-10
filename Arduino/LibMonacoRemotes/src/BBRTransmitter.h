#if !defined(BBRTRANSMITTER_H)
#define BBRTRANSMITTER_H

#include <string>
#include <vector>

#include "BBRTypes.h"
#include "BBRProtocol.h"
#include "BBRAxisInputManager.h"

namespace bb {
namespace rmt {

class Protocol;

//! Abstract transmitter superclass
class Transmitter: public AxisInputManager {
public:
    /* 
        Axis handling

        Transmitters can have fixed or dynamic number of axes. Fixed number is used for static
        protocols like Monaco, where an a priori fixed number of axes per packet is mapped to an
        unknown number of droid inputs. Dynamic number of axes is used for protocols like BLE
        where a fixed number of inputs determines how many axes can be mapped. In these protocols,
        it is customary to use axisWithName() to implicitly create a virtual axis (although you can 
        of course create axes separately).
    */
    virtual uint8_t numAxes();
    virtual const std::string& axisName(uint8_t axis);
    virtual bool setAxisName(uint8_t axis, const std::string& name);
    virtual uint8_t axisWithName(const std::string& name, bool add = false, uint8_t bitDepth = 8);
    virtual uint8_t bitDepthForAxis(uint8_t axis);
    virtual bool canAddAxes() { return true; };
    virtual uint8_t addAxis(const std::string& name, uint8_t bitDepth);

    virtual bool receiverSideMapping() = 0;
    virtual bool syncReceiverSideMapping() = 0;

    /*
        Value setting, transmission and computation

        setAxisValue() and axisValue() perform unit conversion, and are the ones that should be used. 
        The Raw functions can be used to do any different raw value handling than the one done implicitly
        with the axes_ vector.

        computeInputValue() uses the axis-input mix table to compute the value for a given input from
        the axis values and the mix curves. It is provided in the transmitter for fixed-input protocols like
        DroidDepot. In protocols that do receiver-side mapping, the input value is computed in the receiver.

        transmit() is usually called from the Protocol's step() method, but can be called explicitly if
        no feedback / return value to a transmission is expected.
    */
    virtual bool setAxisValue(uint8_t axis, float value, Unit unit);
    virtual float axisValue(uint8_t axis, Unit unit);
    virtual bool setRawAxisValue(uint8_t axis, uint32_t value);
    virtual uint32_t rawAxisValue(uint8_t axis);

    //! Always computed as a percentage from -1 to 1 internally
    virtual float computeInputValue(uint8_t input);
    virtual float computeInputValue(const std::string& name) { return computeInputValue(inputWithName(name)); }
    
    virtual bool transmit() = 0;

protected:
    std::vector<Axis> axes_;
};

template <typename P> class TransmitterBase: public Transmitter {
public:
    TransmitterBase() = delete; // disallow default constructor

protected:
    friend P;

    TransmitterBase(P* protocol): protocol_(protocol) {}
    P* protocol_;
};

};
};

#endif // BBRTRANSMITTER_H