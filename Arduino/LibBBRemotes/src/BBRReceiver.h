#if !defined(BBRRECEIVER_H)
#define BBRRECEIVER_H

#include <string>
#include <vector>
#include <functional>

#include "BBRTypes.h"
#include "BBRAxisInputManager.h"

namespace bb {
namespace rmt {

//! Abstract receiver superclass
class Receiver: public AxisInputManager {
public:
    /*
        Input handling
    */
    virtual uint8_t numInputs();

    /*! 
        \brief Returns input id if OK, INPUT_INVALID if there is already an input for the given name or id.
    */
    virtual uint8_t addInput(const std::string& name, std::function<void(float)> callback);
    virtual uint8_t addInput(const std::string& name, float& variable);
    virtual const std::string& inputName(uint8_t input);
    virtual uint8_t inputWithName(const std::string& name);

    virtual void setDataReceivedCallback(std::function<void(const NodeAddr&, uint8_t)> cb) { dataReceivedCB_ = cb; }
    virtual void setDataFinishedCallback(std::function<void(const NodeAddr&, uint8_t)> cb) { dataFinishedCB_ = cb; }
    virtual void setTimeoutCallback(std::function<void(const NodeAddr&)> cb) { timeoutCB_ = cb; }

protected:
    struct Input {
        std::string name;
        std::function<void(float)> callback;
    };

    std::vector<Input> inputs_;
    std::function<void(const NodeAddr&,uint8_t)> dataReceivedCB_ = nullptr, dataFinishedCB_ = nullptr;
    std::function<void(const NodeAddr&)> timeoutCB_ = nullptr;
};

};
};

#endif // BBRCONFIGURATOR_H