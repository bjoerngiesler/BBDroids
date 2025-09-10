#if !defined(BBRCONFIGURATOR_H)
#define BBRCONFIGURATOR_H

namespace bb {
namespace rmt {

//! Abstract configurator superclass
class Configurator {
public:
    virtual bool handle() = 0;
};

};
};

#endif // BBRCONFIGURATOR_H