#if !defined(LIBBB_H)
#define LIBBB_H

// LibBB - Bavarian Builders Droid Base Library for Arduino
// Written 2022-2023 by Björn Giesler <bjoern@giesler.de>

#define EPS 0.001
#define EPSILON(x) ((x>-EPS)&&(x<EPS))

#include "BBSubsystem.h"
#include "BBXBee.h"
#include "BBWifiServer.h"
#include "BBConsole.h"
#include "BBRunloop.h"
#include "BBConfigStorage.h"
#include "BBControllers.h"
#include "BBLowPassFilter.h"
#include "BBDCMotor.h"
#if defined(ARDUINO_ARCH_SAMD)
#include "BBEncoder.h"
#endif

#endif // LIBBB8_H