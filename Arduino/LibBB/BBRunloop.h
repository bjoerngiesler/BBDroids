#if !defined(BBRUNLOOP_H)
#define BBRUNLOOP_H

#include "BBSubsystem.h"

namespace bb {

class Runloop: public Subsystem {
public:
	static Runloop runloop;

	static const uint64_t DEFAULT_CYCLETIME = 10000;

	virtual Result start(ConsoleStream* stream = NULL);
	virtual Result stop(ConsoleStream* stream = NULL);
	virtual Result step();

	uint64_t getSequenceNumber() { return seqnum_; }

	void setCycleTime(uint8_t microseconds); // not milli, micro.
	uint8_t cycleTime();

protected:
	Runloop();
	bool running_;
	uint64_t seqnum_;
	uint64_t cycleTime_;
};

};

#endif