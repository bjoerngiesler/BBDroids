#if !defined(BBRUNLOOP_H)
#define BBRUNLOOP_H

#include "BBSubsystem.h"

#define BBRUNLOOP_CYCLETIME 40

namespace bb {

class Runloop: public Subsystem {
public:
	static Runloop runloop;

	virtual Result start(ConsoleStream* stream = NULL);
	virtual Result stop(ConsoleStream* stream = NULL);
	virtual Result step();

	uint64_t getSequenceNumber() { return seqnum_; }

	void setCycleTime(uint8_t t);
	uint8_t cycleTime();

protected:
	Runloop();
	bool running_;
	uint64_t seqnum_;
	uint8_t cycleTime_;
};

};

#endif