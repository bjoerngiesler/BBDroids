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

	virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

	uint64_t getSequenceNumber() { return seqnum_; }

	void setCycleTime(unsigned int microseconds); // not milli, micro.
	unsigned int cycleTime();

	// Make clear that the current cycle will likely overrun the time budget, and suppress message.
	// Only valid for the current cycle, flag will be cleared after using.
	void excuseOverrun();

	uint64_t millisSinceStart();

protected:
	Runloop();
	bool running_;
	uint64_t seqnum_;
	uint64_t cycleTime_;
	uint64_t startTime_;
	bool runningStatus_;
	bool excuseOverrun_;
};

};

#endif