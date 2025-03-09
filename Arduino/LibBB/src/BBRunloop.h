#if !defined(BBRUNLOOP_H)
#define BBRUNLOOP_H

#include "BBSubsystem.h"

#include <vector>
#include <functional>

namespace bb {

class Runloop: public Subsystem {
public:
	static Runloop runloop;

	static const uint64_t DEFAULT_CYCLETIME = 10000;

	virtual Result start(ConsoleStream* stream = NULL);
	virtual Result stop(ConsoleStream* stream = NULL);
	virtual Result step();

	virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

	unsigned long getSequenceNumber() { return seqnum_; }

	void setCycleTimeMicros(unsigned long microseconds); // not milli, micro.
	unsigned long cycleTimeMicros();
	float cycleTimeSeconds() { return ((float)cycleTimeMicros())/1e6; }

	// Make clear that the current cycle will likely overrun the time budget, and suppress message.
	// Only valid for the current cycle, flag will be cleared after using.
	void excuseOverrun();

	uint64_t millisSinceStart();


	// Schedule a timed callback (oneshot or recurring). Please note that this is currently only working within
	// the runloop granularity. E.g. if you're running a runloop with a cycle time of 10,000us or 10ms, timed
	// callback execution will have a granularity of 10ms. Do not use for high precision events obviously... :-)
	virtual void* scheduleTimedCallback(uint64_t milliseconds, std::function<void(void)> cb, bool oneshot = true);
	virtual Result cancelTimedCallback(void* handle);


protected:

	struct TimedCallback {
		uint64_t triggerMS, deltaMS;
		bool oneshot;
		std::function<void()> cb;
	};

	std::vector<TimedCallback> timedCallbacks_;


	Runloop();
	bool running_;
	unsigned long seqnum_;
	unsigned long cycleTime_;
	unsigned long startTime_;
	bool runningStatus_, suppressOverrun_;
	bool excuseOverrun_;
};

};

#endif