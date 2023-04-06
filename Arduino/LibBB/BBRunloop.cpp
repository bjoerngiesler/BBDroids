#include <Arduino.h>
#include <limits.h>
#include "BBRunloop.h"
#include "BBConsole.h"

bb::Runloop bb::Runloop::runloop;

bb::Runloop::Runloop() {
	name_ = "runloop";
	description_ = "Main runloop";
	help_ = "This gets started once after all subsystems are added. It runs through all of them, calling their step() function. Its start() never returns directly, but will return if its stop() is called.";
	cycleTime_ = BBRUNLOOP_CYCLETIME;
}

bb::Result bb::Runloop::start(ConsoleStream* stream) {
	running_ = true;
	started_ = true;
	operationStatus_ = RES_OK;
	seqnum_ = 0;

	while(running_) {
		unsigned long millis_start_loop = millis();

		seqnum_++;

		std::vector<Subsystem*> subsys = SubsystemManager::manager.subsystems();
		for(unsigned int i=0; i<subsys.size(); i++) {
			if(subsys[i]->isStarted() && subsys[i]->operationStatus() == RES_OK) {
				subsys[i]->step();
			}
		}

		unsigned long millis_end_loop = millis();
		unsigned long looptime;
		if(millis_end_loop >= millis_start_loop)
			looptime = millis_end_loop-millis_start_loop;
		else
			looptime = ULONG_MAX - millis_start_loop + millis_end_loop;
		//Console::console.printlnBroadcast(String("Elapsed: ") + looptime);
		if(looptime < cycleTime_) {
			delay(cycleTime_-looptime);
		} else if(stream) {
			stream->print((int)looptime); stream->println("ms spent in loop - something is wrong!");
		}
	}

	started_ = false;
	operationStatus_ = RES_SUBSYS_NOT_STARTED;
	return RES_OK;
}

bb::Result bb::Runloop::stop(ConsoleStream *stream) {
	stream = stream; // make compiler happy
	if(!started_) return RES_SUBSYS_NOT_STARTED;
	return RES_SUBSYS_NOT_STOPPABLE;
}

bb::Result bb::Runloop::step() {
	return RES_OK;
}

void bb::Runloop::setCycleTime(uint8_t t) {
 	cycleTime_ = t;
}

uint8_t bb::Runloop::cycleTime() {
	return cycleTime_;
}
