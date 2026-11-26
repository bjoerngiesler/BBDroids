#include <Arduino.h>
#include <limits.h>
#include <LibBB.h>
#include "BBRunloop.h"
#include "BBConsole.h"

bb::Runloop bb::Runloop::runloop;

bb::Runloop::Runloop() {
	name_ = "runloop";
	description_ = "Main runloop";
	help_ = "Started once after all subsystems are added. Its start() only returns if stop() is called.\n"\
"Commands:\n"\
"\trunning_status [on|off]:    Print running status on timing\n"\
"\tsuppress_overrun [on|off]:  Suppress overrun messages";
	cycleTime_ = DEFAULT_CYCLETIME;
	runningStatus_ = false;
	suppressOverrun_ = false;
	excuseOverrun_ = false;
}

bb::Result bb::Runloop::start(ConsoleStream* stream) {
	(void)stream;
	running_ = true;
	started_ = true;
	operationStatus_ = RES_OK;
	seqnum_ = 0;
	startTime_ = millis();

	while(running_) {
		unsigned long micros_start_loop = micros();
		seqnum_++;

		// First of all run any timed callbacks...
		uint64_t m = millis();
		for(std::vector<TimedCallback>::iterator iter = timedCallbacks_.begin(); iter != timedCallbacks_.end(); iter++) {
			if(iter->triggerMS < m) { // FIXME there is highly likely an integer wrap in here...?
				iter->cb();
				if(true == iter->oneshot) {
					iter = timedCallbacks_.erase(iter);
					if(iter == timedCallbacks_.end()) break;
				} else {
					iter->triggerMS = m + iter->deltaMS;
				}
			}
		}

		// ...then run step() on all subsystems...
		std::vector<String> timingInfo;

		std::vector<Subsystem*> subsys = SubsystemManager::manager.subsystems();
		for(auto& s: subsys) {
			unsigned long us = micros();
			if(s->isStarted() && s->operationStatus() == RES_OK) {
				s->step();
			} else {
				s->stepIfNotStarted();
			}
			String str = String(s->name())  + ": " + (micros()-us) + "us ";
			timingInfo.push_back(str);
			if(runningStatus_) Console::console.printfBroadcast(str.c_str());
		}

		// ...and run streams once...
		runStreams();

		// ...find out how long we took...
		unsigned long micros_end_loop = micros();
		unsigned long looptime;
		if(micros_end_loop >= micros_start_loop) {
			looptime = micros_end_loop - micros_start_loop;
		} else {
			looptime = ULONG_MAX - micros_start_loop + micros_end_loop;
		}
		if(runningStatus_) Console::console.printfBroadcast("Total: %dus", looptime);

		// ...not overran? Then fill everything with stream callbacks and busy loop.
		if(looptime <= cycleTime_) {
			while(looptime <= cycleTime_) {
				unsigned long usStream = micros();
				delayMicroseconds(1);
				runStreams();
				looptime += WRAPPEDDIFF(micros(), usStream, ULONG_MAX);
			}
		} else if(excuseOverrun_ == false && suppressOverrun_ == false) {
			// overran? Bicker.
			String msg;
			char buf[255];
			snprintf(buf, 255, "%ld/%ldus spent in loop: ", looptime, cycleTime_);
			msg = buf;
			for(auto& t: timingInfo) {
				msg = msg + t.c_str() + " ";
			}
			LOG(LOG_WARN, "%s\n", msg.c_str());
		}

		excuseOverrun_ = false;
	}

	started_ = false;
	operationStatus_ = RES_SUBSYS_NOT_STARTED;
	return RES_OK;
}

void bb::Runloop::runStreams() {
	size_t ran = 0;
	for(auto& c: streamCallbacks_) {
		if(c.type & STREAM_READ && c.stream->available()) { c.cb(c.stream); ran++; }
		if(c.type & STREAM_WRITE && c.stream->availableForWrite()) { c.cb(c.stream); ran++; }
	}
	//if(ran>0) bb::printf("Ran %d streams\n", ran);
}

bb::Result bb::Runloop::stop(ConsoleStream *stream) {
	stream = stream; // make compiler happy
	if(!started_) return RES_SUBSYS_NOT_STARTED;
	return RES_SUBSYS_NOT_STOPPABLE;
}

bb::Result bb::Runloop::step() {
	return RES_OK;
}

bb::Result bb::Runloop::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
	if(words.size() == 0) return RES_CMD_UNKNOWN_COMMAND;
	if(words[0] == "running_status") {
		if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
		runningStatus_ = words[1] == "on" ? true : false;
		return RES_OK;
	}
	if(words[0] == "suppress_overrun") {
		if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
		suppressOverrun_ = words[1] == "on" ? true : false;
		return RES_OK;
	}

	return bb::Subsystem::handleConsoleCommand(words, stream);;
}

void bb::Runloop::setCycleTimeMicros(unsigned long t) {
 	cycleTime_ = t;
}

unsigned long bb::Runloop::cycleTimeMicros() {
	return cycleTime_;
}

void bb::Runloop::excuseOverrun() {
	excuseOverrun_ = true;
}

uint64_t bb::Runloop::millisSinceStart() {
	return millis() - startTime_;
}

void* bb::Runloop::scheduleTimedCallback(uint64_t ms, std::function<void(void)> cb, bool oneshot) {
	TimedCallback c = {millis() + ms, ms, oneshot, cb};
	timedCallbacks_.push_back(c);
	return &timedCallbacks_.back();
}

bb::Result bb::Runloop::cancelTimedCallback(void* handle) {
	for(std::vector<TimedCallback>::iterator iter = timedCallbacks_.begin(); iter != timedCallbacks_.end(); iter++) {
		if(handle == (void*)&(*iter)) {
			iter = timedCallbacks_.erase(iter);
			return RES_OK;
		}
	}
	Console::console.printfBroadcast("Callback not found!\n");
	return RES_COMMON_NOT_IN_LIST;
}

void* bb::Runloop::addStreamCallback(Stream* s, StreamCallbackType type, std::function<void(Stream*)> cb) {
	StreamCallback c = {s, type, cb};
	streamCallbacks_.push_back(c);
	return &streamCallbacks_.back();
}

bb::Result bb::Runloop::cancelStreamCallback(void* handle) {
	for(std::vector<StreamCallback>::iterator iter = streamCallbacks_.begin(); iter != streamCallbacks_.end(); iter++) {
		if(handle == (void*)&(*iter)) {
			iter = streamCallbacks_.erase(iter);
			return RES_OK;
		}
	}
	Console::console.printfBroadcast("Callback not found!\n");
	return RES_COMMON_NOT_IN_LIST;
}
