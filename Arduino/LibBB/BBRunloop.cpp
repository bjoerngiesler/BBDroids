#include <Arduino.h>
#include <limits.h>
#include "BBRunloop.h"
#include "BBConsole.h"

bb::Runloop bb::Runloop::runloop;

bb::Runloop::Runloop() {
	name_ = "runloop";
	description_ = "Main runloop";
	help_ = "This gets started once after all subsystems are added. It runs through all of them, calling their step() function. Its start() never returns directly, but will return if its stop() is called.\n"\
"Commands:\n"\
"\trunning_status [on|off]: Print running status on timing";
	cycleTime_ = DEFAULT_CYCLETIME;
	runningStatus_ = false;
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

		std::vector<Subsystem*> subsys = SubsystemManager::manager.subsystems();
		for(unsigned int i=0; i<subsys.size(); i++) {
			unsigned long us = micros();
			if(subsys[i]->isStarted() && subsys[i]->operationStatus() == RES_OK) {
				subsys[i]->step();
			}

			if(runningStatus_) Console::console.printBroadcast(subsys[i]->name() + ": " + (micros()-us) + "us ");
		}

		unsigned long micros_end_loop = micros();
		unsigned long looptime;
		if(micros_end_loop >= micros_start_loop)
			looptime = micros_end_loop - micros_start_loop;
		else
			looptime = ULONG_MAX - micros_start_loop + micros_end_loop;
		if(runningStatus_) Console::console.printlnBroadcast(String("Total: ") + looptime + "us");
		if(looptime < cycleTime_) {
			delayMicroseconds(cycleTime_-looptime);
		} else {
			Console::console.printlnBroadcast(String(looptime) + "us spent in loop - something is wrong!");
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

bb::Result bb::Runloop::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
	stream->println(String("Command: ") + words[0]);
	if(words[0] == "running_status") {
		if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
		runningStatus_ = words[1] == "on" ? true : false;
		return RES_OK;
	}

	return RES_CMD_UNKNOWN_COMMAND;
}

void bb::Runloop::setCycleTime(uint8_t t) {
 	cycleTime_ = t;
}

uint8_t bb::Runloop::cycleTime() {
	return cycleTime_;
}

uint64_t bb::Runloop::millisSinceStart() {
	return millis() - startTime_;
}
