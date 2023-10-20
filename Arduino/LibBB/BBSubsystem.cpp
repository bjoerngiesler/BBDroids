#include "BBSubsystem.h"
#include "BBConsole.h"	

#if defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#elif defined(ARDUINO_ARCH_MBED_NANO)
#include <FlashAsEEPROM.h>
#endif

bb::SubsystemManager bb::SubsystemManager::manager;
	
bb::Result bb::SubsystemManager::registerSubsystem(Subsystem* subsys) {
	if(subsystemWithName(subsys->name()) != NULL) return RES_SUBSYS_ALREADY_REGISTERED; // already have this
	subsys_.push_back(subsys);
	return RES_OK;
}
	
bb::Subsystem* bb::SubsystemManager::subsystemWithName(const String& name) {
	for(size_t i=0; i<subsys_.size(); i++) {
		if(subsys_[i]->name() == name) return subsys_[i];
	}
	return NULL;
}

const std::vector<bb::Subsystem*>& bb::SubsystemManager::subsystems() {
	return subsys_;
}


bb::SubsystemManager::SubsystemManager() {
}

bb::Result bb::Subsystem::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
	if(words[0] == "help") {
		if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
		printHelp(stream);
		return RES_OK;
	} 

	else if(words[0] == "status") {
		if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
		printStatus(stream);
		return RES_OK;
	} 

	else if(words[0] == "start") {
		if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
		if(isStarted()) stream->println(name() + "is already running.");
		else {
			stream->print(String("Starting ") + name() + "... ");
			stream->println(errorMessage(start(stream)));
		}
		return RES_OK;
	} 

	else if(words[0] == "stop") {
		if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
		if(!isStarted()) stream->println(name() + " is not running.");
		else {
			stream->print(String("Stopping ") + name() + "... ");
			stream->println(errorMessage(stop(stream)));
		}
		return RES_OK;
	} 

	else if(words[0] == "get") {
		if(words.size() != 2) return RES_CMD_INVALID_ARGUMENT_COUNT;
		for(auto& p: parameters_) {
			if(p.first == words[1]) {
				printParameter(stream, p.first, p.second);
				return RES_OK;
			}
		}
		return RES_PARAM_NO_SUCH_PARAMETER;
	}

	else if(words[0] == "set") {
		if(words.size() != 3) return RES_CMD_INVALID_ARGUMENT_COUNT;
		return setParameterValue(words[1], words[2]);
	}

	return RES_CMD_UNKNOWN_COMMAND;
}

void bb::Subsystem::printStatus(ConsoleStream* stream) {
	stream->print(name() + " (" + description() + "): ");

	if(isStarted()) {
		stream->print("started, ");
		switch(operationStatus()) {
		case RES_OK:
			stream->println("operational");
			break;
		default:
			stream->print("not operational: ");
			stream->println(errorMessage(operationStatus()));
			break;
		}
	} else stream->println("stopped");
}

void bb::Subsystem::printHelp(ConsoleStream* stream) {
	stream->println(help());
	if(parameters_.size()) {
		stream->println("Parameters:");
		printParameters(stream);
	} else {
		stream->println("No parameters.");
	}
}

void bb::Subsystem::printParameters(ConsoleStream* stream) {
	for(auto &p: parameters_) {
		printParameter(stream, p.first, p.second);
	}	
}

void bb::Subsystem::printParameter(ConsoleStream* stream, const String& name, const Parameter* p) {
	if(NULL == stream) return;

	stream->println(name + " = " + p->description());
}

bb::Result bb::Subsystem::addParameter(const String& name, const String& help, int& val, int min, int max) {
	if(parameters_.find(name) != parameters_.end()) return RES_COMMON_DUPLICATE_IN_LIST;
	parameters_[name] = new IntParameter(val, help, min, max);
	return RES_OK;
}

bb::Result bb::Subsystem::addParameter(const String& name, const String& help, float& val, float min, float max) {
	if(parameters_.find(name) != parameters_.end()) return RES_COMMON_DUPLICATE_IN_LIST;
	parameters_[name] = new FloatParameter(val, help, min, max);
	return RES_OK;
}

bb::Result bb::Subsystem::addParameter(const String& name, const String& help, String& val, int maxlen) {
	if(parameters_.find(name) != parameters_.end()) return RES_COMMON_DUPLICATE_IN_LIST;
	parameters_[name] = new StringParameter(val, help, maxlen);
	return RES_OK;
}

bb::Result bb::Subsystem::addParameter(const String& name, const String& help, bool& val) {
	if(parameters_.find(name) != parameters_.end()) return RES_COMMON_DUPLICATE_IN_LIST;
	parameters_[name] = new BoolParameter(val, help);
	return RES_OK;	
}


bb::Result bb::Subsystem::setParameterValue(const String& name, const String& stringval) {
	if(parameters_.find(name) == parameters_.end()) return RES_PARAM_NO_SUCH_PARAMETER;
	Parameter* p = parameters_[name];
	bb::Result retval = p->fromString(stringval);
	return retval;
}


