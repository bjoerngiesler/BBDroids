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
			if(p->name() == words[1]) {
				p->print(stream);
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
		p->print(stream);
	}	
}


bb::Subsystem::Parameter* bb::Subsystem::findParameter(const String& name) {
	for(auto p: parameters_) {
		if(p->name() == name) return p;
	}
	return NULL;
}


bb::Result bb::Subsystem::addParameter(const String& name, const String& help, int& val, int min, int max) {
	if(findParameter(name) != NULL) return RES_COMMON_DUPLICATE_IN_LIST;
	parameters_.push_back(new IntParameter(name, val, help, min, max));
	return RES_OK;
}

bb::Result bb::Subsystem::addParameter(const String& name, const String& help, float& val, float min, float max) {
	if(findParameter(name) != NULL) return RES_COMMON_DUPLICATE_IN_LIST;
	parameters_.push_back(new FloatParameter(name, val, help, min, max));
	return RES_OK;
}

bb::Result bb::Subsystem::addParameter(const String& name, const String& help, String& val, int maxlen) {
	if(findParameter(name) != NULL) return RES_COMMON_DUPLICATE_IN_LIST;
	parameters_.push_back(new StringParameter(name, val, help, maxlen));
	return RES_OK;
}

bb::Result bb::Subsystem::addParameter(const String& name, const String& help, bool& val) {
	if(findParameter(name) != NULL) return RES_COMMON_DUPLICATE_IN_LIST;
	parameters_.push_back(new BoolParameter(name, val, help));
	return RES_OK;	
}


bb::Result bb::Subsystem::setParameterValue(const String& name, const String& stringval) {
	Parameter* p = findParameter(name);
	if(p == NULL) return RES_PARAM_NO_SUCH_PARAMETER;
	bb::Result retval = p->fromString(stringval);
	return retval;
}

const void bb::Subsystem::Parameter::print(ConsoleStream* stream) {
	if(stream) stream->print(name_ + ": " + description());
}

