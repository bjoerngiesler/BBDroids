#include "BBSubsystem.h"
#include "BBConsole.h"	

#if defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#elif defined(ARDUINO_ARCH_MBED_NANO)
#include <FlashAsEEPROM.h>
#endif

bb::SubsystemManager bb::SubsystemManager::manager;
	
bb::SubsystemManager::SubsystemManager() {
}

bb::Result bb::SubsystemManager::registerSubsystem(Subsystem* subsys) {
	if(subsystemWithName(subsys->name()) != NULL) return RES_SUBSYS_ALREADY_REGISTERED; // already have this
	subsys_.push_back(subsys);
	return RES_OK;
}
	
bb::Subsystem* bb::SubsystemManager::subsystemWithName(const String& name) {
	for(size_t i=0; i<subsys_.size(); i++) {
		if(name == subsys_[i]->name()) return subsys_[i];
	}
	return NULL;
}

const std::vector<bb::Subsystem*>& bb::SubsystemManager::subsystems() {
	return subsys_;
}

bb::Result bb::Subsystem::handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream) {
	if(words[0] == "help") {
		if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
		printHelp(stream);
		return RES_OK;
	} 

	else if(words[0] == "status") {
		if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
		printExtendedStatus(stream);
		return RES_OK;
	} 

	else if(words[0] == "start") {
		if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
		if(isStarted()) stream->printf("%s is already running.", name());
		else {
			stream->printf("Starting %s...", name());
			stream->printf(errorMessage(start(stream)));
			stream->printf("\n");
		}
		return RES_OK;
	} 

	else if(words[0] == "stop") {
		if(words.size() != 1) return RES_CMD_INVALID_ARGUMENT_COUNT;
		if(!isStarted()) stream->printf("%s is not running.", name());
		else {
			stream->printf("Stopping %s...", name());
			stream->printf(errorMessage(stop(stream)));
			stream->printf("\n");
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

String bb::Subsystem::statusLine() {
	String line = String(name()) + " (" + description() + "): ";
	if(isStarted()) {
		line += "started, ";
		switch(operationStatus()) {
		case RES_OK:
			line += "operational";
			break;
		default:
			line += "not operational: ";
			line += errorMessage(operationStatus());
			break;
		}
	} else line += "stopped";
	
	return line;
}

void bb::Subsystem::printStatusLine(ConsoleStream* stream) {
	if(stream) 
		stream->printf("%s\n", statusLine().c_str());
	else
		Console::console.printfBroadcast("%s\n", statusLine().c_str());
}

void bb::Subsystem::printExtendedStatus(ConsoleStream* stream) {
	printStatusLine(stream);
}

void bb::Subsystem::printHelp(ConsoleStream* stream) {
	stream->printf(help());
	if(parameters_.size()) {
		stream->printf("Parameters:\n");
		printParameters(stream);
	} else {
		stream->printf("No parameters.\n");
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
	if(stream) stream->printf("%s: %s\n", name_.c_str(), description().c_str());
}

