#include "BBConsole.h"
#include "BBConfigStorage.h"

bb::Console bb::Console::console;

bb::SerialConsoleStream::SerialConsoleStream(HardwareSerial& ser): ser_(ser), opened_(false), curStr_("") {
}

bool bb::SerialConsoleStream::available() { 
	printGreetingIfOpened();
	return ser_.available(); 
}

bool bb::SerialConsoleStream::readStringUntil(unsigned char c, String& str) { 
	printGreetingIfOpened();

	unsigned char input = ser_.read();

	if(input == '\b') {
		if(curStr_.length() > 0) curStr_.remove(curStr_.length()-1);
		ser_.print("\r"); ser_.print("> "); ser_.print(curStr_); ser_.print(" \b");
	} else {
		curStr_ += (char)input;
		ser_.print("\r"); ser_.print("> "); ser_.print(curStr_);
	}

	ser_.flush();
	str = curStr_;
	if(input == c) {
		curStr_ = "";
		return true;
	}

	return false;
}

void bb::SerialConsoleStream::print(size_t val) { 
	printGreetingIfOpened();
	ser_.print(val); 
}

void bb::SerialConsoleStream::print(int val) {
	printGreetingIfOpened();
	ser_.print(val); 
}

void bb::SerialConsoleStream::print(float val)  { 
	printGreetingIfOpened();
	ser_.print(val); 
}

void bb::SerialConsoleStream::print(const String& val)  { 
	printGreetingIfOpened();
	if(val.length() >  0)
		ser_.print(val); 
}

void bb::SerialConsoleStream::println(int val)  { 
	printGreetingIfOpened();
	ser_.println(val); 
	ser_.print("\n");
}

void bb::SerialConsoleStream::println(float val) { 
	printGreetingIfOpened();
	ser_.println(val); 
	ser_.print("\n");
}

void bb::SerialConsoleStream::println(const String& val) { 
	printGreetingIfOpened();
	ser_.println(val); 
	ser_.print("\n");
}

void bb::SerialConsoleStream::println() { 
	printGreetingIfOpened();
	ser_.println(); 
	ser_.print("\n");
}

void bb::SerialConsoleStream::printGreetingIfOpened() {
	if(!opened_ && ser_) {
		opened_ = true;
		ser_.println("Console ready. Type \"help\" for instructions.");
		ser_.print("> ");
		ser_.flush();
	}
}

bb::Console::Console() {
	name_ = "console";
	description_ = "Console interaction facility";
	help_ = "No help available";
}

bb::Result bb::Console::start(ConsoleStream *stream) {
	if(stream) stream = stream; // make compiler happy
	addConsoleStream(new SerialConsoleStream(Serial));
	started_ = true;
	operationStatus_ = RES_OK;
	return RES_OK;
}

bb::Result bb::Console::stop(ConsoleStream *stream) {
	if(stream) stream = stream; // make compiler happy
	if(!started_) return RES_SUBSYS_NOT_STARTED;
	return RES_SUBSYS_NOT_STOPPABLE;
	return RES_OK;
}

bb::Result bb::Console::step() {
	if(!started_) return RES_SUBSYS_NOT_STARTED;

	for(size_t i=0; i<streams_.size(); i++) handleStreamInput(streams_[i]);

	return RES_OK;
}

void bb::Console::addConsoleStream(ConsoleStream* stream) {
	for(size_t i=0; i<streams_.size(); i++) {
		if(streams_[i] == stream) return; // already have this
	}
	streams_.push_back(stream);
	stream->println("Console ready. Type \"help\" for instructions. Please set your terminal to send LF or CR+LF as line ending.\n> ");
}

void bb::Console::removeConsoleStream(ConsoleStream* stream) {
	for(size_t i=0; i<streams_.size(); i++) {
		if(streams_[i] == stream) {
			streams_.erase(streams_.begin()+i);
			return;
		} 
	}
}

void bb::Console::handleStreamInput(ConsoleStream* stream) {
	if(!stream->available()) return;

	String str;
	if(stream->readStringUntil('\n', str) == false) return;

	stream->print("\r");
	str.trim();
	std::vector<String> words = split(str);

	if(words.size() == 0) {
		stream->print("> ");
		return;
	}

	if(words[0] == "help") {
		if(words.size() == 1) printHelp(stream);
		else {
			for(size_t i=1; i<words.size(); i++) {
				Subsystem *subsys = SubsystemManager::manager.subsystemWithName(words[i]);
				if(subsys == NULL) {
					stream->print("No subsystem named \""); stream->print(words[i]); stream->println("\".");
				} else {
					printHelp(stream, subsys);
				}
			}
		}
	} else if(words[0] == "status") {
		if(words.size() == 1) printStatus(stream);
		else {
			for(size_t i=1; i<words.size(); i++) {
				Subsystem *subsys = SubsystemManager::manager.subsystemWithName(words[i]);
				if(subsys == NULL) {
					stream->print("No subsystem named \""); stream->print(words[i]); stream->println("\".");
				} else {
					printStatus(stream, subsys);
				}
			}
		}
	} else if(words[0] == "start") {
		if(words.size() == 1) {
			Serial.println("Starting all stopped subsystems");
			std::vector<Subsystem*> subsystems = SubsystemManager::manager.subsystems();
			for(size_t i=0; i<subsystems.size(); i++) {
				if(!subsystems[i]->isStarted()) {
					stream->print("Starting ");
					stream->print(subsystems[i]->name());
					stream->print("... ");
					stream->println(errorMessage(subsystems[i]->start(stream)));
				}
			}
		} else {
			for(size_t i=1; i<words.size(); i++) {
				Subsystem *subsys = SubsystemManager::manager.subsystemWithName(words[i]);
				if(subsys == NULL) {
					stream->print("No subsystem named \""); stream->print(words[i]); stream->println("\".");
				} else {
					stream->print("Starting ");
					stream->print(subsys->name());
					stream->print("... ");
					stream->println(errorMessage(subsys->start(stream)));
				}
			}
		}
	} else if(words[0] == "stop") {
		if(words.size() == 1) {
			stream->println("Stopping all running subsystems");
			std::vector<Subsystem*> subsystems = SubsystemManager::manager.subsystems();
			for(size_t i=0; i<subsystems.size(); i++) {
				if(subsystems[i]->isStarted()) {
					stream->print("Stopping ");
					stream->print(subsystems[i]->name());
					stream->print("... ");
					stream->println(errorMessage(subsystems[i]->stop(stream)));
				}
			}
		} else {
			for(size_t i=1; i<words.size(); i++) {
				Subsystem *subsys = SubsystemManager::manager.subsystemWithName(words[i]);
				if(subsys == NULL) {
					stream->print("No subsystem named \""); stream->print(words[i]); stream->println("\".");
				} else {
					stream->print("Stopping ");
					stream->print(subsys->name());
					stream->print("... ");
					stream->println(errorMessage(subsys->stop(stream)));
				}
			}
		}
	} else if(words[0] == "get") {
		if(words.size() == 1) {
			stream->println("Missing subsys argument to \"get\".");
		} else if(words.size() == 2) {
			Subsystem *subsys = SubsystemManager::manager.subsystemWithName(words[1]);
			if(subsys == NULL) {
				stream->print("No subsystem named \""); stream->print(words[1]); stream->println("\".");
			} else {
				printParameters(stream, subsys);
			}
		} else if(words.size() == 3) {
			Subsystem *subsys = SubsystemManager::manager.subsystemWithName(words[1]);
			if(subsys == NULL) {
				stream->print("No subsystem named \""); stream->print(words[1]); stream->println("\".");
			} else {
				const std::vector<Subsystem::ParameterDescription>& params = subsys->parameters();
				for(size_t i=0; i<params.size(); i++) {
					if(params[i].name == words[2]) {
						printParameter(stream, subsys, params[i]);
						return;
					}
				}
				stream->print("No parameter named \""); 
				stream->print(words[2]); 
				stream->print("\" in subsystem "); 
				stream->print(words[1]);
				stream->println(".");
			}
		} else {
			stream->println("Too many arguments to \"get\".");
		}
	} else if(words[0] == "set") {
		if(words.size() != 4) {
			stream->println("Wrong number of arguments to \"set\".");
		} else {
			Subsystem *subsys = SubsystemManager::manager.subsystemWithName(words[1]);			
			if(subsys == NULL) {
				stream->print("No subsystem named \""); stream->print(words[1]); stream->println("\".");
				return;
			} 

			stream->println(errorMessage(subsys->setParameterValue(words[2], words[3])));
		}
	} else if(words[0] == "store") {
		stream->println(errorMessage(ConfigStorage::storage.store()));
	} else /*if(words[0].length() > 0) */{
		Subsystem *subsys = SubsystemManager::manager.subsystemWithName(words[0]);
		if(subsys == NULL) {
			stream->print("Unknown command \""); stream->print(words[0]); stream->println("\" (and no subsystem with that name either).");
		} else {
			words.erase(words.begin());;
			stream->println(errorMessage(subsys->handleConsoleCommand(words, stream)));
		}
	}

	stream->print("> ");
}

void bb::Console::printBroadcast(const String& val) {
	for(size_t i = 0; i<streams_.size(); i++) streams_[i]->print(val);
}

void bb::Console::printlnBroadcast(const String& val) {
	for(size_t i = 0; i<streams_.size(); i++) streams_[i]->println(val);
}

void bb::Console::printHelp(ConsoleStream* stream) {
	stream->println("The following commands are available:");
	stream->println("help [<subsys1> ...]                   Print this help text, or help on individual subsystem(s)");
	stream->println("status [<subsys1> ...]                 Print status on all or individual subsystem(s)");
	stream->println("start [<subsys1> ...]                  Start all stopped subsystems, or start individual named subsystem(s)");
	stream->println("stop [<subsys1> ...]                   Stop all started subsystems, or stop individual named subsystem(s)");
	stream->println("restart [<subsys1> ...]                Restart (stop, then start) all started subsystems or individual named subsystem(s)");
	stream->println("get <subsys>                           Get all parameters on subsystem");
	stream->println("get <subsys> [<parameter1> ...]        Get parameter on subsystem (use \"help subsys\" for what's available)");
	stream->println("set <subsys> <parameter> <value>       Set parameter on subsystem (use \"help subsys\" for what's available)");
	stream->println("store [<subsys1> ...]                  Store all parameters for all or individual named subsystems to flash");
	stream->println("store <subsys> [<parameter1> ...]      Store individual parameter(s) for subsystem to flash");
	stream->println("<subsys> <command> [<parameter1> ...]  Execute subsystem-specific command (use \"help subsys\" for what's available)");
}

void bb::Console::printHelp(ConsoleStream* stream, Subsystem* subsys) {
	stream->print("Help on subsystem "); stream->print(subsys->name()); stream->println(":");
	stream->println(subsys->help());
	if(subsys->parameters().size()) {
		stream->println("Parameters:");
		printParameters(stream, subsys);
	} else {
		stream->println("No parameters.");
	}
}

void bb::Console::printParameters(ConsoleStream* stream, Subsystem* subsys) {
	const std::vector<Subsystem::ParameterDescription>& params = subsys->parameters();
	if(!params.size()) return;
	for(size_t i=0; i<params.size(); i++) {
		printParameter(stream, subsys, params[i]);
	}	
}

void bb::Console::printParameter(ConsoleStream* stream, Subsystem* subsys, const bb::Subsystem::ParameterDescription& parameter) {
	stream->print(parameter.name);
	stream->print(" = ");

	String val;
	if(subsys->parameterValue(parameter.name, val) == RES_OK) {
		if(parameter.type == PARAMETER_STRING) {
			stream->print("\"");
			stream->print(val);
			stream->print("\"");
		} else {
			stream->print(val);
		}
	} else { 
		stream->print("ERR");
	}

	switch(parameter.type) {
	case PARAMETER_UINT:   stream->print(" (uint)"); break;
	case PARAMETER_INT:    stream->print(" (int)"); break;
	case PARAMETER_FLOAT:  stream->print(" (float)"); break;
	case PARAMETER_STRING: stream->print(" (string)"); break;
	default: stream->print(" (unknown)"); break;
	}

	stream->print(" -- ");
	stream->println(parameter.help);
}


void bb::Console::printStatus(ConsoleStream* stream) {
	stream->println("System status:");
	const std::vector<Subsystem*> subsystems = SubsystemManager::manager.subsystems();
	for(size_t i=0; i<subsystems.size(); i++) printStatus(stream, subsystems[i]);
}

void bb::Console::printStatus(ConsoleStream* stream, Subsystem* subsys) {
	stream->print(subsys->name());
	stream->print(" (");
	stream->print(subsys->description());
	stream->print("): ");

	if(subsys->isStarted()) stream->print("started");
	else stream->print("stopped");

	switch(subsys->operationStatus()) {
	case RES_OK:
		stream->print(", operational");
		break;
	case RES_SUBSYS_NOT_STARTED:
		if(subsys->isStarted()) { // this contradicts
			stream->print(", not operational: ");
			stream->print(errorMessage(subsys->operationStatus()));
		}
		break;
	default:
		stream->print(", not operational: ");
		stream->print(errorMessage(subsys->operationStatus()));
		break;
	}
	stream->println();
}

std::vector<String> bb::Console::split(const String& str) {
  std::vector<String> words;
  unsigned int start = 0, end = 0;
  bool quotes = false;

  while(end < str.length()) {
    if(quotes == true) {
      if(str[end] != '"') {
        end++;
        if(end == str.length()) {
          String substr = str.substring(start, end);
          if(substr.length()) words.push_back(substr);
        }
      } else if(end >= start) {
        quotes = false;
        String substr = str.substring(start, end);
        if(substr.length()) words.push_back(substr);
        start = end+1;
        end = end+1;
      }
    } else {
      if(str[end] == '"') {
        quotes = true;
        start++;
        end++;
        continue;
      }
  
      if(str[end] != ' ') {
        end++;
        if(end == str.length()) {
          String substr = str.substring(start, end);
          if(substr.length()) words.push_back(substr);
        }
      } else if(end >= start) {
        String substr = str.substring(start, end);
        if(substr.length()) words.push_back(substr);
        start = end+1;
        end = end+1;
      } 
    }
  }

#ifdef SERIALCOMMANDS_DEBUG
  Serial.print("Split: ");
  for(int i=0; i<words.size(); i++) {
    Serial.print("'"); Serial.print(words[i]); Serial.print("' ");
  }
  Serial.println();
#endif

  return words;
}
