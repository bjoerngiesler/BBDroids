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
		if(words.size() != 1) stream->println(errorMessage(RES_CMD_INVALID_ARGUMENT_COUNT));
		printHelpAllSubsystems(stream);
	} 

	else if(words[0] == "status") {
		if(words.size() != 1) stream->println(errorMessage(RES_CMD_INVALID_ARGUMENT_COUNT));
		printStatusAllSubsystems(stream);
	} 

	else if(words[0] == "start") {
		if(words.size() != 1) stream->println(errorMessage(RES_CMD_INVALID_ARGUMENT_COUNT));
		stream->println("Starting all stopped subsystems");
		for(auto& s: SubsystemManager::manager.subsystems()) {
			if(!s->isStarted()) {
				stream->print(String("Starting ") + s->name() + "... ");
				stream->println(errorMessage(s->start(stream)));
			}
		}
	}
		
	else if(words[0] == "stop") {
		if(words.size() != 1) stream->println(errorMessage(RES_CMD_INVALID_ARGUMENT_COUNT));
		stream->println("Stopping all running subsystems");
		std::vector<Subsystem*> subsystems = SubsystemManager::manager.subsystems();
		for(auto& s: SubsystemManager::manager.subsystems()) {
			if(s->isStarted()) {
				stream->print(String("Stopping ") + s->name() + "... ");
				stream->println(errorMessage(s->stop(stream)));
			}
		}
	}

	else if(words[0] == "store") {
		stream->println(errorMessage(ConfigStorage::storage.store()));
	} 

	else {
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

void bb::Console::printHelpAllSubsystems(ConsoleStream* stream) {
	stream->println("The following commands are available on top level:");
	stream->println("    help                    Print this help text (use '<subsys> help' for help on individual subsystem)"); 
	stream->println("    status                  Print status on all subsystems (use '<subsys> status' for help on individual subsystem)");
	stream->println("    start                   Start all stopped subsystems (use '<subsys> start' to start individual subsystem)");
	stream->println("    stop                    Stop all started subsystems (use '<subsys> stop' to stop individual subsystem)");
	stream->println("    restart                 Restart (stop, then start) all started subsystems");
	stream->println("    store                   Store all parameters oto flash");
	stream->println("The following standard commands are supported by all subsystems:");
	stream->println("    <subsys> help");
	stream->println("    <subsys> status");
	stream->println("    <subsys> start");
	stream->println("    <subsys> stop");
	stream->println("    <subsys> restart");
	stream->println("Please use '<subsys> help' for additional commands supported by individual subsystems.");
}

void bb::Console::printStatusAllSubsystems(ConsoleStream* stream) {
	stream->println("System status:");
	const std::vector<Subsystem*> subsystems = SubsystemManager::manager.subsystems();
	for(auto& s: subsystems) s->printStatus(stream);
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
