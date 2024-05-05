#include "BBConsole.h"
#include "BBConfigStorage.h"
#include "BBRunloop.h"
#include <cstdarg>

bb::Console bb::Console::console;

bb::SerialConsoleStream::SerialConsoleStream(HardwareSerial& ser): ser_(ser), opened_(false), curStr_("") {
	lastCheck_ = micros();
	checkInterval_ = 1000000;
	if(ser_) {
		opened_ = true;
	}
}

bool bb::SerialConsoleStream::checkIfOpened() {
	if(!opened_ && ser_) {
		opened_ = true;
		printGreeting();
	} else if(opened_ && !ser_) {
		opened_ = false;
	}

	lastCheck_ = micros();

	return opened_;
}

bool bb::SerialConsoleStream::available() {
	if(opened_ == false) {
		if((micros() - lastCheck_) > checkInterval_)
			checkIfOpened();
	}

	if(!opened_) return 0;
	return ser_.available();
}

bool bb::SerialConsoleStream::readStringUntil(unsigned char c, String& str) { 
	if(!opened_) return false;

	unsigned char input = ser_.read();

	if(input == '\b') {
		if(curStr_.length() > 0) curStr_.remove(curStr_.length()-1);
		ser_.print(String("\r> ") + curStr_ + " \b");
	} else {
		curStr_ += (char)input;
		ser_.print(String("\r> ") + curStr_);
	}

	ser_.flush();
	str = curStr_;
	if(input == c) {
		curStr_ = "";
		return true;
	}

	return false;
}

void bb::SerialConsoleStream::printfFinal(const char* buf) {
	ser_.print(buf);
}

bb::BroadcastStream bb::BroadcastStream::bc;

void bb::BroadcastStream::printfFinal(const char* str) {
	const std::vector<ConsoleStream*>& streams = Console::console.streams();
	for(auto* s: streams) s->printfFinal(str);
}


bb::Console::Console() {
	name_ = "console";
	description_ = "Console interaction facility";
	help_ = "No help available";
	firstResponder_ = this;
}

bb::Result bb::Console::start(ConsoleStream *stream) {
	if(stream) stream = stream; // make compiler happy
	started_ = true;
	operationStatus_ = RES_OK;
	return RES_OK;
}

bb::Result bb::Console::stop(ConsoleStream *stream) {
	if(stream) stream = stream; // make compiler happy
	if(!started_) return RES_SUBSYS_NOT_STARTED;
	return RES_SUBSYS_NOT_STOPPABLE;
}

bb::Result bb::Console::step() {
	if(!started_) return RES_SUBSYS_NOT_STARTED;

	for(size_t i=0; i<streams_.size(); i++) {
		handleStreamInput(streams_[i]);
	}

	return RES_OK;
}

void bb::Console::addConsoleStream(ConsoleStream* stream) {
	for(size_t i=0; i<streams_.size(); i++) {
		if(streams_[i] == stream) return; // already have this
	}
	streams_.push_back(stream);
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
	if(stream->available() == 0) return;

	String str;
	if(stream->readStringUntil('\n', str) == false) return;

	stream->printf("\r");
	str.trim();
	std::vector<String> words = split(str);

	if(words.size() == 0) {
		stream->printf("> ");
		return;
	}

	Result res = firstResponder_->handleConsoleCommand(words, stream);
	stream->printf(errorMessage(res));
	stream->printf(".\n> ");
}

bb::Result bb::Console::handleConsoleCommand(const std::vector<String>& words, ConsoleStream* stream) {

	if(words[0] == "help") {
		bb::Runloop::runloop.excuseOverrun();
		if(words.size() != 1) {
			return RES_CMD_INVALID_ARGUMENT_COUNT;
		}

		printHelpAllSubsystems(stream);
		return RES_OK;
	} 

	else if(words[0] == "status") {
		bb::Runloop::runloop.excuseOverrun();
		if(words.size() != 1) {
			return RES_CMD_INVALID_ARGUMENT_COUNT;
		}

		printStatusAllSubsystems(stream);
		return RES_OK;
	} 

	else if(words[0] == "start") {
		bb::Runloop::runloop.excuseOverrun();
		if(words.size() != 1) {
			return RES_CMD_INVALID_ARGUMENT_COUNT;
		} 

		stream->printf("Starting all stopped subsystems\n");
		for(auto& s: SubsystemManager::manager.subsystems()) {
			if(!s->isStarted()) {
				stream->printf("Starting %s... ", s->name());
				stream->printf(errorMessage(s->start(stream)));
				stream->printf("\n");
			}
		}
		return RES_OK;
	}
		
	else if(words[0] == "stop") {
		bb::Runloop::runloop.excuseOverrun();
		if(words.size() != 1) {
			return RES_CMD_INVALID_ARGUMENT_COUNT;
		}

		stream->printf("Stopping all running subsystems\n");
		std::vector<Subsystem*> subsystems = SubsystemManager::manager.subsystems();
		for(auto& s: SubsystemManager::manager.subsystems()) {
			if(s->isStarted()) {
				stream->printf("Stopping %s... ", s->name());
				stream->printf(errorMessage(s->stop(stream)));
				stream->printf("\n");
			}
		}
		return RES_OK;
	}

	else if(words[0] == "store") {
		return ConfigStorage::storage.store();
	} 

	else {
		Subsystem *subsys = SubsystemManager::manager.subsystemWithName(words[0]);
		if(subsys == NULL) {
			return RES_CMD_UNKNOWN_COMMAND;
		} else {
			std::vector<String> wordsminusone = words;
			wordsminusone.erase(wordsminusone.begin());
			return subsys->handleConsoleCommand(wordsminusone, stream);
		}
	}

	return RES_CMD_UNKNOWN_COMMAND;
}

#define PRINTF_MAXLEN 254
void bb::Console::printfBroadcast(const char* format, ...) {
	char str[PRINTF_MAXLEN+1];

	va_list args;
	va_start(args, format);
	int len = vsnprintf(NULL, 0, format, args)+1;
	if(len > PRINTF_MAXLEN) len = PRINTF_MAXLEN;
	memset(str, 0, len);
	vsnprintf(str, len, format, args);
	va_end(args);
	for(auto& s: streams_) {
		s->printf(str);
	}
}

void bb::Console::printHelpAllSubsystems(ConsoleStream* stream) {
	stream->printf("The following commands are available on top level:\n");
	stream->printf("    help                    Print this help text (use '<subsys> help' for help on individual subsystem)\n"); 
	stream->printf("    status                  Print status on all subsystems (use '<subsys> status' for help on individual subsystem)\n");
	stream->printf("    start                   Start all stopped subsystems (use '<subsys> start' to start individual subsystem)\n");
	stream->printf("    stop                    Stop all started subsystems (use '<subsys> stop' to stop individual subsystem)\n");
	stream->printf("    restart                 Restart (stop, then start) all started subsystems\n");
	stream->printf("    store                   Store all parameters oto flash\n");
	stream->printf("The following standard commands are supported by all subsystems:\n");
	stream->printf("    <subsys> help\n");
	stream->printf("    <subsys> status\n");
	stream->printf("    <subsys> start\n");
	stream->printf("    <subsys> stop\n");
	stream->printf("    <subsys> restart\n");
	stream->printf("Please use '<subsys> help' for additional commands supported by individual subsystems.\n");
}

void bb::Console::printStatusAllSubsystems(ConsoleStream* stream) {
	stream->printf("System status:\n");
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

  return words;
}

void bb::Console::setFirstResponder(Subsystem* subsys) {
	firstResponder_ = subsys;
}

