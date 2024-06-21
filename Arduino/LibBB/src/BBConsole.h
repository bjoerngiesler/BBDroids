#if !defined(BBCONSOLE_H)
#define BBCONSOLE_H

#include "BBSubsystem.h"

#include <vector>
#include <cstdarg>


namespace bb {

class Subsystem;	

class ConsoleStream {
public:
	virtual bool available() = 0;
	virtual bool readStringUntil(unsigned char c, String& str) = 0;

	void printf(const char* format, ...) {
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
	}

	void vprintf(const char* format, va_list args) {
		int len = vsnprintf(NULL, 0, format, args) + 1;
		char *buf = new char[len];
		vsnprintf(buf, len, format, args);
		printfFinal(buf);
		free(buf);
	}

	virtual void printfFinal(const char* str) = 0;

	void printGreeting() {
		printfFinal("Console ready. Type \"help\" for instructions.\n> ");
	}
};

#if defined(ARDUINO_ARCH_ESP32)
#define HWSERIAL_CLASS HWCDC
#else
#define HWSERIAL_CLASS HardwareSerial
#endif

class SerialConsoleStream: public ConsoleStream {
public:
	SerialConsoleStream(HWSERIAL_CLASS& ser);

	bool checkIfOpened();
	void setCheckInterval(unsigned long microseconds);
	unsigned long checkInterval() { return checkInterval_; }

	virtual bool available();
	virtual bool readStringUntil(unsigned char c, String& str);

	virtual void printfFinal(const char* str);
protected:
	HWSERIAL_CLASS& ser_;
	bool opened_;
	String curStr_;
	unsigned long checkInterval_, lastCheck_;
};

class BroadcastStream: public ConsoleStream {
public:
	static BroadcastStream bc;

	virtual bool available() { return false; }
	virtual bool readStringUntil(unsigned char c, String& str) { return false; }
	virtual void printfFinal(const char* str);
};

class Console: public Subsystem {
public:
	static Console console;

	virtual Result initialize(int bps=115200) { 
		Serial.begin(bps); 
		serialStream_ = new SerialConsoleStream(Serial);
		addConsoleStream(serialStream_); 
		return Subsystem::initialize(); 
	}
	virtual Result start(ConsoleStream* stream = NULL);
	virtual Result stop(ConsoleStream* stream = NULL);
	virtual Result step();
	virtual void addConsoleStream(ConsoleStream* stream);
	virtual void removeConsoleStream(ConsoleStream* stream);
	ConsoleStream* serialStream() { return serialStream_; }
	ConsoleStream* broadcastStream() { return &BroadcastStream::bc; }
	const std::vector<ConsoleStream*>& streams() { return streams_; }

	void handleStreamInput(ConsoleStream* stream);
	Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream* stream);
	
	void printfBroadcast(const char* format, ...);
	void printHelpAllSubsystems(ConsoleStream* stream);
	void printStatusAllSubsystems(ConsoleStream* stream);

	void setFirstResponder(Subsystem* subsys);

protected:
	Console();
	std::vector<String> split(const String& str);
	ConsoleStream *serialStream_;
	std::vector<ConsoleStream*> streams_;
	Subsystem* firstResponder_;
};

};

#endif // BBCONSOLE_H