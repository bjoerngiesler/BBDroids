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

	virtual void print(size_t val) = 0;
	virtual void print(int val) = 0;
	virtual void print(float val) = 0;
	virtual void print(const String& val) = 0;
	virtual void println(int val) = 0;
	virtual void println(float val) = 0;
	virtual void println(const String& val) = 0;
	virtual void println() = 0;
};

class SerialConsoleStream: public ConsoleStream {
public:
	SerialConsoleStream(HardwareSerial& ser);

	bool checkIfOpened();
	void setCheckInterval(unsigned long microseconds);
	unsigned long checkInterval() { return checkInterval_; }

	virtual bool available();
	virtual bool readStringUntil(unsigned char c, String& str);

	virtual void printfFinal(const char* str);

	virtual void print(size_t val);
	virtual void print(int val);
	virtual void print(float val);
	virtual void print(const String& val);
	virtual void println(int val);
	virtual void println(float val);
	virtual void println(const String& val);
	virtual void println();
protected:
	HardwareSerial& ser_;
	bool opened_;
	String curStr_;
	unsigned long checkInterval_, lastCheck_;
};

class Console: public Subsystem {
public:
	static Console console;

	virtual Result initialize(int bps=115200) { Serial.begin(bps); addConsoleStream(new SerialConsoleStream(Serial)); return Subsystem::initialize(); }
	virtual Result start(ConsoleStream* stream = NULL);
	virtual Result stop(ConsoleStream* stream = NULL);
	virtual Result step();
	virtual void addConsoleStream(ConsoleStream* stream);
	virtual void removeConsoleStream(ConsoleStream* stream);

	void handleStreamInput(ConsoleStream* stream);
	Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream* stream);
	
	void printBroadcast(const String& val = "");
	void printlnBroadcast(const String& val = "");
	void printfBroadcast(const char* format, ...);
	void printHelpAllSubsystems(ConsoleStream* stream);
	void printStatusAllSubsystems(ConsoleStream* stream);

	void setFirstResponder(Subsystem* subsys);

protected:
	Console();
	std::vector<String> split(const String& str);
	std::vector<ConsoleStream*> streams_;
	Subsystem* firstResponder_;
};

};

#endif // BBCONSOLE_H