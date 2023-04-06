#if !defined(BBCONSOLE_H)
#define BBCONSOLE_H

#include "BBSubsystem.h"

#include <vector>

namespace bb {

class Subsystem;	

class ConsoleStream {
public:
	virtual bool available() = 0;
	virtual bool readStringUntil(unsigned char c, String& str) = 0;
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
	virtual bool available();
	virtual bool readStringUntil(unsigned char c, String& str);
	virtual void print(size_t val);
	virtual void print(int val);
	virtual void print(float val);
	virtual void print(const String& val);
	virtual void println(int val);
	virtual void println(float val);
	virtual void println(const String& val);
	virtual void println();
protected:
	void printGreetingIfOpened();
	HardwareSerial& ser_;
	bool opened_;
	String curStr_;
};

class Console: public Subsystem {
public:
	static Console console;

	virtual Result initialize(int bps=115200) { Serial.begin(bps); return Subsystem::initialize(); }
	virtual Result start(ConsoleStream* stream = NULL);
	virtual Result stop(ConsoleStream* stream = NULL);
	virtual Result step();
	virtual void addConsoleStream(ConsoleStream* stream);
	virtual void removeConsoleStream(ConsoleStream* stream);

	void handleStreamInput(ConsoleStream* stream);
	
	void printBroadcast(const String& val = "");
	void printlnBroadcast(const String& val = "");
	void printHelp(ConsoleStream* stream);
	void printHelp(ConsoleStream* stream, Subsystem *subsys);
	void printStatus(ConsoleStream* stream);
	void printStatus(ConsoleStream* stream, Subsystem* subsys);
	void printParameters(ConsoleStream* stream, Subsystem* subsys);
	void printParameter(ConsoleStream* stream, Subsystem* subsys, const bb::Subsystem::ParameterDescription& parameter);

protected:
	Console();
	std::vector<String> split(const String& str);
	std::vector<ConsoleStream*> streams_;
};

};

#endif // BBCONSOLE_H