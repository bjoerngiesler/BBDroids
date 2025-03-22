#if !defined(BBCONSOLE_H)
#define BBCONSOLE_H

#include "BBSubsystem.h"

#include <vector>
#include <cstdarg>


namespace bb {

class Subsystem;	
class ConsoleStream;

//! Variadic printf() that uses the given ConsoleStream, or BroadcastStream if stream is NULL.
int printf(ConsoleStream* stream, const char* format, ...);

//! Variadic printf() that uses BroadcastStream.
int printf(const char* format, ...);

/*!
	\brief Base class for console streams.

*/
class ConsoleStream {
public:
	virtual bool available() = 0;
	virtual bool readStringUntil(unsigned char c, String& str) = 0;

	int printf(const char* format, ...) {
		int retval;
		va_list args;
		va_start(args, format);
		retval = vprintf(format, args);
		va_end(args);
		return retval;
	}

	int vprintf(const char* format, va_list args) {
		int len = vsnprintf(NULL, 0, format, args) + 1;
		char *buf = new char[len];
		vsnprintf(buf, len, format, args);
		printfFinal(buf);
		free(buf);
		return len;
	}

	virtual int printfFinal(const char* str) = 0;

	void printGreeting() {
		printfFinal("Console ready. Type \"help\" for instructions.\n> ");
	}
};

#if defined(ARDUINO_ARCH_ESP32)
#define HWSERIAL_CLASS HWCDC
#else
#define HWSERIAL_CLASS HardwareSerial
#endif

/*!
	\brief Console stream interacting with a serial port. 
	
	One of these is created by default on the Serial line.
*/
class SerialConsoleStream: public ConsoleStream {
public:
	SerialConsoleStream(HWSERIAL_CLASS& ser);

	bool checkIfOpened();
	void setCheckInterval(unsigned long microseconds);
	unsigned long checkInterval() { return checkInterval_; }

	virtual bool available();
	static bool readStringUntil(HWSERIAL_CLASS& ser, char c, String& str);
	virtual bool readStringUntil(unsigned char c, String& str) { return readStringUntil(ser_, c, str); }

	virtual int printfFinal(const char* str);
protected:
	HWSERIAL_CLASS& ser_;
	bool opened_;
	unsigned long checkInterval_, lastCheck_;
};

/*!
	\brief Virtual stream that will broadcast to all opened console streams. 
	
	This stream cannot be read from.
*/
class BroadcastStream: public ConsoleStream {
public:
	static BroadcastStream bc;

	virtual bool available() { return false; }
	virtual bool readStringUntil(unsigned char c, String& str) { return false; }
	virtual int printfFinal(const char* str);
};

/*!
	\brief Console subsystem.

	This subsystem handles console I/O. It can manage several consoles; one is created with the Arduino's
	Serial port by default but more can be added, e.g. by the Wifi subsystem.

	Commands that are entered into a console are handled by the first responder; this is customarily
	bb::Console::console itself, but another subsystem can be defined as the first responder.

	The console subsystem handles some toplevel commands, like "help", "status" etc., by itself. Other commands
	need to be prefixed with a subsystem name, and will then be forwarded to that subsystem's handleConsoleCommand()
	method. The stream the command was entered on is passed to the command handler to be able to output

*/
class Console: public Subsystem {
public:
	static Console console;

	static std::vector<String> split(const String& str);

	//! Initialize, opening the Serial port using Serial.begin(bps)
	virtual Result initialize(int bps) { 
		Serial.begin(bps); 
		return initialize();
	}

	//! Initialize, assuming the Serial port is already opened
	virtual Result initialize() { 
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
	ConsoleStream *serialStream_;
	std::vector<ConsoleStream*> streams_;
	Subsystem* firstResponder_;
};

};

#endif // BBCONSOLE_H