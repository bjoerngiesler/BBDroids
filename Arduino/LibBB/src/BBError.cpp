#include "BBError.h"

static const char* messages[] = {
	"OK", // 0
	
	"No such parameter", // 1
	"Invalid parameter type", // 2
	"Invalid parameter value", // 3
	"Read-only parameter", // 4
	
	"Hardware dependency missing (eg specify serial port in code)", // 5
	"Hardware dependency locked (shutdown subsys first)", // 6
	"Subsystem already registered", // 7
	"No such subsystem", // 8
	"Resource not available", // 9
	"Communication error", // 10
	"Protocol error", // 11
	"Subsystem already started", // 12
	"Subsystem not started", // 13
	"Subsystem not operational", // 14
	"Subsystem not stoppable", // 15
	"Subsystem not initialized", // 16
	"Subsystem already initialized", // 17
	"Wrong mode", // 18

	"Invalid handle", // 19

	"Communication timeout", // 20

	"Unknown command", // 21
	"Invalid number of arguments", // 22
	"Invalid argument", // 23
	"Failure", // 24

	"Not in list", // 25
	"Duplicate in list", // 26
	"Out of range", // 27

	"Packet too short", // 28
	"Packet too long", // 29
	"Packet invalid", // 30

	"Voltage too low", // 31
	"Voltage too high", // 32
	"Current too high", // 33
	"Wrong direction" // 34¨
};

static const char* UnknownError = "Unknown Error";

static size_t numMessages = 35;

const char* bb::errorMessage(Result res) {
	if((size_t)res >= numMessages) return UnknownError;
	else return messages[(size_t)res];
}
