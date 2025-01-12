#include "BBError.h"

static const char* messages[] = {
	"OK", // 0
	
	"No such parameter", // 1
	"Invalid parameter type", // 2
	"Invalid parameter value", // 3
	"Read-only parameter", // 4
	
	"HW dependency missing", // 5
	"HW dependency locked", // 6
	"Subsys already registered", // 7
	"No such subsys", // 8
	"Resource not available", // 9
	"Comm error", // 10
	"Protocol error", // 11
	"Subsys already started", // 12
	"Subsys not started", // 13
	"Subsys not operational", // 14
	"Subsys not stoppable", // 15
	"Subsys not initialized", // 16
	"Subsys already initialized", // 17
	"Wrong mode", // 18

	"Invalid handle", // 19

	"Comm timeout", // 20

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
