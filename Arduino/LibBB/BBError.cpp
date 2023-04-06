#include "BBError.h"

static String messages[] = {
	"OK.", // 0
	"No such parameter.", // 1
	"Invalid parameter type.", // 2
	"Hardware dependency missing (eg specify serial port in code).", // 3
	"Hardware dependency locked (shutdown subsys first).", // 4
	"Subsystem already registered.", // 5
	"No such subsystem.", // 6
	"Resource not available.", // 7
	"Communication error.", // 8
	"Protocol error.", // 9
	"Subsystem already started.", // 10
	"Subsystem not started.", // 11
	"Subsystem not operational.", // 12
	"Subsystem not stoppable.", // 13
	"Invalid parameter value.", // 14
	"Read-only parameter.", // 15
	"Invalid handle.", // 16
	"Subsystem not initialized.", // 17
	"Subsystem already initialized.", // 18
	"Communication timeout.", // 19
	"Unknown command.", // 20
	"Invalid number of arguments.", // 21
	"Invalid argument.", // 22
	"Not in list.", // 23
	"Duplicate in list.", // 24
	"Wrong mode.", // 25
	"Packet too short.", // 26
	"Packet too long.", // 27
	"Packet invalid.", // 28
	"Failure." // 29
};

static String UnknownError = "Unknown Error";

static size_t numMessages = 30;

const String& bb::errorMessage(Result res) {
	if((size_t)res >= numMessages) return UnknownError;
	else return messages[(size_t)res];
}
