#if !defined(BBSUBSYSTEM_H)
#define BBSUBSYSTEM_H

#include <Arduino.h>
#include <vector>
#include "BBError.h"

namespace bb {

class Subsystem;
class ConsoleStream;

class SubsystemManager {
public:
	static SubsystemManager manager;
	Result registerSubsystem(Subsystem* subsys);
	Subsystem* subsystemWithName(const String& name);
	const std::vector<Subsystem*>& subsystems();

protected:
	SubsystemManager();
	std::vector<Subsystem*> subsys_;
};

class Subsystem {
public:
	typedef enum {
		PARAMETER_UINT,
		PARAMETER_INT,
		PARAMETER_FLOAT,
		PARAMETER_STRING
	} ParameterType;

	typedef struct {
		String name;
		ParameterType type;
		String help;
	} ParameterDescription;

	virtual const String& name() { return name_; }
	virtual const String& description() { return description_; }
	virtual const String& help() { return help_; }
	virtual const std::vector<ParameterDescription>& parameters() { return parameters_; }
	virtual Result parameterValue(const String& name, String& value) { String n = name; String v = value; return RES_PARAM_NO_SUCH_PARAMETER; }
	virtual Result setParameterValue(const String& name, const String& value) { String n = name; String v = value; return RES_PARAM_NO_SUCH_PARAMETER; }
	virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

	virtual Result initialize() { operationStatus_ = RES_SUBSYS_NOT_STARTED; return registerWithManager(); };
	virtual Result start(ConsoleStream *stream) = 0;
	virtual Result stop(ConsoleStream *stream) = 0;
	virtual Result step() = 0;
	virtual bool isStarted() { return started_; }
	virtual Result operationStatus() { return operationStatus_; }

	virtual Result registerWithManager() { return SubsystemManager::manager.registerSubsystem(this); }

	virtual void printStatus(ConsoleStream *stream);
	virtual void printHelp(ConsoleStream *stream);
	virtual void printParameters(ConsoleStream *stream);
	virtual void printParameter(ConsoleStream *stream, const ParameterDescription& p);

protected:
	std::vector<ParameterDescription> parameters_;
	bool started_, begun_;
	Result operationStatus_;
	String name_, description_, help_;
	Subsystem(): started_(false), begun_(false), operationStatus_(RES_SUBSYS_NOT_INITIALIZED), name_(""), description_(""), help_("") {}
	virtual ~Subsystem() { }
};


};

#endif // BBSUBSYSTEM_H
