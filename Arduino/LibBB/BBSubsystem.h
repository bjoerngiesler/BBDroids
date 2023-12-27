#if !defined(BBSUBSYSTEM_H)
#define BBSUBSYSTEM_H

#include <Arduino.h>
#include <limits.h>
#include <map>
#include <vector>
#include "BBError.h"
#include "BBConfigStorage.h"

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
	virtual const String& name() { return name_; }
	virtual const String& description() { return description_; }
	virtual const String& help() { return help_; }
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

	virtual Result addParameter(const String& name, const String& help, int& param, int min = INT_MIN, int max = INT_MAX);
	virtual Result addParameter(const String& name, const String& help, float& param, float min = INT_MIN, float max = INT_MAX);
	virtual Result addParameter(const String& name, const String& help, String& param, int maxlen = 0);
	virtual Result addParameter(const String& name, const String& help, bool& val);

	virtual Result setParameterValue(const String& name, const String& stringVal);
	virtual void parameterChangedCallback(const String& name) {} // override if you want to do something if the parameter was changed

protected:
	class Parameter {
	public:
		virtual Result fromString(const String& str) = 0;
		virtual String toString() const = 0;
		virtual String description() const = 0;
		virtual const String& name() const { return name_; }
		virtual const void print(ConsoleStream* stream);
	protected:
		String name_;
	};

	virtual Parameter* findParameter(const String& name);

	class IntParameter: public Parameter {
	public:
		IntParameter(const String& name, int& val, String help, int min=INT_MIN, int max=INT_MAX): 
			val_(val), help_(help), min_(min), max_(max) {name_ = name;}
		virtual Result fromString(const String& str) {
			int v = str.toInt();
			if(v<min_ || v>max_) return RES_COMMON_OUT_OF_RANGE;
			val_ = v;
			return RES_OK;
		}
		virtual String toString() const { return String(val_); }
		virtual String description() const { 
			String str = toString() + " [";
			if(min_==INT_MAX) str+="-inf"; else str+=min_;
			str += "..";
			if(max_==INT_MAX) str+="inf"; else str+=max_;
			str += "]";
			if(help_.length() != 0) str = str + ": " + help_; 
			return str;
		}
	protected:
		int& val_;
		int min_, max_;
		String help_;
	};

	class FloatParameter: public Parameter {
	public:
		FloatParameter(const String& name, float& val, String help, float min=INT_MIN, float max=INT_MAX): 
			val_(val), help_(help), min_(min), max_(max) {name_ = name;}
		virtual Result fromString(const String& str) {
			float v = str.toFloat();
			if(v<min_ || v>max_) return RES_COMMON_OUT_OF_RANGE;
			val_ = v;
			return RES_OK;
		}
		virtual String toString() const { return String(val_, 10); }
		virtual String description() const { 
			String str = toString() + " [";
			if(min_==INT_MAX) str+="-inf"; else str+=min_;
			str += "..";
			if(max_==INT_MAX) str+="inf"; else str+=max_;
			str += "]";
			if(help_.length() != 0) str = str + ": " + help_; 
			return str;
		}
	protected:
		float& val_;
		float min_, max_;
		String help_;
	};

	class StringParameter: public Parameter {
	public:
		StringParameter(const String& name, String& val, String help, int maxlen=0): val_(val), help_(help), maxlen_(maxlen) {name_ = name;}
		virtual Result fromString(const String& str) { 
			if(maxlen_ > 0 && str.length() <= maxlen_) {
				val_ = str; 
				return RES_OK; 
			} 
			return RES_COMMON_OUT_OF_RANGE;
		}
		virtual String toString() const { return val_; }
		virtual String description() const { 
			String str = toString();
			if(maxlen_ != 0) str = str + " (max length " + maxlen_ + ")";
			if(help_.length() != 0) str = str + ": " + help_; 
			return str;
		}
	protected:
		String& val_;
		int maxlen_;
		String help_;
	};	

	class BoolParameter: public Parameter {
	public:
		BoolParameter(const String& name, bool& val, String help): val_(val), help_(help) {name_ = name;}
		virtual Result fromString(const String& str) { 
			if(str == "true" || str == "yes" || str == "1") {
				val_ = true;
				return RES_OK;
			} else if(str == "false" || str == "no" || str == "0") {
				val_ = false;
				return RES_OK;
			}
			return RES_COMMON_OUT_OF_RANGE;
		}
		virtual String toString() const { return val_ ? "true" : "false"; }
		virtual String description() const { 
			String str = toString();
			if(help_.length() != 0) str = str + ": " + help_; 
			return str;
		}
	protected:
		bool& val_;
		String help_;
	};

	std::vector<Parameter*> parameters_;
	bool started_, begun_;
	Result operationStatus_;
	String name_, description_, help_;
	Subsystem(): started_(false), begun_(false), operationStatus_(RES_SUBSYS_NOT_INITIALIZED), name_(""), description_(""), help_("") {}
	virtual ~Subsystem() { }
};


};

#endif // BBSUBSYSTEM_H
