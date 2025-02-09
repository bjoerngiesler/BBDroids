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
	static const unsigned int LOG_ALL   = 0;
	static const unsigned int LOG_DEBUG = 1;
	static const unsigned int LOG_INFO  = 2;
	static const unsigned int LOG_WARN  = 3;
	static const unsigned int LOG_ERROR = 4;
	static const unsigned int LOG_FATAL = 5;

#define LOGS(stream, level, args...) if(level>=loglevel_) { bb::printf(stream, "%s(%d):", name_, level); bb::printf(stream, args); }
#define LOG(level, args...) if(level>=loglevel_) { bb::printf("%s(%d):", name_, level); bb::printf(args); }

	virtual const char* name() { return name_; }
	virtual const char* description() { return description_; }
	virtual const char* help() { return help_; }
	virtual Result handleConsoleCommand(const std::vector<String>& words, ConsoleStream *stream);

	virtual Result initialize();
	virtual Result start(ConsoleStream *stream) = 0;
	virtual Result stop(ConsoleStream *stream) = 0;
	virtual Result step() = 0;
	virtual bool isStarted() { return started_; }
	virtual Result operationStatus() { return operationStatus_; }
	virtual unsigned long sequenceNumber(bool autoincrement = true) { unsigned long s = seqnum_; if(autoincrement) seqnum_++; return s; }

	void setLogLevel(unsigned int lvl) { loglevel_ = lvl; }

	virtual Result registerWithManager() { return SubsystemManager::manager.registerSubsystem(this); }

	virtual String statusLine();
	virtual void printStatusLine(ConsoleStream *stream = NULL);
	virtual void printExtendedStatus(ConsoleStream *stream = NULL);
	virtual void printHelp(ConsoleStream *stream);
	virtual void printParameters(ConsoleStream *stream);

	virtual Result addParameter(const String& name, const String& help, unsigned int& param, int max = INT_MAX);
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
		virtual void print(ConsoleStream* stream);
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
		String help_;
		int min_, max_;
	};

	class UIntParameter: public Parameter {
	public:
		UIntParameter(const String& name, unsigned int& val, String help, int max=INT_MAX): 
			val_(val), help_(help), max_(max) { name_ = name; }
		virtual Result fromString(const String& str) {
			int v = str.toInt();
			if(v<0 || v>int(max_)) return RES_COMMON_OUT_OF_RANGE;
			val_ = v;
			return RES_OK;
		}
		virtual String toString() const { return String(val_); }
		virtual String description() const { 
			String str = toString() + " [0..";
			if(max_==INT_MAX) str+="inf"; else str+=max_;
			str += "]";
			if(help_.length() != 0) str = str + ": " + help_; 
			return str;
		}
	protected:
		unsigned int& val_;
		String help_;
		unsigned int max_;
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
		String help_;
		float min_, max_;
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
		String help_;
		unsigned int maxlen_;
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
	bool started_;
	Result operationStatus_;
	const char *name_, *description_, *help_;
	unsigned long seqnum_;
	unsigned int loglevel_;

	Subsystem(): started_(false), operationStatus_(RES_SUBSYS_NOT_INITIALIZED), name_(""), description_(""), help_(""), seqnum_(0), loglevel_(LOG_INFO) {}
	virtual ~Subsystem() { }
};


};

#endif // BBSUBSYSTEM_H
