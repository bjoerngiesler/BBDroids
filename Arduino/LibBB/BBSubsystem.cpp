#include "BBSubsystem.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#elif defined(ARDUINO_ARCH_MBED_NANO)
#include <FlashAsEEPROM.h>
#endif

bb::SubsystemManager bb::SubsystemManager::manager;
	
bb::Result bb::SubsystemManager::registerSubsystem(Subsystem* subsys) {
	if(subsystemWithName(subsys->name()) != NULL) return RES_SUBSYS_ALREADY_REGISTERED; // already have this
	subsys_.push_back(subsys);
	return RES_OK;
}
	
bb::Subsystem* bb::SubsystemManager::subsystemWithName(const String& name) {
	for(size_t i=0; i<subsys_.size(); i++) {
		if(subsys_[i]->name() == name) return subsys_[i];
	}
	return NULL;
}

const std::vector<bb::Subsystem*>& bb::SubsystemManager::subsystems() {
	return subsys_;
}


bb::SubsystemManager::SubsystemManager() {
}
