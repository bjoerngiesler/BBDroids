#include "BBConfigStorage.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
static const size_t MAXSIZE=1024;
#elif defined(ARDUINO_ARCH_SAMD)
#include <FlashAsEEPROM.h>
static const size_t MAXSIZE=EEPROM_EMULATION_SIZE;
#else
#error Unsupported architecture
#endif


bb::ConfigStorage bb::ConfigStorage::storage;

bb::ConfigStorage::HANDLE bb::ConfigStorage::reserveBlock(size_t size) {
	if(!initialized_) initialize();

	if(nextHandle_ + 1 + size > maxSize_) return 0;

	HANDLE retval = nextHandle_ + 1;
	blocks_[retval] = size;
	nextHandle_ = nextHandle_ + size + 1;
	return retval;
}

bb::Result bb::ConfigStorage::writeBlock(HANDLE handle, uint8_t* data) {
	if(!initialized_) return RES_CONFIG_INVALID_HANDLE;
	std::map<HANDLE, size_t>::iterator it = blocks_.begin();
 	
 	// Iterate through the map and print the elements
  	while (it != blocks_.end()) {
  		if(it->first == handle) {
  			size_t size = it->second;
  			Serial.print("Storing "); Serial.print(size); Serial.print(" bytes of data at address "); Serial.println(handle, HEX);
  			for(size_t i=0; i<size; i++) {
  				Serial.print(handle+i);
  				Serial.print(".");
  				EEPROM.write(handle+i, data[i]);	
  			} 
  			Serial.println("Marking as valid");
  			EEPROM.write(handle-1, 1);
  			Serial.println("Done.");
  			return RES_OK;
  		}
    	++it;
  	}
  	return RES_CONFIG_INVALID_HANDLE;
}

bb::Result bb::ConfigStorage::readBlock(HANDLE handle, uint8_t* data) {
	if(!initialized_) return RES_CONFIG_INVALID_HANDLE;
	std::map<HANDLE, size_t>::iterator it = blocks_.begin();
 	
 	// Iterate through the map and print the elements
  	while (it != blocks_.end()) {
  		if(it->first == handle) {
  			size_t size = it->second;
  			for(size_t i=0; i<size; i++) {
  				data[i] = EEPROM.read(handle+i);
  			} 
  			return RES_OK;
  		}
    	++it;
  	}
  	return RES_CONFIG_INVALID_HANDLE;
}

bool bb::ConfigStorage::blockIsValid(HANDLE handle) {
	if(handle == 0 || !initialized_) return false;
	std::map<HANDLE, size_t>::iterator it = blocks_.begin();
 	
 	// Iterate through the map and print the elements
  	while (it != blocks_.end()) {
  		if(it->first == handle) {
  			if(EEPROM.read(handle-1) == 1) return true;
  			else return false;
		} 
    	++it;
	}
  	return false;
}

bool bb::ConfigStorage::initialize() {
	if(initialized_) return true;
#if defined(NANO_RP2040_CONNECT)
	EEPROM.begin(MAXSIZE);
#else
	maxSize_ = MAXSIZE;
#endif

	nextHandle_ = 0;
	initialized_ = true;
	return true;
}

bb::Result bb::ConfigStorage::store() {
	if(!initialized_) return RES_SUBSYS_NOT_INITIALIZED;
	EEPROM.commit();
	return RES_OK;
}

bb::ConfigStorage::ConfigStorage() {
	initialized_ = false;
}
