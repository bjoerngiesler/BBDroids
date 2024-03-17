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
	if(!initialized_) {
		Serial.println("Not initialized, returning 0.");
		return 0;
	}

	if(nextHandle_ + 1 + size > maxSize_) {
		Serial.println("Size too large");
		return 0;
	}

	Block block = {nextHandle_+1, size};
	blocks_.push_back(block);
	nextHandle_ = nextHandle_ + size + 1;

	return block.handle;
}

bb::Result bb::ConfigStorage::writeBlock(HANDLE handle, uint8_t* data) {
	if(!initialized_) return RES_CONFIG_INVALID_HANDLE;
	for(auto block: blocks_) {
  		if(block.handle == handle) {
  			size_t size = block.size;
  			Serial.print("Storing "); Serial.print(size); Serial.print(" bytes of data at address "); Serial.println(handle, HEX);
  			for(size_t i=0; i<size; i++) {
  				Serial.print(handle+i);
  				Serial.print(".");
  				EEPROM.write(handle+i, data[i]);	
  			} 
  			Serial.println("Marking as valid");
  			EEPROM.write(handle-1, 0xba);
  			Serial.println("Done.");
  			return RES_OK;
  		}
  	}
  	return RES_CONFIG_INVALID_HANDLE;
}

bb::Result bb::ConfigStorage::readBlock(HANDLE handle, uint8_t* data) {
	if(!initialized_) return RES_CONFIG_INVALID_HANDLE;
	for(auto block: blocks_) {
  		if(block.handle == handle) {
  			size_t size = block.size;
  			for(size_t i=0; i<size; i++) {
  				data[i] = EEPROM.read(handle+i);
  			} 
  			return RES_OK;
  		}
  	}
  	return RES_CONFIG_INVALID_HANDLE;
}

bool bb::ConfigStorage::blockIsValid(HANDLE handle) {
	if(handle == 0 || !initialized_) return false;
	for(auto& block: blocks_) {
  		if(block.handle == handle) {
  			if(EEPROM.read(handle-1) == 0xba) return true;
  			else return false;
		} 
	}
  	return false;
}

bool bb::ConfigStorage::initialize() {
	if(initialized_) return true;

#if defined(ARDUINO_NANO_RP2040_CONNECT)
	EEPROM.begin(MAXSIZE);
#endif
	maxSize_ = MAXSIZE;

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
