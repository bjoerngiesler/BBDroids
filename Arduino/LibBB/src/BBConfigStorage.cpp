#include "BBConfigStorage.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
static const size_t MAXSIZE=1024;
#elif defined(ARDUINO_ARCH_SAMD)
#include <FlashAsEEPROM.h>
static const size_t MAXSIZE=EEPROM_EMULATION_SIZE;
#elif defined(ARDUINO_ARCH_ESP32)
#include <nvs_flash.h>
#include <nvs.h>
static const size_t MAXSIZE=1024;
#else
#error Unsupported architecture
#endif


bb::ConfigStorage bb::ConfigStorage::storage;

bb::ConfigStorage::HANDLE bb::ConfigStorage::reserveBlock(const char* name, size_t size) {
	if(!initialized_) {
		Serial.println("Not initialized, returning 0.");
		return 0;
	}

	if(nextHandle_ + 1 + size > maxSize_) {
		Serial.println("Size too large");
		return 0;
	}

#if defined(ARDUINO_ARCH_ESP32)
	nvs_handle_t handle;
	esp_err_t err = nvs_open(name, NVS_READWRITE, &handle);
	if(err != ESP_OK) {
		Serial.println(String("nvs_open() returned ") + err);
		return RES_CONFIG_INVALID_HANDLE;
	}
	Block block = {handle, size};
#else
	Block block = {nextHandle_+1, size};
	nextHandle_ = nextHandle_ + size + 1;
#endif
	blocks_.push_back(block);
	return block.handle;
}

bb::Result bb::ConfigStorage::writeBlock(HANDLE handle, uint8_t* data) {
	if(!initialized_) return RES_CONFIG_INVALID_HANDLE;

	for(auto block: blocks_) {
  		if(block.handle == handle) {
  			size_t size = block.size;
  			Serial.print("Storing "); Serial.print(size); Serial.print(" bytes of data at address "); Serial.println(handle, HEX);
#if defined(ARDUINO_ARCH_ESP32)
			esp_err_t err;
			err = nvs_set_blob(handle, "blob", data, size);
			if(err != ESP_OK) {
				Serial.println(String("nvs_set_blob() returned ") + err);
				return RES_CONFIG_INVALID_HANDLE;
			}
			
			err = nvs_set_i8(handle, "valid", 1);
			if(err != ESP_OK) {
				Serial.println(String("nvs_set_i8() returned ") + err);
				return RES_CONFIG_INVALID_HANDLE;
			}

			err = nvs_commit(handle);
			if(err != ESP_OK) {
				Serial.println(String("nvs_commit() returned ") + err);
				return RES_CONFIG_INVALID_HANDLE;
			}

  			Serial.println("Commited blob.");
#else
  			for(size_t i=0; i<size; i++) {
  				Serial.print(handle+i);
  				Serial.print(".");
  				EEPROM.write(handle+i, data[i]);	
  			} 
  			Serial.println("Marking as valid");
  			EEPROM.write(handle-1, 0xba);
#endif
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
#if defined(ARDUINO_ARCH_ESP32)
			esp_err_t err = nvs_get_blob(handle, "blob", data, &block.size);
			if(err != ESP_OK) {
				Serial.println(String("nvs_get_blob() returned ") + err);
				return RES_CONFIG_INVALID_HANDLE;
			}
#else
  			size_t size = block.size;
  			for(size_t i=0; i<size; i++) {
  				data[i] = EEPROM.read(handle+i);
  			} 
#endif
  			return RES_OK;
  		}
  	}
  	return RES_CONFIG_INVALID_HANDLE;
}

bool bb::ConfigStorage::blockIsValid(HANDLE handle) {
	if(handle == 0 || !initialized_) return false;

	for(auto& block: blocks_) {
  		if(block.handle == handle) {
#if defined(ARDUINO_ARCH_ESP32)
			int8_t i8 = 0;
			esp_err_t err = nvs_get_i8(handle, "valid", &i8);
			if(err == ESP_OK && i8 == 1) return true;
#else
  			if(EEPROM.read(handle-1) == 0xba) return true;
#endif
  			else return false;
		} 
	}
  	return false;
}

bool bb::ConfigStorage::initialize() {
	if(initialized_) return true;

#if defined(ARDUINO_NANO_RP2040_CONNECT)
	EEPROM.begin(MAXSIZE);
#elif defined(ARDUINO_ARCH_ESP32)
	esp_err_t err = nvs_flash_init();
	if(err != ESP_OK) {
		Serial.println(String("Error initializing flash: ") + err);
		return false;
	}
	Serial.println("Flash initialized OK.\n");
#endif
	maxSize_ = MAXSIZE;

	nextHandle_ = 0;
	initialized_ = true;
	return true;
}

bb::Result bb::ConfigStorage::store() {
	if(!initialized_) return RES_SUBSYS_NOT_INITIALIZED;
#if !defined(ARDUINO_ARCH_ESP32)
	EEPROM.commit();
#endif
	return RES_OK;
}

bb::Result bb::ConfigStorage::factoryReset() {
#if defined(ARDUINO_ARCH_ESP32)
	esp_err_t err = nvs_flash_erase();
	if(err != ESP_OK) {
		Serial.println(String("Error erasing flash: ") + err);
		return RES_CONFIG_INVALID_HANDLE;
	}
#else
#warning Factory reset not implemented for current architecture!
#endif
	return RES_OK;
}

bb::ConfigStorage::ConfigStorage() {
	initialized_ = false;
}
