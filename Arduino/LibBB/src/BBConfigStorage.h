#if !defined(BBCONFIGSTORAGE_H)
#define BBCONFIGSTORAGE_H

#include <vector>

#include "BBError.h"

namespace bb {

/*!
	\brief An abstraction class to manage persistent parameter storage.

	The way this works is you reserve named blocks of a given size, then write them to flash and read them back.
	Each block has a flag that states whether what's stored in flash is valid, so that if it's not it can be
	initialized. 

	Erasing flash memory is processor dependent. SAMD architectures (like Arduino MKR Wifi 1010) erase the
	parameter flash at each software upload. For other architectures, like ESP32, that conveniently keep the parameter 
	flash across software uploads, the class implements the factoryReset() method.

	This is a singleton class that can be accessed using the static bb::ConfigStorage::storage member.
*/
class ConfigStorage {
public:
	typedef unsigned int HANDLE;

	static ConfigStorage storage;

	bool initialize();
	HANDLE reserveBlock(const char* name, size_t size, uint8_t* mem);
	Result writeBlock(HANDLE);
	Result readBlock(HANDLE);
	bool blockIsValid(HANDLE);
	Result factoryReset();
	Result writeAll();
	Result commit();

protected:
	ConfigStorage();

	struct Block {
		HANDLE handle;
		size_t size;
		uint8_t *mem;
	};
	std::vector<Block> blocks_;
	bool initialized_;
	HANDLE nextHandle_;
	size_t maxSize_;
};

};

#endif // BBCONFIGSTORAGE_H