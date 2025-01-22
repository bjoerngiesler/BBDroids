#if !defined(BBCONFIGSTORAGE_H)
#define BBCONFIGSTORAGE_H

#include <vector>

#include "BBError.h"

namespace bb {

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