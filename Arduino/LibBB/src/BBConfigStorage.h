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
	HANDLE reserveBlock(const char* name, size_t size);
	Result writeBlock(HANDLE, uint8_t* block);
	Result readBlock(HANDLE, uint8_t* block);
	bool blockIsValid(HANDLE);
	Result factoryReset();
	Result store();

protected:
	ConfigStorage();

	struct Block {
		HANDLE handle;
		size_t size;
	};
	std::vector<Block> blocks_;
	bool initialized_;
	HANDLE nextHandle_;
	size_t maxSize_;
};

};

#endif // BBCONFIGSTORAGE_H