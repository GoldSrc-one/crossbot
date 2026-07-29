#include "crc32.h"

#include <vector>

namespace navgen
{

namespace
{

const uint32_t *Table()
{
	static uint32_t table[256];
	static bool initialized = false;
	if (!initialized)
	{
		for (uint32_t i = 0; i < 256; ++i)
		{
			uint32_t c = i;
			for (int k = 0; k < 8; ++k)
				c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
			table[i] = c;
		}
		initialized = true;
	}
	return table;
}

}

uint32_t Crc32(const void *data, size_t length)
{
	const uint32_t *table = Table();
	uint32_t crc = 0xFFFFFFFFu;
	const unsigned char *bytes = static_cast<const unsigned char *>(data);
	for (size_t i = 0; i < length; ++i)
		crc = table[(crc ^ bytes[i]) & 0xFF] ^ (crc >> 8);
	return crc ^ 0xFFFFFFFFu;
}

uint32_t BspChecksumLumps(const void *planes, size_t planesBytes,
	const void *clipnodes, size_t clipnodesBytes,
	const void *models, size_t modelsBytes)
{
	std::vector<unsigned char> buf;
	buf.reserve(planesBytes + clipnodesBytes + modelsBytes);

	auto append = [&](const void *data, size_t bytes)
	{
		const unsigned char *p = static_cast<const unsigned char *>(data);
		buf.insert(buf.end(), p, p + bytes);
	};
	append(planes, planesBytes);
	append(clipnodes, clipnodesBytes);
	append(models, modelsBytes);

	return Crc32(buf.data(), buf.size());
}

}

