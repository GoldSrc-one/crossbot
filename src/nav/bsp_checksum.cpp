#include "bsp_checksum.h"

#include <cstdio>
#include <cstring>
#include <vector>

#include <extdll.h>
#include <bspfile.h>

namespace Nav
{

namespace
{

uint32_t Crc32(const void *data, size_t length)
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

	uint32_t crc = 0xFFFFFFFFu;
	const unsigned char *bytes = static_cast<const unsigned char *>(data);
	for (size_t i = 0; i < length; ++i)
		crc = table[(crc ^ bytes[i]) & 0xFF] ^ (crc >> 8);
	return crc ^ 0xFFFFFFFFu;
}

bool ReadLump(const std::vector<unsigned char> &data, const lump_t &lump, size_t elemSize,
	std::vector<unsigned char> &out)
{
	if (lump.filelen < 0 || static_cast<size_t>(lump.fileofs) + static_cast<size_t>(lump.filelen) > data.size())
		return false;
	if (static_cast<size_t>(lump.filelen) % elemSize != 0)
		return false;
	out.assign(data.data() + lump.fileofs, data.data() + lump.fileofs + lump.filelen);
	return true;
}

} // namespace

uint32_t ComputeLiveBspChecksum(const char *bspPath)
{
	FILE *f = std::fopen(bspPath, "rb");
	if (!f)
		return 0;

	std::fseek(f, 0, SEEK_END);
	long size = std::ftell(f);
	std::fseek(f, 0, SEEK_SET);
	if (size < static_cast<long>(sizeof(dheader_t)))
	{
		std::fclose(f);
		return 0;
	}

	std::vector<unsigned char> data(static_cast<size_t>(size));
	size_t readBytes = std::fread(data.data(), 1, data.size(), f);
	std::fclose(f);
	if (readBytes != data.size())
		return 0;

	dheader_t header;
	std::memcpy(&header, data.data(), sizeof(header));
	if (header.version != HLBSP_VERSION)
		return 0;

	std::vector<unsigned char> planes, clipnodes, models;
	if (!ReadLump(data, header.lumps[LUMP_PLANES], sizeof(dplane_t), planes) ||
		!ReadLump(data, header.lumps[LUMP_CLIPNODES], sizeof(dclipnode_t), clipnodes) ||
		!ReadLump(data, header.lumps[LUMP_MODELS], sizeof(dmodel_t), models))
		return 0;

	std::vector<unsigned char> combined;
	combined.reserve(planes.size() + clipnodes.size() + models.size());
	combined.insert(combined.end(), planes.begin(), planes.end());
	combined.insert(combined.end(), clipnodes.begin(), clipnodes.end());
	combined.insert(combined.end(), models.begin(), models.end());

	return Crc32(combined.data(), combined.size());
}

} // namespace Nav
