#include "bsp_file.h"

#include <cstdio>
#include <cstring>
#include <stdexcept>

#include "crc32.h"

namespace navgen
{

namespace
{

std::vector<char> ReadWholeFile(const std::string &path)
{
	FILE *f = std::fopen(path.c_str(), "rb");
	if (!f)
		throw std::runtime_error("cannot open BSP file: " + path);

	std::fseek(f, 0, SEEK_END);
	long size = std::ftell(f);
	std::fseek(f, 0, SEEK_SET);
	if (size < static_cast<long>(sizeof(dheader_t)))
	{
		std::fclose(f);
		throw std::runtime_error("BSP file too small: " + path);
	}

	std::vector<char> data(static_cast<size_t>(size));
	size_t read = std::fread(data.data(), 1, data.size(), f);
	std::fclose(f);
	if (read != data.size())
		throw std::runtime_error("failed to read BSP file: " + path);

	return data;
}

template <typename T>
std::vector<T> ReadLump(const std::vector<char> &data, const lump_t &lump, const char *lumpName)
{
	if (lump.filelen < 0 ||
		static_cast<size_t>(lump.fileofs) + static_cast<size_t>(lump.filelen) > data.size())
		throw std::runtime_error(std::string("corrupt lump: ") + lumpName);
	if (static_cast<size_t>(lump.filelen) % sizeof(T) != 0)
		throw std::runtime_error(std::string("lump size not a multiple of element size: ") + lumpName);

	size_t count = static_cast<size_t>(lump.filelen) / sizeof(T);
	std::vector<T> result(count);
	if (count > 0)
		std::memcpy(result.data(), data.data() + lump.fileofs, static_cast<size_t>(lump.filelen));
	return result;
}

}

BspFile::BspFile(const std::string &path)
{
	std::vector<char> data = ReadWholeFile(path);

	dheader_t header;
	std::memcpy(&header, data.data(), sizeof(header));

	if (header.version != HLBSP_VERSION)
		throw std::runtime_error(
			"unsupported BSP version (navgen targets stock GoldSrc BSP30 only): " + path);

	m_planes = ReadLump<dplane_t>(data, header.lumps[LUMP_PLANES], "LUMP_PLANES");
	m_clipnodes = ReadLump<dclipnode_t>(data, header.lumps[LUMP_CLIPNODES], "LUMP_CLIPNODES");
	m_models = ReadLump<dmodel_t>(data, header.lumps[LUMP_MODELS], "LUMP_MODELS");

	const lump_t &entLump = header.lumps[LUMP_ENTITIES];
	if (entLump.filelen < 0 ||
		static_cast<size_t>(entLump.fileofs) + static_cast<size_t>(entLump.filelen) > data.size())
		throw std::runtime_error("corrupt lump: LUMP_ENTITIES");
	m_entityText.assign(data.data() + entLump.fileofs, static_cast<size_t>(entLump.filelen));

	if (m_models.empty())
		throw std::runtime_error("BSP has no models (corrupt or empty map): " + path);
}

uint32_t BspFile::Checksum() const
{
	return BspChecksumLumps(
		m_planes.data(), m_planes.size() * sizeof(dplane_t),
		m_clipnodes.data(), m_clipnodes.size() * sizeof(dclipnode_t),
		m_models.data(), m_models.size() * sizeof(dmodel_t));
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

