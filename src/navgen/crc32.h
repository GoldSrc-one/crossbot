#pragma once

#include <cstddef>
#include <cstdint>

namespace navgen
{

uint32_t Crc32(const void *data, size_t length);

uint32_t BspChecksumLumps(const void *planes, size_t planesBytes,
	const void *clipnodes, size_t clipnodesBytes,
	const void *models, size_t modelsBytes);

}

