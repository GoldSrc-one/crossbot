#pragma once

#include <cstddef>
#include <cstdint>

namespace navgen
{

uint32_t Crc32(const void *data, size_t length);

// The identity of a map's collision geometry. Both navgen and the runtime plugin checksum the
// same three lumps so a .nav file can be rejected when it no longer matches the live BSP; keeping
// it here rather than in bsp_file.cpp is what lets the plugin link it without the BSP reader.
uint32_t BspChecksumLumps(const void *planes, size_t planesBytes,
	const void *clipnodes, size_t clipnodesBytes,
	const void *models, size_t modelsBytes);

}

