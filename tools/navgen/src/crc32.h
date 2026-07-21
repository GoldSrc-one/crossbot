#pragma once

#include <cstddef>
#include <cstdint>

namespace navgen
{

// Standard IEEE 802.3 CRC32 over a single buffer. Used only as an internal
// tamper/staleness check between navgen's output and the runtime loader's
// recomputed value (Milestone 2c) -- not required to interoperate with any
// external CRC32 implementation.
uint32_t Crc32(const void *data, size_t length);

} // namespace navgen
