#pragma once

#include <cstdint>

namespace Nav
{

// Recomputes CRC32-over-planes+clipnodes+models (same as navgen's BspFile::Checksum()), reading live map's .bsp off disk. Independent duplicate of navgen's logic (separate build targets) -- mirror any change here if navgen's Checksum() algorithm changes. Returns 0 on read/parse failure, used by loader as "couldn't verify" sentinel (never a valid checksum in practice).
uint32_t ComputeLiveBspChecksum(const char *bspPath);

} // namespace Nav
