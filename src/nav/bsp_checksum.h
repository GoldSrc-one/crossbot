#pragma once

#include <cstdint>

namespace Nav
{

// Recomputes the same CRC32-over-planes+clipnodes+models used by navgen's
// BspFile::Checksum(), reading the live map's .bsp directly off disk. An
// independent duplicate of navgen's logic (not shared code, since the
// runtime plugin and the offline tool are separate build targets) --
// mirror any change here if navgen's Checksum() algorithm ever changes.
// Returns 0 on any read/parse failure, used by the loader as a sentinel
// meaning "couldn't verify" (never a valid checksum in practice).
uint32_t ComputeLiveBspChecksum(const char *bspPath);

} // namespace Nav
