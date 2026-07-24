#pragma once

#include <memory>

#include "nav_mesh.h"

namespace Nav
{

// Loads <moddir>/crossbot_data/nav/<mapName>.nav, validating header magic/version and bspChecksum against live recompute over <moddir>/maps/<mapName>.bsp. Returns nullptr (logging via ALERT) on any failure -- missing file, corrupt header, stale/mismatched checksum -- so bad/absent nav file degrades to "no nav loaded" (bot falls back to wandering), never a crash or trust of stale data.
std::unique_ptr<NavMesh> LoadNavFile(const char *mapName);

} // namespace Nav
