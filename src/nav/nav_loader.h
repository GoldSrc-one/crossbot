#pragma once

#include <memory>

#include "nav_mesh.h"

namespace Nav
{

// Loads <moddir>/crossbot_data/nav/<mapName>.nav, validating its header
// magic/version and bspChecksum against a live recompute over
// <moddir>/maps/<mapName>.bsp. Returns nullptr (logging via ALERT) on any
// failure -- missing file, corrupt header, or a stale/mismatched checksum
// -- so a bad or absent nav file degrades to "no nav loaded" (the bot
// falls back to wandering), never a crash or a trust of stale data.
std::unique_ptr<NavMesh> LoadNavFile(const char *mapName);

} // namespace Nav
