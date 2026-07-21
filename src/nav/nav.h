#pragma once

#include "nav_mesh.h"

// Facade over the loaded nav graph, matching the existing GameProfile_Detect()
// free-function convention (src/game/game_profile.h) rather than introducing
// a new interface hierarchy for what is, for now, a single global resource.
namespace Nav
{

void LoadForMap(const char *mapName);
void Unload();
bool IsLoaded();
const NavMesh *Get();

} // namespace Nav
