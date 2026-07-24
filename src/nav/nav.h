#pragma once

#include "nav_mesh.h"

// Facade over loaded nav graph, matching GameProfile_Detect() free-function convention (game_profile.h) rather than new interface hierarchy for single global resource.
namespace Nav
{

void LoadForMap(const char *mapName);
void Unload();
bool IsLoaded();
const NavMesh *Get();

} // namespace Nav
