#pragma once

#include "nav_mesh.h"

namespace Nav
{

void LoadForMap(const char *mapName);
void Unload();
bool IsLoaded();
const NavMesh *Get();

}

