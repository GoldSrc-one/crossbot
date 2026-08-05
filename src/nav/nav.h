#pragma once

#include "nav_mesh.h"

namespace Nav
{

void LoadForMap(const char *mapName);
void Unload();
bool IsLoaded();
const NavMesh *Get();

void PenalizeConnection(uint32_t connIndex);
void RewardConnection(uint32_t connIndex);
float ConnectionPenalty(uint32_t connIndex);

}

