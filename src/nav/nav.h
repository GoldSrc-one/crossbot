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

int AddTeleport(const Vector &launch, uint32_t fromArea, const Vector &landing, uint32_t toArea,
	float waitSeconds, uint32_t teamMask);
void RemoveTeleport(int connIndex);
void SetTeleportWait(int connIndex, float waitSeconds);
bool IsAddedTeleport(int connIndex);

void SetDetourTeams(const Vector &actCenter, uint32_t teamMask);

void AddCamp(const CampSpot &spot);

}

