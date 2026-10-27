#pragma once

#include <cstdint>

#include "nav_mesh.h"

namespace Nav
{

bool BarrierClosed(const NavMesh &mesh, uint16_t barrier);

inline bool AreaBarred(const NavMesh &mesh, const Area &area)
{
	return BarrierClosed(mesh, area.barrier) ||
		(area.barrierFloor != crossbot_nav::kNoBarrier && !BarrierClosed(mesh, area.barrierFloor));
}

inline bool ConnBarred(const NavMesh &mesh, const Connection &conn)
{
	return AreaBarred(mesh, mesh.areas[conn.areaA]) || AreaBarred(mesh, mesh.areas[conn.areaB]);
}

uint32_t BarrierGeneration(const NavMesh &mesh);

void ResetBarrierCache();

}
