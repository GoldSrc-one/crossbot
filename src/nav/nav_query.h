#pragma once

#include <extdll.h>

#include "nav_mesh.h"

namespace Nav
{

bool AreaContainsPoint(const Area &area, const Vector &point);

float AreaFloorZ(const Area &area, float x, float y);

const Area *FindAreaExact(const NavMesh &mesh, const Vector &point);

const Area *FindAreaNearest(const NavMesh &mesh, const Vector &point);

const Area *FindAreaById(const NavMesh &mesh, uint32_t id);

}

