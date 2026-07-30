#pragma once

#include <extdll.h>

#include "nav_mesh.h"

namespace Nav
{

const Area *FindAreaForPoint(const NavMesh &mesh, const Vector &point);

const Area *FindAreaById(const NavMesh &mesh, uint32_t id);

}

