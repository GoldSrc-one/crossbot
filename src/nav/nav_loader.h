#pragma once

#include <memory>

#include "nav_mesh.h"

namespace Nav
{

std::unique_ptr<NavMesh> LoadNavFile(const char *mapName);

}

