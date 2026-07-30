#include "nav.h"

#include "nav_loader.h"

namespace Nav
{

namespace
{
std::unique_ptr<NavMesh> g_mesh;
}

void LoadForMap(const char *mapName)
{
	g_mesh = LoadNavFile(mapName);
}

void Unload()
{
	g_mesh.reset();
}

bool IsLoaded()
{
	return g_mesh != nullptr;
}

const NavMesh *Get()
{
	return g_mesh.get();
}

}

