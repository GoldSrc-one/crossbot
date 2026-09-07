#include <extdll.h>
#include <meta_api.h>

#include "crossplay.h"
#include "give_item.h"

#define SF_NORESPAWN (1 << 30)

namespace
{

edict_t *CreateIn(int iGame, const char *classname)
{
	edict_t *pent = nullptr;
	XPlay::InGame(iGame, [&] { pent = g_engfuncs.pfnCreateNamedEntity(g_engfuncs.pfnAllocString(classname)); });

	if (!pent || g_engfuncs.pfnIndexOfEdict(pent) == 0)
		return nullptr;

	return pent;
}

}

namespace GiveItem
{

bool Give(edict_t *player, const char *classname)
{
	if (!player || !classname || !classname[0])
		return false;

	edict_t *pent = CreateIn(XPlay::OwnerOf(player), classname);
	if (!pent)
		return false;

	pent->v.origin = player->v.origin;
	pent->v.spawnflags |= SF_NORESPAWN;

	gpGamedllFuncs->dllapi_table->pfnSpawn(pent);
	gpGamedllFuncs->dllapi_table->pfnTouch(pent, player);
	return true;
}

edict_t *Spawn(int iGame, const char *classname, const Vector &origin)
{
	if (!classname || !classname[0])
		return nullptr;

	edict_t *pent = CreateIn(iGame, classname);
	if (!pent)
		return nullptr;

	pent->v.origin = origin;
	pent->v.spawnflags |= SF_NORESPAWN;

	gpGamedllFuncs->dllapi_table->pfnSpawn(pent);

	if (pent->free)
		return nullptr;

	return pent;
}

}
