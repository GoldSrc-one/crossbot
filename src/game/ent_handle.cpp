#include <extdll.h>
#include <meta_api.h>

#include "crossplay.h"
#include "ent_handle.h"

const char *CBotPlayerNameOf(const edict_t *ent)
{
	if (!ent || !gpGlobals->pStringBase)
		return "?";
	const char *s = gpGlobals->pStringBase + static_cast<unsigned int>(ent->v.netname);
	return (s && s[0]) ? s : "?";
}

const char *CBotTargetNameOf(const edict_t *ent)
{
	if (!ent)
		return "?";
	if ((ent->v.flags & FL_CLIENT) != 0)
		return CBotPlayerNameOf(ent);
	return CBotClassOf(ent);
}

void *CBotPrivateData(const edict_t *ent)
{
	return XPlay::PrivateData(ent, XPlay::Current());
}

bool CBotIsLivePlayer(const edict_t *ent)
{
	if (!ent || ent->free || (ent->v.flags & FL_CLIENT) == 0)
		return false;

	return XPlay::PrivateData(ent, XPlay::OwnerOf(ent)) != nullptr;
}
