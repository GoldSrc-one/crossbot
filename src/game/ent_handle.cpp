#include <extdll.h>
#include <meta_api.h>

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

bool CBotIsLivePlayer(const edict_t *ent)
{
	return ent && !ent->free && ent->pvPrivateData && (ent->v.flags & FL_CLIENT) != 0;
}
