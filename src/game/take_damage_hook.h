#pragma once

#ifdef CROSSBOT_DPS

#include <extdll.h>

namespace TakeDamageHook
{

typedef void (*Callback)(edict_t *victim, edict_t *attacker, edict_t *inflictor,
	float damage, int bits);

bool Install(Callback cb);

void Remove();

}

#endif
