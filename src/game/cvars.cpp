#include <extdll.h>
#include <meta_api.h>

#include <cstdint>
#include <cstring>

#include "cvars.h"

namespace
{
	constexpr size_t kSlots = 256;

	struct Slot
	{
		const char *key = nullptr;
		cvar_t *cvar = nullptr;
	};

	Slot g_slots[kSlots];

	size_t Hash(const char *name)
	{
		return (reinterpret_cast<uintptr_t>(name) >> 2) & (kSlots - 1);
	}
}

cvar_t *CBotCvar(const char *name)
{
	if (!name)
		return nullptr;

	size_t h = Hash(name);
	for (size_t probe = 0; probe < kSlots; ++probe)
	{
		Slot &slot = g_slots[(h + probe) & (kSlots - 1)];
		if (slot.key == name)
			return strcmp(slot.cvar->name, name) == 0 ? slot.cvar : g_engfuncs.pfnCVarGetPointer(name);
		if (!slot.key)
		{
			cvar_t *cvar = g_engfuncs.pfnCVarGetPointer(name);
			if (cvar)
			{
				slot.key = name;
				slot.cvar = cvar;
			}
			return cvar;
		}
	}
	return g_engfuncs.pfnCVarGetPointer(name);
}

float CBotCvarFloat(const char *name)
{
	cvar_t *cvar = CBotCvar(name);
	return cvar ? cvar->value : 0.f;
}

const char *CBotCvarString(const char *name)
{
	cvar_t *cvar = CBotCvar(name);
	return cvar && cvar->string ? cvar->string : "";
}
