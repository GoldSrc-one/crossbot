#include <extdll.h>
#include <meta_api.h>

#ifdef CROSSBOT_DPS

#include <stdio.h>

#include "take_damage_hook.h"
#include "gamedata.h"

#if defined(_WIN32)

#include <windows.h>

namespace
{
	typedef int (__fastcall *TakeDamageFn)(void *self, void *edx, entvars_t *pevInflictor,
		entvars_t *pevAttacker, float flDamage, int bitsDamageType);

	TakeDamageFn g_original = nullptr;
	void **g_slot = nullptr;
	TakeDamageHook::Callback g_callback = nullptr;

	edict_t *EdictOf(entvars_t *pev)
	{
		return pev ? pev->pContainingEntity : nullptr;
	}

	int __fastcall Detour(void *self, void *edx, entvars_t *pevInflictor, entvars_t *pevAttacker,
		float flDamage, int bitsDamageType)
	{
		entvars_t *pev = self ? GameData::Peek<entvars_t *>(self, g_offsets.pev) : nullptr;
		float before = pev ? pev->health : 0.f;

		int result = g_original(self, edx, pevInflictor, pevAttacker, flDamage, bitsDamageType);

		if (g_callback && pev)
		{
			edict_t *victim = EdictOf(pev);
			bool fatal = before > 0.f && pev->health <= 0.f;
			if (victim)
				g_callback(victim, EdictOf(pevAttacker), EdictOf(pevInflictor), flDamage,
					bitsDamageType, fatal);
		}

		return result;
	}

	bool Patch(void **slot, void *value, void **previous)
	{
		DWORD old = 0;
		if (!VirtualProtect(slot, sizeof(void *), PAGE_READWRITE, &old))
			return false;

		if (previous)
			*previous = *slot;
		*slot = value;

		DWORD ignored = 0;
		VirtualProtect(slot, sizeof(void *), old, &ignored);
		return true;
	}
}

namespace TakeDamageHook
{

bool Install(Callback cb)
{
	g_callback = cb;

	if (g_slot)
		return true;

	edict_t *probe = g_engfuncs.pfnCreateNamedEntity(g_engfuncs.pfnAllocString("player"));
	if (!probe || !probe->pvPrivateData)
	{
		SERVER_PRINT("CrossBot: TakeDamage hook failed, cannot create a player probe entity.\n");
		if (probe)
			g_engfuncs.pfnRemoveEntity(probe);
		return false;
	}

	void **vtable = *reinterpret_cast<void ***>(probe->pvPrivateData);
	g_engfuncs.pfnRemoveEntity(probe);

	if (!vtable)
	{
		SERVER_PRINT("CrossBot: TakeDamage hook failed, the player probe carried no vtable.\n");
		return false;
	}

	void **slot = &vtable[g_offsets.vt_TakeDamage];
	void *previous = nullptr;
	if (!Patch(slot, reinterpret_cast<void *>(&Detour), &previous))
	{
		SERVER_PRINT("CrossBot: TakeDamage hook failed, the vtable page refused to unprotect.\n");
		return false;
	}

	g_original = reinterpret_cast<TakeDamageFn>(previous);
	g_slot = slot;

	char msg[160];
	snprintf(msg, sizeof(msg), "CrossBot: hooked CBasePlayer::TakeDamage at vtable index %d.\n",
		g_offsets.vt_TakeDamage);
	SERVER_PRINT(msg);
	return true;
}

void Remove()
{
	g_callback = nullptr;

	if (!g_slot)
		return;

	Patch(g_slot, reinterpret_cast<void *>(g_original), nullptr);
	g_slot = nullptr;
	g_original = nullptr;
}

}

#else

namespace TakeDamageHook
{

bool Install(Callback cb)
{
	(void)cb;
	SERVER_PRINT("CrossBot: the TakeDamage hook is Windows-only, dps measurement is unavailable.\n");
	return false;
}

void Remove()
{
}

}

#endif

#endif
