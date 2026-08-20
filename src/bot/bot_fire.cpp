#include <extdll.h>
#include <meta_api.h>

#include <algorithm>

#include "bot_fire.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_weapons.h"
#include "../game/fake_cmd.h"
#include "../game/game_profile.h"
#include "../nav/nav_math.h"

namespace
{
	using NavMath::kPiF;

	constexpr float kHitscanRange = 8192.f;

	constexpr float kSelectInterval = 0.5f;

	constexpr int kMaxChoices = 64;

	struct Choice
	{
		int index;
		BotFire::Mode mode;
	};

	float RandomUnit(BotFire::State &fire)
	{
		fire.rngState = fire.rngState * 1664525u + 1013904223u;
		return static_cast<float>((fire.rngState >> 8) & 0xFFFFFFu) / 16777216.f;
	}

	void SeedFire(CBot &bot)
	{
		BotFire::State &fire = bot.fire;
		if (fire.rngSeeded)
			return;

		fire.rngSeeded = true;
		fire.rngState = static_cast<unsigned int>(
			g_engfuncs.pfnIndexOfEdict(bot.pEdict)) * 2654435761u + 2131u;
	}

	bool Slab(float origin, float dir, float lo, float hi, float &tmin, float &tmax)
	{
		if (fabsf(dir) < 1e-6f)
			return origin >= lo && origin <= hi;

		float inv = 1.f / dir;
		float t1 = (lo - origin) * inv;
		float t2 = (hi - origin) * inv;
		if (t1 > t2)
			std::swap(t1, t2);

		if (t1 > tmin)
			tmin = t1;
		if (t2 < tmax)
			tmax = t2;

		return tmin <= tmax;
	}

	bool RayHitsBox(const Vector &origin, const Vector &dir, const Vector &centre, const Vector &half,
		float range, float &entry)
	{
		float tmin = 0.f;
		float tmax = range;

		if (!Slab(origin.x, dir.x, centre.x - half.x, centre.x + half.x, tmin, tmax))
			return false;
		if (!Slab(origin.y, dir.y, centre.y - half.y, centre.y + half.y, tmin, tmax))
			return false;
		if (!Slab(origin.z, dir.z, centre.z - half.z, centre.z + half.z, tmin, tmax))
			return false;

		entry = tmin;
		return true;
	}

	int AmmoCount(const BotInventory &inv, int type)
	{
		if (type < 0 || type >= MAX_AMMO_SLOTS)
			return 0;
		return inv.ammo[type];
	}

	void CountShots(CBot &bot)
	{
		BotFire::State &fire = bot.fire;
		const BotInventory &inv = bot.inventory;

		bool pressed = fire.pressed;
		fire.pressed = false;

		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
		{
			fire.lastTimerId = 0;
			return;
		}

		const BotWeapon &w = inv.weapons[inv.activeIndex];

		if (pressed && fire.lastTimerId == w.id)
		{
			if (fire.lastPrimaryTimer <= 0.f && w.nextPrimaryAttack > 0.f)
				++fire.shots;
			else if (fire.lastSecondaryTimer <= 0.f && w.nextSecondaryAttack > 0.f)
				++fire.shots;
		}

		fire.lastTimerId = w.id;
		fire.lastPrimaryTimer = w.nextPrimaryAttack;
		fire.lastSecondaryTimer = w.nextSecondaryAttack;
	}

	void Idle(BotFire::State &fire, const char *reason)
	{
		fire.mode = BotFire::MODE_NONE;
		fire.weaponId = 0;
		fire.wantReload = false;
		fire.lastReason = reason;
	}
}

namespace BotFire
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_fire");
	return cvar && cvar->value != 0.f;
}

const char *ModeName(Mode mode)
{
	switch (mode)
	{
	case MODE_NONE: return "none";
	case MODE_PRIMARY: return "primary";
	case MODE_SECONDARY: return "secondary";
	}
	return "?";
}

bool Supported(const WeaponMode &mode)
{
	if (mode.fire != FIRE_HOLD || mode.aim != AIM_TARGET || mode.toggle != TOGGLE_NONE)
		return false;
	if (mode.speed != 0.f || mode.gravity != 0.f || mode.splash != 0.f)
		return false;
	return (mode.quirks & (WPN_Q_HIDDEN_STATE | WPN_Q_KEEP_AIMING)) == 0;
}

const WeaponMode *ModeOf(const WeaponSpec &spec, Mode mode)
{
	if (mode == MODE_PRIMARY)
		return &spec.primary;
	if (mode == MODE_SECONDARY)
		return &spec.secondary;
	return nullptr;
}

bool ModeUsable(const CBot &bot, const BotWeapon &w, const WeaponMode &m, float dist)
{
	if (!Supported(m))
		return false;

	if ((m.quirks & WPN_Q_NO_UNDERWATER) != 0 && bot.pEdict->v.waterlevel == 3)
		return false;

	float range = (m.range > 0.f) ? m.range : kHitscanRange;
	if (dist > range)
		return false;

	const BotInventory &inv = bot.inventory;
	int need = (m.ammoPerShot > 0) ? m.ammoPerShot : 1;

	switch (m.ammo)
	{
	case AMMO_NONE:
		return true;
	case AMMO_CLIP:
		if (w.clip >= need)
			return true;
		return AmmoCount(inv, w.primaryAmmoType) > 0;
	case AMMO_PRIMARY:
		return AmmoCount(inv, w.primaryAmmoType) >= need;
	case AMMO_SECONDARY:
		return AmmoCount(inv, w.secondaryAmmoType) >= need;
	}

	return false;
}

void Reset(CBot &bot)
{
	bot.fire = State();
}

void Think(CBot &bot)
{
	State &fire = bot.fire;

	fire.lastMiss = -1.f;
	fire.lastTol = -1.f;

	CountShots(bot);

	if (!Enabled() || !g_pGameProfile)
	{
		Idle(fire, "off");
		return;
	}

	SeedFire(bot);

	edict_t *target = bot.sense.target.Get();
	if (target)
	{
		fire.targetSize = target->v.maxs - target->v.mins;
		fire.haveSize = true;
	}

	Vector aimPoint;
	if (!fire.haveSize || !BotAim::ReadDelayed(bot, BotAim::Delay(), aimPoint))
	{
		Idle(fire, "no-target");
		return;
	}

	Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
	float dist = (aimPoint - eye).Length();

	const BotInventory &inv = bot.inventory;

	if (inv.activeIndex >= 0 && inv.activeIndex < inv.count)
	{
		const BotWeapon &active = inv.weapons[inv.activeIndex];
		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(active.id);
		if (spec)
		{
			Mode order[2] = { MODE_PRIMARY, MODE_SECONDARY };
			if (fire.weaponId == active.id && fire.mode == MODE_SECONDARY)
			{
				order[0] = MODE_SECONDARY;
				order[1] = MODE_PRIMARY;
			}

			for (Mode candidate : order)
			{
				const WeaponMode *m = ModeOf(*spec, candidate);
				if (!m || !ModeUsable(bot, active, *m, dist))
					continue;

				int need = (m->ammoPerShot > 0) ? m->ammoPerShot : 1;

				bool wasReload = fire.wantReload;

				fire.mode = candidate;
				fire.weaponId = active.id;
				fire.wantReload = m->ammo == AMMO_CLIP && active.clip >= 0 && active.clip < need &&
					!active.reloading && AmmoCount(inv, active.primaryAmmoType) > 0;
				fire.lastReason = fire.wantReload ? "reloading" : "ready";

				if (fire.wantReload && !wasReload)
					++fire.reloads;

				return;
			}
		}
	}

	if (gpGlobals->time < fire.nextSelectTime)
	{
		Idle(fire, "switching");
		return;
	}

	Choice choices[kMaxChoices];
	int count = 0;

	for (int i = 0; i < inv.count && count < kMaxChoices; ++i)
	{
		const BotWeapon &w = inv.weapons[i];
		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
		if (!spec)
			continue;

		if (count < kMaxChoices && ModeUsable(bot, w, spec->primary, dist))
		{
			choices[count].index = i;
			choices[count].mode = MODE_PRIMARY;
			++count;
		}
		if (count < kMaxChoices && ModeUsable(bot, w, spec->secondary, dist))
		{
			choices[count].index = i;
			choices[count].mode = MODE_SECONDARY;
			++count;
		}
	}

	if (count == 0)
	{
		Idle(fire, "nothing-usable");
		return;
	}

	int pick = static_cast<int>(RandomUnit(fire) * count);
	if (pick >= count)
		pick = count - 1;

	const BotWeapon &want = inv.weapons[choices[pick].index];
	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(want.id);

	fire.nextSelectTime = gpGlobals->time + kSelectInterval;
	fire.pendingId = want.id;

	Idle(fire, "switching");

	if (spec && spec->classname && FakeCmd::Run(bot.pEdict, spec->classname))
		++fire.switches;
}

unsigned short Buttons(CBot &bot, const Vector &angles)
{
	State &fire = bot.fire;

	if (!Enabled() || fire.mode == MODE_NONE || !g_pGameProfile)
		return 0;

	const BotInventory &inv = bot.inventory;
	if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
		return 0;

	const BotWeapon &w = inv.weapons[inv.activeIndex];
	if (w.id != fire.weaponId)
		return 0;

	if (fire.wantReload)
		return IN_RELOAD;

	if (w.reloading || inv.nextAttack > 0.f)
		return 0;

	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
	if (!spec)
		return 0;

	const WeaponMode *m = ModeOf(*spec, fire.mode);
	if (!m)
		return 0;

	int need = (m->ammoPerShot > 0) ? m->ammoPerShot : 1;
	if (m->ammo == AMMO_CLIP && w.clip >= 0 && w.clip < need)
	{
		fire.lastReason = "empty";
		return 0;
	}

	Vector aimPoint;
	if (!BotAim::ReadDelayed(bot, BotAim::Delay(), aimPoint))
		return 0;

	edict_t *pEdict = bot.pEdict;
	Vector eye = pEdict->v.origin + pEdict->v.view_ofs;

	MAKE_VECTORS(angles + pEdict->v.punchangle);
	Vector dir = gpGlobals->v_forward;

	Vector half = fire.targetSize * 0.5f;
	Vector los = aimPoint - eye;
	float dist = los.Length();

	if (dist > 0.001f)
	{
		float dot = DotProduct(dir, los * (1.f / dist));
		if (dot > 1.f)
			dot = 1.f;
		else if (dot < -1.f)
			dot = -1.f;

		fire.lastMiss = acosf(dot) * (180.f / kPiF);

		float big = std::max(half.x, std::max(half.y, half.z));
		fire.lastTol = atanf(big / dist) * (180.f / kPiF);
	}

	float range = (m->range > 0.f) ? m->range : kHitscanRange;

	float entry = 0.f;
	if (!RayHitsBox(eye, dir, aimPoint, half, range, entry))
	{
		fire.lastReason = "off-target";
		return 0;
	}

	TraceResult tr;
	TRACE_LINE(eye, eye + dir * range, ignore_monsters, pEdict, &tr);
	if (tr.flFraction * range < entry)
	{
		fire.lastReason = "blocked";
		return 0;
	}

	fire.lastReason = "firing";
	fire.pressed = true;
	return (fire.mode == MODE_SECONDARY) ? IN_ATTACK2 : IN_ATTACK;
}

}
