#include <extdll.h>
#include <meta_api.h>
#include <algorithm>
#include <in_buttons.h>

#include <cmath>
#include <cstdio>

#include "bot_break.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_weapons.h"
#include "../game/game_profile.h"
#include "../nav/nav_math.h"
#include "../nav/path_follower.h"
#include "../nav/pm_sim.h"

namespace
{
	constexpr float kBreakMaxHealth = 100.f;
	constexpr float kBreakReach = 28.f;
	constexpr float kBreakFloorReach = 31.f;
	constexpr float kBreakStandRadius = 8.f;
	constexpr float kBreakAimTolerance = 8.f;
	constexpr float kBreakNudge = 2.f;

	constexpr float kBreakSwingInterval = 0.5f;
	constexpr float kBreakSelectInterval = 0.5f;
	constexpr float kBreakNoDamageGrace = 2.f;
	constexpr float kBreakTimeout = 8.f;
	constexpr float kBreakRefuseTime = 15.f;
	constexpr float kBreakRetreatTime = 4.f;

	constexpr float kBreakMoveDeadzone = 0.01f;

	using NavMath::kPiF;
	using NavMath::Norm2D;


	Vector NearestPoint(const Vector &from, const Vector &mins, const Vector &maxs)
	{
		Vector center = (mins + maxs) * 0.5f;
		Vector p(std::clamp(from.x, mins.x, maxs.x), std::clamp(from.y, mins.y, maxs.y),
			std::clamp(from.z, mins.z, maxs.z));

		Vector inward = center - p;
		float len = inward.Length();
		if (len <= kBreakNudge)
			return center;

		return p + inward * (kBreakNudge / len);
	}

	bool CanHit(edict_t *pEdict, const Vector &eye, const Vector &at, edict_t *ent)
	{
		TraceResult tr;
		g_engfuncs.pfnTraceLine(eye, at, dont_ignore_monsters, pEdict, &tr);
		return tr.flFraction >= 1.f || tr.pHit == ent;
	}

	bool SolveAim(edict_t *pEdict, edict_t *ent, Vector &aimPoint, Vector &aimAngles)
	{
		Vector eye = pEdict->v.origin + pEdict->v.view_ofs;

		aimPoint = NearestPoint(eye, ent->v.absmin, ent->v.absmax);
		bool canHit = CanHit(pEdict, eye, aimPoint, ent);
		if (!canHit)
		{
			aimPoint = (ent->v.absmin + ent->v.absmax) * 0.5f;
			canHit = CanHit(pEdict, eye, aimPoint, ent);
		}

		Vector los = aimPoint - eye;
		float flat = sqrtf(los.x * los.x + los.y * los.y);
		if (flat > 0.0001f || fabsf(los.z) > 0.0001f)
			aimAngles = Vector(-atan2f(los.z, flat) * (180.f / kPiF),
				atan2f(los.y, los.x) * (180.f / kPiF), 0.f);

		return canHit;
	}

	int ActiveWeaponId(const CBot &bot)
	{
		const BotInventory &inv = bot.inventory;
		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
			return 0;
		return inv.weapons[inv.activeIndex].id;
	}

	bool Carries(const CBot &bot, int weaponId)
	{
		const BotInventory &inv = bot.inventory;
		for (int i = 0; i < inv.count; ++i)
		{
			if (inv.weapons[i].id == weaponId)
				return true;
		}
		return false;
	}

	int WantWeaponId(const CBot &bot)
	{
		if (!g_pGameProfile)
			return ActiveWeaponId(bot);

		int melee = g_pGameProfile->MeleeWeapon();

		if (bot.brk.standoff > 0.f)
		{
			int active = ActiveWeaponId(bot);
			if (active != 0 && !g_pGameProfile->IsMeleeWeapon(active))
				return active;

			const BotInventory &inv = bot.inventory;
			for (int i = 0; i < inv.count; ++i)
			{
				if (inv.weapons[i].id != 0 && !g_pGameProfile->IsMeleeWeapon(inv.weapons[i].id))
					return inv.weapons[i].id;
			}
			return active;
		}

		if (melee != 0 && Carries(bot, melee))
			return melee;

		const BotInventory &inv = bot.inventory;
		for (int i = 0; i < inv.count; ++i)
		{
			if (inv.weapons[i].id != 0 && g_pGameProfile->IsMeleeWeapon(inv.weapons[i].id))
				return inv.weapons[i].id;
		}
		return ActiveWeaponId(bot);
	}

	const char *SelectWeapon(CBot &bot, int weaponId)
	{
		if (weaponId == 0 || !g_pGameProfile)
			return NULL;

		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(weaponId);
		if (!spec || !spec->classname)
			return NULL;

		return g_pGameProfile->SelectWeapon(bot.pEdict, *spec) ? spec->classname : NULL;
	}

	void RestoreWeapon(CBot &bot)
	{
		BotBreak::State &st = bot.brk;
		if (st.prevWeaponId == 0 || st.prevWeaponId == ActiveWeaponId(bot))
			return;
		if (!Carries(bot, st.prevWeaponId))
			return;

		const char *drawn = SelectWeapon(bot, st.prevWeaponId);
		if (!drawn)
			return;

		BOT_VLOG("CrossBot: %s went back to %s after %s %s\n", bot.name, drawn,
			st.standoff > 0.f ? "shooting" : "breaking", st.targetName);
	}

	void Report(CBot &bot, const char *verb)
	{
		BOT_VLOG("CrossBot: %s %s %s after %.1fs and %d swings\n", bot.name, verb,
			bot.brk.targetName, gpGlobals->time - bot.brk.startTime, bot.brk.swings);
	}

	void Finish(CBot &bot)
	{
		Report(bot, bot.brk.standoff > 0.f ? "destroyed" : "broke");
		RestoreWeapon(bot);

		BotBreak::State &st = bot.brk;
		st.active = false;
		st.standoff = 0.f;
		st.target.Clear();

		bot.nextRepathTime = gpGlobals->time;
	}

	void Abort(CBot &bot, const char *why)
	{
		Report(bot, why);
		RestoreWeapon(bot);

		BotBreak::State &st = bot.brk;
		st.refused = st.target;
		st.refusedUntil = gpGlobals->time + kBreakRefuseTime;
		st.active = false;
		st.standoff = 0.f;
		st.target.Clear();

		Bot_PenalizeCurrentConnection(bot);
		bot.nextRepathTime = gpGlobals->time;
	}
}

namespace BotBreak
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_nav_break");
	return !cvar || cvar->value != 0.f;
}

bool IsTarget(edict_t *ent)
{
	if (!ent || ent->free)
		return false;
	if (g_engfuncs.pfnIndexOfEdict(ent) == 0)
		return false;
	if (ent->v.solid != SOLID_BSP && ent->v.solid != SOLID_BBOX)
		return false;
	if (ent->v.movetype != MOVETYPE_PUSH && ent->v.movetype != MOVETYPE_PUSHSTEP)
		return false;
	if ((ent->v.flags & (FL_CLIENT | FL_MONSTER)) != 0)
		return false;
	if (ent->v.takedamage == DAMAGE_NO)
		return false;

	return ent->v.health > 0.f && ent->v.health <= kBreakMaxHealth;
}

bool Active(const CBot &bot)
{
	return bot.brk.active;
}

PhaseThrough::PhaseThrough()
{
	if (!Enabled())
		return;

	for (int i = 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!IsTarget(ent))
			continue;

		m_saved.push_back(std::make_pair(ent, static_cast<int>(ent->v.solid)));
		ent->v.solid = SOLID_NOT;
	}
}

PhaseThrough::~PhaseThrough()
{
	for (size_t i = 0; i < m_saved.size(); ++i)
	{
		if (!m_saved[i].first->free)
			m_saved[i].first->v.solid = m_saved[i].second;
	}
}

bool AimSwing(CBot &bot, edict_t *ent, Vector &aimAngles, bool &swing)
{
	swing = false;

	if (!Enabled() || !IsTarget(ent))
		return false;

	edict_t *pEdict = bot.pEdict;
	Vector eye = pEdict->v.origin + pEdict->v.view_ofs;
	Vector aimPoint;

	aimAngles = bot.desiredAngles;
	if (!SolveAim(pEdict, ent, aimPoint, aimAngles))
		return false;

	if ((aimPoint - eye).Length() > kBreakReach)
		return true;

	const Vector &view = pEdict->v.v_angle;
	if (fabsf(Bot_NormalizeAngle(aimAngles.y - view.y)) > kBreakAimTolerance ||
		fabsf(Bot_NormalizeAngle(aimAngles.x - view.x)) > kBreakAimTolerance)
		return true;

	State &st = bot.brk;
	if (bot.inventory.nextAttack > 0.f || gpGlobals->time < st.nextSwingTime)
		return true;

	swing = true;
	st.nextSwingTime = gpGlobals->time + kBreakSwingInterval;
	return true;
}

bool EngageStandoff(CBot &bot, edict_t *ent, float minRange)
{
	State &st = bot.brk;

	if (st.active)
		return st.target.Is(ent);

	if (!Enabled() || !g_pGameProfile || !ent || ent->free)
		return false;

	if (st.refused.Is(ent) && gpGlobals->time < st.refusedUntil)
		return false;

	st.target.Set(ent);
	st.active = true;
	st.standoff = minRange;
	st.prevWeaponId = ActiveWeaponId(bot);
	st.swings = 0;
	st.startTime = gpGlobals->time;
	st.lastDamageTime = gpGlobals->time;
	st.lastHealth = ent->v.health;
	st.nextSelectTime = 0.f;
	st.nextSwingTime = 0.f;
	snprintf(st.targetName, sizeof(st.targetName), "%s", CBotClassOf(ent));

	Vector center = (ent->v.absmin + ent->v.absmax) * 0.5f;
	BOT_VLOG("CrossBot: %s is shooting %s at (%.1f,%.1f,%.1f) from %.0f units\n", bot.name,
		st.targetName, center.x, center.y, center.z, minRange);

	return true;
}

bool Engage(CBot &bot, edict_t *ent)
{
	State &st = bot.brk;

	if (st.active)
		return st.target.Is(ent);

	if (!Enabled() || !g_pGameProfile || !IsTarget(ent))
		return false;

	if (st.refused.Is(ent) && gpGlobals->time < st.refusedUntil)
		return false;

	st.target.Set(ent);
	st.active = true;
	st.standoff = 0.f;
	st.prevWeaponId = ActiveWeaponId(bot);
	st.swings = 0;
	st.startTime = gpGlobals->time;
	st.lastDamageTime = gpGlobals->time;
	st.lastHealth = ent->v.health;
	st.nextSelectTime = 0.f;
	st.nextSwingTime = 0.f;
	snprintf(st.targetName, sizeof(st.targetName), "%s", CBotClassOf(ent));

	BOT_VLOG("CrossBot: %s is breaking %s at (%.1f,%.1f,%.1f), health %.0f\n", bot.name,
		st.targetName, ent->v.absmin.x, ent->v.absmin.y, ent->v.absmin.z, ent->v.health);

	return true;
}

bool Run(CBot &bot)
{
	State &st = bot.brk;
	if (!st.active)
		return false;

	if (!Enabled() || !g_pGameProfile)
	{
		st.active = false;
		st.target.Clear();
		return false;
	}

	edict_t *pEdict = bot.pEdict;
	edict_t *ent = st.target.Get();
	float now = gpGlobals->time;

	if (!ent || ent->v.solid == SOLID_NOT || ent->v.health <= 0.f)
	{
		Finish(bot);
		return false;
	}

	bot.lastProgressTime = now;

	if (ent->v.health < st.lastHealth)
		st.lastDamageTime = now;
	st.lastHealth = ent->v.health;

	if (now - st.startTime > kBreakTimeout)
	{
		Abort(bot, "gave up breaking");
		return false;
	}


	const Vector &origin = pEdict->v.origin;
	Vector eye = origin + pEdict->v.view_ofs;
	Vector aimPoint;
	bool canHit = SolveAim(pEdict, ent, aimPoint, bot.desiredAngles);

	Vector angles = Bot_UpdateFacing(bot);

	Vector los = aimPoint - eye;
	bool standoff = st.standoff > 0.f;
	bool underfoot = !standoff && pEdict->v.groundentity == ent;
	bool inRange = canHit && (standoff ? los.Length() >= st.standoff
		: los.Length() <= (underfoot ? kBreakFloorReach : kBreakReach));
	bool aimed =
		fabsf(Bot_NormalizeAngle(bot.desiredAngles.y - angles.y)) <= kBreakAimTolerance &&
		fabsf(Bot_NormalizeAngle(bot.desiredAngles.x - angles.x)) <= kBreakAimTolerance;

	Vector to(aimPoint.x - origin.x, aimPoint.y - origin.y, 0.f);
	float dist = sqrtf(to.x * to.x + to.y * to.y);

	if (standoff && !inRange && now - st.startTime > kBreakRetreatTime)
		inRange = canHit;

	float drive = 0.f;
	Vector moveDir(0.f, 0.f, 0.f);
	if (!inRange && (standoff || dist > kBreakStandRadius))
	{
		moveDir = Norm2D(to);
		if (standoff)
			moveDir = moveDir * -1.f;
		drive = Bot_MaxSpeed(pEdict) * PmSim::kDuckingMultiplier;

		NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
			Bot_DrivePhysics(pEdict));
		if (solved.wishSpeed > 0.f)
		{
			moveDir = solved.wishDir;
			drive = solved.wishSpeed;
		}
	}

	if ((pEdict->v.flags & FL_DUCKING) != 0)
		drive /= PmSim::kDuckingMultiplier;

	float fmove = 0.f, smove = 0.f;
	Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, drive, fmove, smove);
	bot.moveForward = fmove;
	bot.moveSide = smove;

	unsigned short buttons = 0;
	if (fmove > kBreakMoveDeadzone) buttons |= IN_FORWARD;
	else if (fmove < -kBreakMoveDeadzone) buttons |= IN_BACK;
	if (smove > kBreakMoveDeadzone) buttons |= IN_MOVERIGHT;
	else if (smove < -kBreakMoveDeadzone) buttons |= IN_MOVELEFT;
	if (underfoot) buttons |= IN_DUCK;

	int wantId = WantWeaponId(bot);
	int activeId = ActiveWeaponId(bot);

	if (wantId != 0 && wantId != activeId)
	{
		if (now >= st.nextSelectTime)
		{
			if (const char *drawn = SelectWeapon(bot, wantId))
			{
				BOT_VLOG("CrossBot: %s drew %s to break %s\n", bot.name, drawn, st.targetName);
			}
			st.nextSelectTime = now + kBreakSelectInterval;
		}
	}
	else if (aimed && inRange && bot.inventory.nextAttack <= 0.f && now >= st.nextSwingTime)
	{
		buttons |= IN_ATTACK;
		st.nextSwingTime = now + kBreakSwingInterval;
		++st.swings;
	}

	if (!aimed || !inRange)
		st.lastDamageTime = now;

	if (st.swings > 0 && now - st.lastDamageTime > kBreakNoDamageGrace)
	{
		Abort(bot, "could not damage");
		return false;
	}

	Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
	return true;
}

}
