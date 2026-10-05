#include <extdll.h>
#include <meta_api.h>

#include <math.h>

#include "../game/game_profile.h"
#include "../game/teams.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_query.h"
#include "../nav/nav_vantage.h"
#include "../nav/path_follower.h"
#include "bot.h"
#include "bot_combat.h"
#include "bot_difficulty.h"
#include "bot_fun.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sense.h"

extern IGameProfile *g_pGameProfile;

namespace
{
	using NavMath::kPiF;
	using NavMath::Norm2D;

	constexpr float kSightTimeout = 5.f;

	constexpr float kEnemyViewCone = 90.f;

	constexpr float kProbeDist = 160.f;
	constexpr float kProbeInterval = 0.7f;
	constexpr float kProbeReached = 32.f;

	const float kProbeFractions[] = {1.f, 0.7f, 0.45f, 0.25f};

	constexpr float kAreaStandLift = 18.f;

	constexpr float kMoveButtonDeadzone = 0.01f;

	constexpr float kDuelSameFloor = 72.f;

	constexpr float kDuelBackoff = 0.4f;

	constexpr float kDuelCloseFrac = 0.8f;

	struct Spot
	{
		Vector origin;
		uint32_t areaId = 0;
	};

	float NextRandom(BotCombat::State &st)
	{
		st.rngState = st.rngState * 1664525u + 1013904223u;
		return static_cast<float>((st.rngState >> 8) & 0xFFFFFFu) / 16777216.f;
	}

	void Seed(CBot &bot)
	{
		BotCombat::State &st = bot.combat;
		if (st.rngSeeded)
			return;

		st.rngSeeded = true;
		st.rngState = static_cast<unsigned int>(
			g_engfuncs.pfnIndexOfEdict(bot.pEdict)) * 2654435761u + 12345u;
	}

	bool TestArea(const Nav::Area &area, const Vector &xy, Spot &out)
	{
		Vector spot(xy.x, xy.y, area.centroid.z);
		if (!Nav::AreaContainsPoint(area, spot))
			return false;

		bool crouch = (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		out.origin = crouch ? spot : spot + Vector(0.f, 0.f, kAreaStandLift);
		out.areaId = area.id;
		return true;
	}

	bool ResolveSpot(const Nav::NavMesh &mesh, const CBot &bot, const Vector &xy, Spot &out)
	{
		if (bot.currentAreaId < 0)
			return false;

		const Nav::Area *cur = Nav::FindAreaById(mesh, static_cast<uint32_t>(bot.currentAreaId));
		if (!cur)
			return false;

		if (TestArea(*cur, xy, out))
			return true;

		for (uint32_t ci : cur->connIndices)
		{
			if (ci >= mesh.conns.size())
				continue;

			const Nav::Connection &conn = mesh.conns[ci];
			if (conn.connType != crossbot_nav::CONN_WALK)
				continue;
			if (!crossbot_nav::ConnAllowsDirection(conn.connFlags, conn.areaA, cur->id))
				continue;

			uint32_t other = (conn.areaA == cur->id) ? conn.areaB : conn.areaA;
			const Nav::Area *area = Nav::FindAreaById(mesh, other);
			if (area && TestArea(*area, xy, out))
				return true;
		}

		return false;
	}

	bool ResolveAlongRay(const Nav::NavMesh &mesh, const CBot &bot, const Vector &from,
		const Vector &dir, Spot &out)
	{
		for (float f : kProbeFractions)
			if (ResolveSpot(mesh, bot, from + dir * (kProbeDist * f), out))
				return true;

		return false;
	}

	bool SampleExposure(const Nav::NavMesh &mesh, const CBot &bot, const Vector &origin, float &out)
	{
		Nav::VantageBlend blend;
		if (!Nav::BlendVantage(mesh.vantageField, origin + bot.pEdict->v.view_ofs, blend))
			return false;
		if (!blend.valid)
			return false;

		out = blend.exposure;
		return true;
	}

	bool EnemySees(const CBot &bot, edict_t *enemy)
	{
		if ((enemy->v.flags & FL_CLIENT) == 0)
			return true;

		Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
		Vector theirEye = enemy->v.origin + enemy->v.view_ofs;

		Vector los = eye - theirEye;
		float dist = los.Length();
		if (dist < 0.001f)
			return true;

		Vector theirForward, theirRight, theirUp;
		g_engfuncs.pfnAngleVectors(enemy->v.v_angle, theirForward, theirRight, theirUp);

		float fov = enemy->v.fov;
		if (fov <= 0.f)
			fov = kEnemyViewCone;

		return DotProduct(theirForward, los * (1.f / dist)) >= cosf(fov * 0.5f * kPiF / 180.f);
	}

	void SetAware(CBot &bot, const char *how)
	{
		BotCombat::State &st = bot.combat;
		if (st.enemyAware)
			return;

		st.enemyAware = true;

		bot.fire.nextSelectTime = 0.f;

		if (BotCombat::DebugEnabled())
			BOT_VLOG("CrossBot: %s is spotted (%s), ambush off\n", bot.name, how);
	}

	bool IsMelee(int id)
	{
		return id != 0 && g_pGameProfile && g_pGameProfile->IsMeleeWeapon(id);
	}

	bool EnemyInOurArea(const CBot &bot, edict_t *enemy)
	{
		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh || bot.currentAreaId < 0)
			return false;

		const Nav::Area *cur = Nav::FindAreaById(*mesh, static_cast<uint32_t>(bot.currentAreaId));
		if (!cur)
			return false;

		Vector spot(enemy->v.origin.x, enemy->v.origin.y, cur->centroid.z);
		if (!Nav::AreaContainsPoint(*cur, spot))
			return false;

		return fabsf(enemy->v.origin.z - bot.pEdict->v.origin.z) <= kDuelSameFloor;
	}

	void UpdateDuel(CBot &bot, edict_t *enemy, bool visible)
	{
		BotCombat::State &st = bot.combat;

		if (visible)
			st.enemyMelee = (enemy->v.flags & FL_CLIENT) != 0 &&
				IsMelee(BotWeapons::ActiveWeaponId(enemy));

		st.enemyInArea = st.enemyMelee && EnemyInOurArea(bot, enemy);

		bool now = st.enemyMelee && st.enemyInArea;
		if (now == st.dueling)
			return;

		st.dueling = now;

		bot.fire.nextSelectTime = 0.f;

		if (BotCombat::DebugEnabled())
		{
			BOT_VLOG("CrossBot: %s %s a duel with %s (melee=%d area=%d)\n", bot.name,
				now ? "enters" : "leaves", CBotTargetNameOf(enemy), st.enemyMelee ? 1 : 0,
				st.enemyInArea ? 1 : 0);
		}
	}

	void ClearDuel(CBot &bot)
	{
		BotCombat::State &st = bot.combat;
		st.enemyMelee = false;
		st.enemyInArea = false;
		st.dueling = false;
	}

	void Enter(CBot &bot, edict_t *enemy, const char *how, bool aware)
	{
		BotCombat::State &st = bot.combat;

		st.active = true;
		st.driving = false;
		st.enemy.Set(enemy);
		st.startTime = gpGlobals->time;
		st.lastSightTime = gpGlobals->time;
		st.enemyAware = aware;
		st.haveProbe = false;
		st.probeArea = -1;
		st.nextProbeTime = 0.f;
		st.lastReason = how;

		ClearDuel(bot);
		UpdateDuel(bot, enemy, bot.sense.target.Get() == enemy);

		BOT_VLOG("CrossBot: %s entered combat with %s (%s, aware=%d, melee=%d)\n", bot.name,
			CBotTargetNameOf(enemy), how, aware ? 1 : 0, st.enemyMelee ? 1 : 0);
	}

	void UpdateProbe(CBot &bot, const Nav::NavMesh &mesh)
	{
		BotCombat::State &st = bot.combat;
		edict_t *pEdict = bot.pEdict;

		if (st.haveProbe)
		{
			Vector to(st.probePoint.x - pEdict->v.origin.x, st.probePoint.y - pEdict->v.origin.y,
				0.f);
			if (to.Length() <= kProbeReached)
				st.haveProbe = false;
		}

		if (st.haveProbe && gpGlobals->time < st.nextProbeTime)
			return;

		st.nextProbeTime = gpGlobals->time + kProbeInterval;
		st.haveProbe = false;
		st.probeArea = -1;

		if (bot.currentAreaId < 0)
		{
			st.lastReason = "off the mesh";
			return;
		}

		Vector dir = Bot_YawDir(NextRandom(st) * 360.f);

		Spot spot;
		bool forward = ResolveAlongRay(mesh, bot, pEdict->v.origin, dir, spot);

		bool haveHere = bot.sense.vantage.valid;
		st.hereExposure = haveHere ? bot.sense.vantage.exposure : 0.f;

		const char *why = "no vantage";
		bool take = forward;

		float exposure = 0.f;
		if (!forward)
		{
			why = "walled off";
		}
		else if (haveHere && SampleExposure(mesh, bot, spot.origin, exposure))
		{
			if (exposure < st.hereExposure)
			{
				why = "less exposed";
			}
			else
			{
				take = false;
				why = "turned around";
			}
		}

		if (!take)
		{
			Spot back;
			if (!ResolveAlongRay(mesh, bot, pEdict->v.origin, dir * -1.f, back))
			{
				st.lastReason = forward ? "boxed in" : "nowhere to go";
				return;
			}
			spot = back;
		}

		st.haveProbe = true;
		st.probePoint = spot.origin;
		st.probeArea = static_cast<int>(spot.areaId);
		st.lastReason = why;
		if (!SampleExposure(mesh, bot, spot.origin, st.probeExposure))
			st.probeExposure = 0.f;

		if (BotCombat::DebugEnabled())
		{
			BOT_VLOG("CrossBot: %s combat probe %s to (%.0f,%.0f,%.0f) area %d, exposure %.3f "
				"here %.3f\n", bot.name, why, spot.origin.x, spot.origin.y, spot.origin.z,
				st.probeArea, st.probeExposure, st.hereExposure);
		}
	}

	int HeldId(const CBot &bot)
	{
		const BotInventory &inv = bot.inventory;
		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
			return 0;
		return inv.weapons[inv.activeIndex].id;
	}

	bool DuelDrive(CBot &bot, const Nav::NavMesh &mesh, const PmSim::PmPhysics &physics,
		Vector &moveDir, float &drive)
	{
		BotCombat::State &st = bot.combat;

		if (!BotCombat::Dueling(bot))
			return false;

		int melee = HeldId(bot);

		if (!IsMelee(melee))
			return false;

		edict_t *enemy = st.enemy.Get();
		if (!enemy)
			return false;

		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(melee);
		if (!spec)
			return false;

		edict_t *pEdict = bot.pEdict;
		Vector to(enemy->v.origin.x - pEdict->v.origin.x, enemy->v.origin.y - pEdict->v.origin.y,
			0.f);
		float dist = to.Length();

		Spot spot;
		bool backing = gpGlobals->time - bot.fire.lastShotTime < kDuelBackoff;

		if (backing)
		{
			if (dist < 0.001f)
				return false;
			if (!ResolveAlongRay(mesh, bot, pEdict->v.origin, Norm2D(to) * -1.f, spot))
				return false;
			st.lastReason = "backing off";
		}
		else if (spec->primary.range > 0.f && dist > spec->primary.range * kDuelCloseFrac)
		{
			if (!ResolveSpot(mesh, bot, enemy->v.origin, spot))
				return false;
			st.lastReason = "charging";
		}
		else
		{
			moveDir = Vector(0.f, 0.f, 0.f);
			drive = 0.f;
			st.lastReason = "swinging";
			return true;
		}

		Vector step(spot.origin.x - pEdict->v.origin.x, spot.origin.y - pEdict->v.origin.y, 0.f);
		if (step.Length() <= 1.f)
		{
			moveDir = Vector(0.f, 0.f, 0.f);
			drive = 0.f;
			return true;
		}

		moveDir = Norm2D(step);
		drive = Bot_MaxSpeed(pEdict);

		NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
			physics);
		if (solved.wishSpeed > 0.f)
		{
			moveDir = solved.wishDir;
			drive = solved.wishSpeed;
		}

		return true;
	}
}

bool BotCombat::Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_combat");
	return !cvar || cvar->value != 0.f;
}

bool BotCombat::AmbushEnabled()
{
	return BotDiff::AmbushOverride();
}

bool BotCombat::Ambushing(const CBot &bot)
{
	return bot.combat.active && !bot.combat.enemyAware && AmbushEnabled();
}

bool BotCombat::Dueling(const CBot &bot)
{
	return bot.combat.active && bot.combat.dueling;
}

bool BotCombat::DebugEnabled()
{
#if BOT_VLOG_ON
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_combat_debug");
	return cvar && cvar->value != 0.f;
#else
	return false;
#endif
}

bool BotCombat::Active(const CBot &bot)
{
	return bot.combat.active;
}

edict_t *BotCombat::Enemy(const CBot &bot)
{
	return bot.combat.enemy.Get();
}

edict_t *BotCombat::DamageSource(edict_t *pEdict)
{
	if (!pEdict || pEdict->v.dmg_take == 0.f)
		return nullptr;

	return Attacker(pEdict);
}

edict_t *BotCombat::Attacker(edict_t *pEdict)
{
	edict_t *ent = pEdict->v.dmg_inflictor;
	if (!ent || ent->free || ent == pEdict || BotSense::NoTarget(ent))
		return nullptr;
	if (g_engfuncs.pfnIndexOfEdict(ent) == 0)
		return nullptr;

	if (!CBotIsLivePlayer(ent) && g_pGameProfile && !g_pGameProfile->IsDead(ent))
	{
		edict_t *foe = g_pGameProfile->MonsterEnemy(ent);
		Teams::Relation rel = Teams::Of(pEdict, ent);
		if (foe && rel != Teams::REL_FRIEND && (rel == Teams::REL_ENEMY || foe == pEdict))
			return ent;
	}

	if (!CBotIsLivePlayer(ent))
		ent = ent->v.owner;

	if (!ent || ent->free || ent == pEdict || !CBotIsLivePlayer(ent) || BotSense::NoTarget(ent))
		return nullptr;
	if (g_pGameProfile && g_pGameProfile->IsDead(ent))
		return nullptr;
	if (!Teams::Attackable(pEdict, ent))
		return nullptr;

	return ent;
}

void BotCombat::Exit(CBot &bot, const char *why)
{
	State &st = bot.combat;
	bool was = st.active;

	edict_t *had = st.enemy.ent;
	const char *hadName = (was && had) ? CBotTargetNameOf(had) : "-";

	st.active = false;
	st.driving = false;
	st.enemy.Clear();
	st.enemyAware = false;
	st.haveProbe = false;
	st.probeArea = -1;
	st.nextProbeTime = 0.f;
	st.lastReason = why;

	ClearDuel(bot);

	if (!was)
		return;

	BOT_VLOG("CrossBot: %s left combat with %s after %.1fs: %s\n", bot.name, hadName,
		gpGlobals->time - st.startTime, why);

	bot.nextSelectTime = 0.f;
}

void BotCombat::OnRespawn(CBot &bot)
{
	Exit(bot, "respawned");
}

void BotCombat::Update(CBot &bot)
{
	State &st = bot.combat;

	if (!Enabled())
	{
		Exit(bot, "combat disabled");
		return;
	}

	if (g_pGameProfile && g_pGameProfile->HoldFire(bot))
	{
		Exit(bot, "disguised");
		return;
	}

	Seed(bot);

	edict_t *seen = bot.sense.target.Get();

	if (st.active)
	{
		edict_t *enemy = st.enemy.Get();

		if (!enemy)
			Exit(bot, "the enemy handle went stale");
		else if (g_pGameProfile && g_pGameProfile->IsDead(enemy))
			Exit(bot, "the enemy died");
		else if (BotSense::NoTarget(enemy))
			Exit(bot, "the enemy went notarget");
		else if (!Teams::Attackable(bot.pEdict, enemy))
			Exit(bot, "the enemy is on our team");
		else if ((enemy->v.flags & FL_CLIENT) == 0 && g_pGameProfile &&
			Teams::Of(bot.pEdict, enemy) != Teams::REL_ENEMY &&
			g_pGameProfile->MonsterEnemy(enemy) != bot.pEdict)
			Exit(bot, "the monster stopped hunting us");
		else if (seen && seen != enemy)
		{
			BOT_VLOG("CrossBot: %s switches from %s to %s\n", bot.name,
				CBotTargetNameOf(enemy), CBotTargetNameOf(seen));
			st.enemy.Set(seen);
			st.lastSightTime = gpGlobals->time;

			st.enemyAware = false;
			if (EnemySees(bot, seen))
				SetAware(bot, "new opponent");

			ClearDuel(bot);
			UpdateDuel(bot, seen, true);
		}
		else
		{
			if (seen == enemy)
			{
				st.lastSightTime = gpGlobals->time;

				if (!st.enemyAware && EnemySees(bot, enemy))
					SetAware(bot, "turned around");
			}

			if (!st.enemyAware && DamageSource(bot.pEdict) == enemy)
				SetAware(bot, "shot at us");

			UpdateDuel(bot, enemy, seen == enemy);

			if (!BotSense::Senses(bot, enemy))
				Exit(bot, "lost every trace of the enemy");
			else if (gpGlobals->time - st.lastSightTime > kSightTimeout)
				Exit(bot, "went too long without line of sight");
		}
	}

	if (st.active)
		return;

	if (seen)
	{
		Enter(bot, seen, "sight", EnemySees(bot, seen));
		return;
	}

	edict_t *hurt = DamageSource(bot.pEdict);
	if (hurt)
		Enter(bot, hurt, "damage", true);
}

bool BotCombat::Run(CBot &bot, const Nav::NavMesh &mesh)
{
	State &st = bot.combat;
	bool blind = Enabled() && BotDaze::Blind(bot);
	if (!st.active && !blind)
	{
		if (st.driving)
		{
			st.driving = false;
			st.haveProbe = false;
			st.probeArea = -1;
			bot.nextSelectTime = 0.f;
		}
		return false;
	}

	if (Bot_ManeuverActive(bot))
	{
		st.lastReason = "finishing a maneuver";
		return false;
	}

	if (!BotDiff::CombatInterrupt())
	{
		st.driving = false;
		st.lastReason = "difficulty";
		return false;
	}

	if (BotFun::CombatTank(bot, mesh))
	{
		st.driving = false;
		st.lastReason = "going for a tank";
		return false;
	}

	edict_t *pEdict = bot.pEdict;

	if (!st.driving)
	{
		st.driving = true;
		BotGoal::Clear(bot);
		BOT_VLOG("CrossBot: %s drops its goal to fight\n", bot.name);
	}

	bot.lastProgressTime = gpGlobals->time;

	BotSense::ApplyLook(bot);
	Vector angles = Bot_UpdateFacing(bot);

	PmSim::PmPhysics physics = Bot_DrivePhysics(pEdict);

	Vector moveDir(0.f, 0.f, 0.f);
	float drive = 0.f;

	if (DuelDrive(bot, mesh, physics, moveDir, drive))
	{
		st.haveProbe = false;
		st.probeArea = -1;
	}
	else
	{
		UpdateProbe(bot, mesh);

		if (st.haveProbe)
		{
			Vector to(st.probePoint.x - pEdict->v.origin.x,
				st.probePoint.y - pEdict->v.origin.y, 0.f);
			float dist = to.Length();
			if (dist > 1.f)
			{
				moveDir = Norm2D(to);
				drive = Bot_MaxSpeed(pEdict);

				NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive,
					pEdict->v.velocity, physics);
				if (solved.wishSpeed > 0.f)
				{
					moveDir = solved.wishDir;
					drive = solved.wishSpeed;
				}
			}
		}
	}

	if ((pEdict->v.flags & FL_DUCKING) != 0)
		drive /= PmSim::kDuckingMultiplier;
	if (drive > physics.maxSpeed)
		drive = physics.maxSpeed;

	float fmove = 0.f, smove = 0.f;
	Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, drive, fmove, smove);
	bot.moveForward = fmove;
	bot.moveSide = smove;

	unsigned short buttons = 0;
	if (fmove > kMoveButtonDeadzone) buttons |= IN_FORWARD;
	else if (fmove < -kMoveButtonDeadzone) buttons |= IN_BACK;
	if (smove > kMoveButtonDeadzone) buttons |= IN_MOVERIGHT;
	else if (smove < -kMoveButtonDeadzone) buttons |= IN_MOVELEFT;

	const Nav::Area *cur = (bot.currentAreaId >= 0)
		? Nav::FindAreaById(mesh, static_cast<uint32_t>(bot.currentAreaId)) : nullptr;
	if (cur && (cur->areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0)
		buttons |= IN_DUCK;

	Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
	return true;
}
