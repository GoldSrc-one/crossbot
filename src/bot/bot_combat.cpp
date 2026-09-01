#include <extdll.h>
#include <meta_api.h>

#include <math.h>

#include "../game/game_profile.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_query.h"
#include "../nav/nav_vantage.h"
#include "../nav/path_follower.h"
#include "bot.h"
#include "bot_combat.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sense.h"

extern IGameProfile *g_pGameProfile;

namespace
{
	using NavMath::Norm2D;

	// The bot gives up on an enemy it cannot see for this long. Suspicion entries expire on
	// crossbot_sense_decay, which defaults to the same 5s, so on stock settings the two rules
	// retire an enemy at about the same moment and either one may be the one that logs it.
	constexpr float kSightTimeout = 5.f;

	constexpr float kProbeDist = 160.f;
	constexpr float kProbeInterval = 0.7f;
	constexpr float kProbeReached = 32.f;

	// A full step lands outside the mesh most of the time -- crossfire's areas are routinely
	// narrower than kProbeDist, and a walk neighbour at a different height fails the z test. Back
	// off along the same ray rather than throwing the direction away, so "nothing that way" means
	// the bot really is against something.
	const float kProbeFractions[] = {1.f, 0.7f, 0.45f, 0.25f};

	// Nav areas are in duck-origin z-space, so a standing origin sits this far above one.
	constexpr float kAreaStandLift = 18.f;

	constexpr float kMoveButtonDeadzone = 0.01f;

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
		// Areas merge into non-convex prisms, so this has to be the bound-plane test rather than
		// a clamp into the AABB.
		Vector spot(xy.x, xy.y, area.centroid.z);
		if (!Nav::AreaContainsPoint(area, spot))
			return false;

		bool crouch = (area.areaFlags & crossbot_nav::NAV_AREA_CROUCH) != 0;
		out.origin = crouch ? spot : spot + Vector(0.f, 0.f, kAreaStandLift);
		out.areaId = area.id;
		return true;
	}

	// Local means local: the bot may stand anywhere in the area it occupies or in one joined to
	// it by a plain walk. Jumps, ladders, lifts and teleports are route-building, and combat is
	// what drops routes.
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
			if ((conn.connFlags & crossbot_nav::CONN_FLAG_ONE_WAY) != 0 && conn.areaA != cur->id)
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

	void Enter(CBot &bot, edict_t *enemy, const char *how)
	{
		BotCombat::State &st = bot.combat;

		st.active = true;
		st.driving = false;
		st.enemy.Set(enemy);
		st.startTime = gpGlobals->time;
		st.lastSightTime = gpGlobals->time;
		st.haveProbe = false;
		st.probeArea = -1;
		st.nextProbeTime = 0.f;
		st.lastReason = how;

		BOT_VLOG("CrossBot: %s entered combat with %s (%s)\n", bot.name,
			CBotPlayerNameOf(enemy), how);
	}

	// Pick where to stand next. One random direction, scored on exposure alone; if it is no
	// better than standing still, go the other way instead, unscored.
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

		// bot_sense re-blends the bot's own eye every 0.1s, so there is no second blend here.
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
}

bool BotCombat::Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_combat");
	return !cvar || cvar->value != 0.f;
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

// Who just hurt this bot, if a player hurt it at all. Falling, drowning and crush damage all
// arrive from worldspawn at edict index 0; a trigger_hurt is a brush entity that no player owns;
// and a rocket or grenade names itself as the inflictor, carrying its shooter in pev->owner.
edict_t *BotCombat::DamageSource(edict_t *pEdict)
{
	if (!pEdict || pEdict->v.dmg_take == 0.f)
		return nullptr;

	edict_t *ent = pEdict->v.dmg_inflictor;
	if (!ent || ent->free || ent == pEdict)
		return nullptr;
	if (g_engfuncs.pfnIndexOfEdict(ent) == 0)
		return nullptr;

	if (!CBotIsLivePlayer(ent))
		ent = ent->v.owner;

	if (!ent || ent->free || ent == pEdict || !CBotIsLivePlayer(ent))
		return nullptr;
	if (g_pGameProfile && g_pGameProfile->IsDead(ent))
		return nullptr;

	return ent;
}

void BotCombat::Exit(CBot &bot, const char *why)
{
	State &st = bot.combat;
	bool was = st.active;

	st.active = false;
	st.driving = false;
	st.enemy.Clear();
	st.haveProbe = false;
	st.probeArea = -1;
	st.nextProbeTime = 0.f;
	st.lastReason = why;

	if (!was)
		return;

	BOT_VLOG("CrossBot: %s left combat after %.1fs: %s\n", bot.name,
		gpGlobals->time - st.startTime, why);

	// Whatever it was walking to was dropped when combat took over, so choose again now rather
	// than idling out the gather retry interval.
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

	Seed(bot);

	edict_t *seen = bot.sense.target.Get();

	if (st.active)
	{
		edict_t *enemy = st.enemy.Get();

		if (!enemy)
			Exit(bot, "the enemy handle went stale");
		else if (g_pGameProfile && g_pGameProfile->IsDead(enemy))
			Exit(bot, "the enemy died");
		else if (seen && seen != enemy)
		{
			// Still a fight, just a different opponent. Dropping out and re-entering would
			// throw away the probe and the goal it already gave up, for nothing.
			BOT_VLOG("CrossBot: %s switches from %s to %s\n", bot.name,
				CBotPlayerNameOf(enemy), CBotPlayerNameOf(seen));
			st.enemy.Set(seen);
			st.lastSightTime = gpGlobals->time;
		}
		else
		{
			if (seen == enemy)
				st.lastSightTime = gpGlobals->time;

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
		Enter(bot, seen, "sight");
		return;
	}

	edict_t *hurt = DamageSource(bot.pEdict);
	if (hurt)
		Enter(bot, hurt, "damage");
}

bool BotCombat::Run(CBot &bot, const Nav::NavMesh &mesh)
{
	State &st = bot.combat;
	if (!st.active)
		return false;

	// A bot mid-flight, mid-ladder or mid-ride needs its waypoints, and tearing the path out from
	// under one would drop it. The state is entered on time either way -- only the movement
	// takeover waits.
	if (Bot_ManeuverActive(bot))
	{
		st.lastReason = "finishing a maneuver";
		return false;
	}

	edict_t *pEdict = bot.pEdict;

	if (!st.driving)
	{
		st.driving = true;
		BotGoal::Clear(bot);
		BOT_VLOG("CrossBot: %s drops its goal to fight\n", bot.name);
	}

	// Nothing below plans a route, so keep the stall detector from reading a bot holding a corner
	// as a bot stuck against geometry.
	bot.lastProgressTime = gpGlobals->time;

	// Aim belongs to whoever it is shooting: ApplyLook gives BotFire first refusal.
	BotSense::ApplyLook(bot);
	Vector angles = Bot_UpdateFacing(bot);

	UpdateProbe(bot, mesh);

	PmSim::PmPhysics physics = Bot_DrivePhysics(pEdict);

	Vector moveDir(0.f, 0.f, 0.f);
	float drive = 0.f;
	if (st.haveProbe)
	{
		Vector to(st.probePoint.x - pEdict->v.origin.x, st.probePoint.y - pEdict->v.origin.y,
			0.f);
		float dist = to.Length();
		if (dist > 1.f)
		{
			moveDir = Norm2D(to);
			drive = Bot_MaxSpeed(pEdict);

			NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
				physics);
			if (solved.wishSpeed > 0.f)
			{
				moveDir = solved.wishDir;
				drive = solved.wishSpeed;
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
