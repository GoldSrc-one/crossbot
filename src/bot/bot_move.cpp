#include <extdll.h>
#include <meta_api.h>

#include <string.h>

#include <vector>

#include "bot_move.h"
#include "../game/cvars.h"
#include "bot_break.h"
#include "bot_fire.h"
#include "bot_deploy.h"
#include "bot_log.h"
#include "bot_perf.h"
#include "bot_timing.h"
#include "../game/game_profile.h"
#include "../game/teams.h"
#include "../nav/baked_blockers.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_query.h"
#include "../nav/pm_sim.h"

namespace
{
	constexpr byte kMaxJumpCommandMsec = 50;

	constexpr float kMoveSpeed = 250.f;

	constexpr float kBraceShare = 0.9f;

	constexpr int kMaxPlayers = 33;

	constexpr int kHullHuman = 1;
	constexpr int kHullDuck = 3;

	constexpr float kDefaultPushSpeed = 100.f;
	constexpr float kDefaultMaxVelocity = 2000.f;

	using NavMath::kPiF;

	float g_frameTime = 0.f;

	float g_moveMaxSpeed[kMaxPlayers] = {};
}

bool Bot_HasLongJumpModule(edict_t *pEdict)
{
	const char *slj = g_engfuncs.pfnGetPhysicsKeyValue(pEdict, "slj");
	return slj && atoi(slj) == 1;
}

float Bot_NormalizeAngle(float a)
{
	while (a > 180.f) a -= 360.f;
	while (a < -180.f) a += 360.f;
	return a;
}

void Bot_BeginFrame()
{
	g_frameTime = gpGlobals->frametime;
}

float Bot_FrameTime()
{
	return g_frameTime;
}

byte Bot_FrameMsec()
{
	int msec = static_cast<int>(Bot_FrameTime() * 1000.f);
	if (msec < 1)
		msec = 1;
	else if (msec > 100)
		msec = 100;
	return static_cast<byte>(msec);
}

namespace
{
	constexpr float kBeamGuardMin = 56.f;
	constexpr float kBeamGuardTime = 0.35f;
	constexpr float kBeamGuardMax = 144.f;
	constexpr float kBeamGuardDeadzone = 0.01f;

	Vector ComposeMove(float viewYawDeg, float forwardMove, float sideMove)
	{
		float yawRad = viewYawDeg * (NavMath::kPiF / 180.f);
		float cy = cosf(yawRad), sy = sinf(yawRad);

		return Vector(forwardMove * cy + sideMove * sy, forwardMove * sy - sideMove * cy, 0.f);
	}

	void ApplyBeamGuard(CBot &bot, const Vector &angles, float &fmove, float &smove,
		unsigned short &buttons)
	{
		if (BotDeploy::Beams().empty())
			return;

		Vector world = ComposeMove(angles.y, fmove, smove);
		float len = sqrtf(world.x * world.x + world.y * world.y);
		if (len < 1.f)
			return;

		Vector dir(world.x / len, world.y / len, 0.f);

		const Vector &vel = bot.pEdict->v.velocity;
		float speed = sqrtf(vel.x * vel.x + vel.y * vel.y);
		float lookahead = speed * kBeamGuardTime;
		if (lookahead < kBeamGuardMin) lookahead = kBeamGuardMin;
		if (lookahead > kBeamGuardMax) lookahead = kBeamGuardMax;

		const BotDeploy::Beam *beam = NULL;
		BotDeploy::BeamBlock guard = BotDeploy::BeamGuard(bot, dir, lookahead, &beam);
		if (guard == BotDeploy::BEAM_CLEAR)
			return;

		float now = gpGlobals->time;
		bot.avoid.guardBeam = beam ? beam->ent : NULL;
		bot.avoid.guardTime = now;
		if (beam && now >= bot.avoid.guardLogTime)
		{
			bot.avoid.guardLogTime = now + 1.f;
			const Vector &o = bot.pEdict->v.origin;
			Vector seg = beam->end - beam->start;
			float segLen2 = seg.x * seg.x + seg.y * seg.y;
			float t = segLen2 > 0.f
				? ((o.x - beam->start.x) * seg.x + (o.y - beam->start.y) * seg.y) / segLen2 : 0.f;
			t = t < 0.f ? 0.f : (t > 1.f ? 1.f : t);
			float lx = o.x - (beam->start.x + seg.x * t);
			float ly = o.y - (beam->start.y + seg.y * t);
			BOT_VLOG("CrossBot: %s beam guard %s at (%.1f,%.1f,%.1f) dir (%.2f,%.2f) beam (%.0f,%.0f,%.0f)-"
				"(%.0f,%.0f,%.0f) lateral %.0f speed %.0f\n",
				bot.name, guard == BotDeploy::BEAM_STOP ? "stop" : "duck", o.x, o.y, o.z, dir.x, dir.y,
				beam->start.x, beam->start.y, beam->start.z, beam->end.x, beam->end.y, beam->end.z,
				sqrtf(lx * lx + ly * ly), speed);
		}

		if (guard == BotDeploy::BEAM_DUCK)
			buttons |= IN_DUCK;

		buttons &= ~(IN_FORWARD | IN_BACK | IN_MOVERIGHT | IN_MOVELEFT | IN_JUMP);

		float vlen = sqrtf(vel.x * vel.x + vel.y * vel.y);
		if (vlen > 1.f)
		{
			Vector brake(-vel.x / vlen, -vel.y / vlen, 0.f);
			Bot_DecomposeMove(angles.y, brake, Bot_MaxSpeed(bot.pEdict), fmove, smove);

			if (fmove > kBeamGuardDeadzone) buttons |= IN_FORWARD;
			else if (fmove < -kBeamGuardDeadzone) buttons |= IN_BACK;
			if (smove > kBeamGuardDeadzone) buttons |= IN_MOVERIGHT;
			else if (smove < -kBeamGuardDeadzone) buttons |= IN_MOVELEFT;
		}
		else
		{
			fmove = 0.f;
			smove = 0.f;
		}

		bot.moveForward = fmove;
		bot.moveSide = smove;
	}

	constexpr float kSpacingRadius = 64.f;
	constexpr float kSpacingHeight = 72.f;
	constexpr float kSpacingProbe = 24.f;
	constexpr float kSpacingGoldenAngle = 2.39996f;
	constexpr int kSpacingWaterlevel = 2;
	constexpr float kRangePref = 0.5f;

	bool SpacingProbeOnMesh(const CBot &bot, const Vector &probe)
	{
		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh)
			return false;
		if (bot.currentAreaId >= 0 && static_cast<size_t>(bot.currentAreaId) < mesh->areas.size() &&
			Nav::AreaContainsPoint(mesh->areas[bot.currentAreaId], probe))
			return true;
		return Nav::FindAreaExact(*mesh, probe) != nullptr;
	}

	void ApplySpacing(CBot &bot, const Vector &angles, float &fmove, float &smove,
		unsigned short &buttons)
	{
		edict_t *pEdict = bot.pEdict;
		if (!Teams::Active(pEdict))
			return;
		if ((pEdict->v.flags & FL_ONGROUND) == 0 || pEdict->v.movetype != MOVETYPE_WALK ||
			pEdict->v.waterlevel >= kSpacingWaterlevel || (buttons & IN_JUMP) != 0)
			return;
		if (bot.rideActive || bot.ladderActive || bot.airmoveActive || bot.fun.aboard)
			return;

		int self = g_engfuncs.pfnIndexOfEdict(pEdict);
		float px = 0.f, py = 0.f;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent == pEdict || !CBotIsLivePlayer(ent) || (g_pGameProfile && g_pGameProfile->IsDead(ent)) ||
				Teams::Of(pEdict, ent) != Teams::REL_FRIEND)
				continue;

			Vector d = pEdict->v.origin - ent->v.origin;
			if (fabsf(d.z) >= kSpacingHeight)
				continue;
			float dist = sqrtf(d.x * d.x + d.y * d.y);
			if (dist >= kSpacingRadius)
				continue;

			float dx, dy;
			if (dist < 1.f)
			{
				dx = cosf(self * kSpacingGoldenAngle);
				dy = sinf(self * kSpacingGoldenAngle);
			}
			else
			{
				dx = d.x / dist;
				dy = d.y / dist;
			}
			float w = 1.f - dist / kSpacingRadius;
			px += dx * w;
			py += dy * w;
		}

		float len = sqrtf(px * px + py * py);
		if (len < 0.001f)
			return;
		Vector dir(px / len, py / len, 0.f);
		if (len > 1.f)
			len = 1.f;
		if (!SpacingProbeOnMesh(bot, pEdict->v.origin + dir * kSpacingProbe))
			return;

		Vector world = ComposeMove(angles.y, fmove, smove);
		float speed = sqrtf(world.x * world.x + world.y * world.y);
		float drive;
		if (speed < 1.f)
		{
			world = dir;
			drive = len * Bot_MaxSpeed(pEdict);
		}
		else
		{
			world = world + dir * (len * speed);
			if (sqrtf(world.x * world.x + world.y * world.y) < 1.f)
				return;
			drive = speed;
		}

		Bot_DecomposeMove(angles.y, world, drive, fmove, smove);

		buttons &= ~(IN_FORWARD | IN_BACK | IN_MOVERIGHT | IN_MOVELEFT);
		if (fmove > kBeamGuardDeadzone) buttons |= IN_FORWARD;
		else if (fmove < -kBeamGuardDeadzone) buttons |= IN_BACK;
		if (smove > kBeamGuardDeadzone) buttons |= IN_MOVERIGHT;
		else if (smove < -kBeamGuardDeadzone) buttons |= IN_MOVELEFT;

		bot.moveForward = fmove;
		bot.moveSide = smove;
	}

	constexpr float kLineSticky = 0.5f;
	constexpr float kLineSightHalf = 24.f;
	constexpr float kLineFireHalf = 64.f;
	constexpr float kLineSightTurn = 0.785398f;

	float LineBlock(CBot &bot, const Vector &target, float halfWidth)
	{
		edict_t *pEdict = bot.pEdict;
		const Vector &from = pEdict->v.origin;
		float dx = target.x - from.x, dy = target.y - from.y;
		float len = sqrtf(dx * dx + dy * dy);
		float left = 0.f, right = 0.f;
		if (len >= 1.f && Teams::Active(pEdict) && (pEdict->v.flags & FL_ONGROUND) != 0)
		{
			dx /= len;
			dy /= len;
			float fallback = (g_engfuncs.pfnIndexOfEdict(pEdict) & 1) ? 1.f : -1.f;
			for (int i = 1; i <= gpGlobals->maxClients; ++i)
			{
				edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
				if (!ent || ent == pEdict || !CBotIsLivePlayer(ent) || (g_pGameProfile && g_pGameProfile->IsDead(ent)) ||
					Teams::Of(pEdict, ent) != Teams::REL_FRIEND)
					continue;

				Vector f = ent->v.origin - from;
				float along = f.x * dx + f.y * dy;
				if (along <= 0.f || along >= len)
					continue;
				if (fabsf(f.z - (target.z - from.z) * along / len) >= kSpacingHeight)
					continue;
				float lat = dx * f.y - dy * f.x;
				float w = 1.f - fabsf(lat) / halfWidth;
				if (w <= 0.f)
					continue;

				float sticky = halfWidth * kLineSticky;
				float side = (lat > sticky) ? -1.f : (lat < -sticky) ? 1.f
					: (bot.lineSide != 0.f ? bot.lineSide : fallback);
				if (side > 0.f)
					left = fmaxf(left, w);
				else
					right = fmaxf(right, w);
			}
		}

		float side = 0.f, w = 0.f;
		if (left > 0.f && right == 0.f)
		{
			side = 1.f;
			w = left;
		}
		else if (right > 0.f && left == 0.f)
		{
			side = -1.f;
			w = right;
		}

		if (side != bot.lineSide)
		{
			BOT_VLOG("CrossBot: %s line to (%.0f,%.0f,%.0f) %s\n", bot.name, target.x, target.y, target.z,
				side > 0.f ? "steers left" : side < 0.f ? "steers right"
					: (left > 0.f ? "blocked both sides, goes middle" : "is clear"));
			bot.lineSide = side;
		}
		return side * w;
	}
}

Vector Bot_ClearLineOfSight(CBot &bot, const Vector &moveDir, const Vector &target)
{
	float s = LineBlock(bot, target, kLineSightHalf);
	if (s == 0.f)
		return moveDir;

	float a = s * kLineSightTurn;
	float c = cosf(a), sn = sinf(a);
	Vector turned(moveDir.x * c - moveDir.y * sn, moveDir.x * sn + moveDir.y * c, 0.f);
	float len = sqrtf(turned.x * turned.x + turned.y * turned.y);
	if (len < 0.001f || !SpacingProbeOnMesh(bot, bot.pEdict->v.origin + turned * (kSpacingProbe / len)))
		return moveDir;
	return turned;
}

void Bot_ClearLineOfFire(CBot &bot, Vector &moveDir, float &drive, const Vector &enemy)
{
	float s = LineBlock(bot, enemy, kLineFireHalf);
	if (s == 0.f)
		return;

	edict_t *pEdict = bot.pEdict;
	float dx = enemy.x - pEdict->v.origin.x, dy = enemy.y - pEdict->v.origin.y;
	float len = sqrtf(dx * dx + dy * dy);
	float side = (s > 0.f) ? 1.f : -1.f;
	Vector perp(-dy / len * side, dx / len * side, 0.f);
	if (!SpacingProbeOnMesh(bot, pEdict->v.origin + perp * kSpacingProbe))
		return;

	float push = fabsf(s) * Bot_MaxSpeed(pEdict);
	Vector world = moveDir * drive + perp * push;
	float wl = sqrtf(world.x * world.x + world.y * world.y);
	if (wl < 1.f)
		return;
	moveDir = Vector(world.x / wl, world.y / wl, 0.f);
	drive = fmaxf(drive, push);
}

void Bot_RunPlayerMove(CBot &bot, const Vector &angles, float fmove, float smove,
	unsigned short buttons, bool allowLongJump, float upmove)
{
	BOTPERF_SCOPE(ZONE_RUNMOVE);

	edict_t *pEdict = bot.pEdict;
	byte msec = Bot_FrameMsec();

	unsigned short fireButtons = BotFire::Buttons(bot, bot.aim.perceived);
	if ((buttons & IN_USE) != 0)
		fireButtons &= ~IN_RELOAD;
	buttons |= fireButtons;

	Vector view = angles;
	BotGrenade::Run(bot, view);

	bool braceDuck = false;
	bool stanceOnly = false;
	float braceCap = 0.f;
	if (BotFire::Brace(bot, buttons, braceDuck, stanceOnly, braceCap))
	{
		float drive = sqrtf(fmove * fmove + smove * smove);
		float allowed = braceCap * kBraceShare;
		if (!stanceOnly && drive > allowed)
		{
			fmove *= allowed / drive;
			smove *= allowed / drive;
		}
		buttons &= ~(IN_FORWARD | IN_BACK | IN_MOVELEFT | IN_MOVERIGHT);
		if (braceDuck)
			buttons |= IN_DUCK;
	}
	else
		ApplySpacing(bot, angles, fmove, smove, buttons);

	if (g_pGameProfile)
		g_pGameProfile->MoveAdjust(bot, fmove, smove, buttons);

	if (g_pGameProfile && ((bot.camp.charging && bot.goal.stop.camp) || BotFire::BraceCreep(bot)))
	{
		buttons |= g_pGameProfile->CreepButtons();
		float creep = g_pGameProfile->CreepSpeed();
		float drive = sqrtf(fmove * fmove + smove * smove);
		if (creep > 0.f && drive > creep)
		{
			fmove *= creep / drive;
			smove *= creep / drive;
		}
	}

	ApplyBeamGuard(bot, angles, fmove, smove, buttons);

	if (view.y != angles.y)
	{
		Vector world = ComposeMove(angles.y, fmove, smove);
		Bot_DecomposeMove(view.y, world, world.Length(), fmove, smove);
	}

	if (!allowLongJump && (buttons & (IN_JUMP | IN_DUCK)) == (IN_JUMP | IN_DUCK) &&
		Bot_HasLongJumpModule(pEdict))
		buttons &= ~IN_DUCK;

	if ((buttons & IN_JUMP) != 0 && msec > kMaxJumpCommandMsec)
	{
		BotTiming::BeginMove(pEdict, kMaxJumpCommandMsec);
		g_engfuncs.pfnRunPlayerMove(pEdict, view, fmove, smove, upmove, buttons, 0,
			kMaxJumpCommandMsec);
		BotTiming::EndMove(pEdict);
		buttons &= ~IN_JUMP;
		msec -= kMaxJumpCommandMsec;
	}

	BotTiming::BeginMove(pEdict, msec);
	g_engfuncs.pfnRunPlayerMove(pEdict, view, fmove, smove, upmove, buttons, 0, msec);
	BotTiming::EndMove(pEdict);
}

void Bot_DecomposeMove(float viewYawDeg, const Vector &worldDir, float driveLength, float &forwardMove,
	float &sideMove)
{
	forwardMove = 0.f;
	sideMove = 0.f;

	float len = sqrtf(worldDir.x * worldDir.x + worldDir.y * worldDir.y);
	if (len < 0.0001f)
		return;

	float nx = worldDir.x / len, ny = worldDir.y / len;

	float yawRad = viewYawDeg * (NavMath::kPiF / 180.f);
	float cy = cosf(yawRad), sy = sinf(yawRad);

	forwardMove = (nx * cy + ny * sy) * driveLength;
	sideMove = (nx * sy - ny * cy) * driveLength;
}

Vector Bot_YawDir(float yawDeg)
{
	float r = yawDeg * (kPiF / 180.f);
	return Vector(cosf(r), sinf(r), 0.f);
}

bool Bot_TraceHitEntity(const TraceResult &tr)
{
	edict_t *hit = tr.pHit;
	if (!hit || g_engfuncs.pfnIndexOfEdict(hit) == 0)
		return false;
	if (hit->v.solid == SOLID_BSP && (hit->v.flags & FL_WORLDBRUSH) != 0)
		return false;
	return !crossbot_nav::IsBakedBlocker(STRING(hit->v.classname),
		static_cast<int>(hit->v.spawnflags), hit->v.targetname != 0);
}

std::shared_ptr<const Nav::WorldSnapshot> Bot_WorldSnapshot()
{
	static std::shared_ptr<const Nav::WorldSnapshot> s_snap;
	static float s_time = -1.f;

	const Nav::NavMesh *mesh = Nav::Get();
	if (!mesh || !mesh->world)
		return nullptr;
	if (s_snap && s_time == gpGlobals->time && s_snap->world == mesh->world)
		return s_snap;

	auto snap = std::make_shared<Nav::WorldSnapshot>();
	snap->world = mesh->world;
	int models = static_cast<int>(mesh->world->models.size());

	for (int i = 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free || ent->v.solid == SOLID_NOT)
			continue;

		const char *model = STRING(ent->v.model);
		int sub = (model && model[0] == '*') ? atoi(model + 1) : -1;
		if (sub >= models)
			sub = -1;

		const char *classname = STRING(ent->v.classname);
		Nav::WorldSolid solid;
		solid.index = i;
		solid.owner = ent->v.owner ? g_engfuncs.pfnIndexOfEdict(ent->v.owner) : 0;
		solid.origin = ent->v.origin;
		solid.absmin = ent->v.absmin;
		solid.absmax = ent->v.absmax;

		if (classname && strcmp(classname, "trigger_push") == 0)
		{
			if (sub <= 0)
				continue;
			solid.model = sub;
			Nav::SetSolidAngles(solid, ent->v.angles);
			solid.push = ent->v.movedir * (ent->v.speed != 0.f ? ent->v.speed : kDefaultPushSpeed);
			snap->pushes.push_back(solid);
			continue;
		}

		if (ent->v.solid == SOLID_TRIGGER || ent->v.size.x == 0.f)
			continue;

		bool named = ent->v.targetname != 0;
		bool baked = crossbot_nav::IsBakedBlocker(classname, static_cast<int>(ent->v.spawnflags), named);
		if (ent->v.solid == SOLID_BSP)
		{
			if (sub <= 0 || (ent->v.flags & FL_MONSTERCLIP) != 0)
				continue;
			solid.model = sub;
			Nav::SetSolidAngles(solid, ent->v.angles);
			solid.counts = crossbot_nav::IsBarrierWall(classname, named) ||
				((ent->v.flags & FL_WORLDBRUSH) == 0 && !baked);
		}
		else if (ent->v.movetype == MOVETYPE_PUSHSTEP)
		{
			solid.boxMins = ent->v.mins;
			solid.boxMaxs = ent->v.maxs;
			solid.counts = !baked;
		}
		else
			continue;

		solid.breakable = BotBreak::IsTarget(ent);
		snap->solids.push_back(solid);
	}

	s_snap = std::move(snap);
	s_time = gpGlobals->time;
	return s_snap;
}

namespace
{
	Nav::TraceFilter WorldFilter(edict_t *ignore, int hull, bool phaseBreakables)
	{
		Nav::TraceFilter filter;
		filter.hull = hull;
		filter.ignore = ignore ? g_engfuncs.pfnIndexOfEdict(ignore) : 0;
		filter.phaseBreakables = phaseBreakables;
		return filter;
	}
}

PmSim::PmTraceFn Bot_WorldTraceFn(edict_t *ignore, int hull, bool phaseBreakables)
{
	std::shared_ptr<const Nav::WorldSnapshot> snap = Bot_WorldSnapshot();
	if (snap)
	{
		Nav::TraceFilter filter = WorldFilter(ignore, hull, phaseBreakables);
		return [snap, filter](const Vector &start, const Vector &end)
		{
			return Nav::TraceWorld(*snap, filter, start, end);
		};
	}

	return [ignore, hull](const Vector &start, const Vector &end)
	{
		TraceResult tr;
		g_engfuncs.pfnTraceHull(start, end, ignore_monsters, hull, ignore, &tr);

		PmSim::PmTrace out;
		out.endpos = tr.vecEndPos;
		out.planeNormal = tr.vecPlaneNormal;
		out.fraction = tr.flFraction;
		out.allsolid = tr.fAllSolid != 0;
		out.startsolid = tr.fStartSolid != 0;
		out.hitEntity = Bot_TraceHitEntity(tr);
		return out;
	};
}

PmSim::PmArcFn Bot_WorldArcFn(edict_t *ignore, int hull, bool phaseBreakables)
{
	std::shared_ptr<const Nav::WorldSnapshot> snap = Bot_WorldSnapshot();
	if (!snap)
		return nullptr;

	Nav::TraceFilter filter = WorldFilter(ignore, hull, phaseBreakables);
	return [snap, filter](const Vector &start, const Vector &vel, float gravity, float maxTime)
	{
		return Nav::SafeArcTime(*snap, filter, start, vel, gravity, maxTime);
	};
}

Vector Bot_DriftVelocity(edict_t *pEdict, const PmSim::PmPushFn &push)
{
	Vector drift(0.f, 0.f, 0.f);
	if (!pEdict)
		return drift;

	edict_t *ground = pEdict->v.groundentity;
	if (ground && !ground->free && (pEdict->v.flags & FL_ONGROUND) != 0 &&
		(ground->v.flags & FL_CONVEYOR) != 0)
		drift = ground->v.movedir * ground->v.speed;

	if (push)
		drift = drift + push(pEdict->v.origin);

	drift.z = 0.f;
	return drift;
}

PmSim::PmPushFn Bot_PushFields(int hull)
{
	std::shared_ptr<const Nav::WorldSnapshot> snap = Bot_WorldSnapshot();
	if (!snap || snap->pushes.empty())
		return nullptr;

	return [snap, hull](const Vector &origin)
	{
		return Nav::PushAt(*snap, hull, origin);
	};
}

float Bot_GravityMultiplier(edict_t *pEdict)
{
	return (pEdict && pEdict->v.gravity > 0.f) ? pEdict->v.gravity : 1.f;
}

float Bot_Carefulness(const CBot &bot)
{
	if (!g_pGameProfile || !bot.pEdict)
		return 0.f;

	cvar_t *cvar = CBotCvar("crossbot_careful");
	float scale = cvar ? cvar->value : 1.f;
	if (scale <= 0.f)
		return 0.f;
	if (scale > 1.f)
		scale = 1.f;

	float readiness = g_pGameProfile->EvaluateCombat(bot.pEdict, bot.inventory);
	if (readiness < 0.f)
		readiness = 0.f;
	if (readiness > 1.f)
		readiness = 1.f;

	return (1.f - readiness) * scale;
}

PmSim::FallRule Bot_FallDamage(const CBot &bot)
{
	PmSim::FallRule rule = g_pGameProfile ? g_pGameProfile->FallDamage() : PmSim::kHlFall;
	if (bot.pEdict)
		rule.health = bot.pEdict->v.health;
	return rule;
}

WeaponRangeClass Bot_RangeClass(const CBot &bot)
{
	const BotInventory &inv = bot.inventory;
	if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
		return RANGE_NONE;

	const WeaponSpec *spec = g_pGameProfile ? g_pGameProfile->GetWeaponSpec(inv.weapons[inv.activeIndex].id) : nullptr;
	return spec ? spec->rangeClass : RANGE_NONE;
}

NavPlan::RangeBias Bot_RangeBias(const CBot &bot, bool readiness)
{
	NavPlan::RangeBias bias;

	if (!g_pGameProfile || !bot.pEdict)
		return bias;

	WeaponRangeClass cls = Bot_RangeClass(bot);
	if (cls == RANGE_NONE)
		return bias;

	bias.preferLong = (cls == RANGE_LONG);

	if (!readiness)
	{
		bias.weight = kRangePref;
		return bias;
	}

	float ready = g_pGameProfile->EvaluateCombat(bot.pEdict, bot.inventory);
	if (ready < 0.f)
		ready = 0.f;
	if (ready > 1.f)
		ready = 1.f;

	bias.weight = ready * kRangePref;
	return bias;
}

void Bot_NoteMoveMaxSpeed(int playerIndex, float maxspeed)
{
	int idx = playerIndex + 1;
	if (idx < 1 || idx >= kMaxPlayers)
		return;
	g_moveMaxSpeed[idx] = (maxspeed > 1.f) ? maxspeed : 0.f;
}

void Bot_ForgetMoveMaxSpeed(edict_t *pEdict)
{
	int idx = g_engfuncs.pfnIndexOfEdict(pEdict);
	if (idx >= 1 && idx < kMaxPlayers)
		g_moveMaxSpeed[idx] = 0.f;
}

bool Bot_MoveMaxSpeedKnown(edict_t *pEdict)
{
	int idx = g_engfuncs.pfnIndexOfEdict(pEdict);
	return idx >= 1 && idx < kMaxPlayers && g_moveMaxSpeed[idx] > 1.f;
}

bool Bot_Frozen(const edict_t *pEdict)
{
	return pEdict->v.maxspeed > 0.f && pEdict->v.maxspeed < 10.f;
}

float Bot_MaxSpeed(edict_t *pEdict)
{
	int idx = g_engfuncs.pfnIndexOfEdict(pEdict);
	float maxspeed = 0.f;
	if (idx >= 1 && idx < kMaxPlayers && g_moveMaxSpeed[idx] > 1.f)
		maxspeed = g_moveMaxSpeed[idx];
	else
	{
		maxspeed = pEdict->v.maxspeed;
		if (maxspeed <= 1.f)
			maxspeed = CBotCvarFloat("sv_maxspeed");
		if (maxspeed <= 1.f)
			maxspeed = kMoveSpeed;
	}
	return g_pGameProfile ? g_pGameProfile->MaxSpeed(pEdict, maxspeed) : maxspeed;
}

PmSim::PmPhysics Bot_Physics(edict_t *pEdict)
{
	PmSim::PmPhysics physics;

	float maxspeed = Bot_MaxSpeed(pEdict);

	float gravity = PmSim::kGravity * Bot_GravityMultiplier(pEdict);

	float airAccel = CBotCvarFloat("sv_airaccelerate");
	if (airAccel <= 0.f)
		airAccel = 10.f;

	float accel = CBotCvarFloat("sv_accelerate");
	if (accel <= 0.f)
		accel = 10.f;

	float groundFriction = CBotCvarFloat("sv_friction");
	if (groundFriction <= 0.f)
		groundFriction = 4.f;

	float stopSpeed = CBotCvarFloat("sv_stopspeed");
	if (stopSpeed <= 0.f)
		stopSpeed = 100.f;

	float edgeFriction = CBotCvarFloat("sv_edgefriction");
	if (edgeFriction <= 0.f)
		edgeFriction = 2.f;

	float maxVelocity = CBotCvarFloat("sv_maxvelocity");
	if (maxVelocity <= 0.f)
		maxVelocity = kDefaultMaxVelocity;

	float frameTime = Bot_FrameTime();
	if (frameTime < 1.f / 128.f || frameTime > 1.f / 20.f)
		frameTime = 1.f / 60.f;

	physics.maxSpeed = maxspeed;
	physics.gravity = gravity;
	physics.airAccelerate = airAccel;
	physics.accelerate = accel;
	physics.groundFriction = groundFriction;
	physics.stopSpeed = stopSpeed;
	physics.edgeFriction = edgeFriction;
	physics.friction = 1.f;
	physics.frameTime = frameTime;
	physics.maxVelocity = maxVelocity;
	return physics;
}
