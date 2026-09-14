#include <extdll.h>
#include <meta_api.h>

#include <string.h>

#include <vector>

#include "bot_move.h"
#include "bot_fire.h"
#include "bot_deploy.h"
#include "bot_perf.h"
#include "bot_timing.h"
#include "../game/game_profile.h"
#include "../game/hldm_weapons.h"
#include "../nav/baked_blockers.h"
#include "../nav/nav_math.h"
#include "../nav/pm_sim.h"

namespace
{
	constexpr byte kMaxJumpCommandMsec = 50;

	constexpr float kMoveSpeed = 250.f;

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

		BotDeploy::BeamBlock guard = BotDeploy::BeamGuard(bot, dir, lookahead);
		if (guard == BotDeploy::BEAM_CLEAR)
			return;

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
}

void Bot_RunPlayerMove(CBot &bot, const Vector &angles, float fmove, float smove,
	unsigned short buttons, bool allowLongJump, float upmove)
{
	BOTPERF_SCOPE(ZONE_RUNMOVE);

	edict_t *pEdict = bot.pEdict;
	byte msec = Bot_FrameMsec();

	buttons |= BotFire::Buttons(bot, angles);

	ApplyBeamGuard(bot, angles, fmove, smove, buttons);

	if (!allowLongJump && (buttons & (IN_JUMP | IN_DUCK)) == (IN_JUMP | IN_DUCK) &&
		Bot_HasLongJumpModule(pEdict))
		buttons &= ~IN_DUCK;

	if ((buttons & IN_JUMP) != 0 && msec > kMaxJumpCommandMsec)
	{
		BotTiming::BeginMove(pEdict, kMaxJumpCommandMsec);
		g_engfuncs.pfnRunPlayerMove(pEdict, angles, fmove, smove, upmove, buttons, 0,
			kMaxJumpCommandMsec);
		BotTiming::EndMove(pEdict);
		buttons &= ~IN_JUMP;
		msec -= kMaxJumpCommandMsec;
	}

	BotTiming::BeginMove(pEdict, msec);
	g_engfuncs.pfnRunPlayerMove(pEdict, angles, fmove, smove, upmove, buttons, 0, msec);
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
		static_cast<int>(hit->v.spawnflags));
}

PmSim::PmTraceFn Bot_EngineTraceFn(edict_t *ignore, int hull)
{
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
	struct Field
	{
		Vector mins, maxs;
		Vector push;
		edict_t *ent;
	};

	std::vector<Field> fields;

	for (int i = 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free || ent->v.solid == SOLID_NOT)
			continue;

		const char *classname = STRING(ent->v.classname);
		if (!classname || strcmp(classname, "trigger_push") != 0)
			continue;

		Field field;
		field.mins = ent->v.absmin;
		field.maxs = ent->v.absmax;
		field.push = ent->v.movedir * (ent->v.speed != 0.f ? ent->v.speed : kDefaultPushSpeed);
		field.ent = ent;
		fields.push_back(field);
	}

	if (fields.empty())
		return nullptr;

	Vector hullMins = (hull == kHullDuck) ? Vector(-16.f, -16.f, -18.f) : Vector(-16.f, -16.f, -36.f);
	Vector hullMaxs = (hull == kHullDuck) ? Vector(16.f, 16.f, 18.f) : Vector(16.f, 16.f, 36.f);

	return [fields, hull, hullMins, hullMaxs](const Vector &origin)
	{
		Vector total(0.f, 0.f, 0.f);

		for (const Field &field : fields)
		{
			if (origin.x < field.mins.x - hullMaxs.x || origin.x > field.maxs.x - hullMins.x ||
				origin.y < field.mins.y - hullMaxs.y || origin.y > field.maxs.y - hullMins.y ||
				origin.z < field.mins.z - hullMaxs.z || origin.z > field.maxs.z - hullMins.z)
				continue;

			TraceResult tr;
			g_engfuncs.pfnTraceModel(origin, origin, hull, field.ent, &tr);
			if (!tr.fStartSolid && !tr.fAllSolid)
				continue;

			total = total + field.push;
		}

		return total;
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

	cvar_t *pin = g_engfuncs.pfnCVarGetPointer("crossbot_careful_pin");
	if (pin && pin->value >= 0.f)
		return pin->value > 1.f ? 1.f : pin->value;

	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_careful");
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

WeaponRangeClass Bot_RangeClass(const CBot &bot)
{
	cvar_t *force = g_engfuncs.pfnCVarGetPointer("crossbot_range_force");
	if (force && force->value >= 0.f)
	{
		int forced = static_cast<int>(force->value);
		if (forced == RANGE_CLOSE || forced == RANGE_LONG)
			return static_cast<WeaponRangeClass>(forced);
		return RANGE_NONE;
	}

	const BotInventory &inv = bot.inventory;
	if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
		return RANGE_NONE;

	const WeaponSpec *spec = HLDM_WeaponSpec(inv.weapons[inv.activeIndex].id);
	return spec ? spec->rangeClass : RANGE_NONE;
}

NavPlan::RangeBias Bot_RangeBias(const CBot &bot)
{
	NavPlan::RangeBias bias;

	if (!g_pGameProfile || !bot.pEdict)
		return bias;

	WeaponRangeClass cls = Bot_RangeClass(bot);
	if (cls == RANGE_NONE)
		return bias;

	cvar_t *shortCvar = g_engfuncs.pfnCVarGetPointer("crossbot_range_short");
	cvar_t *longCvar = g_engfuncs.pfnCVarGetPointer("crossbot_range_long");
	if (shortCvar && shortCvar->value > 0.f)
		bias.shortDist = shortCvar->value;
	if (longCvar && longCvar->value > bias.shortDist)
		bias.longDist = longCvar->value;

	bias.preferLong = (cls == RANGE_LONG);

	cvar_t *pin = g_engfuncs.pfnCVarGetPointer("crossbot_range_pin");
	if (pin && pin->value >= 0.f)
	{
		bias.weight = pin->value > 1.f ? 1.f : pin->value;
		return bias;
	}

	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_range_pref");
	float scale = cvar ? cvar->value : 0.f;
	if (scale <= 0.f)
		return bias;
	if (scale > 1.f)
		scale = 1.f;

	float readiness = g_pGameProfile->EvaluateCombat(bot.pEdict, bot.inventory);
	if (readiness < 0.f)
		readiness = 0.f;
	if (readiness > 1.f)
		readiness = 1.f;

	bias.weight = readiness * scale;
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

float Bot_MaxSpeed(edict_t *pEdict)
{
	int idx = g_engfuncs.pfnIndexOfEdict(pEdict);
	if (idx >= 1 && idx < kMaxPlayers && g_moveMaxSpeed[idx] > 1.f)
		return g_moveMaxSpeed[idx];

	float maxspeed = pEdict->v.maxspeed;
	if (maxspeed <= 1.f)
		maxspeed = g_engfuncs.pfnCVarGetFloat("sv_maxspeed");
	if (maxspeed <= 1.f)
		maxspeed = kMoveSpeed;
	return maxspeed;
}

PmSim::PmPhysics Bot_Physics(edict_t *pEdict)
{
	PmSim::PmPhysics physics;

	float maxspeed = Bot_MaxSpeed(pEdict);

	float gravity = PmSim::kGravity * Bot_GravityMultiplier(pEdict);

	float airAccel = g_engfuncs.pfnCVarGetFloat("sv_airaccelerate");
	if (airAccel <= 0.f)
		airAccel = 10.f;

	float accel = g_engfuncs.pfnCVarGetFloat("sv_accelerate");
	if (accel <= 0.f)
		accel = 10.f;

	float groundFriction = g_engfuncs.pfnCVarGetFloat("sv_friction");
	if (groundFriction <= 0.f)
		groundFriction = 4.f;

	float stopSpeed = g_engfuncs.pfnCVarGetFloat("sv_stopspeed");
	if (stopSpeed <= 0.f)
		stopSpeed = 100.f;

	float edgeFriction = g_engfuncs.pfnCVarGetFloat("sv_edgefriction");
	if (edgeFriction <= 0.f)
		edgeFriction = 2.f;

	float maxVelocity = g_engfuncs.pfnCVarGetFloat("sv_maxvelocity");
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
