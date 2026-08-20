#include <extdll.h>
#include <meta_api.h>

#include "bot_move.h"
#include "bot_fire.h"
#include "../nav/nav_math.h"
#include "../nav/pm_sim.h"

namespace
{
	constexpr byte kMaxJumpCommandMsec = 50;

	constexpr float kMoveSpeed = 250.f;

	using NavMath::kPiF;
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

byte Bot_FrameMsec()
{

	int msec = static_cast<int>(gpGlobals->frametime * 1000.f);
	if (msec < 1)
		msec = 1;
	else if (msec > 100)
		msec = 100;
	return static_cast<byte>(msec);
}

void Bot_RunPlayerMove(CBot &bot, const Vector &angles, float fmove, float smove,
	unsigned short buttons, bool allowLongJump)
{
	edict_t *pEdict = bot.pEdict;
	byte msec = Bot_FrameMsec();

	buttons |= BotFire::Buttons(bot, angles);

	if (!allowLongJump && (buttons & (IN_JUMP | IN_DUCK)) == (IN_JUMP | IN_DUCK) &&
		Bot_HasLongJumpModule(pEdict))
		buttons &= ~IN_DUCK;

	if ((buttons & IN_JUMP) != 0 && msec > kMaxJumpCommandMsec)
	{
		g_engfuncs.pfnRunPlayerMove(pEdict, angles, fmove, smove, 0.f, buttons, 0,
			kMaxJumpCommandMsec);
		buttons &= ~IN_JUMP;
		msec -= kMaxJumpCommandMsec;
	}

	g_engfuncs.pfnRunPlayerMove(pEdict, angles, fmove, smove, 0.f, buttons, 0, msec);
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
		return out;
	};
}

float Bot_MaxSpeed(edict_t *pEdict)
{
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

	float gravity = g_engfuncs.pfnCVarGetFloat("sv_gravity");
	if (gravity <= 1.f)
		gravity = 800.f;
	if (pEdict->v.gravity > 0.f)
		gravity *= pEdict->v.gravity;

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

	float frameTime = gpGlobals->frametime;
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
	return physics;
}
