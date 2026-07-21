#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

#include "bot_move_nav.h"
#include "bot_move.h"
#include "../nav/nav.h"
#include "../nav/nav_avoid.h"
#include "../nav/nav_query.h"
#include "../nav/path_astar.h"
#include "../nav/path_funnel.h"

namespace
{
	constexpr float kPi = 3.14159265f;
	constexpr float kMoveSpeed = 250.f;
	constexpr float kWaypointReachedDistance = 32.f;
	// Distance/facing gates before committing to a jump, matching FoxBot's
	// precedent (bot_navigate.cpp): jump only once reasonably close and
	// roughly lined up, rather than jk_botti's simpler "press jump the
	// instant you're within a small radius" -- CrossBot already does
	// rate-limited turn smoothing (Bot_UpdateFacing), so jumping before
	// that smoothing has caught up would send the bot off at the wrong
	// angle.
	constexpr float kJumpTriggerDistance = 100.f;
	constexpr float kJumpFacingToleranceDeg = 20.f;

	float NormalizeAngle(float a)
	{
		while (a > 180.f) a -= 360.f;
		while (a < -180.f) a += 360.f;
		return a;
	}
	// More than any single tick's max travel at kMoveSpeed, even at
	// Bot_FrameMsec()'s 100ms clamp ceiling (250 u/s -> 25 u/tick) -- a jump
	// this big means something moved the bot behind our back (respawn, or a
	// trigger_teleport once Milestone 2h lands), not normal movement. See
	// the plan's "Detecting respawn and teleport" note.
	constexpr float kTeleportJumpThreshold = 150.f;
	constexpr float kRepathInterval = 5.f; // periodic safety re-path, cheap at the area counts seen so far

	Vector RandomPointInArea(const Nav::Area &area)
	{
		// The baked format only stores each area's AABB/centroid, not its
		// exact polytope (see nav_format.h), so there's nothing to sample
		// a genuinely random interior point from -- using the centroid
		// directly is a deliberate simplification, same class of
		// approximation as nav_query's AABB-based point-to-area lookup.
		return area.centroid;
	}

	void ResetPath(CBot &bot)
	{
		bot.waypoints.clear();
		bot.waypointIndex = 0;
	}

	void PickRandomGoal(CBot &bot, const Nav::NavMesh &mesh)
	{
		if (mesh.areas.empty())
			return;
		const Nav::Area &area = mesh.areas[static_cast<size_t>(rand()) % mesh.areas.size()];
		bot.goalPoint = RandomPointInArea(area);
		bot.hasGoal = true;
		ResetPath(bot);
	}

	bool Repath(CBot &bot, const Nav::NavMesh &mesh, edict_t *pEdict)
	{
		const Nav::Area *startArea = Nav::FindAreaForPoint(mesh, pEdict->v.origin);
		const Nav::Area *goalArea = Nav::FindAreaForPoint(mesh, bot.goalPoint);
		if (!startArea || !goalArea)
			return false;

		Nav::PathResult path = Nav::FindPath(mesh, startArea->id, goalArea->id);
		if (!path.found)
			return false;

		bot.waypoints = Nav::BuildWaypointPath(mesh, path, pEdict->v.origin, bot.goalPoint, pEdict);
		bot.waypointIndex = 0;
		return true;
	}
}

void Bot_MoveNav(CBot &bot)
{
	edict_t *pEdict = bot.pEdict;
	const Nav::NavMesh *mesh = Nav::Get();
	if (!mesh)
		return; // Bot_Move only calls us when Nav::IsLoaded(), but stay safe

	if (bot.hasLastOrigin && (pEdict->v.origin - bot.lastOrigin).Length() > kTeleportJumpThreshold)
		ResetPath(bot); // discontinuous jump -- the old path is meaningless now
	bot.lastOrigin = pEdict->v.origin;
	bot.hasLastOrigin = true;

	if (!bot.hasGoal)
		PickRandomGoal(bot, *mesh);

	if (bot.waypoints.empty() || bot.nextRepathTime <= gpGlobals->time)
	{
		if (!Repath(bot, *mesh, pEdict))
		{
			// Goal unreachable, or a transient point-in-area miss -- drop
			// it and let the next frame pick a fresh one rather than
			// getting stuck retrying the same failing repath forever.
			bot.hasGoal = false;
			return;
		}
		bot.nextRepathTime = gpGlobals->time + kRepathInterval;
	}

	if (bot.waypointIndex >= bot.waypoints.size())
	{
		bot.hasGoal = false; // reached the goal
		ResetPath(bot);
		return;
	}

	const Nav::Waypoint &waypoint = bot.waypoints[bot.waypointIndex];
	Vector toTarget = waypoint.point - pEdict->v.origin;
	float horizDist = sqrtf(toTarget.x * toTarget.x + toTarget.y * toTarget.y);

	if (horizDist < kWaypointReachedDistance)
	{
		++bot.waypointIndex; // pick up the next waypoint next frame with a fresh direction
		return;
	}

	bool onLadder = (pEdict->v.movetype == MOVETYPE_FLY);
	bool needsJump = !onLadder && waypoint.requiresJump;

	if (onLadder)
	{
		// GoldSrc ladder-touch movement reads climb direction from view
		// pitch combined with forward-move input -- the same technique
		// reference bots use (e.g. jk_botti's BotOnLadder): look up to
		// climb up, down to climb down, hold forward either way.
		float climbPitch = (toTarget.z > 0.f) ? -60.f : 60.f;
		bot.desiredAngles = Vector(climbPitch, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
	}
	else if (needsJump)
	{
		// No avoidance deflection while lined up on a jump -- committing to
		// the gap shouldn't get nudged sideways by a nearby bot.
		bot.desiredAngles = Vector(0.f, atan2f(toTarget.y, toTarget.x) * (180.f / kPi), 0.f);
	}
	else
	{
		// Avoidance only applies to normal ground steering -- deflecting a
		// bot mid-climb would fight the ladder's precise alignment needs.
		Vector steerDir = NavAvoid::Deflect(bot, toTarget);
		bot.desiredAngles = Vector(0.f, atan2f(steerDir.y, steerDir.x) * (180.f / kPi), 0.f);
	}

	Vector angles = Bot_UpdateFacing(bot);
	unsigned short buttons = IN_FORWARD;

	if (needsJump)
	{
		bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
		float facingError = fabsf(NormalizeAngle(angles.y - bot.desiredAngles.y));
		if (onGround && horizDist < kJumpTriggerDistance && facingError < kJumpFacingToleranceDeg)
			buttons |= IN_JUMP;

		// Stock HL duck-jumping doesn't add jump velocity -- PM_Jump's
		// ducking branch produces the identical sqrt(2*800*45) impulse as
		// the non-ducking case, it only shrinks the hull (VEC_DUCK_HULL_MIN/
		// MAX). So this is purely for clearance once airborne through a
		// tight gap, the same technique FoxBot and yapb both use, not an
		// attempt to jump higher or further. (A future GameProfile-driven
		// long-jump-module upgrade, per HLSDK's item_longjump/PM_Jump "slj"
		// check, is a real distance boost -- tracked separately, not this.)
		if (pEdict->v.velocity.z > 0.f)
			buttons |= IN_DUCK;
	}

	g_engfuncs.pfnRunPlayerMove(pEdict, angles, kMoveSpeed, 0.f, 0.f, buttons, 0, Bot_FrameMsec());
}
