#include <extdll.h>
#include <meta_api.h>

#include <math.h>

#include "../game/hldm_weapons.h"
#include "../nav/nav.h"
#include "../nav/nav_camp.h"
#include "../nav/nav_format.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_path.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_camp.h"
#include "bot_difficulty.h"
#include "bot_goal.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sense.h"

namespace
{
	constexpr float kDefaultBudget = 3.f;
	constexpr float kDefaultScareRadius = 1024.f;
	constexpr float kDefaultReachRadius = 512.f;

	// Slack so a contact jittering in place does not read as closing in.
	constexpr float kCampScareEpsilon = 16.f;
	// Nothing has come closer for this long -- the scare is over.
	constexpr float kCampStale = 1.f;
	// A committed camp whose route is longer than this was never a sidestep.
	constexpr float kCampMaxTravel = 3.f;
	constexpr float kCampCooldown = 5.f;
	// Duck unless standing sees more than 5% more.
	constexpr float kCampDuckSlack = 0.95f;
	// How much of the zoomed field of view the sweep may fill and still be worth zooming into.
	constexpr float kDefaultZoomShare = 0.85f;
	// Below this the targets are close enough to find without magnification.
	constexpr float kDefaultZoomDist = 768.f;

	// Crouching is the default. A camper is a smaller and steadier target, so the tie and the near
	// tie both go to crouch; only a spot that really needs the extra eye height stands up.
	bool ChooseDuck(const Nav::CampSpot &camp)
	{
		if ((camp.flags & crossbot_nav::CAMP_NO_STAND) != 0)
			return true;

		return camp.crouchScore >= kCampDuckSlack * camp.standScore;
	}

	// The spot's spread is the RMS angle its visible targets sit off the measured facing, so twice
	// it is the arc the camper actually has to sweep. Zoom when that sweep fits inside the scope
	// and the targets are far enough away for the magnification to buy anything.
	bool ChooseZoom(const Nav::CampSpot &camp, bool duck, float zoomFov)
	{
		if (zoomFov <= 0.f)
			return false;

		float spread = duck ? camp.crouchSpread : camp.standSpread;
		float dist = duck ? camp.crouchDist : camp.standDist;
		if (spread <= 0.f || dist < BotCamp::ZoomDist())
			return false;

		return spread * 2.f <= zoomFov * BotCamp::ZoomShare();
	}

	// The scope of whatever the bot is holding when the route is built, 0 when it has none.
	float HeldZoomFov(const CBot &bot)
	{
		const BotInventory &inv = bot.inventory;
		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
			return 0.f;

		const WeaponSpec *spec = HLDM_WeaponSpec(inv.weapons[inv.activeIndex].id);
		return spec ? spec->zoomFov : 0.f;
	}

	float CvarValue(const char *name, float fallback)
	{
		cvar_t *cvar = g_engfuncs.pfnCVarGetPointer(name);
		float v = cvar ? cvar->value : fallback;
		return v < 0.f ? 0.f : v;
	}
}

namespace BotCamp
{

bool Enabled()
{
	return BotDiff::Camp();
}

bool DebugEnabled()
{
#if BOT_VLOG_ON
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_camp_debug");
	return cvar && cvar->value != 0.f;
#else
	return false;
#endif
}

float Budget()
{
	return CvarValue("crossbot_camp_budget", kDefaultBudget);
}

float ScareRadius()
{
	return CvarValue("crossbot_camp_radius", kDefaultScareRadius);
}

float ReachRadius()
{
	return CvarValue("crossbot_camp_reach", kDefaultReachRadius);
}

float ZoomShare()
{
	return CvarValue("crossbot_camp_zoom", kDefaultZoomShare);
}

float ZoomDist()
{
	return CvarValue("crossbot_camp_zoom_dist", kDefaultZoomDist);
}

bool Holding(const CBot &bot)
{
	return bot.camp.holding;
}

void BuildRoute(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot)
{
	State &st = bot.camp;
	st.route.clear();

	if (!Enabled() || mesh.camps.empty() || bot.currentAreaId < 0)
		return;

	const std::vector<float> &toGoal = bot.goal.toTargetCosts;
	if (static_cast<size_t>(bot.currentAreaId) >= toGoal.size())
		return;

	float here = toGoal[bot.currentAreaId];
	if (here >= Nav::kUnreachableSeconds)
		return;

	float budget = Budget();
	float zoomFov = HeldZoomFov(bot);

	for (size_t i = 0; i < mesh.camps.size(); ++i)
	{
		const Nav::CampSpot &camp = mesh.camps[i];
		if (camp.areaId < 0)
			continue;

		size_t area = static_cast<size_t>(camp.areaId);
		if (area >= toGoal.size() || area >= fromBot.size())
			continue;

		float in = toGoal[area];
		float out = fromBot[area];
		if (in >= Nav::kUnreachableSeconds || out >= Nav::kUnreachableSeconds)
			continue;
		// The gather-detour rule verbatim: a camp may never cost the bot ground on its target.
		if (in >= here)
			continue;
		if (out + in - here > budget)
			continue;

		RouteSpot spot;
		spot.campIndex = static_cast<int>(i);
		spot.areaId = camp.areaId;
		spot.pos = camp.pos;
		spot.yaw = camp.yaw;
		spot.pitch = camp.pitch;
		spot.toGoal = in;
		spot.flags = camp.flags;
		spot.duck = ChooseDuck(camp);
		spot.zoom = ChooseZoom(camp, spot.duck, zoomFov);
		st.route.push_back(spot);
	}

	// Retargeting is frequent -- a moving strategic goal rebuilds this four times a second -- so
	// this one line is the reason crossbot_camp_debug exists.
	if (DebugEnabled())
		BOT_VLOG("CrossBot: %s prepares %zu camp spots along the way of %zu, %.1fs from the target\n",
			bot.name, st.route.size(), mesh.camps.size(), here);
}

void Track(CBot &bot)
{
	State &st = bot.camp;

	// The zoom we owe the weapon outlives the camp that asked for it, so settle it first and
	// independently of whether camping is still switched on.
	if (st.unzoom && !bot.fire.zoomed)
		st.unzoom = false;

	if (!Enabled() || !BotSense::Enabled() || !bot.pEdict)
	{
		st.scared = false;
		return;
	}

	bool was = st.scared;

	float dist = 0.f;
	const BotSense::Suspicion *contact = BotSense::NearestLive(bot, bot.pEdict->v.origin,
		ScareRadius(), true, &dist);

	// NearestLive has already applied the sense decay, so anything it returns is a contact the bot
	// still believes in. Nothing further gates this on the slot's own age: footsteps arrive in
	// bursts, and an age test tight enough to mean anything flickers the whole tracker on and off
	// between them. Distance is measured from the bot, so walking toward a remembered position also
	// reads as closing in -- which is the right answer for somewhere the bot heard something a
	// moment ago, and the decay is what stops it being the answer forever.
	if (contact)
	{
		if (!st.scared || dist < st.scareDist - kCampScareEpsilon)
		{
			st.scared = true;
			st.scareDist = dist;
			st.scareTime = gpGlobals->time;
		}
	}

	// Nothing has come closer for a while. The running minimum resets with the scare, so the same
	// contact can frighten the bot again if it starts closing a second time.
	if (st.scared && gpGlobals->time - st.scareTime > kCampStale)
		st.scared = false;

	if (DebugEnabled() && st.scared != was)
		BOT_VLOG("CrossBot: %s %s\n", bot.name, st.scared
			? "picks up something closing in" : "stops hearing anything come closer");
}

bool Consider(CBot &bot)
{
	State &st = bot.camp;

	if (bot.goal.stop.camp)
	{
		// Already on a camp leg. A stale scare deliberately does NOT abandon the approach: the
		// stale rule is about how long to *stay*, and applying it to the walk as well meant the
		// bot gave up on every camp it ever picked -- a spot 350 units out is a second away, which
		// is the whole window. Arriving somewhere it was going anyway costs nothing, because the
		// camp was only eligible if it was already on the way to the target.
		if (!st.holding && !Enabled())
		{
			Abandon(bot, "camping is off");
			BotGoal::Advance(bot);
			return true;
		}
		return false;
	}

	if (!Enabled() || !st.scared || st.route.empty())
		return false;
	if (!bot.hasGoal || !bot.goal.active || bot.currentAreaId < 0)
		return false;
	if (gpGlobals->time < st.nextCampTime)
		return false;
	if (Bot_ManeuverActive(bot))
		return false;

	const std::vector<float> &toGoal = bot.goal.toTargetCosts;
	if (static_cast<size_t>(bot.currentAreaId) >= toGoal.size())
		return false;

	float here = toGoal[bot.currentAreaId];
	if (here >= Nav::kUnreachableSeconds)
		return false;

	float reach = ReachRadius();
	const RouteSpot *best = nullptr;
	float bestDist = 0.f;

	for (const RouteSpot &spot : st.route)
	{
		// The live half of the detour rule. toGoal is a fixed field cached on the bot, so re-testing
		// it as the bot advances is free -- that is the whole reason the route set is prepared.
		if (spot.toGoal >= here)
			continue;

		float d = (spot.pos - bot.pEdict->v.origin).Length();
		if (d > reach)
			continue;
		if (!best || d < bestDist)
		{
			best = &spot;
			bestDist = d;
		}
	}

	if (!best)
		return false;

	st.spot = *best;
	st.checkTravel = true;
	++st.camps;

	BOT_VLOG("CrossBot: %s is scared (%.0f out, closing %.1fs ago) and camps at (%.1f,%.1f,%.1f) "
		"area %d, %.0f away, %.1fs to the target from there (was %.1fs), duck=%d zoom=%d\n",
		bot.name, st.scareDist, gpGlobals->time - st.scareTime, best->pos.x, best->pos.y,
		best->pos.z, best->areaId, bestDist, best->toGoal, here, best->duck ? 1 : 0,
		best->zoom ? 1 : 0);

	BotGoal::CommitCampStop(bot, best->pos, best->areaId, best->campIndex);
	Bot_ResetNavPath(bot);
	return true;
}

bool OnArrived(CBot &bot)
{
	State &st = bot.camp;

	st.holding = true;
	st.holdStart = gpGlobals->time;
	st.checkTravel = false;
	if (st.spot.zoom)
		st.unzoom = true;

	// Re-seed the tracker from the camp itself. "Stale for more than a second" is a rule about
	// camping and the bot has only just started, and the running minimum it carried here was
	// measured from wherever it heard the noise rather than from where it is now. With nothing in
	// radius the scare flag is left alone, so a bot that arrives to silence leaves again at once.
	float dist = 0.f;
	if (BotSense::NearestLive(bot, bot.pEdict->v.origin, ScareRadius(), true, &dist))
	{
		st.scared = true;
		st.scareDist = dist;
	}
	st.scareTime = gpGlobals->time;

	BOT_VLOG("CrossBot: %s settles into its camp at (%.1f,%.1f,%.1f), watching yaw %.0f pitch %.0f, "
		"%s%s\n", bot.name, st.spot.pos.x, st.spot.pos.y, st.spot.pos.z, st.spot.yaw, st.spot.pitch,
		st.spot.duck ? "crouched" : "standing", st.spot.zoom ? ", zoomed in" : "");
	return true;
}

bool Run(CBot &bot)
{
	State &st = bot.camp;
	if (!st.holding)
		return false;

	if (!Enabled() || !bot.goal.stop.camp)
	{
		Leave(bot, "camping is off");
		BotGoal::Advance(bot);
		return false;
	}

	if (!st.scared)
	{
		Leave(bot, "nothing has come closer");
		BotGoal::Advance(bot);
		return false;
	}

	// Nothing below plans a route, so keep the stall detector from reading a bot holding a corner
	// as a bot stuck against geometry.
	bot.lastProgressTime = gpGlobals->time;

	// Default the head to the facing the spot was measured at, then let ApplyLook blend away from
	// it toward whatever the bot can hear. The camp direction is the base, not an override -- a
	// camper that ignored the noise it is camping against would be worse than one that never camped.
	float yaw = 0.f, pitch = 0.f;
	if (Facing(bot, yaw, pitch))
		bot.desiredAngles = Vector(pitch, yaw, 0.f);

	BotSense::ApplyLook(bot);
	Vector angles = Bot_UpdateFacing(bot);

	unsigned short buttons = 0;
	if (st.spot.duck)
		buttons |= IN_DUCK;

	bot.moveForward = 0.f;
	bot.moveSide = 0.f;
	Bot_RunPlayerMove(bot, angles, 0.f, 0.f, buttons);
	return true;
}

bool CheckTravel(CBot &bot, float seconds)
{
	State &st = bot.camp;
	if (!st.checkTravel)
		return false;

	st.checkTravel = false;
	if (seconds <= kCampMaxTravel)
		return false;

	// The reach radius is euclidean, so it cannot tell a camp across the room from a camp across a
	// chasm. The planned route can, and it has already been built by the time we get here.
	Abandon(bot, "the route to it was longer than it looked");
	return true;
}

void Leave(CBot &bot, const char *why)
{
	State &st = bot.camp;
	if (!st.holding)
		return;

	BOT_VLOG("CrossBot: %s leaves its camp after %.1fs (%s)\n", bot.name,
		gpGlobals->time - st.holdStart, why);

	st.holding = false;
	st.checkTravel = false;
	st.nextCampTime = gpGlobals->time + kCampCooldown;
}

void Abandon(CBot &bot, const char *why)
{
	State &st = bot.camp;

	BOT_VLOG("CrossBot: %s gives up on its camp at (%.1f,%.1f,%.1f) (%s)\n", bot.name,
		st.spot.pos.x, st.spot.pos.y, st.spot.pos.z, why);

	st.holding = false;
	st.checkTravel = false;
	st.nextCampTime = gpGlobals->time + kCampCooldown;
}

void Reset(CBot &bot)
{
	State &st = bot.camp;
	st.route.clear();
	st.holding = false;
	st.checkTravel = false;
	st.spot = RouteSpot();
}

bool Facing(const CBot &bot, float &yaw, float &pitch)
{
	const State &st = bot.camp;
	if (!st.holding)
		return false;

	yaw = Bot_NormalizeAngle(st.spot.yaw);
	pitch = st.spot.pitch;
	return true;
}

bool WantZoom(const CBot &bot, bool &want)
{
	const State &st = bot.camp;

	if (st.holding)
	{
		want = st.spot.zoom;
		return true;
	}

	// The camp is over but the scope is still up. One opinion, held until the weapon agrees.
	if (st.unzoom)
	{
		want = false;
		return true;
	}

	return false;
}

void OnRespawn(CBot &bot)
{
	State &st = bot.camp;
	Reset(bot);
	st.scared = false;
	st.scareDist = 0.f;
	st.scareTime = 0.f;
	st.unzoom = false;
	st.nextCampTime = 0.f;
}

}
