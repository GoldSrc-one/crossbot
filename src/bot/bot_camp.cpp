#include <extdll.h>
#include <meta_api.h>

#include <math.h>

#include "../game/game_profile.h"
#include "../game/game_profile.h"
#include "../game/teams.h"
#include "../nav/nav.h"
#include "../nav/nav_camp.h"
#include "../nav/nav_format.h"
#include "../nav/nav_math.h"
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

	constexpr float kCampScareEpsilon = 16.f;
	constexpr float kCampStale = 1.f;
	constexpr float kCampMaxTravel = 3.f;
	constexpr float kCampCooldown = 5.f;
	constexpr float kCampDuckSlack = 0.95f;
	constexpr float kDefaultZoomShare = 0.85f;
	constexpr float kDefaultZoomDist = 768.f;
	constexpr float kFriendCheckInterval = 0.2f;
	constexpr float kFriendFallbackCone = 90.f;

	bool ChooseDuck(const Nav::CampSpot &camp)
	{
		if ((camp.flags & crossbot_nav::CAMP_NO_STAND) != 0)
			return true;

		return camp.crouchScore >= kCampDuckSlack * camp.standScore;
	}

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

	float HeldZoomFov(const CBot &bot)
	{
		const BotInventory &inv = bot.inventory;
		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
			return 0.f;

		const WeaponSpec *spec = g_pGameProfile ? g_pGameProfile->GetWeaponSpec(inv.weapons[inv.activeIndex].id) : nullptr;
		return spec ? spec->zoomFov : 0.f;
	}

	float CvarValue(const char *name, float fallback)
	{
		cvar_t *cvar = g_engfuncs.pfnCVarGetPointer(name);
		float v = cvar ? cvar->value : fallback;
		return v < 0.f ? 0.f : v;
	}

	bool FriendInView(const CBot &bot, float range)
	{
		if (!Teams::Active(bot.pEdict))
			return false;
		if (!CBotIsLivePlayer(bot.pEdict))
			return false;

		edict_t *pEdict = bot.pEdict;
		Vector eye = pEdict->v.origin + pEdict->v.view_ofs;

		MAKE_VECTORS(pEdict->v.v_angle);
		Vector forward = gpGlobals->v_forward;

		float fov = pEdict->v.fov;
		if (fov <= 0.f)
			fov = kFriendFallbackCone;
		float minDot = cosf(fov * 0.5f * NavMath::kPiF / 180.f);

		edict_t *closest = nullptr;
		float closestDist = range;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent == pEdict || !CBotIsLivePlayer(ent))
				continue;
			if (g_pGameProfile && g_pGameProfile->IsDead(ent))
				continue;
			if (Teams::Of(pEdict, ent) != Teams::REL_FRIEND)
				continue;

			Vector centre = ent->v.origin + ent->v.view_ofs;
			Vector los = centre - eye;
			float dist = los.Length();
			if (dist > closestDist || dist < 0.001f)
				continue;

			Vector dir = los * (1.f / dist);
			if (DotProduct(forward, dir) < minDot)
				continue;

			closest = ent;
			closestDist = dist;
		}

		if (!closest)
			return false;

		TraceResult tr;
		Vector centre = closest->v.origin + closest->v.view_ofs;
		TRACE_LINE(eye, centre, dont_ignore_monsters, pEdict, &tr);
		return tr.flFraction >= 1.f || tr.pHit == closest;
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

	if (DebugEnabled())
		BOT_VLOG("CrossBot: %s prepares %zu camp spots along the way of %zu, %.1fs from the target\n",
			bot.name, st.route.size(), mesh.camps.size(), here);
}

void Track(CBot &bot)
{
	State &st = bot.camp;

	if (st.unzoom && !bot.fire.zoomed)
		st.unzoom = false;

	if (!Enabled() || !BotSense::Enabled() || !bot.pEdict)
	{
		st.scared = false;
		st.friendInView = false;
		return;
	}

	bool was = st.scared;

	float radius = ScareRadius();
	float dist = 0.f;
	const BotSense::Suspicion *contact = BotSense::NearestLive(bot, bot.pEdict->v.origin,
		radius, true, &dist);

	if (contact || st.scared || st.holding)
	{
		if (gpGlobals->time >= st.nextFriendCheck)
		{
			st.friendInView = FriendInView(bot, radius);
			st.nextFriendCheck = gpGlobals->time + kFriendCheckInterval;
		}
	}
	else
		st.friendInView = false;

	if (st.friendInView)
		st.scared = false;
	else if (contact)
	{
		if (!st.scared || dist < st.scareDist - kCampScareEpsilon)
		{
			st.scared = true;
			st.scareDist = dist;
			st.scareTime = gpGlobals->time;
		}
	}

	if (st.scared && gpGlobals->time - st.scareTime > kCampStale)
		st.scared = false;

	if (DebugEnabled() && st.scared != was)
		BOT_VLOG("CrossBot: %s %s\n", bot.name, st.scared
			? "picks up something closing in" : (st.friendInView
				? "calms down, a friend is in view"
				: "stops hearing anything come closer"));
}

bool Consider(CBot &bot)
{
	State &st = bot.camp;

	if (bot.goal.stop.camp)
	{
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

	if (st.friendInView)
	{
		Leave(bot, "a friend is in view");
		BotGoal::Advance(bot);
		return false;
	}

	if (!st.scared)
	{
		Leave(bot, "nothing has come closer");
		BotGoal::Advance(bot);
		return false;
	}

	bot.lastProgressTime = gpGlobals->time;

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
	st.friendInView = false;
	st.nextFriendCheck = 0.f;
	st.unzoom = false;
	st.nextCampTime = 0.f;
}

}
