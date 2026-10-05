#include <extdll.h>
#include <meta_api.h>

#include <math.h>
#include <stdio.h>

#include "../game/game_profile.h"
#include "../game/objectives.h"
#include "../game/teams.h"
#include "../nav/nav.h"
#include "../nav/nav_camp.h"
#include "../nav/nav_format.h"
#include "../nav/nav_math.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_path.h"
#include "../nav/nav_query.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_camp.h"
#include "bot_difficulty.h"
#include "bot_goal.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sense.h"
#include "bot_strategic.h"
#include "bot_tactics.h"

namespace
{
	constexpr float kBudget = 3.f;
	constexpr float kScareRadius = 1024.f;
	constexpr float kReachRadius = 512.f;

	constexpr float kCampScareEpsilon = 16.f;
	constexpr float kCampStale = 1.f;
	constexpr float kCampMaxTravel = 3.f;
	constexpr float kCampCooldown = 5.f;
	constexpr float kCampDuckSlack = 0.95f;
	constexpr float kZoomShare = 0.85f;
	constexpr float kZoomDist = 768.f;
	constexpr float kFriendCheckInterval = 0.2f;
	constexpr float kFriendFallbackCone = 90.f;
	constexpr float kSpotForever = 1.0e9f;
	constexpr float kSpotReuse = 30.f;
	constexpr float kDeployTraverse = 45.f;
	constexpr float kStandEyeLift = 46.f;
	constexpr float kDuckEyeLift = 12.f;
	constexpr float kSnipeCheckInterval = 0.5f;

	BotCamp::Bipod ChooseBipod(const CBot &bot, const Nav::CampSpot &camp)
	{
		if (!g_pGameProfile || !g_pGameProfile->CanDeploy(bot.pEdict))
			return BotCamp::BIPOD_NONE;
		if ((camp.flags & crossbot_nav::CAMP_DEPLOY) != 0)
			return BotCamp::BIPOD_SANDBAG;
		return camp.standScore <= camp.crouchScore ? BotCamp::BIPOD_PRONE : BotCamp::BIPOD_NONE;
	}

	bool Faces(const Vector &pos, float yaw, float arc, const Vector &threat)
	{
		Vector d = threat - pos;
		if (d.x == 0.f && d.y == 0.f)
			return true;
		float toward = atan2f(d.y, d.x) * 180.f / NavMath::kPiF;
		return fabsf(Bot_NormalizeAngle(toward - yaw)) <= arc;
	}

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

	void FillSpot(const CBot &bot, const Nav::CampSpot &camp, BotCamp::RouteSpot &spot, float zoomFov)
	{
		spot.flags = camp.flags;
		spot.arc = camp.arc;
		spot.bipod = ChooseBipod(bot, camp);
		spot.duck = spot.bipod == BotCamp::BIPOD_NONE && ChooseDuck(camp);
		spot.zoom = spot.bipod == BotCamp::BIPOD_NONE && ChooseZoom(camp, spot.duck, zoomFov);
	}

	bool NowClosest(CBot &bot)
	{
		BotCamp::State &st = bot.camp;
		if (st.spot.snipeId <= 0 || gpGlobals->time < st.nextSnipeCheck)
			return false;

		st.nextSnipeCheck = gpGlobals->time + kSnipeCheckInterval;
		return BotTactics::Closest(bot);
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
	return kBudget;
}

float ScareRadius()
{
	return kScareRadius;
}

float ReachRadius()
{
	return kReachRadius;
}

float ZoomShare()
{
	return kZoomShare;
}

float ZoomDist()
{
	return kZoomDist;
}

bool Holding(const CBot &bot)
{
	return bot.camp.holding;
}

void BuildRoute(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot)
{
	State &st = bot.camp;
	st.route.clear();

	if (!Enabled() || mesh.camps.empty() || bot.currentAreaId < 0 || BotStrategic::Urgent(bot))
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
		if (camp.areaId < 0 || !SpotAllowed(bot, camp))
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
		FillSpot(bot, camp, spot, zoomFov);
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

	if (st.charging && !bot.goal.stop.camp)
		st.charging = false;

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
		if (!st.scared || (contact->time > st.scareTime && dist < st.scareDist - kCampScareEpsilon))
		{
			st.scared = true;
			st.scareDist = dist;
			st.scarePos = contact->pos;
			st.scareTime = gpGlobals->time;
		}
	}

	if (st.scared && gpGlobals->time - st.scareTime > kCampStale)
		st.scared = false;

	if (!st.scared)
		st.rolled = false;

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
		if (st.charging && !st.scared)
		{
			st.charging = false;
			BotGoal::Advance(bot);
			return true;
		}
		if (!st.holding && st.spot.timed && st.spot.raceOut > 0.f && st.scared)
		{
			NoteSpot(bot, st.spot.campIndex, kSpotForever);
			Abandon(bot, "something closed in before it got there");
			BotGoal::Advance(bot);
			return true;
		}
		if (!st.holding && !st.spot.timed && !Enabled())
		{
			Abandon(bot, "camping is off");
			BotGoal::Advance(bot);
			return true;
		}
		if (!st.holding && NowClosest(bot))
		{
			Abandon(bot, "it is now the closest to the target");
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

	if (!st.rolled)
	{
		st.rolled = true;
		const Nav::NavMesh *mesh = Nav::Get();
		float ready = g_pGameProfile ? g_pGameProfile->EvaluateCombat(bot.pEdict, bot.inventory) : 0.f;
		if (mesh && g_pGameProfile && (g_pGameProfile->CreepButtons() != 0 || g_pGameProfile->CreepSpeed() > 0.f) &&
			g_engfuncs.pfnRandomFloat(0.f, 1.f) < ready)
		{
			const Nav::Area *area = Nav::FindAreaNearest(*mesh, st.scarePos);
			if (area)
			{
				st.spot = RouteSpot();
				st.spot.pos = st.scarePos;
				st.spot.areaId = static_cast<int>(area->id);
				st.charging = true;

				BOT_VLOG("CrossBot: %s is scared (%.0f out) but ready (%.2f) and charges at (%.1f,%.1f,%.1f)\n",
					bot.name, st.scareDist, ready, st.scarePos.x, st.scarePos.y, st.scarePos.z);

				BotGoal::CommitCampStop(bot, st.scarePos, st.spot.areaId, -1);
				Bot_ResetNavPath(bot);
				return true;
			}
		}
	}

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
		if ((spot.flags & crossbot_nav::CAMP_DEPLOY) != 0 && !Faces(spot.pos, spot.yaw, spot.arc, st.scarePos))
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

	if (st.charging)
	{
		st.charging = false;
		st.nextCampTime = gpGlobals->time + kCampCooldown;
		return BotGoal::Advance(bot);
	}

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
		"%s%s%s\n", bot.name, st.spot.pos.x, st.spot.pos.y, st.spot.pos.z, st.spot.yaw, st.spot.pitch,
		st.spot.bipod == BIPOD_SANDBAG ? "deploying on sandbags" : st.spot.bipod == BIPOD_PRONE ? "deploying prone" :
		st.spot.duck ? "crouched" : "standing", st.spot.zoom ? ", zoomed in" : "", "");
	return true;
}

bool Run(CBot &bot)
{
	State &st = bot.camp;
	if (!st.holding)
		return false;

	if ((!Enabled() && !st.spot.timed) || !bot.goal.stop.camp)
	{
		Leave(bot, "camping is off");
		BotGoal::Advance(bot);
		return false;
	}

	if (st.spot.timed)
	{
		if (gpGlobals->time - st.holdStart >= st.spot.holdFor)
		{
			Leave(bot, "its time is up");
			BotGoal::Advance(bot);
			return false;
		}
		if (NowClosest(bot))
		{
			Leave(bot, "it is now the closest to the target");
			BotGoal::Advance(bot);
			return false;
		}
	}
	else if (st.friendInView)
	{
		Leave(bot, "a friend is in view");
		BotGoal::Advance(bot);
		return false;
	}
	else if (!st.scared)
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
	if (st.spot.timed)
	{
		if (st.spot.raceOut <= 0.f || seconds + st.spot.raceLead < st.spot.raceRunner * seconds / st.spot.raceOut)
			return false;

		BOT_VLOG("CrossBot: %s lost the race to spot %d (%.1fs planned, %.1fs estimated, runner %.1fs, lead %.1fs), "
			"not trying it again this life\n", bot.name, st.spot.campIndex, seconds, st.spot.raceOut,
			st.spot.raceRunner, st.spot.raceLead);
		NoteSpot(bot, st.spot.campIndex, kSpotForever);
		Abandon(bot, "it lost the race");
		return true;
	}
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
	st.charging = false;
	st.rolled = false;
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

void CommitTimed(CBot &bot, const Nav::NavMesh &mesh, int campIndex, float holdFor, float runner, float out,
	float lead)
{
	State &st = bot.camp;
	const Nav::CampSpot &camp = mesh.camps[campIndex];
	const std::vector<float> &toGoal = bot.goal.toTargetCosts;

	RouteSpot spot;
	spot.campIndex = campIndex;
	spot.areaId = camp.areaId;
	spot.pos = camp.pos;
	spot.yaw = camp.yaw;
	spot.pitch = camp.pitch;
	spot.toGoal = (camp.areaId >= 0 && static_cast<size_t>(camp.areaId) < toGoal.size())
		? toGoal[camp.areaId] : Nav::kUnreachableSeconds;
	FillSpot(bot, camp, spot, HeldZoomFov(bot));
	spot.timed = true;
	spot.holdFor = holdFor;
	spot.raceRunner = runner;
	spot.raceOut = out;
	spot.raceLead = lead;

	st.holding = false;
	st.spot = spot;
	st.checkTravel = true;
	NoteSpot(bot, campIndex, kSpotReuse);

	BotGoal::CommitCampStop(bot, camp.pos, camp.areaId, campIndex);
}

void CommitSnipe(CBot &bot, const Nav::NavMesh &mesh, int campIndex, float holdFor, int weaponId,
	const Vector &watch)
{
	CommitTimed(bot, mesh, campIndex, holdFor, 0.f, 0.f, 0.f);

	RouteSpot &spot = bot.camp.spot;
	spot.snipeId = weaponId;
	bot.camp.nextSnipeCheck = 0.f;
	BotAim::LosAngles(SpotEye(mesh.camps[campIndex]), watch, spot.yaw, spot.pitch);

	const WeaponSpec *spec = g_pGameProfile ? g_pGameProfile->GetWeaponSpec(weaponId) : nullptr;
	spot.zoom = spot.bipod == BIPOD_NONE && spec && spec->zoomFov > 0.f;
}

int SnipeId(const CBot &bot)
{
	return bot.goal.stop.camp ? bot.camp.spot.snipeId : 0;
}

void NoteSpot(CBot &bot, int campIndex, float seconds)
{
	float ready = gpGlobals->time + seconds;
	for (SpotMemo &m : bot.camp.memo)
	{
		if (m.campIndex != campIndex)
			continue;
		if (ready > m.readyTime)
			m.readyTime = ready;
		return;
	}

	SpotMemo memo;
	memo.campIndex = campIndex;
	memo.readyTime = ready;
	bot.camp.memo.push_back(memo);
}

bool SpotReady(const CBot &bot, int campIndex)
{
	for (const SpotMemo &m : bot.camp.memo)
	{
		if (m.campIndex == campIndex)
			return gpGlobals->time >= m.readyTime;
	}
	return true;
}

float SpotDist(const Nav::CampSpot &camp)
{
	return ChooseDuck(camp) ? camp.crouchDist : camp.standDist;
}

Vector SpotEye(const Nav::CampSpot &camp)
{
	return camp.pos + Vector(0.f, 0.f, ChooseDuck(camp) ? kDuckEyeLift : kStandEyeLift);
}

void OnRespawn(CBot &bot)
{
	State &st = bot.camp;
	Reset(bot);
	st.memo.clear();
	st.scared = false;
	st.scareDist = 0.f;
	st.scareTime = 0.f;
	st.friendInView = false;
	st.nextFriendCheck = 0.f;
	st.unzoom = false;
	st.nextCampTime = 0.f;
}

bool SpotAllowed(const CBot &bot, const Nav::CampSpot &camp)
{
	if ((camp.flags & crossbot_nav::CAMP_DEPLOY) == 0)
		return true;
	if (!g_pGameProfile || !g_pGameProfile->CanDeploy(bot.pEdict))
		return false;

	const Objectives::Sandbag *bag = Objectives::FindSandbag(camp.entIndex);
	return bag && bag->open;
}

bool FacesThreat(const Nav::CampSpot &camp, const Vector &threat)
{
	if ((camp.flags & crossbot_nav::CAMP_DEPLOY) == 0)
		return true;
	return Faces(camp.pos, camp.yaw, camp.arc, threat);
}

Bipod HeldBipod(const CBot &bot)
{
	return bot.camp.holding ? bot.camp.spot.bipod : BIPOD_NONE;
}

void AddSandbags()
{
	const Nav::NavMesh *mesh = Nav::Get();
	if (!mesh)
		return;

	int added = 0;
	const std::vector<Objectives::Sandbag> &bags = Objectives::Get().sandbags;
	for (const Objectives::Sandbag &bag : bags)
	{
		edict_t *ent = bag.ent.Get();
		if (!ent)
			continue;

		std::vector<BotItems::Placement> places;
		BotItems::VolumeAreas(*mesh, ent, bag.point, places);

		const BotItems::Placement *best = nullptr;
		for (const BotItems::Placement &place : places)
		{
			if (!best || (place.point - bag.point).LengthSquared() < (best->point - bag.point).LengthSquared())
				best = &place;
		}
		if (!best)
			continue;

		Nav::CampSpot spot;
		spot.pos = best->point;
		spot.yaw = bag.yaw;
		spot.score = 1.f;
		spot.standScore = 1.f;
		spot.areaId = static_cast<int>(best->areaId);
		spot.flags = crossbot_nav::CAMP_DEPLOY;
		spot.arc = bag.range + kDeployTraverse;
		spot.entIndex = g_engfuncs.pfnIndexOfEdict(ent);
		Nav::AddCamp(spot);
		++added;
	}

	if (!bags.empty())
	{
		char msg[128];
		snprintf(msg, sizeof(msg), "CrossBot: %d of %d sandbags became camp spots.\n", added,
			static_cast<int>(bags.size()));
		SERVER_PRINT(msg);
	}
}

}
