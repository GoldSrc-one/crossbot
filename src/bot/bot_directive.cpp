#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "../brain/brain_feed.h"
#include "../game/cvars.h"
#include "../game/crossplay.h"
#include "../game/fake_cmd.h"
#include "../game/game_profile.h"
#include "../game/teams.h"
#include "../nav/nav.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_query.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_camp.h"
#include "bot_deploy.h"
#include "bot_fire.h"
#include "bot_directive.h"
#include "bot_goal.h"
#include "bot_grenade.h"
#include "bot_items.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sense.h"
#include "bot_strategic.h"

extern IGameProfile *g_pGameProfile;

namespace
{
	constexpr float kDefaultExpire = 120.f;
	constexpr float kGoWithin = 64.f;
	constexpr float kFollowWithin = 128.f;
	constexpr float kFollowResume = 1.5f;
	constexpr float kFollowSeconds = 60.f;
	constexpr float kHoldWithin = 32.f;
	constexpr float kHoldSeconds = 30.f;
	constexpr float kLookSeconds = 2.f;
	constexpr float kWaitSeconds = 2.f;
	constexpr float kArriveHeight = 72.f;
	constexpr float kRetargetMove = 128.f;
	constexpr float kRetargetShare = 0.3f;
	constexpr float kCommitInterval = 0.5f;
	constexpr int kMaxGoalFails = 3;
	constexpr float kSelectTimeout = 2.f;
	constexpr float kDropSettle = 0.3f;
	constexpr float kDropIgnore = 20.f;
	constexpr float kDropScan = 160.f;
	constexpr float kSpotRadius = 384.f;
	constexpr float kDefaultRules = 300.f;
	constexpr float kFireSeconds = 10.f;
	constexpr float kPlaceWithin = 64.f;
	constexpr float kPlaceStand = 24.f;

	const char *const kStanceNames[] = { "any", "stand", "crouch", "prone" };

	const char *const kStepNames[BotDirective::STEP_COUNT] = {
		"go", "follow", "hold", "look", "select", "drop", "ability", "wait", "fire", "place",
	};

	void Note(CBot &bot, const char *fmt, ...)
	{
		BotDirective::State &st = bot.directive;
		va_list args;
		va_start(args, fmt);
		vsnprintf(st.note, sizeof(st.note), fmt, args);
		va_end(args);
		st.noteTime = gpGlobals->time;
		BOT_VLOG("CrossBot: %s directive %s\n", bot.name, st.note);
	}

	bool LiveTarget(const BotDirective::Target &t)
	{
		if (!t.valid)
			return false;
		if (!t.ent.ent)
			return true;
		edict_t *ent = t.ent.Get();
		return ent && CBotIsLivePlayer(ent) && !(g_pGameProfile && g_pGameProfile->IsDead(ent));
	}

	Vector PointOf(const BotDirective::Target &t)
	{
		edict_t *ent = t.ent.Get();
		return ent ? ent->v.origin : t.point;
	}

	bool Moving(const BotDirective::Target &t)
	{
		return t.ent.ent != nullptr;
	}

	bool Ours(const CBot &bot)
	{
		return bot.hasGoal && bot.goal.active && bot.goal.target.tier == BotGoal::GOAL_DIRECTIVE;
	}

	bool Near(const CBot &bot, const Vector &point, float within)
	{
		Vector d = point - bot.pEdict->v.origin;
		return d.Length2D() <= within && fabsf(d.z) <= kArriveHeight;
	}

	void Face(CBot &bot, const Vector &point)
	{
		Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
		float yaw = 0.f, pitch = 0.f;
		if (!BotAim::LosAngles(eye, point, yaw, pitch))
			return;
		float limit = BotAim::PitchLimit();
		if (pitch > limit)
			pitch = limit;
		else if (pitch < -limit)
			pitch = -limit;
		bot.desiredAngles = Vector(pitch, yaw, 0.f);
	}

	bool Idle(CBot &bot, unsigned short buttons = 0)
	{
		Vector angles = Bot_UpdateFacing(bot);
		bot.moveForward = 0.f;
		bot.moveSide = 0.f;
		Bot_RunPlayerMove(bot, angles, 0.f, 0.f, buttons);
		return true;
	}

	bool IdleLooking(CBot &bot)
	{
		BotSense::ApplyLook(bot);
		return Idle(bot);
	}

	void DropOurGoal(CBot &bot)
	{
		if (Ours(bot))
			BotGoal::Clear(bot);
	}

	void EndOrders(CBot &bot)
	{
		BotCamp::EndDirected(bot);
		BotFire::CancelOrder(bot);
		BotGrenade::CancelOrder(bot);
	}

	void Finish(CBot &bot, bool ok, const char *why)
	{
		BotDirective::State &st = bot.directive;
		const char *name = st.index < st.count ? kStepNames[st.steps[st.index].kind] : "-";
		DropOurGoal(bot);
		EndOrders(bot);
		if (!ok)
		{
			++st.failed;
			Note(bot, "step %d/%d %s failed: %s", st.index + 1, st.count, name, why);
			st.count = 0;
			st.index = 0;
			return;
		}

		++st.done;
		Note(bot, "step %d/%d %s done: %s", st.index + 1, st.count, name, why);
		++st.index;
		st.stepBegun = false;
		if (st.index >= st.count)
		{
			st.count = 0;
			st.index = 0;
		}
	}

	bool CommitTo(CBot &bot, const Nav::NavMesh &mesh, const Vector &where)
	{
		BotDirective::State &st = bot.directive;
		st.nextCommit = gpGlobals->time + kCommitInterval;

		uint32_t areaId = 0;
		Vector point;
		if (!BotStrategic::ResolvePoint(mesh, where, areaId, point))
			return false;

		BotGoal::Candidate cand;
		cand.tier = BotGoal::GOAL_DIRECTIVE;
		cand.areaId = areaId;
		cand.point = point;
		BotGoal::Commit(bot, cand);
		st.committed = where;
		st.resolved = point;
		st.hasResolved = true;
		st.resolvedArea = static_cast<int>(areaId);
		return true;
	}

	enum Travel
	{
		TRAVEL_ARRIVED = 0,
		TRAVEL_MOVING,
		TRAVEL_WAITING,
		TRAVEL_FAILED,
	};

	Travel TravelTo(CBot &bot, const Nav::NavMesh &mesh, const Vector &where, bool moving, float within)
	{
		BotDirective::State &st = bot.directive;
		if (Near(bot, where, within))
			return TRAVEL_ARRIVED;
		if (st.hasResolved && (where - st.committed).Length() < 1.f && (Near(bot, st.resolved, within) ||
			(bot.currentAreaId == st.resolvedArea && (st.resolved - bot.pEdict->v.origin).Length2D() <= within)))
			return TRAVEL_ARRIVED;

		if (st.fails >= kMaxGoalFails)
			return TRAVEL_FAILED;

		bool ours = Ours(bot);
		float slack = std::max(kRetargetMove, kRetargetShare * (where - bot.pEdict->v.origin).Length());
		bool drifted = moving && (where - st.committed).Length() > slack && !Bot_ManeuverActive(bot);
		if (ours && !drifted)
			return TRAVEL_MOVING;

		if (gpGlobals->time < st.nextCommit)
			return ours ? TRAVEL_MOVING : TRAVEL_WAITING;

		if (!CommitTo(bot, mesh, where))
			return TRAVEL_FAILED;
		return TRAVEL_MOVING;
	}

	bool TravelFrame(CBot &bot, Travel travel, const char *what)
	{
		if (travel == TRAVEL_FAILED)
		{
			Finish(bot, false, what);
			return IdleLooking(bot);
		}
		if (travel == TRAVEL_WAITING)
			return IdleLooking(bot);
		return false;
	}

	int PickSpot(const Nav::NavMesh &mesh, const Vector &where)
	{
		int best = -1;
		float bestDist = kSpotRadius;
		for (size_t i = 0; i < mesh.camps.size(); ++i)
		{
			float dist = (mesh.camps[i].pos - where).Length();
			if (dist >= bestDist)
				continue;
			best = static_cast<int>(i);
			bestDist = dist;
		}
		return best;
	}

	int FindWeapon(const CBot &bot, const char *classname)
	{
		if (!g_pGameProfile || !classname || !classname[0])
			return -1;
		const BotInventory &inv = bot.inventory;
		for (int i = 0; i < inv.count; ++i)
		{
			const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(inv.weapons[i].id);
			if (spec && spec->classname && CB_STRICMP(spec->classname, classname) == 0)
				return i;
		}
		return -1;
	}

	void IgnoreDropped(CBot &bot, const char *classname)
	{
		edict_t *ent = nullptr;
		for (;;)
		{
			ent = g_engfuncs.pfnFindEntityInSphere(ent, bot.pEdict->v.origin, kDropScan);
			if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
				break;
			const char *cls = CBotClassOf(ent);
			if (strcmp(cls, "weaponbox") == 0 || (classname[0] && CB_STRICMP(cls, classname) == 0))
				BotItems::NoteIgnored(bot, ent, kDropIgnore);
		}
	}

	const AbilityInfo *FindAbility(const char *name)
	{
		if (!g_pGameProfile)
			return nullptr;
		const AbilityInfo *list = nullptr;
		int count = g_pGameProfile->Abilities(list);
		for (int i = 0; i < count; ++i)
		{
			if (CB_STRICMP(list[i].name, name) == 0)
				return &list[i];
		}
		return nullptr;
	}

	bool RunGo(CBot &bot, const Nav::NavMesh &mesh, BotDirective::Step &step)
	{
		if (!LiveTarget(step.target))
		{
			Finish(bot, false, "target gone");
			return IdleLooking(bot);
		}
		float within = step.within > 0.f ? step.within : kGoWithin;
		Travel travel = TravelTo(bot, mesh, PointOf(step.target), Moving(step.target), within);
		if (travel == TRAVEL_ARRIVED)
		{
			Finish(bot, true, "arrived");
			return false;
		}
		return TravelFrame(bot, travel, "no route");
	}

	bool RunFollow(CBot &bot, const Nav::NavMesh &mesh, BotDirective::Step &step)
	{
		BotDirective::State &st = bot.directive;
		float seconds = step.seconds > 0.f ? step.seconds : kFollowSeconds;
		if (gpGlobals->time - st.stepStart >= seconds)
		{
			Finish(bot, true, "follow time over");
			return false;
		}
		if (!LiveTarget(step.target))
		{
			Finish(bot, false, "leader gone");
			return IdleLooking(bot);
		}

		float within = step.within > 0.f ? step.within : kFollowWithin;
		Vector where = PointOf(step.target);
		if (st.phase == 1 && Near(bot, where, within * kFollowResume))
			return IdleLooking(bot);

		Travel travel = TravelTo(bot, mesh, where, true, within);
		if (travel == TRAVEL_ARRIVED)
		{
			st.phase = 1;
			DropOurGoal(bot);
			return IdleLooking(bot);
		}
		st.phase = 0;
		return TravelFrame(bot, travel, "cannot reach the leader");
	}

	bool RunHold(CBot &bot, const Nav::NavMesh &mesh, BotDirective::Step &step)
	{
		BotDirective::State &st = bot.directive;
		if (!LiveTarget(step.target))
		{
			Finish(bot, false, "target gone");
			return IdleLooking(bot);
		}

		if (st.phase == 0)
		{
			if (step.stance == BotDirective::STANCE_PRONE && !g_pGameProfile->CanProne(bot.pEdict))
			{
				Finish(bot, false, "cannot go prone in this game");
				return IdleLooking(bot);
			}
			st.mark = PickSpot(mesh, PointOf(step.target));
			st.spot = st.mark >= 0 ? mesh.camps[st.mark].pos : PointOf(step.target);
			st.phase = 1;
		}

		if (st.phase == 1)
		{
			Travel travel = TravelTo(bot, mesh, st.spot, false, step.within > 0.f ? step.within : kHoldWithin);
			if (travel != TRAVEL_ARRIVED)
				return TravelFrame(bot, travel, "cannot reach the spot");
			DropOurGoal(bot);
			st.phase = 2;
			st.phaseTime = gpGlobals->time;

			Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
			float yaw = st.mark >= 0 ? mesh.camps[st.mark].yaw : bot.pEdict->v.v_angle.y;
			float pitch = st.mark >= 0 ? mesh.camps[st.mark].pitch : 0.f;
			if (LiveTarget(step.watch))
				BotAim::LosAngles(eye, PointOf(step.watch), yaw, pitch);
			BotCamp::HoldDirected(bot, mesh, st.mark, yaw, pitch, step.stance);
		}

		float seconds = step.seconds > 0.f ? step.seconds : kHoldSeconds;
		if (gpGlobals->time - st.phaseTime >= seconds)
		{
			Finish(bot, true, "held");
			return false;
		}

		if (bot.sense.lookValid && bot.sense.lookAuthority > 0.f)
			BotSense::ApplyLook(bot);
		else if (LiveTarget(step.watch))
			Face(bot, PointOf(step.watch));
		else
			BotSense::ApplyLook(bot);
		return Idle(bot, bot.camp.spot.duck ? IN_DUCK : 0);
	}

	bool RunLook(CBot &bot, BotDirective::Step &step)
	{
		BotDirective::State &st = bot.directive;
		float seconds = step.seconds > 0.f ? step.seconds : kLookSeconds;
		if (!LiveTarget(step.target) || gpGlobals->time - st.stepStart >= seconds)
		{
			Finish(bot, true, "looked");
			return false;
		}
		edict_t *ent = step.target.ent.Get();
		Face(bot, ent ? ent->v.origin + ent->v.view_ofs : step.target.point);
		return Idle(bot);
	}

	bool RunSelect(CBot &bot, BotDirective::Step &step)
	{
		BotDirective::State &st = bot.directive;
		int index = FindWeapon(bot, step.arg);
		if (index < 0)
		{
			Finish(bot, false, "not carrying it");
			return IdleLooking(bot);
		}
		if (bot.inventory.activeIndex == index)
		{
			Finish(bot, true, "selected");
			return false;
		}
		if (gpGlobals->time - st.stepStart > kSelectTimeout)
		{
			Finish(bot, false, "would not switch");
			return IdleLooking(bot);
		}
		if (st.phase == 0)
		{
			const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(bot.inventory.weapons[index].id);
			if (spec)
				g_pGameProfile->SelectWeapon(bot.pEdict, *spec);
			st.phase = 1;
		}
		return IdleLooking(bot);
	}

	bool RunDrop(CBot &bot, BotDirective::Step &step)
	{
		BotDirective::State &st = bot.directive;
		if (st.phase == 0)
		{
			if (step.arg[0] && FindWeapon(bot, step.arg) < 0)
			{
				Finish(bot, false, "not carrying it");
				return IdleLooking(bot);
			}
			char cmd[FakeCmd::kMaxLength];
			snprintf(cmd, sizeof(cmd), step.arg[0] ? "drop %s" : "drop", step.arg);
			FakeCmd::Run(bot.pEdict, cmd);
			st.phase = 1;
			st.phaseTime = gpGlobals->time;
			return Idle(bot);
		}
		if (gpGlobals->time - st.phaseTime < kDropSettle)
			return Idle(bot);
		IgnoreDropped(bot, step.arg);
		Finish(bot, true, "dropped");
		return false;
	}

	unsigned int Successes(const BotAbility::State &a)
	{
		return a.heals + a.repairs + a.builds + a.detpacks + a.discards + a.plants + a.stabs;
	}

	bool RunAbility(CBot &bot, BotDirective::Step &step)
	{
		BotDirective::State &st = bot.directive;
		if (st.phase == 1)
		{
			bool ok = static_cast<int>(Successes(bot.ability)) > st.mark;
			Finish(bot, ok, bot.ability.why ? bot.ability.why : "finished");
			if (ok)
				return false;
			return IdleLooking(bot);
		}

		const AbilityInfo *info = FindAbility(step.arg);
		if (!info)
		{
			Finish(bot, false, "unknown ability");
			return IdleLooking(bot);
		}
		if (info->command)
		{
			char cmd[FakeCmd::kMaxLength];
			char arg[BotDirective::kArgLength] = {};
			if (info->args && !g_pGameProfile->AbilityArg(info->name, step.value, arg, sizeof(arg)))
			{
				Finish(bot, false, "bad argument");
				return IdleLooking(bot);
			}
			snprintf(cmd, sizeof(cmd), info->args ? "%s %s" : "%s", info->command, arg);
			FakeCmd::Run(bot.pEdict, cmd);
			Finish(bot, true, "issued");
			return Idle(bot);
		}
		st.mark = static_cast<int>(Successes(bot.ability));
		if (!g_pGameProfile->StartAbility(bot, info->name, step.value, step.target.ent.Get()))
		{
			Finish(bot, false, bot.ability.why ? bot.ability.why : "cannot start");
			return IdleLooking(bot);
		}
		st.phase = 1;
		return IdleLooking(bot);
	}

	int OffhandSlot(const char *item)
	{
		if (CB_STRICMP(item, "gren1") == 0)
			return 0;
		if (CB_STRICMP(item, "gren2") == 0)
			return 1;
		return -1;
	}

	bool RunFire(CBot &bot, BotDirective::Step &step)
	{
		BotDirective::State &st = bot.directive;
		float now = gpGlobals->time;
		float seconds = step.seconds > 0.f ? step.seconds : kFireSeconds;
		int count = step.count > 0 ? step.count : 1;
		edict_t *ent = step.target.ent.Get();
		if (!LiveTarget(step.target))
		{
			Finish(bot, false, "target gone");
			return IdleLooking(bot);
		}
		if (ent && Teams::Of(bot.pEdict, ent) == Teams::REL_FRIEND)
		{
			Finish(bot, false, "will not shoot a teammate");
			return IdleLooking(bot);
		}
		if (st.holdFire)
		{
			Finish(bot, false, "hold_fire is set");
			return IdleLooking(bot);
		}

		Vector point = ent ? (ent->v.absmin + ent->v.absmax) * 0.5f : step.target.point;
		int slot = OffhandSlot(step.arg);
		const char *why = "cannot fire";
		if (st.phase == 0)
		{
			int id = 0;
			if (slot < 0)
			{
				int index = step.arg[0] ? FindWeapon(bot, step.arg) : bot.inventory.activeIndex;
				if (index < 0 || index >= bot.inventory.count)
				{
					Finish(bot, false, "not carrying it");
					return IdleLooking(bot);
				}
				id = bot.inventory.weapons[index].id;
				if (BotDirective::MeleeOnly(bot) && !g_pGameProfile->MeleeOnlyAllows(id))
				{
					Finish(bot, false, "melee_only is set");
					return IdleLooking(bot);
				}
			}
			else if (BotDirective::MeleeOnly(bot))
			{
				Finish(bot, false, "melee_only is set");
				return IdleLooking(bot);
			}
			st.mark = slot >= 0 ? BotGrenade::Throws(bot) : BotFire::Shots(bot);
			st.phase = slot >= 0 ? 2 : 1;
			st.weaponId = id;
		}

		int done = (st.phase == 2 ? BotGrenade::Throws(bot) : BotFire::Shots(bot)) - st.mark;
		if (done >= count)
		{
			Finish(bot, true, "fired");
			return false;
		}

		bool ordered = st.phase == 2
			? BotGrenade::Order(bot, slot, point, st.stepStart + seconds, why)
			: BotFire::Order(bot, st.weaponId, point, st.stepStart + seconds, why);
		if (!ordered)
		{
			Finish(bot, false, done > 0 ? "fired fewer than asked" : why);
			return IdleLooking(bot);
		}
		if (now - st.stepStart >= seconds)
		{
			Finish(bot, false, done > 0 ? "fired fewer than asked" : "no clear shot");
			return IdleLooking(bot);
		}
		return IdleLooking(bot);
	}

	bool RunPlace(CBot &bot, const Nav::NavMesh &mesh, BotDirective::Step &step)
	{
		BotDirective::State &st = bot.directive;
		if (st.phase == 0)
		{
			if (!LiveTarget(step.target))
			{
				Finish(bot, false, "target gone");
				return IdleLooking(bot);
			}
			if (BotDirective::MeleeOnly(bot))
			{
				Finish(bot, false, "melee_only is set");
				return IdleLooking(bot);
			}
			float within = step.within > 0.f ? step.within : kPlaceWithin;
			Travel travel = TravelTo(bot, mesh, PointOf(step.target), false, within);
			if (travel != TRAVEL_ARRIVED)
				return TravelFrame(bot, travel, "cannot reach the spot");
			DropOurGoal(bot);

			const char *why = "cannot place";
			if (!BotDeploy::Reserve(bot, mesh, step.arg, st.spot, why))
			{
				Finish(bot, false, why);
				return IdleLooking(bot);
			}
			st.phase = 1;
			st.fails = 0;
			st.hasResolved = false;
			st.nextCommit = 0.f;
		}

		if (st.phase == 1)
		{
			Travel travel = TravelTo(bot, mesh, st.spot, false, kPlaceStand);
			if (travel != TRAVEL_ARRIVED)
				return TravelFrame(bot, travel, "cannot reach the mining spot");
			DropOurGoal(bot);
			st.mark = bot.deploy.placements;
			BotDeploy::StartReserved(bot);
			st.phase = 2;
			return IdleLooking(bot);
		}

		if (BotDeploy::Active(bot))
			return IdleLooking(bot);
		if (bot.deploy.placements > st.mark)
		{
			Finish(bot, true, "placed");
			return false;
		}
		Finish(bot, false, bot.deploy.lastReason);
		return IdleLooking(bot);
	}

	bool RunWait(CBot &bot, BotDirective::Step &step)
	{
		float seconds = step.seconds > 0.f ? step.seconds : kWaitSeconds;
		if (gpGlobals->time - bot.directive.stepStart >= seconds)
		{
			Finish(bot, true, "waited");
			return false;
		}
		return IdleLooking(bot);
	}
}

namespace BotDirective
{

const char *StepName(StepKind kind)
{
	return kind < STEP_COUNT ? kStepNames[kind] : "?";
}

bool ParseStep(const char *name, StepKind &out)
{
	for (int i = 0; i < STEP_COUNT; ++i)
	{
		if (name && CB_STRICMP(name, kStepNames[i]) == 0)
		{
			out = static_cast<StepKind>(i);
			return true;
		}
	}
	return false;
}

bool ParseStance(const char *name, Stance &out)
{
	for (int i = 0; i < static_cast<int>(sizeof(kStanceNames) / sizeof(kStanceNames[0])); ++i)
	{
		if (name && CB_STRICMP(name, kStanceNames[i]) == 0)
		{
			out = static_cast<Stance>(i);
			return true;
		}
	}
	return false;
}

bool Active(const CBot &bot)
{
	return bot.directive.count > 0;
}

bool Busy(const CBot &bot)
{
	const State &st = bot.directive;
	if (st.count == 0)
		return false;
	StepKind kind = st.steps[st.index].kind;
	return kind == STEP_ABILITY || kind == STEP_FIRE || kind == STEP_PLACE;
}

void Set(CBot &bot, const Step *steps, int count, float seconds)
{
	State &st = bot.directive;
	DropOurGoal(bot);
	if (count > kMaxSteps)
		count = kMaxSteps;
	for (int i = 0; i < count; ++i)
		st.steps[i] = steps[i];
	st.count = count;
	st.index = 0;
	st.stepBegun = false;
	st.expire = gpGlobals->time + (seconds > 0.f ? seconds : kDefaultExpire);
	Note(bot, "set: %d step(s), first %s, expires in %.0fs", count, count > 0 ? kStepNames[steps[0].kind] : "-",
		st.expire - gpGlobals->time);
}

void Release(CBot &bot, const char *why)
{
	State &st = bot.directive;
	if (st.count == 0)
		return;
	DropOurGoal(bot);
	EndOrders(bot);
	st.count = 0;
	st.index = 0;
	st.stepBegun = false;
	Note(bot, "released: %s", why);
}

void SetRules(CBot &bot, bool meleeOnly, bool holdFire, bool noAutoObjective, float seconds)
{
	State &st = bot.directive;
	st.meleeOnly = meleeOnly;
	st.holdFire = holdFire;
	st.noAutoObjective = noAutoObjective;
	st.rulesUntil = gpGlobals->time + (seconds > 0.f ? seconds : kDefaultRules);
	Note(bot, "rules: melee_only=%d hold_fire=%d no_auto_objective=%d for %.0fs", meleeOnly ? 1 : 0,
		holdFire ? 1 : 0, noAutoObjective ? 1 : 0, st.rulesUntil - gpGlobals->time);
}

void Alert(CBot &bot, const Vector &point, float seconds)
{
	State &st = bot.directive;
	st.alertPoint = point;
	st.alertUntil = gpGlobals->time + seconds;
	Note(bot, "alert at (%.0f,%.0f,%.0f) for %.0fs", point.x, point.y, point.z, seconds);
}

bool Queue(CBot &bot, const char *cmd)
{
	State &st = bot.directive;
	if (!cmd || !cmd[0] || st.pendingCount >= kMaxPending)
		return false;
	snprintf(st.pending[st.pendingCount], kPendingLength, "%s", cmd);
	++st.pendingCount;
	return true;
}

void Think(CBot &bot)
{
	State &st = bot.directive;
	for (int i = 0; i < st.pendingCount; ++i)
	{
		const char *cmd = st.pending[i];
		FakeCmd::Run(bot.pEdict, cmd);
		if (strncmp(cmd, "say_team ", 9) == 0)
			BrainFeed::NoteChat(bot.pEdict, true, cmd + 9);
		else if (strncmp(cmd, "say ", 4) == 0)
			BrainFeed::NoteChat(bot.pEdict, false, cmd + 4);
	}
	st.pendingCount = 0;

	float now = gpGlobals->time;
	if (st.rulesUntil > 0.f && now >= st.rulesUntil)
	{
		st.meleeOnly = false;
		st.holdFire = false;
		st.noAutoObjective = false;
		st.rulesUntil = 0.f;
		Note(bot, "rules expired");
	}
	if (st.count > 0 && now >= st.expire)
		Release(bot, "expired");
}

bool Run(CBot &bot, const Nav::NavMesh &mesh)
{
	State &st = bot.directive;
	if (st.count == 0)
		return false;

	for (int guard = 0; guard < kMaxSteps && st.count > 0; ++guard)
	{
		Step &step = st.steps[st.index];
		if (!st.stepBegun)
		{
			st.stepBegun = true;
			st.stepStart = gpGlobals->time;
			st.phase = 0;
			st.fails = 0;
			st.hasResolved = false;
			st.phaseTime = 0.f;
			st.nextCommit = 0.f;
			Note(bot, "step %d/%d %s", st.index + 1, st.count, kStepNames[step.kind]);
		}

		int before = st.index;
		int beforeCount = st.count;
		bool owned = false;
		switch (step.kind)
		{
		case STEP_GO: owned = RunGo(bot, mesh, step); break;
		case STEP_FOLLOW: owned = RunFollow(bot, mesh, step); break;
		case STEP_HOLD: owned = RunHold(bot, mesh, step); break;
		case STEP_LOOK: owned = RunLook(bot, step); break;
		case STEP_SELECT: owned = RunSelect(bot, step); break;
		case STEP_DROP: owned = RunDrop(bot, step); break;
		case STEP_ABILITY: owned = RunAbility(bot, step); break;
		case STEP_WAIT: owned = RunWait(bot, step); break;
		case STEP_FIRE: owned = RunFire(bot, step); break;
		case STEP_PLACE: owned = RunPlace(bot, mesh, step); break;
		default: Finish(bot, false, "bad step"); break;
		}

		if (owned)
			return true;
		if (st.count == beforeCount && st.index == before)
			return false;
	}

	if (st.count == 0)
		return false;
	return IdleLooking(bot);
}

void OnGoalFailed(CBot &bot, const char *why)
{
	State &st = bot.directive;
	if (st.count == 0)
		return;
	++st.fails;
	Note(bot, "route trouble (%s), %d/%d", why, st.fails, kMaxGoalFails);
}

void OnRespawn(CBot &bot)
{
	State &st = bot.directive;
	if (st.count > 0)
		Release(bot, "died");
	st.alertUntil = 0.f;
}

void OnNewRound(CBot &bot)
{
	State &st = bot.directive;
	Release(bot, "new round");
	st.alertUntil = 0.f;
	if (st.rulesUntil > 0.f)
	{
		st.meleeOnly = false;
		st.holdFire = false;
		st.noAutoObjective = false;
		st.rulesUntil = 0.f;
		Note(bot, "rules cleared for the new round");
	}
}

bool MeleeOnly(const CBot &bot)
{
	if (bot.directive.meleeOnly)
		return true;
	cvar_t *cvar = CBotCvar("crossbot_melee_only");
	return cvar && cvar->value != 0.f;
}

bool HoldFire(const CBot &bot)
{
	return bot.directive.holdFire;
}

bool NoAutoObjective(const CBot &bot)
{
	return bot.directive.noAutoObjective;
}

bool AlertLook(CBot &bot)
{
	const State &st = bot.directive;
	if (st.alertUntil <= gpGlobals->time)
		return false;
	Face(bot, st.alertPoint);
	return true;
}

void Describe(const CBot &bot, char *out, size_t size)
{
	const State &st = bot.directive;
	float now = gpGlobals->time;
	int n = 0;
	if (st.count > 0)
	{
		const Step &step = st.steps[st.index];
		n = snprintf(out, size, "step %d/%d %s", st.index + 1, st.count, kStepNames[step.kind]);
		edict_t *ent = step.target.ent.Get();
		if (n > 0 && static_cast<size_t>(n) < size && ent)
			n += snprintf(out + n, size - n, " %s", CBotPlayerNameOf(ent));
		if (n > 0 && static_cast<size_t>(n) < size && step.arg[0])
			n += snprintf(out + n, size - n, " %s", step.arg);
		if (n > 0 && static_cast<size_t>(n) < size && step.value[0])
			n += snprintf(out + n, size - n, " %s", step.value);
		if (n > 0 && static_cast<size_t>(n) < size)
			n += snprintf(out + n, size - n, ", expires in %.0fs", st.expire - now);
	}
	else
	{
		n = snprintf(out, size, "autonomous");
	}

	if (n > 0 && static_cast<size_t>(n) < size && st.rulesUntil > now)
		n += snprintf(out + n, size - n, "; rules%s%s%s %.0fs", st.meleeOnly ? " melee_only" : "",
			st.holdFire ? " hold_fire" : "", st.noAutoObjective ? " no_auto_objective" : "", st.rulesUntil - now);
	if (n > 0 && static_cast<size_t>(n) < size && st.alertUntil > now)
		n += snprintf(out + n, size - n, "; alert (%.0f,%.0f,%.0f) %.0fs", st.alertPoint.x, st.alertPoint.y,
			st.alertPoint.z, st.alertUntil - now);
	if (n > 0 && static_cast<size_t>(n) < size && st.note[0])
		snprintf(out + n, size - n, "; last: %s (%.0fs ago)", st.note, now - st.noteTime);
}

void PrintStatus(const CBot &bot)
{
	char desc[384];
	Describe(bot, desc, sizeof(desc));
	const Vector &o = bot.pEdict->v.origin;
	char msg[512];
	snprintf(msg, sizeof(msg), "CrossBot: %s at (%.0f,%.0f,%.0f) directive: %s; done=%u failed=%u\n", bot.name,
		o.x, o.y, o.z, desc, bot.directive.done, bot.directive.failed);
	SERVER_PRINT(msg);
}

}
