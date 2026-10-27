#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cmath>

#include "bot.h"
#include "../game/cvars.h"
#include "bot_aim.h"
#include "bot_difficulty.h"
#include "bot_fire.h"
#include "bot_grenade.h"
#include "bot_log.h"
#include "bot_sense.h"
#include "../game/fake_cmd.h"
#include "../game/game_profile.h"
#include "../game/teams.h"

namespace
{
	constexpr float kDecideInterval = 0.25f;
	constexpr float kTargetFresh = 0.3f;
	constexpr float kPrimeChance = 0.35f;
	constexpr float kCooldownMin = 4.f;
	constexpr float kCooldownMax = 9.f;
	constexpr float kPrimeConfirm = 0.3f;
	constexpr float kBounceSlack = 0.3f;
	constexpr float kForceLeft = 1.f;
	constexpr float kReleaseHold = 0.1f;
	constexpr float kFeetLift = 8.f;
	constexpr float kScaredShare = 0.6f;
	constexpr float kScaredReachMin = 200.f;
	constexpr float kScaredReachMax = 300.f;
	constexpr float kScaredSlack = 64.f;
	constexpr float kFriendClear = 400.f;
	constexpr float kLandClear = 300.f;
	constexpr float kOpenPitch = -20.f;
	constexpr float kOpenReach = 600.f;
	constexpr int kOpenYaws = 8;
	constexpr float kLookReach = 100.f;

	bool FriendNear(edict_t *self, const Vector &point, float radius)
	{
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent == self || !CBotIsLivePlayer(ent) || g_pGameProfile->IsDead(ent))
				continue;
			if (Teams::Of(self, ent) != Teams::REL_FRIEND)
				continue;
			if ((ent->v.origin - point).Length() < radius)
				return true;
		}
		return false;
	}

	bool AttackPoint(CBot &bot, Vector &base, Vector &vel, bool &heard)
	{
		heard = false;
		edict_t *target = bot.sense.target.Get();
		if (!target || gpGlobals->time - bot.sense.targetTime > kTargetFresh)
		{
			if (!BotSense::HeardEnemy(bot, base, vel, &target))
				return false;
			heard = true;
			base.z += target->v.mins.z + kFeetLift;
			return true;
		}

		base = target->v.origin;
		base.z += target->v.mins.z + kFeetLift;
		vel = target->v.velocity;
		return true;
	}

	bool AttackClear(CBot &bot, const WeaponMode &mode, const Vector &base, const Vector &vel, bool heard)
	{
		edict_t *pEdict = bot.pEdict;
		if ((mode.quirks & WPN_Q_TEAM_UNSAFE) != 0 && BotFire::FriendCloser(pEdict, base))
			return false;
		if (!heard)
			return true;

		Vector look, impact;
		float flight = 0.f;
		return BotFire::SolveOffhand(mode, pEdict->v.origin, base, vel, look, flight, pEdict, &impact) &&
			BotFire::HarmFavors(pEdict, impact, base + vel * flight, mode.splash);
	}

	bool ScaredPoint(CBot &bot, Vector &base, Vector &vel)
	{
		if (!bot.camp.scared)
			return false;

		edict_t *pEdict = bot.pEdict;
		float dist = 0.f;
		const BotSense::Suspicion *contact = BotSense::NearestLive(bot, pEdict->v.origin,
			bot.camp.scareDist + kScaredSlack, true, &dist);
		if (!contact)
			return false;

		Vector to = contact->pos - pEdict->v.origin;
		float len = to.Length();
		if (len < 1.f)
			return false;
		float reach = std::clamp(len * kScaredShare, kScaredReachMin, kScaredReachMax);
		Vector want = pEdict->v.origin + to * (reach / len);

		TraceResult tr;
		TRACE_LINE(pEdict->v.origin, want, ignore_monsters, pEdict, &tr);
		base = tr.vecEndPos;
		vel = Vector(0.f, 0.f, 0.f);
		return !FriendNear(pEdict, pEdict->v.origin, kFriendClear) && !FriendNear(pEdict, base, kFriendClear);
	}

	void OpenLook(CBot &bot)
	{
		edict_t *pEdict = bot.pEdict;
		Vector eye = pEdict->v.origin + pEdict->v.view_ofs;
		float best = -1.f;
		for (int i = 0; i < kOpenYaws; ++i)
		{
			MAKE_VECTORS(Vector(kOpenPitch, pEdict->v.v_angle.y + 360.f * i / kOpenYaws, 0.f));
			Vector dir = gpGlobals->v_forward;
			TraceResult tr;
			TRACE_LINE(eye, eye + dir * kOpenReach, ignore_monsters, pEdict, &tr);
			if (tr.flFraction > best)
			{
				best = tr.flFraction;
				bot.grenade.look = dir;
			}
		}
		bot.grenade.haveLook = true;
	}

	void Apply(CBot &bot, Vector &angles)
	{
		BotGrenade::State &st = bot.grenade;
		if (!st.haveLook)
			return;

		Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
		float yaw = 0.f, pitch = 0.f;
		if (BotAim::LosAngles(eye, eye + st.look * kLookReach, yaw, pitch))
			angles = Vector(pitch, yaw, 0.f);
	}

	void Finish(BotGrenade::State &st, const char *why)
	{
		st.slot = -1;
		st.ordered = false;
		st.releaseTime = 0.f;
		st.haveLook = false;
		st.why = why;
		st.nextTry = gpGlobals->time + RANDOM_FLOAT(kCooldownMin, kCooldownMax);
	}

	void PrimeOrdered(CBot &bot)
	{
		BotGrenade::State &st = bot.grenade;
		edict_t *pEdict = bot.pEdict;
		OffhandInfo info;
		if (!g_pGameProfile->OffhandRead(pEdict, st.orderSlot, info) || info.count <= 0)
			return;
		if (!FakeCmd::Run(pEdict, g_pGameProfile->OffhandCommand(st.orderSlot, true)))
			return;

		st.slot = st.orderSlot;
		st.scared = false;
		st.ordered = true;
		st.primeTime = gpGlobals->time;
		st.releaseTime = 0.f;
		st.pin = info.pin;
		st.fuse = info.fuse;
		st.minRange = 0.f;
		st.mode = info.mode;
		st.haveLook = false;
		st.why = "primed on orders";
		++st.primes;
		BOT_VLOG("CrossBot: %s primes grenade slot %d type %d on orders (%d left)\n", bot.name, st.slot + 1, info.type,
			info.count - 1);
	}

	void Hold(CBot &bot, Vector &angles)
	{
		BotGrenade::State &st = bot.grenade;
		edict_t *pEdict = bot.pEdict;
		float now = gpGlobals->time;
		float held = now - st.primeTime;
		bool primed = g_pGameProfile->OffhandPrimed(pEdict);

		if (st.releaseTime > 0.f)
		{
			Apply(bot, angles);
			float thrown = std::max(st.releaseTime, st.primeTime + st.pin) + kReleaseHold;
			if (now >= thrown || (!primed && now > st.releaseTime))
				Finish(st, "thrown");
			return;
		}

		if (!primed)
		{
			if (held > kPrimeConfirm)
			{
				++st.refused;
				Finish(st, "refused");
			}
			return;
		}

		Vector base, vel;
		bool heard = false;
		bool target = false;
		if (st.ordered)
		{
			base = st.orderPoint;
			vel = Vector(0.f, 0.f, 0.f);
			target = !FriendNear(pEdict, base, kLandClear) && AttackClear(bot, st.mode, base, vel, false);
		}
		else
		{
			target = st.scared ? ScaredPoint(bot, base, vel) : AttackPoint(bot, base, vel, heard);
		}
		if (target && !st.scared && !st.ordered)
			target = (base - pEdict->v.origin).Length() >= st.minRange && !FriendNear(pEdict, base, kLandClear) &&
				AttackClear(bot, st.mode, base, vel, heard);
		Vector look;
		float flight = 0.f;
		bool solved = target && BotFire::SolveOffhand(st.mode, pEdict->v.origin, base, vel, look, flight);
		if (solved)
		{
			st.look = look;
			st.flight = flight;
			st.haveLook = true;
		}

		float left = st.fuse - held;
		bool release = solved && (st.scared || !BotDiff::GrenadeCook() || left <= flight + kBounceSlack);
		if (!release && left <= kForceLeft)
		{
			release = true;
			++st.forced;
			OpenLook(bot);
		}
		if (!release)
		{
			st.why = solved ? "cooking" : "no-shot";
			return;
		}

		FakeCmd::Run(pEdict, g_pGameProfile->OffhandCommand(st.slot, false));
		st.releaseTime = now;
		st.why = "releasing";
		++st.throws;
		if (st.scared)
			++st.scaredThrows;
		if (heard)
			++st.heardThrows;
		BOT_VLOG("CrossBot: %s throws grenade slot %d after %.2fs, flight %.2fs, %s\n", bot.name,
			st.slot + 1, held, st.flight, st.haveLook ? "aimed" : "unaimed");
		Apply(bot, angles);
	}
}

namespace BotGrenade
{

bool Enabled()
{
	cvar_t *cvar = CBotCvar("crossbot_grenade");
	return !cvar || cvar->value != 0.f;
}

void Run(CBot &bot, Vector &angles)
{
	State &st = bot.grenade;
	if (!g_pGameProfile)
		return;

	if (st.slot >= 0)
	{
		Hold(bot, angles);
		return;
	}

	float now = gpGlobals->time;
	if (st.orderSlot >= 0 && now > st.orderUntil)
		st.orderSlot = -1;
	if (!Enabled() || BotDirective::MeleeOnly(bot) || BotDirective::HoldFire(bot))
		return;
	if (st.orderSlot >= 0)
	{
		PrimeOrdered(bot);
		return;
	}
	if (now < st.nextTry)
		return;
	st.nextTry = now + kDecideInterval;

	edict_t *pEdict = bot.pEdict;
	OffhandInfo pick;
	int slot = -1;
	bool scared = false;
	Vector base, vel;
	bool heard = false;

	for (int s = 0; s < kMaxOffhandSlots && slot < 0; ++s)
	{
		OffhandInfo info;
		bool read = g_pGameProfile->OffhandRead(pEdict, s, info);
		st.types[s] = read ? info.type : -1;
		st.counts[s] = info.count;
		if (!read || info.count <= 0)
			continue;

		if (info.use == OFFHAND_ATTACK && AttackPoint(bot, base, vel, heard))
		{
			++st.sightings;
			float dist = (base - pEdict->v.origin).Length();
			st.lastDist = dist;
			if (st.minDist <= 0.f || dist < st.minDist)
				st.minDist = dist;
			if (dist < info.minRange || dist > info.maxRange || FriendNear(pEdict, base, kLandClear) ||
				!AttackClear(bot, info.mode, base, vel, heard))
			{
				++st.rangeRejects;
				continue;
			}
		}
		else if (!(info.use == OFFHAND_SCARED && ScaredPoint(bot, base, vel)))
		{
			continue;
		}

		pick = info;
		slot = s;
		scared = info.use == OFFHAND_SCARED;
	}

	if (slot < 0)
		return;

	if (RANDOM_FLOAT(0.f, 1.f) > kPrimeChance)
	{
		st.nextTry = now + 1.f;
		return;
	}

	if (!FakeCmd::Run(pEdict, g_pGameProfile->OffhandCommand(slot, true)))
		return;

	st.slot = slot;
	st.scared = scared;
	st.primeTime = now;
	st.releaseTime = 0.f;
	st.pin = pick.pin;
	st.fuse = pick.fuse;
	st.minRange = pick.minRange;
	st.mode = pick.mode;
	st.haveLook = false;
	st.why = "primed";
	++st.primes;
	BOT_VLOG("CrossBot: %s primes grenade slot %d type %d (%d left)%s\n", bot.name, slot + 1, pick.type,
		pick.count - 1, scared ? ", scared" : "");
}

void Reset(CBot &bot)
{
	State &st = bot.grenade;
	State fresh;
	fresh.primes = st.primes;
	fresh.throws = st.throws;
	fresh.forced = st.forced;
	fresh.refused = st.refused;
	fresh.scaredThrows = st.scaredThrows;
	fresh.heardThrows = st.heardThrows;
	st = fresh;
}

bool Order(CBot &bot, int slot, const Vector &point, float until, const char *&why)
{
	State &st = bot.grenade;
	OffhandInfo info;
	if (!Enabled() || !g_pGameProfile)
	{
		why = "grenades are off";
		return false;
	}
	if (st.slot < 0 && (!g_pGameProfile->OffhandRead(bot.pEdict, slot, info) || info.count <= 0))
	{
		why = "no grenade in that slot";
		return false;
	}
	st.orderSlot = slot;
	st.orderPoint = point;
	st.orderUntil = until;
	return true;
}

void CancelOrder(CBot &bot)
{
	bot.grenade.orderSlot = -1;
}

int Throws(const CBot &bot)
{
	return bot.grenade.throws;
}

void PrintStatus(const CBot &bot)
{
	const State &st = bot.grenade;
	char line[320];
	snprintf(line, sizeof(line),
		"CrossBot: %s grenade slot=%d why=%s held=%.2f flight=%.2f primes=%u throws=%u forced=%u "
		"refused=%u scared=%u heard=%u slots=%d:%d,%d:%d sightings=%u rejects=%u dist=%.0f min=%.0f\n",
		bot.name, st.slot + 1, st.why, st.slot >= 0 ? gpGlobals->time - st.primeTime : 0.f, st.flight,
		st.primes, st.throws, st.forced, st.refused, st.scaredThrows, st.heardThrows, st.types[0], st.counts[0], st.types[1],
		st.counts[1], st.sightings, st.rangeRejects, st.lastDist, st.minDist);
	SERVER_PRINT(line);
}

}
