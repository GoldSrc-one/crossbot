#include <extdll.h>
#include <meta_api.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "bot_deploy.h"
#include "../game/cvars.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_combat.h"
#include "bot_fire.h"
#include "bot_log.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sense.h"
#include "bot_weapons.h"
#include "../game/crossplay.h"
#include "../game/ent_handle.h"
#include "../game/game_profile.h"
#include "../nav/nav_math.h"
#include "../nav/nav_query.h"
#include "../nav/path_follower.h"
#include "../nav/pm_sim.h"
#include "../nav/waypoint_plan.h"

namespace
{
	constexpr float kDeployKillRadius = 125.f;
	constexpr float kDeployGateProbe = 176.f;
	constexpr float kDeployOpeningMax = 144.f;
	constexpr float kDeployVerticalMax = 48.f;
	constexpr float kDeployReach = 128.f;
	constexpr float kDeployMountOffset = 8.f;
	constexpr float kDeployNormalTol = 0.94f;
	constexpr float kDeployFloorNormal = 0.7f;

	constexpr float kDeployStandEye = 46.f;
	constexpr float kDeployDuckEye = 12.f;

	constexpr float kDeployBodyTop = 46.f;
	constexpr float kDeployCeilingGap = 8.f;

	constexpr float kDeployMinedRadius = 96.f;

	constexpr float kDeployTryInterval = 2.f;
	constexpr float kDeploySelectInterval = 0.5f;
	constexpr float kDeployStandRadius = 12.f;
	constexpr float kDeployAimTolerance = 4.f;
	constexpr float kDeployTimeout = 8.f;
	constexpr float kDeployMemoTime = 45.f;
	constexpr float kDeployVerifyDelay = 0.3f;
	constexpr float kDeployVerifyRadius = 48.f;
	constexpr float kDeployMoveDeadzone = 0.01f;

	constexpr float kDeployBeamLength = 2048.f;
	constexpr float kHazardLift = 2.f;
	constexpr float kDeployBeamSlack = 24.f;
	constexpr float kDeployGuardStep = 8.f;
	constexpr float kDeployClearMargin = 24.f;
	constexpr float kDeployRetreatTimeout = 1.5f;
	constexpr float kDeployDuckClear = 40.f;
	constexpr float kDeploySatchelRest = 4.f;
	constexpr float kDeploySatchelRadius = 375.f;
	constexpr float kDeployDetonateTimeout = 2.f;

	std::vector<BotDeploy::Beam> g_beams;

	using NavMath::kPiF;
	using NavMath::Norm2D;

	Vector AnglesTo(const Vector &dir)
	{
		float flat = sqrtf(dir.x * dir.x + dir.y * dir.y);
		return Vector(-atan2f(dir.z, flat) * (180.f / kPiF),
			atan2f(dir.y, dir.x) * (180.f / kPiF), 0.f);
	}

	edict_t *EntityNear(const char *classname, const Vector &point, float radius)
	{
		if (!classname)
			return NULL;

		for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free || ent->v.classname == 0)
				continue;
			if (strcmp(CBotClassOf(ent), classname) != 0)
				continue;
			if ((ent->v.origin - point).Length() <= radius)
				return ent;
		}
		return NULL;
	}

	bool GateHasMine(const Vector &mid)
	{
		if (!g_pGameProfile)
			return false;

		for (int k = 0; k < DEPLOY_COUNT; ++k)
		{
			const char *cls = g_pGameProfile->DeployEntity(static_cast<DeployKind>(k));
			if (EntityNear(cls, mid, kDeployMinedRadius))
				return true;
		}
		return false;
	}

	bool Visible(const Vector &from, const Vector &to)
	{
		TraceResult tr;
		g_engfuncs.pfnTraceLine(from, to, ignore_monsters, NULL, &tr);
		return tr.flFraction >= 1.f;
	}

	bool Covers(const Vector &mount, const Vector &left, const Vector &right, float topZ)
	{
		const float fracs[3] = { 0.f, 0.5f, 1.f };

		for (int i = 0; i < 3; ++i)
		{
			Vector base = left + (right - left) * fracs[i];

			for (int h = 0; h < 3; ++h)
			{
				Vector sample = base;
				sample.z += topZ * (0.5f * static_cast<float>(h));

				if ((sample - mount).Length() > kDeployKillRadius)
					return false;
				if (!Visible(mount, sample))
					return false;

				if (topZ <= 0.f)
					break;
			}
		}
		return true;
	}

	struct SurfaceHit
	{
		bool hit = false;
		bool faces = false;
		float dist = kDeployReach;
		Vector mount;
		Vector normal;
	};

	SurfaceHit Probe(const Vector &eye, const Vector &dir, float tol)
	{
		SurfaceHit out;

		TraceResult tr;
		g_engfuncs.pfnTraceLine(eye, eye + dir * kDeployReach, ignore_monsters, NULL, &tr);

		if (tr.flFraction >= 1.f || !tr.pHit)
			return out;

		out.hit = true;
		out.dist = kDeployReach * tr.flFraction;

		if ((tr.pHit->v.flags & FL_CONVEYOR) != 0)
			return out;

		const Vector &n = tr.vecPlaneNormal;
		if (n.x * -dir.x + n.y * -dir.y + n.z * -dir.z < tol)
			return out;

		out.faces = true;
		out.normal = n;
		out.mount = tr.vecEndPos + n * kDeployMountOffset;
		return out;
	}


	constexpr int kDeployHullHuman = 1;
	constexpr int kDeployHullDuck = 3;
	constexpr float kDeployHullHalf = 16.f;
	constexpr float kDeployDuckHullHalf = NavPlan::kPlanStandHeight;
	constexpr float kDeployStandHullHalf = kDeployDuckHullHalf * 2.f;

	constexpr float kDeployArmSeconds = 2.5f;

	bool ClipSegmentToBox(const Vector &a, const Vector &b, const Vector &mins, const Vector &maxs,
		float &t0, float &t1)
	{
		t0 = 0.f;
		t1 = 1.f;

		const float av[3] = { a.x, a.y, a.z };
		const float bv[3] = { b.x, b.y, b.z };
		const float lo[3] = { mins.x, mins.y, mins.z };
		const float hi[3] = { maxs.x, maxs.y, maxs.z };

		for (int i = 0; i < 3; ++i)
		{
			float d = bv[i] - av[i];
			if (fabsf(d) < 1e-6f)
			{
				if (av[i] < lo[i] || av[i] > hi[i])
					return false;
				continue;
			}

			float p0 = (lo[i] - av[i]) / d;
			float p1 = (hi[i] - av[i]) / d;
			if (p0 > p1)
				std::swap(p0, p1);

			if (p0 > t0) t0 = p0;
			if (p1 < t1) t1 = p1;
			if (t0 > t1)
				return false;
		}
		return true;
	}

	bool MineBeam(edict_t *ent, Vector &start, Vector &end)
	{
		if (!ent || ent->free)
			return false;

		Vector aim = ent->v.angles;
		aim.x = -aim.x;

		Vector forward, right, up;
		g_engfuncs.pfnAngleVectors(aim, forward, right, up);

		TraceResult tr;
		g_engfuncs.pfnTraceLine(ent->v.origin, ent->v.origin + forward * kDeployBeamLength,
			ignore_monsters, ent, &tr);

		start = ent->v.origin;
		end = tr.vecEndPos;
		return true;
	}

	bool BotOnBeam(const CBot &bot, const Vector &start, const Vector &end, float pad)
	{
		edict_t *pEdict = bot.pEdict;
		int hull = (pEdict->v.flags & FL_DUCKING) != 0 ? kDeployHullDuck : kDeployHullHuman;

		Vector mins, maxs;
		BotDeploy::HullExtents(hull, mins, maxs);

		mins = pEdict->v.origin + mins - Vector(pad, pad, pad);
		maxs = pEdict->v.origin + maxs + Vector(pad, pad, pad);

		float t0 = 0.f, t1 = 1.f;
		return ClipSegmentToBox(start, end, mins, maxs, t0, t1);
	}

	BotDeploy::PlacedMine *FindMine(BotDeploy::State &st, const edict_t *ent)
	{
		for (int i = 0; i < BotDeploy::kDeployMineSlots; ++i)
		{
			if (st.mines[i].ent.Get() == ent)
				return &st.mines[i];
		}
		return NULL;
	}

	void RememberMine(BotDeploy::State &st, edict_t *ent)
	{
		BotDeploy::PlacedMine &slot = st.mines[st.mineNext];
		st.mineNext = (st.mineNext + 1) % BotDeploy::kDeployMineSlots;

		slot = BotDeploy::PlacedMine();
		slot.ent.Set(ent);
		slot.placeTime = gpGlobals->time;
	}
	bool BeamContained(const Vector &mount, const Vector &normal, float opening)
	{
		TraceResult tr;
		g_engfuncs.pfnTraceLine(mount, mount + normal * kDeployBeamLength, ignore_monsters, NULL,
			&tr);

		float len = kDeployBeamLength * tr.flFraction;
		return len <= opening + kDeployBeamSlack;
	}

	float CeilingAbove(const Vector &eye)
	{
		TraceResult tr;
		g_engfuncs.pfnTraceLine(eye, eye + Vector(0.f, 0.f, kDeployReach), ignore_monsters, NULL,
			&tr);
		return tr.flFraction >= 1.f ? eye.z + kDeployReach : tr.vecEndPos.z;
	}

	int ActiveWeaponId(const CBot &bot)
	{
		const BotInventory &inv = bot.inventory;
		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
			return 0;
		return inv.weapons[inv.activeIndex].id;
	}

	int DeployAmmo(const CBot &bot, int weaponId)
	{
		const BotInventory &inv = bot.inventory;
		for (int i = 0; i < inv.count; ++i)
		{
			if (inv.weapons[i].id != weaponId)
				continue;

			int type = inv.weapons[i].primaryAmmoType;
			if (type < 0 || type >= MAX_AMMO_SLOTS)
				return 0;
			return inv.ammo[type];
		}
		return 0;
	}

	const char *SelectWeapon(CBot &bot, int weaponId)
	{
		if (weaponId == 0 || !g_pGameProfile)
			return NULL;

		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(weaponId);
		if (!spec || !spec->classname)
			return NULL;

		return g_pGameProfile->SelectWeapon(bot.pEdict, *spec) ? spec->classname : NULL;
	}

	void RestoreWeapon(CBot &bot)
	{
		BotDeploy::State &st = bot.deploy;
		if (st.prevWeaponId == 0 || st.prevWeaponId == ActiveWeaponId(bot))
			return;
		if (DeployAmmo(bot, st.prevWeaponId) <= 0 && st.prevWeaponId != 0)
		{
			const BotInventory &inv = bot.inventory;
			bool carries = false;
			for (int i = 0; i < inv.count && !carries; ++i)
				carries = inv.weapons[i].id == st.prevWeaponId;
			if (!carries)
				return;
		}

		SelectWeapon(bot, st.prevWeaponId);
	}

	unsigned int NextRand(BotDeploy::State &st, edict_t *pEdict)
	{
		if (!st.rngSeeded)
		{
			st.rngState = static_cast<unsigned int>(g_engfuncs.pfnIndexOfEdict(pEdict)) * 2654435761u
				+ 1u;
			st.rngSeeded = true;
		}
		st.rngState = st.rngState * 1664525u + 1013904223u;
		return st.rngState;
	}

	void PruneMemo(BotDeploy::State &st)
	{
		float now = gpGlobals->time;
		size_t out = 0;
		for (size_t i = 0; i < st.memo.size(); ++i)
		{
			if (st.memo[i].expireTime <= now)
				continue;
			st.memo[out++] = st.memo[i];
		}
		st.memo.resize(out);
	}

	bool MemoBlocks(const BotDeploy::State &st, uint32_t connIndex)
	{
		float now = gpGlobals->time;
		for (size_t i = 0; i < st.memo.size(); ++i)
		{
			if (st.memo[i].connIndex == connIndex && st.memo[i].expireTime > now)
				return true;
		}
		return false;
	}

	void NoteMemo(BotDeploy::State &st, uint32_t connIndex)
	{
		float until = gpGlobals->time + kDeployMemoTime;
		for (size_t i = 0; i < st.memo.size(); ++i)
		{
			if (st.memo[i].connIndex != connIndex)
				continue;
			st.memo[i].expireTime = until;
			return;
		}

		BotDeploy::Memo memo;
		memo.connIndex = connIndex;
		memo.expireTime = until;
		st.memo.push_back(memo);
	}

	const BotWeapon *WeaponById(const CBot &bot, int weaponId)
	{
		const BotInventory &inv = bot.inventory;
		for (int i = 0; i < inv.count; ++i)
		{
			if (inv.weapons[i].id == weaponId)
				return &inv.weapons[i];
		}
		return NULL;
	}

	int ChargeReady(const CBot &bot, int weaponId)
	{
		const BotWeapon *w = WeaponById(bot, weaponId);
		return w ? w->chargeReady : 0;
	}

	bool AttackReady(const CBot &bot, int weaponId)
	{
		if (bot.inventory.nextAttack > 0.f)
			return false;

		const BotWeapon *w = WeaponById(bot, weaponId);
		return w && w->nextPrimaryAttack <= 0.f && !w->reloading;
	}

	bool TryDetonate(CBot &bot)
	{
		BotDeploy::State &st = bot.deploy;

		const char *cls = g_pGameProfile->DeployEntity(DEPLOY_SATCHEL);
		int satchelId = g_pGameProfile->DeployWeapon(DEPLOY_SATCHEL);
		if (!cls || satchelId == 0 || ChargeReady(bot, satchelId) != 1)
			return false;

		const Vector &origin = bot.pEdict->v.origin;

		for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!ent || ent->free || ent->v.classname == 0)
				continue;
			if (ent->v.owner != bot.pEdict)
				continue;
			if (strcmp(CBotClassOf(ent), cls) != 0)
				continue;

			float senseDist = 0.f;
			if (!BotSense::NearestLive(bot, ent->v.origin, kDeploySatchelRadius, true, &senseDist))
				continue;
			if (senseDist >= (origin - ent->v.origin).Length())
				continue;

			st.active = true;
			st.phase = BotDeploy::PHASE_DETONATE;
			st.prevWeaponId = ActiveWeaponId(bot);
			st.startTime = gpGlobals->time;
			st.nextSelectTime = 0.f;
			st.firedTime = 0.f;
			st.lastReason = "detonating";

			BOT_VLOG("CrossBot: %s is detonating a satchel at (%.0f,%.0f,%.0f), contact %.0f away\n",
				bot.name, ent->v.origin.x, ent->v.origin.y, ent->v.origin.z, senseDist);

			return true;
		}
		return false;
	}

	const char *ManeuverName(const CBot &bot)
	{
		if (bot.deploy.active) return "deploy";
		if (bot.brk.active) return "break";
		if (bot.airmoveActive) return "airmove";
		if (bot.ladderActive) return "ladder";
		if (bot.rideActive) return "ride";
		if (bot.activationActive) return "activation";
		if (bot.chargerActive) return "charger";
		if (BotCombat::Active(bot)) return "combat";
		return "walk";
	}

	float BeamPerp(const Vector &at, const Vector &start, const Vector &end)
	{
		Vector d = end - start;
		float len = d.Length();
		if (len < 1.f)
			return (at - start).Length();

		Vector n = d * (1.f / len);
		float t = DotProduct(at - start, n);
		if (t < 0.f) t = 0.f;
		if (t > len) t = len;

		return (at - (start + n * t)).Length();
	}

	void AuditMine(CBot &bot, edict_t *mine)
	{
		BotDeploy::State &st = bot.deploy;

		Vector start, end;
		if (!MineBeam(mine, start, end))
			return;

		float beamLen = (end - start).Length();
		float escape = beamLen - st.spot.opening;
		if (escape > kDeployBeamSlack)
			++st.escapes;

		if (!BotDeploy::DebugEnabled())
			return;

		TraceResult tr;
		g_engfuncs.pfnTraceLine(start, end + (end - start), ignore_monsters, mine, &tr);
		const char *hitName = tr.pHit ? CBotClassOf(tr.pHit) : "-";

		BOT_VLOG("CrossBot: %s audit conn %u %s open %.0f beam %.0f escape %.0f into %s, "
			"sideA hit %d faces %d %.0f, sideB hit %d faces %d %.0f, onbeam %d\n",
			bot.name, st.spot.connIndex, BotDeploy::SpotKindName(st.spot.kind), st.spot.opening,
			beamLen, escape, hitName,
			st.spot.sideHit[0] ? 1 : 0, st.spot.sideFaces[0] ? 1 : 0, st.spot.sideDist[0],
			st.spot.sideHit[1] ? 1 : 0, st.spot.sideFaces[1] ? 1 : 0, st.spot.sideDist[1],
			BotOnBeam(bot, start, end, 0.f) ? 1 : 0);
	}

	void Stop(CBot &bot, const char *why, bool placed)
	{
		BotDeploy::State &st = bot.deploy;

		RestoreWeapon(bot);

		if (st.phase == BotDeploy::PHASE_DETONATE)
		{
			if (placed)
				++st.detonations;
			else
				++st.aborts;
		}
		else
		{
			NoteMemo(st, st.spot.connIndex);
			if (placed)
				++st.placements;
			else
				++st.aborts;
		}

		BOT_VLOG("CrossBot: %s %s at conn %u after %.1fs\n", bot.name, why, st.spot.connIndex,
			gpGlobals->time - st.startTime);

		st.active = false;
		st.ordered = false;
		st.phase = BotDeploy::PHASE_NONE;
		st.lastReason = why;
		bot.nextRepathTime = gpGlobals->time;
	}
}

namespace BotDeploy
{

bool Enabled()
{
	cvar_t *cvar = CBotCvar("crossbot_deploy");
	return !cvar || cvar->value != 0.f;
}

bool DebugEnabled()
{
#if BOT_VLOG_ON
	cvar_t *cvar = CBotCvar("crossbot_deploy_debug");
	return cvar && cvar->value != 0.f;
#else
	return false;
#endif
}

void HullExtents(int hull, Vector &mins, Vector &maxs)
{
	float half = hull == kDeployHullDuck ? kDeployDuckHullHalf : kDeployStandHullHalf;
	mins = Vector(-kDeployHullHalf, -kDeployHullHalf, -half);
	maxs = Vector(kDeployHullHalf, kDeployHullHalf, half);
}

bool BeamMeetsHull(const Beam &beam, const Vector &at, int hull, float &zLo, float &zHi,
	float pad)
{
	Vector mins, maxs;
	HullExtents(hull, mins, maxs);
	mins = at + mins - Vector(pad, pad, 0.f);
	maxs = at + maxs + Vector(pad, pad, 0.f);

	float t0 = 0.f, t1 = 1.f;
	if (!ClipSegmentToBox(beam.start, beam.end, mins, maxs, t0, t1))
		return false;

	float za = beam.start.z + (beam.end.z - beam.start.z) * t0;
	float zb = beam.start.z + (beam.end.z - beam.start.z) * t1;
	zLo = za < zb ? za : zb;
	zHi = za < zb ? zb : za;
	return true;
}

void TrackMines(CBot &bot)
{
	State &st = bot.deploy;
	if (!DebugEnabled())
		return;

	float now = gpGlobals->time;

	for (int i = 0; i < kDeployMineSlots; ++i)
	{
		PlacedMine &m = st.mines[i];
		edict_t *ent = m.ent.Get();
		if (!ent)
			continue;

		Vector start, end;
		if (!MineBeam(ent, start, end))
			continue;

		bool armed = ent->v.solid == SOLID_BBOX;
		bool onBeam = BotOnBeam(bot, start, end, 0.f);

		if (!m.cleared && !onBeam)
		{
			m.cleared = true;
			m.clearedAt = now;
			BOT_VLOG("CrossBot: %s cleared its own beam after %.2fs (arms at %.2fs)\n", bot.name,
				now - m.placeTime, kDeployArmSeconds);
		}

		if (armed && !m.armed)
		{
			m.armed = true;
			if (onBeam)
				BOT_VLOG("CrossBot: %s STILL IN BEAM AT ARMING, mine at (%.0f,%.0f,%.0f) age %.2fs "
					"-- will detonate when it steps off\n", bot.name, ent->v.origin.x,
					ent->v.origin.y, ent->v.origin.z, now - m.placeTime);
			else
				BOT_VLOG("CrossBot: %s beam armed clear, mine at (%.0f,%.0f,%.0f), cleared %.2fs "
					"before arming\n", bot.name, ent->v.origin.x, ent->v.origin.y, ent->v.origin.z,
					now - m.clearedAt);
		}
	}
}

void NoteDamage(CBot &bot)
{
	edict_t *pEdict = bot.pEdict;
	if (pEdict->v.dmg_take == 0.f)
		return;

	edict_t *inflictor = pEdict->v.dmg_inflictor;
	if (!inflictor || inflictor->free)
		return;

	State &st = bot.deploy;
	PlacedMine *mine = FindMine(st, inflictor);
	if (!mine)
		return;

	++st.selfHits;

	if (!DebugEnabled())
		return;

	float age = gpGlobals->time - mine->placeTime;
	float dist = (inflictor->v.origin - pEdict->v.origin).Length();

	BOT_VLOG("CrossBot: %s took %.0f from its own mine, age %.2fs, %.0f away, %s, %s\n", bot.name,
		pEdict->v.dmg_take, age, dist, mine->armed ? "armed" : "unarmed",
		mine->cleared ? "had cleared the beam" : "never cleared the beam");
}

void NoteDeath(CBot &bot)
{
	State &st = bot.deploy;
	edict_t *pEdict = bot.pEdict;
	edict_t *inflictor = pEdict->v.dmg_inflictor;

	PlacedMine *mine = (inflictor && !inflictor->free) ? FindMine(st, inflictor) : NULL;
	const Beam *beam = inflictor ? BeamFor(inflictor) : NULL;

	if (mine)
		++st.selfHits;

	if (BotAvoid::JumpPendingOn(bot, inflictor))
		BOT_VLOG("CrossBot: %s beam jump FAILED (killed), gap %.0f speed %.0f dist %.0f\n", bot.name,
			bot.avoid.jumpGap, bot.avoid.jumpSpeed, bot.avoid.jumpDist);
	BotAvoid::ClearJump(bot);

	if (!DebugEnabled())
		return;

	const BotAvoid::State &av = bot.avoid;
	const char *what = inflictor && !inflictor->free ? CBotClassOf(inflictor) : "-";

	if (mine)
	{
		float age = gpGlobals->time - mine->placeTime;
		float dist = (inflictor->v.origin - pEdict->v.origin).Length();
		Vector bs, be;
		bool haveBeam = MineBeam(inflictor, bs, be);
		float perp = haveBeam ? BeamPerp(pEdict->v.origin, bs, be) : -1.f;
		bool onBeam = haveBeam && BotOnBeam(bot, bs, be, 0.f);

		BOT_VLOG("CrossBot: %s KILLED BY ITS OWN MINE at (%.0f,%.0f,%.0f), age %.2fs, %.0f away, "
			"perp %.0f, onbeam %d, reg %d, %s, %s, maneuver %s, avoid %s vs %s\n", bot.name,
			pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, age, dist, perp,
			onBeam ? 1 : 0, beam ? 1 : 0, mine->armed ? "armed" : "unarmed",
			mine->cleared ? "had cleared" : "never cleared", ManeuverName(bot),
			BotAvoid::KindName(av.lastKind), av.lastHit[0] ? av.lastHit : "-");
		return;
	}

	BOT_VLOG("CrossBot: %s died at (%.0f,%.0f,%.0f) to %s%s, avoid %s vs %s gap %.0f top %.0f\n",
		bot.name, pEdict->v.origin.x, pEdict->v.origin.y, pEdict->v.origin.z, what,
		beam ? " (A TRIPMINE, not ours)" : "", BotAvoid::KindName(av.lastKind),
		av.lastHit[0] ? av.lastHit : "-", av.lastGap, av.lastTop);
}

SpotKind SurfacePin()
{
	cvar_t *cvar = CBotCvar("crossbot_deploy_surface");
	if (!cvar || cvar->value <= 0.f)
		return SPOT_NONE;

	int v = static_cast<int>(cvar->value);
	if (v < SPOT_WALL || v > SPOT_GROUND)
		return SPOT_NONE;
	return static_cast<SpotKind>(v);
}

const char *SpotKindName(SpotKind kind)
{
	switch (kind)
	{
	case SPOT_WALL: return "wall";
	case SPOT_CEILING: return "ceiling";
	case SPOT_GROUND: return "ground";
	default: return "none";
	}
}

const char *SpotRejectName(SpotReject reject)
{
	switch (reject)
	{
	case REJECT_TYPE: return "not-walk";
	case REJECT_GATE: return "no-gate";
	case REJECT_OPEN: return "open";
	case REJECT_WIDE: return "too-wide";
	case REJECT_SURFACE: return "no-surface";
	case REJECT_COVERAGE: return "no-coverage";
	case REJECT_MINED: return "already-mined";
	case REJECT_MEMO: return "memo";
	case REJECT_ESCAPE: return "beam-escapes";
	default: return "ok";
	}
}

bool EvaluateGate(const Nav::NavMesh &mesh, uint32_t areaId, uint32_t connIndex, DeployKind kind,
	bool duck, Spot &out)
{
	out = Spot();
	out.connIndex = connIndex;
	out.duck = duck;

	if (connIndex >= mesh.conns.size())
	{
		out.reject = REJECT_TYPE;
		return false;
	}

	const Nav::Connection &conn = mesh.conns[connIndex];
	if (conn.connType != crossbot_nav::CONN_WALK)
	{
		out.reject = REJECT_TYPE;
		return false;
	}

	const NavPlan::PlanGate *gate = NULL;
	if (conn.areaA == areaId)
	{
		gate = &conn.gateA;
		out.otherAreaId = conn.areaB;
	}
	else if (conn.areaB == areaId)
	{
		gate = &conn.gateB;
		out.otherAreaId = conn.areaA;
	}
	else
	{
		out.reject = REJECT_TYPE;
		return false;
	}

	if (!gate->valid)
	{
		out.reject = REJECT_GATE;
		return false;
	}

	Vector mid = NavPlan::GateMidpoint(*gate);
	float span = (gate->left - gate->right).Length2D();
	out.gateWidth = span + 4.f * NavPlan::kPlanClearance;

	if (out.gateWidth > kDeployGateProbe)
	{
		out.reject = REJECT_WIDE;
		return false;
	}

	Vector along;
	if (span > 0.1f)
	{
		along = Norm2D(gate->right - gate->left);
	}
	else
	{
		const NavPlan::PlanGate &farGate = (conn.areaA == areaId) ? conn.gateB : conn.gateA;

		Vector across(0.f, 0.f, 0.f);
		if (farGate.valid)
			across = Norm2D(NavPlan::GateMidpoint(farGate) - mid);

		if (across.x == 0.f && across.y == 0.f)
		{
			const Nav::Area *here = Nav::FindAreaById(mesh, areaId);
			const Nav::Area *there = Nav::FindAreaById(mesh, out.otherAreaId);
			if (!here || !there)
			{
				out.reject = REJECT_GATE;
				return false;
			}
			across = Norm2D(there->centroid - here->centroid);
		}

		if (across.x == 0.f && across.y == 0.f)
		{
			out.reject = REJECT_GATE;
			return false;
		}

		along = Vector(-across.y, across.x, 0.f);
	}

	out.stand = mid;

	{
		Vector across(-along.y, along.x, 0.f);
		const Nav::Area *here = Nav::FindAreaById(mesh, areaId);
		if (here && DotProduct(across, here->centroid - mid) < 0.f)
			across = across * -1.f;
		out.retreatDir = across;
	}

	Vector eye = mid;
	eye.z += duck ? kDeployDuckEye : kDeployStandEye;

	float ceilZ = CeilingAbove(eye);
	float topZ = kDeployBodyTop;
	if (ceilZ - kDeployCeilingGap - mid.z < topZ)
		topZ = ceilZ - kDeployCeilingGap - mid.z;
	if (topZ < 0.f)
		topZ = 0.f;

	SurfaceHit side[2];
	side[0] = Probe(eye, along, kDeployNormalTol);
	side[1] = Probe(eye, along * -1.f, kDeployNormalTol);

	for (int s = 0; s < 2; ++s)
	{
		out.sideHit[s] = side[s].hit;
		out.sideFaces[s] = side[s].faces;
		out.sideDist[s] = side[s].dist;
	}

	if (!side[0].hit || !side[1].hit)
	{
		out.reject = REJECT_OPEN;
		return false;
	}

	out.opening = side[0].dist + side[1].dist;
	if (out.opening > kDeployOpeningMax)
	{
		out.reject = REJECT_WIDE;
		return false;
	}

	if (kind == DEPLOY_SATCHEL)
	{
		Vector rest = mid;
		rest.z -= NavPlan::kPlanStandHeight - kDeploySatchelRest;

		if (!Covers(rest, gate->left, gate->right, topZ))
		{
			out.reject = REJECT_COVERAGE;
			return false;
		}

		if (GateHasMine(mid))
		{
			out.reject = REJECT_MINED;
			return false;
		}

		out.kind = SPOT_GROUND;
		out.aimAngles = AnglesTo(Vector(-along.y, along.x, 0.f));
		out.mount = rest;
		out.reject = REJECT_NONE;
		return true;
	}

	SpotKind pin = SurfacePin();

	Vector mount;
	bool haveSurface = false;

	int nearSide = side[0].dist <= side[1].dist ? 0 : 1;
	bool escaped = false;
	for (int i = 0; i < 2 && !haveSurface && pin != SPOT_CEILING && pin != SPOT_GROUND; ++i)
	{
		int s = i == 0 ? nearSide : 1 - nearSide;
		if (!side[s].faces)
			continue;
		if (!Covers(side[s].mount, gate->left, gate->right, topZ))
			continue;
		if (!BeamContained(side[s].mount, side[s].normal, out.opening))
		{
			escaped = true;
			continue;
		}

		mount = side[s].mount;
		out.kind = SPOT_WALL;
		out.aimAngles = AnglesTo(s == 0 ? along : along * -1.f);
		out.beamDir = side[s].normal;
		haveSurface = true;
	}

	if (!haveSurface && pin != SPOT_WALL && (pin != SPOT_NONE || out.opening <= kDeployVerticalMax))
	{
		SurfaceHit up = Probe(eye, Vector(0.f, 0.f, 1.f), kDeployFloorNormal);
		if (pin != SPOT_GROUND && up.faces && Covers(up.mount, gate->left, gate->right, topZ))
		{
			mount = up.mount;
			out.kind = SPOT_CEILING;
			out.aimAngles = Vector(-90.f, AnglesTo(along).y, 0.f);
			haveSurface = true;
		}

		if (!haveSurface && pin != SPOT_CEILING)
		{
			SurfaceHit down = Probe(eye, Vector(0.f, 0.f, -1.f), kDeployFloorNormal);
			if (down.faces && Covers(down.mount, gate->left, gate->right, topZ))
			{
				mount = down.mount;
				out.kind = SPOT_GROUND;
				out.aimAngles = Vector(90.f, AnglesTo(along).y, 0.f);
				haveSurface = true;
			}
		}
	}

	if (!haveSurface)
	{
		out.reject = escaped ? REJECT_ESCAPE : REJECT_SURFACE;
		return false;
	}

	if (GateHasMine(mid))
	{
		out.reject = REJECT_MINED;
		return false;
	}

	out.mount = mount;
	out.reject = REJECT_NONE;
	return true;
}

bool FindBest(const CBot &bot, const Nav::NavMesh &mesh, int areaId, DeployKind kind, bool duck,
	Spot &out)
{
	if (areaId < 0)
		return false;

	const Nav::Area *area = Nav::FindAreaById(mesh, static_cast<uint32_t>(areaId));
	if (!area)
		return false;

	bool found = false;
	Spot best;

	for (size_t i = 0; i < area->connIndices.size(); ++i)
	{
		if (MemoBlocks(bot.deploy, area->connIndices[i]))
			continue;

		Spot spot;
		if (!EvaluateGate(mesh, area->id, area->connIndices[i], kind, duck, spot))
			continue;

		if (!found || spot.opening < best.opening)
		{
			best = spot;
			found = true;
		}
	}

	if (found)
		out = best;
	return found;
}

bool FindSpot(const CBot &bot, const Nav::NavMesh &mesh, int areaId, bool duck, Spot &out)
{
	return FindBest(bot, mesh, areaId, DEPLOY_TRIPMINE, duck, out);
}

bool FindDropSpot(const CBot &bot, const Nav::NavMesh &mesh, int areaId, Spot &out)
{
	return FindBest(bot, mesh, areaId, DEPLOY_SATCHEL, false, out);
}

const char *PhaseName(Phase phase)
{
	switch (phase)
	{
	case PHASE_APPROACH: return "approach";
	case PHASE_SELECT: return "select";
	case PHASE_AIM: return "aim";
	case PHASE_FIRE: return "fire";
	case PHASE_VERIFY: return "verify";
	case PHASE_RETREAT: return "retreat";
	case PHASE_DETONATE: return "detonate";
	default: return "none";
	}
}

void RefreshBeams()
{
	g_beams.clear();

	const char *mines[XPlay::kMaxGames] = {};
	const char *hazards[XPlay::kMaxGames] = {};
	int kinds = 0;
	for (int g = 0; g < XPlay::NumGames(); ++g)
	{
		const IGameProfile *profile = XPlay::ProfileFor(g);
		if (!profile)
			continue;
		mines[g] = profile->DeployEntity(DEPLOY_TRIPMINE);
		hazards[g] = profile->HazardEntity();
		kinds += (mines[g] ? 1 : 0) + (hazards[g] ? 1 : 0);
	}
	if (!kinds)
		return;

	for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free || ent->v.classname == 0)
			continue;

		const char *cls = CBotClassOf(ent);
		bool mine = false;
		bool hazard = false;
		for (int g = 0; g < XPlay::NumGames(); ++g)
		{
			mine = mine || (mines[g] && strcmp(cls, mines[g]) == 0);
			hazard = hazard || (hazards[g] && strcmp(cls, hazards[g]) == 0);
		}

		if (hazard && (ent->v.flags & FL_ONGROUND) != 0)
		{
			Beam floor;
			floor.ent = ent;
			floor.hazard = true;
			floor.start = Vector(ent->v.origin.x, ent->v.origin.y, ent->v.absmin.z);
			floor.end = Vector(ent->v.origin.x, ent->v.origin.y, ent->v.absmax.z + kHazardLift);
			g_beams.push_back(floor);
			continue;
		}

		if (!mine || hazard)
			continue;
		if (ent->v.solid != SOLID_BBOX && ent->v.solid != SOLID_NOT)
			continue;

		Vector aim = ent->v.angles;
		aim.x = -aim.x;

		Vector forward, right, up;
		g_engfuncs.pfnAngleVectors(aim, forward, right, up);

		TraceResult tr;
		g_engfuncs.pfnTraceLine(ent->v.origin, ent->v.origin + forward * kDeployBeamLength,
			ignore_monsters, ent, &tr);

		Beam beam;
		beam.ent = ent;
		beam.start = ent->v.origin;
		beam.armed = ent->v.solid == SOLID_BBOX;
		beam.end = tr.vecEndPos;
		g_beams.push_back(beam);
	}
}

const std::vector<Beam> &Beams()
{
	return g_beams;
}

const Beam *BeamFor(const edict_t *ent)
{
	if (!ent)
		return NULL;

	for (size_t i = 0; i < g_beams.size(); ++i)
	{
		if (g_beams[i].ent == ent)
			return &g_beams[i];
	}
	return NULL;
}

BeamBlock BeamGuard(const CBot &bot, const Vector &moveDir, float lookahead, const Beam **hit)
{
	if (g_beams.empty())
		return BEAM_CLEAR;
	if (moveDir.x == 0.f && moveDir.y == 0.f)
		return BEAM_CLEAR;

	edict_t *pEdict = bot.pEdict;
	bool ducking = (pEdict->v.flags & FL_DUCKING) != 0;
	int hull = ducking ? kDeployHullDuck : kDeployHullHuman;
	float hullHalf = ducking ? kDeployDuckHullHalf : kDeployStandHullHalf;
	float feetZ = pEdict->v.origin.z - hullHalf;

	const Vector &origin = pEdict->v.origin;
	Vector end = origin + moveDir * lookahead;

	int steps = static_cast<int>(lookahead / kDeployGuardStep) + 1;
	BeamBlock worst = BEAM_CLEAR;
	bool walkOver = gpGlobals->time < bot.avoid.walkOverUntil;

	for (size_t i = 0; i < g_beams.size(); ++i)
	{
		const Beam &beam = g_beams[i];
		if (!beam.ent || beam.ent->free || (beam.hazard && walkOver)
			|| BotAvoid::BeamJumpActive(bot, beam.ent))
			continue;

		float zLo = 0.f, zHi = 0.f;
		if (BeamMeetsHull(beam, origin, hull, zLo, zHi, kBeamGuardPad))
			continue;

		bool ahead = false;
		for (int s = 1; s <= steps && !ahead; ++s)
		{
			float t = static_cast<float>(s) / static_cast<float>(steps);
			Vector at = origin + (end - origin) * t;
			ahead = BeamMeetsHull(beam, at, hull, zLo, zHi, kBeamGuardPad);
		}

		if (!ahead)
			continue;

		if (!ducking && zLo - feetZ >= kDeployDuckClear)
		{
			if (worst == BEAM_CLEAR)
			{
				worst = BEAM_DUCK;
				if (hit)
					*hit = &beam;
			}
			continue;
		}

		if (hit)
			*hit = &beam;
		return BEAM_STOP;
	}

	return worst;
}

bool Active(const CBot &bot)
{
	return bot.deploy.active;
}

void OnRespawn(CBot &bot)
{
	State &st = bot.deploy;
	st.active = false;
	st.ordered = false;
	st.phase = PHASE_NONE;
	st.spot = Spot();
	st.prevWeaponId = 0;
	st.nextTryTime = 0.f;
	st.lastReason = "respawned";
	st.memo.clear();
}

static void Begin(CBot &bot, DeployKind kind, const Spot &spot, bool duck)
{
	State &st = bot.deploy;
	st.active = true;
	st.phase = PHASE_APPROACH;
	st.spot = spot;
	st.kind = kind;
	st.prevWeaponId = ActiveWeaponId(bot);
	st.startTime = gpGlobals->time;
	st.nextSelectTime = 0.f;
	st.firedTime = 0.f;
	st.lastReason = "started";

	BOT_VLOG("CrossBot: %s is mining conn %u (open %.0f, %s, %s) at (%.0f,%.0f,%.0f)\n", bot.name,
		spot.connIndex, spot.opening, SpotKindName(spot.kind), duck ? "ducked" : "standing",
		spot.stand.x, spot.stand.y, spot.stand.z);
}

void TryStart(CBot &bot, const Nav::NavMesh &mesh)
{
	State &st = bot.deploy;
	if (st.active)
		return;
	if (!Enabled() || !g_pGameProfile || BotDirective::MeleeOnly(bot) || BotDirective::HoldFire(bot) ||
		BotDirective::Busy(bot))
		return;

	float now = gpGlobals->time;

	if (bot.sense.enemy.Get() || bot.sense.target.Get())
	{
		st.lastReason = "enemy";
		return;
	}

	if (bot.fire.commit != BotFire::COMMIT_NONE)
	{
		st.lastReason = "committed";
		return;
	}

	if (TryDetonate(bot))
		return;

	if (now < st.nextTryTime)
		return;
	st.nextTryTime = now + kDeployTryInterval;

	PruneMemo(st);

	DeployKind kind = DEPLOY_TRIPMINE;
	int weaponId = g_pGameProfile->DeployWeapon(kind);
	if (weaponId == 0 || DeployAmmo(bot, weaponId) <= 0)
	{
		kind = DEPLOY_SATCHEL;
		weaponId = g_pGameProfile->DeployWeapon(kind);
	}
	if (weaponId == 0 || DeployAmmo(bot, weaponId) <= 0)
	{
		st.lastReason = "no-ammo";
		return;
	}

	bool duck = kind == DEPLOY_TRIPMINE && (NextRand(st, bot.pEdict) & 1u) != 0u;

	Spot spot;
	if (kind == DEPLOY_SATCHEL)
	{
		if (!FindDropSpot(bot, mesh, bot.currentAreaId, spot))
		{
			st.lastReason = "no-gate";
			return;
		}
	}
	else if (!FindSpot(bot, mesh, bot.currentAreaId, duck, spot))
	{
		st.lastReason = "no-gate";
		return;
	}

	Begin(bot, kind, spot, duck);
}

bool Reserve(CBot &bot, const Nav::NavMesh &mesh, const char *item, Vector &stand, const char *&why)
{
	State &st = bot.deploy;
	if (st.active)
	{
		why = "already placing";
		return false;
	}

	int kinds = 0;
	int chosen = -1;
	for (int k = 0; k < DEPLOY_COUNT && chosen < 0; ++k)
	{
		int weaponId = g_pGameProfile ? g_pGameProfile->DeployWeapon(static_cast<DeployKind>(k)) : 0;
		const WeaponSpec *spec = weaponId ? g_pGameProfile->GetWeaponSpec(weaponId) : nullptr;
		if (!spec || (item && item[0] && (!spec->classname || CB_STRICMP(spec->classname, item) != 0)))
			continue;
		++kinds;
		if (DeployAmmo(bot, weaponId) > 0)
			chosen = k;
	}
	if (chosen < 0)
	{
		why = kinds ? "none left to place" : "nothing it can place";
		return false;
	}

	DeployKind kind = static_cast<DeployKind>(chosen);
	bool duck = kind == DEPLOY_TRIPMINE && (NextRand(st, bot.pEdict) & 1u) != 0u;
	std::vector<int> areas(1, bot.currentAreaId);
	const Nav::Area *here = bot.currentAreaId >= 0 ? Nav::FindAreaById(mesh, static_cast<uint32_t>(bot.currentAreaId)) : nullptr;
	for (size_t i = 0; here && i < here->connIndices.size(); ++i)
	{
		const Nav::Connection &conn = mesh.conns[here->connIndices[i]];
		areas.push_back(static_cast<int>(conn.areaA == here->id ? conn.areaB : conn.areaA));
	}

	Spot spot;
	bool found = false;
	float bestDist = 0.f;
	for (int areaId : areas)
	{
		Spot candidate;
		bool ok = kind == DEPLOY_SATCHEL ? FindDropSpot(bot, mesh, areaId, candidate)
			: FindSpot(bot, mesh, areaId, duck, candidate);
		float dist = (candidate.stand - bot.pEdict->v.origin).Length();
		if (!ok || (found && dist >= bestDist))
			continue;
		spot = candidate;
		bestDist = dist;
		found = true;
	}
	if (!found)
	{
		why = "no spot to place it here";
		return false;
	}

	st.pending = spot;
	st.pendingKind = kind;
	st.pendingDuck = duck;
	stand = spot.stand;
	return true;
}

void StartReserved(CBot &bot)
{
	State &st = bot.deploy;
	Begin(bot, st.pendingKind, st.pending, st.pendingDuck);
	st.ordered = true;
}

bool Run(CBot &bot)
{
	State &st = bot.deploy;
	if (!st.active)
		return false;

	if ((!Enabled() && !st.ordered) || !g_pGameProfile)
	{
		st.active = false;
		st.ordered = false;
		st.phase = PHASE_NONE;
		return false;
	}

	edict_t *pEdict = bot.pEdict;
	float now = gpGlobals->time;

	bot.lastProgressTime = now;

	if (bot.sense.enemy.Get() || bot.sense.target.Get())
	{
		Stop(bot, "broke off mining", false);
		return false;
	}

	if (now - st.startTime > kDeployTimeout)
	{
		Stop(bot, "gave up mining", false);
		return false;
	}

	int weaponId = g_pGameProfile->DeployWeapon(st.kind);
	const char *entity = g_pGameProfile->DeployEntity(st.kind);

	if (st.phase == PHASE_VERIFY)
	{
		if (now - st.firedTime < kDeployVerifyDelay)
			return true;

		if (st.kind == DEPLOY_SATCHEL)
		{
			bool placed = ChargeReady(bot, weaponId) == 1;
			Stop(bot, placed ? "placed a mine" : "placed nothing", placed);
			return false;
		}

		edict_t *mine = EntityNear(entity, st.spot.mount, kDeployVerifyRadius);
		if (!mine)
		{
			Stop(bot, "placed nothing", false);
			return false;
		}

		RememberMine(st, mine);
		AuditMine(bot, mine);

		st.retreatMine.Set(mine);
		st.retreatStart = now;
		st.phase = PHASE_RETREAT;
		return true;
	}

	if (st.phase == PHASE_RETREAT)
	{
		edict_t *mine = st.retreatMine.Get();
		Vector start, end;

		if (!mine || !MineBeam(mine, start, end)
			|| !BotOnBeam(bot, start, end, kDeployClearMargin))
		{
			Stop(bot, "placed a mine", true);
			return false;
		}

		if (now - st.retreatStart > kDeployRetreatTimeout)
		{
			if (DebugEnabled())
				BOT_VLOG("CrossBot: %s COULD NOT LEAVE its own beam in %.1fs -- the mine will arm "
					"around it\n", bot.name, kDeployRetreatTimeout);
			Stop(bot, "placed a mine", true);
			return false;
		}

		bot.lastProgressTime = now;

		Vector retAngles = Bot_UpdateFacing(bot);

		Vector moveDir = st.spot.retreatDir;
		float drive = Bot_MaxSpeed(pEdict);

		NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
			Bot_DrivePhysics(pEdict));
		if (solved.wishSpeed > 0.f)
		{
			moveDir = solved.wishDir;
			drive = solved.wishSpeed;
		}

		if ((pEdict->v.flags & FL_DUCKING) != 0)
			drive /= PmSim::kDuckingMultiplier;

		float rf = 0.f, rs = 0.f;
		Bot_DecomposeMove(retAngles.y + pEdict->v.punchangle.y, moveDir, drive, rf, rs);
		bot.moveForward = rf;
		bot.moveSide = rs;

		unsigned short retButtons = 0;
		if (rf > kDeployMoveDeadzone) retButtons |= IN_FORWARD;
		else if (rf < -kDeployMoveDeadzone) retButtons |= IN_BACK;
		if (rs > kDeployMoveDeadzone) retButtons |= IN_MOVERIGHT;
		else if (rs < -kDeployMoveDeadzone) retButtons |= IN_MOVELEFT;

		Bot_RunPlayerMove(bot, retAngles, rf, rs, retButtons);
		return true;
	}

	if (st.phase == PHASE_DETONATE)
	{
		Vector detAngles = Bot_UpdateFacing(bot);

		unsigned short detButtons = 0;
		int satchelId = g_pGameProfile->DeployWeapon(DEPLOY_SATCHEL);

		if (st.firedTime > 0.f && now - st.firedTime >= kDeployVerifyDelay)
		{
			bool fired = ChargeReady(bot, satchelId) != 1;
			Stop(bot, fired ? "detonated a satchel" : "could not detonate", fired);
			return false;
		}

		if (st.firedTime <= 0.f && now - st.startTime > kDeployDetonateTimeout)
		{
			Stop(bot, "could not detonate", false);
			return false;
		}

		if (ActiveWeaponId(bot) != satchelId)
		{
			if (now >= st.nextSelectTime)
			{
				SelectWeapon(bot, satchelId);
				st.nextSelectTime = now + kDeploySelectInterval;
			}
		}
		else if (st.firedTime <= 0.f && ChargeReady(bot, satchelId) == 1
			&& AttackReady(bot, satchelId))
		{
			detButtons |= IN_ATTACK;
			st.firedTime = now;
		}

		bot.moveForward = 0.f;
		bot.moveSide = 0.f;
		Bot_RunPlayerMove(bot, detAngles, 0.f, 0.f, detButtons);
		return true;
	}

	const Vector &origin = pEdict->v.origin;
	Vector to(st.spot.stand.x - origin.x, st.spot.stand.y - origin.y, 0.f);
	float dist = sqrtf(to.x * to.x + to.y * to.y);

	if (st.phase == PHASE_APPROACH && dist <= kDeployStandRadius)
		st.phase = PHASE_SELECT;

	bool wantDuck = st.spot.duck && st.phase != PHASE_APPROACH;

	if (st.phase == PHASE_APPROACH)
		bot.desiredAngles = AnglesTo(Norm2D(to));
	else
		bot.desiredAngles = st.spot.aimAngles;

	Vector angles = Bot_UpdateFacing(bot);

	float drive = 0.f;
	Vector moveDir(0.f, 0.f, 0.f);
	if (dist > kDeployStandRadius)
	{
		moveDir = Norm2D(to);
		drive = Bot_MaxSpeed(pEdict) * PmSim::kDuckingMultiplier;

		NavFollow::Drive solved = NavFollow::SolveDrive(moveDir * drive, pEdict->v.velocity,
			Bot_DrivePhysics(pEdict));
		if (solved.wishSpeed > 0.f)
		{
			moveDir = solved.wishDir;
			drive = solved.wishSpeed;
		}
	}

	if ((pEdict->v.flags & FL_DUCKING) != 0)
		drive /= PmSim::kDuckingMultiplier;

	float fmove = 0.f, smove = 0.f;
	Bot_DecomposeMove(angles.y + pEdict->v.punchangle.y, moveDir, drive, fmove, smove);
	bot.moveForward = fmove;
	bot.moveSide = smove;

	unsigned short buttons = 0;
	if (fmove > kDeployMoveDeadzone) buttons |= IN_FORWARD;
	else if (fmove < -kDeployMoveDeadzone) buttons |= IN_BACK;
	if (smove > kDeployMoveDeadzone) buttons |= IN_MOVERIGHT;
	else if (smove < -kDeployMoveDeadzone) buttons |= IN_MOVELEFT;
	if (wantDuck) buttons |= IN_DUCK;

	if (st.phase == PHASE_SELECT)
	{
		if (ActiveWeaponId(bot) == weaponId)
		{
			st.phase = PHASE_AIM;
		}
		else if (now >= st.nextSelectTime)
		{
			SelectWeapon(bot, weaponId);
			st.nextSelectTime = now + kDeploySelectInterval;
		}
	}

	if (st.phase == PHASE_AIM)
	{
		bool aimed =
			fabsf(Bot_NormalizeAngle(bot.desiredAngles.y - angles.y)) <= kDeployAimTolerance &&
			fabsf(Bot_NormalizeAngle(bot.desiredAngles.x - angles.x)) <= kDeployAimTolerance;
		bool stanced = st.spot.duck == ((pEdict->v.flags & FL_DUCKING) != 0);

		const BotWeapon *w = WeaponById(bot, weaponId);
		bool ready = w && bot.inventory.nextAttack <= 0.f
			&& (st.kind == DEPLOY_SATCHEL ? w->nextSecondaryAttack <= 0.f
				: w->nextPrimaryAttack <= 0.f);

		if (ActiveWeaponId(bot) != weaponId)
			st.phase = PHASE_SELECT;
		else if (aimed && stanced && ready)
			st.phase = PHASE_FIRE;
	}

	if (st.phase == PHASE_FIRE)
	{
		buttons |= st.kind == DEPLOY_SATCHEL ? IN_ATTACK2 : IN_ATTACK;
		st.firedTime = now;
		st.phase = PHASE_VERIFY;
	}

	Bot_RunPlayerMove(bot, angles, fmove, smove, buttons);
	return true;
}

void PrintGates(const Nav::NavMesh &mesh)
{
	size_t shown = 0;

	for (size_t a = 0; a < mesh.areas.size(); ++a)
	{
		const Nav::Area &area = mesh.areas[a];

		for (size_t i = 0; i < area.connIndices.size(); ++i)
		{
			uint32_t ci = area.connIndices[i];
			if (ci >= mesh.conns.size() || mesh.conns[ci].connType != crossbot_nav::CONN_WALK)
				continue;

			Spot spot;
			bool ok = EvaluateGate(mesh, area.id, ci, DEPLOY_TRIPMINE, false, spot);
			if (!ok && spot.reject != REJECT_COVERAGE && spot.reject != REJECT_SURFACE
				&& spot.reject != REJECT_ESCAPE
				&& spot.reject != REJECT_MINED)
				continue;

			char msg[224];
			snprintf(msg, sizeof(msg),
				"CrossBot: gate conn %u area %u->%u width %.0f open %.0f %s at "
				"(%.0f,%.0f,%.0f) %s\n",
				ci, area.id, spot.otherAreaId, spot.gateWidth, spot.opening,
				SpotKindName(spot.kind), spot.mount.x, spot.mount.y, spot.mount.z,
				SpotRejectName(spot.reject));
			SERVER_PRINT(msg);

			if (ok)
				++shown;
		}
	}

	char msg[96];
	snprintf(msg, sizeof(msg), "CrossBot: %zu deployable gates\n", shown);
	SERVER_PRINT(msg);
}

}
