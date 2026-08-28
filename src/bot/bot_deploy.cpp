#include <extdll.h>
#include <meta_api.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "bot_deploy.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_fire.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_sense.h"
#include "bot_weapons.h"
#include "../game/ent_handle.h"
#include "../game/fake_cmd.h"
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
	constexpr float kDeploySatchelRest = 4.f;
	constexpr float kDeploySatchelRadius = 375.f;
	constexpr float kDeployDetonateTimeout = 2.f;

	std::vector<BotDeploy::Beam> g_beams;

	using NavMath::kPiF;
	using NavMath::Norm2D;

	float Dist2D(const Vector &a, const Vector &b)
	{
		float dx = a.x - b.x;
		float dy = a.y - b.y;
		return sqrtf(dx * dx + dy * dy);
	}

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
		out.mount = tr.vecEndPos + n * kDeployMountOffset;
		return out;
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

		return FakeCmd::Run(bot.pEdict, spec->classname) ? spec->classname : NULL;
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

			char msg[256];
			snprintf(msg, sizeof(msg),
				"CrossBot: %s is detonating a satchel at (%.0f,%.0f,%.0f), contact %.0f away\n",
				bot.name, ent->v.origin.x, ent->v.origin.y, ent->v.origin.z, senseDist);
			SERVER_PRINT(msg);

			return true;
		}
		return false;
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

		char msg[224];
		snprintf(msg, sizeof(msg), "CrossBot: %s %s at conn %u after %.1fs\n", bot.name, why,
			st.spot.connIndex, gpGlobals->time - st.startTime);
		SERVER_PRINT(msg);

		st.active = false;
		st.phase = BotDeploy::PHASE_NONE;
		st.lastReason = why;
		bot.nextRepathTime = gpGlobals->time;
	}
}

namespace BotDeploy
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_deploy");
	return !cvar || cvar->value != 0.f;
}

SpotKind SurfacePin()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_deploy_surface");
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
	float span = Dist2D(gate->left, gate->right);
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
	for (int i = 0; i < 2 && !haveSurface && pin != SPOT_CEILING && pin != SPOT_GROUND; ++i)
	{
		int s = i == 0 ? nearSide : 1 - nearSide;
		if (!side[s].faces)
			continue;
		if (!Covers(side[s].mount, gate->left, gate->right, topZ))
			continue;

		mount = side[s].mount;
		out.kind = SPOT_WALL;
		out.aimAngles = AnglesTo(s == 0 ? along : along * -1.f);
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
		out.reject = REJECT_SURFACE;
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
	case PHASE_DETONATE: return "detonate";
	default: return "none";
	}
}

void RefreshBeams()
{
	g_beams.clear();

	if (!g_pGameProfile)
		return;

	const char *cls = g_pGameProfile->DeployEntity(DEPLOY_TRIPMINE);
	if (!cls)
		return;

	for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free || ent->v.classname == 0)
			continue;
		if (ent->v.solid != SOLID_BBOX)
			continue;
		if (strcmp(CBotClassOf(ent), cls) != 0)
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

bool Active(const CBot &bot)
{
	return bot.deploy.active;
}

void OnRespawn(CBot &bot)
{
	State &st = bot.deploy;
	st.active = false;
	st.phase = PHASE_NONE;
	st.spot = Spot();
	st.prevWeaponId = 0;
	st.nextTryTime = 0.f;
	st.lastReason = "respawned";
	st.memo.clear();
}

void TryStart(CBot &bot, const Nav::NavMesh &mesh)
{
	State &st = bot.deploy;
	if (st.active)
		return;
	if (!Enabled() || !g_pGameProfile)
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

	st.active = true;
	st.phase = PHASE_APPROACH;
	st.spot = spot;
	st.kind = kind;
	st.prevWeaponId = ActiveWeaponId(bot);
	st.startTime = now;
	st.nextSelectTime = 0.f;
	st.firedTime = 0.f;
	st.lastReason = "started";

	char msg[288];
	snprintf(msg, sizeof(msg),
		"CrossBot: %s is mining conn %u (open %.0f, %s, %s) at (%.0f,%.0f,%.0f)\n", bot.name,
		spot.connIndex, spot.opening, SpotKindName(spot.kind), duck ? "ducked" : "standing",
		spot.stand.x, spot.stand.y, spot.stand.z);
	SERVER_PRINT(msg);
}

bool Run(CBot &bot)
{
	State &st = bot.deploy;
	if (!st.active)
		return false;

	if (!Enabled() || !g_pGameProfile)
	{
		st.active = false;
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

		bool placed = st.kind == DEPLOY_SATCHEL
			? ChargeReady(bot, weaponId) == 1
			: EntityNear(entity, st.spot.mount, kDeployVerifyRadius) != NULL;
		Stop(bot, placed ? "placed a mine" : "placed nothing", placed);
		return false;
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
