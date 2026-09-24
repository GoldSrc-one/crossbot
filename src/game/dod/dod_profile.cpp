#include <extdll.h>
#include <meta_api.h>
#include <commonmacros.h>
#include <in_buttons.h>

#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <cmath>

#include "../../bot/bot.h"
#include "../../bot/bot_ability.h"
#include "../../bot/bot_aim.h"
#include "../../bot/bot_camp.h"
#include "../../bot/bot_combat.h"
#include "../../bot/bot_difficulty.h"
#include "../../bot/bot_fire.h"
#include "../../bot/bot_log.h"
#include "../../bot/bot_move.h"
#include "../../bot/bot_timing.h"
#include "../../bot/bot_weapons.h"
#include "../../nav/nav_query.h"
#include "../crossplay.h"
#include "../ent_handle.h"
#include "../fake_cmd.h"
#include "../gamedata.h"
#include "../teams.h"
#include "dod_profile.h"
#include "dod_weapons.h"

namespace
{
	constexpr int kMaxPlayers = 32;
	constexpr int kAllies = 1;
	constexpr int kAxis = 2;
	constexpr int kTeamAuto = 5;
	constexpr int kJoinTries = 3;
	constexpr float kJoinInterval = 1.2f;

	constexpr int kProne = 1;
	constexpr int kProneDeployed = 2;
	constexpr float kSandbagDeployed = 2.f;
	constexpr float kSandbagZone = 1.f;
	constexpr int kStateScoped = 1;

	constexpr float kRecoilTime = 0.1f;
	constexpr float kSwayStand = 0.5f;
	constexpr float kSwayDuck = 0.2f;
	constexpr float kSwayProne = 0.1f;
	constexpr float kDeployYaw = 45.f;
	constexpr float kDeployPitch = 20.f;
	constexpr float kDeployPitchUp = -10.f;

	constexpr float kToggleInterval = 1.f;
	constexpr float kProneInterval = 1.2f;
	constexpr float kSelectInterval = 0.5f;
	constexpr float kDeployTimeout = 5.f;
	constexpr float kDeployFacing = 20.f;

	constexpr float kStaminaCap = 100.f;
	constexpr float kWalkBase = 120.f;
	constexpr float kSprintBase = 230.f;
	constexpr float kSprintStart = 90.f;
	constexpr float kSprintStop = 70.f;

	constexpr float kThrowRate = 4.f;
	constexpr float kThrowSpeedMax = 600.f;

	constexpr float kFallSafeSpeed = 500.f;
	constexpr float kFallDamage = 100.f / 400.f;

	constexpr float kDecideInterval = 0.5f;
	constexpr float kAimTolerance = 4.f;
	constexpr float kDiscardTimeout = 1.5f;
	constexpr float kDiscardRetry = 10.f;
	constexpr float kOncePerLife = 1.0e6f;
	constexpr float kStabRange = 300.f;
	constexpr float kStabLost = 400.f;
	constexpr float kStabBehind = -0.3f;
	constexpr float kStabSpotted = 0.3f;
	constexpr float kStabReach = 56.f;
	constexpr float kStabTimeout = 4.f;

	struct ClassInfo
	{
		const char *name;
		const char *cmd;
		int team;
	};

	const ClassInfo kClasses[] = {
		{ "garand", "cls_garand", kAllies },
		{ "carbine", "cls_carbine", kAllies },
		{ "tommy", "cls_tommy", kAllies },
		{ "grease", "cls_grease", kAllies },
		{ "spring", "cls_spring", kAllies },
		{ "bar", "cls_bar", kAllies },
		{ "30cal", "cls_30cal", kAllies },
		{ "bazooka", "cls_bazooka", kAllies },
		{ "enfield", "cls_enfield", kAllies },
		{ "sten", "cls_sten", kAllies },
		{ "enfields", "cls_enfields", kAllies },
		{ "bren", "cls_bren", kAllies },
		{ "piat", "cls_piat", kAllies },
		{ "k98", "cls_k98", kAxis },
		{ "k43", "cls_k43", kAxis },
		{ "mp40", "cls_mp40", kAxis },
		{ "mp44", "cls_mp44", kAxis },
		{ "k98s", "cls_k98s", kAxis },
		{ "fg42", "cls_fg42", kAxis },
		{ "fg42_s", "cls_fg42_s", kAxis },
		{ "mg34", "cls_mg34", kAxis },
		{ "mg42", "cls_mg42", kAxis },
		{ "pschreck", "cls_pschreck", kAxis },
		{ "random", "cls_random", 0 },
	};

	constexpr int kRandomClass = ARRAYSIZE(kClasses) - 1;

	struct Slot
	{
		int cls = kRandomClass;
		int tries = 0;
		float nextJoin = 0.f;

		bool sprint = false;

		int lastId = 0;
		int lastClip = 0;
		float kickPitch = 0.f;
		float kickYaw = 0.f;
		float kickLeft = 0.f;

		bool deployed = false;
		float deployYaw = 0.f;
		float deployPitch = 0.f;

		bool wantGun = false;
		float nextToggle = 0.f;
		float nextProne = 0.f;
		float nextSelect = 0.f;
	};

	Slot g_slots[kMaxPlayers + 1];

	int PlayerSlot(edict_t *pEdict)
	{
		int index = g_engfuncs.pfnIndexOfEdict(pEdict);
		return (index >= 1 && index <= kMaxPlayers) ? index : 0;
	}

	void IdleMove(edict_t *pEdict)
	{
		byte msec = Bot_FrameMsec();
		BotTiming::BeginMove(pEdict, msec);
		g_engfuncs.pfnRunPlayerMove(pEdict, pEdict->v.v_angle, 0.f, 0.f, 0.f, 0, 0, msec);
		BotTiming::EndMove(pEdict);
	}

	bool Own(const edict_t *ent)
	{
		return XPlay::OwnerOf(ent) == XPlay::Current();
	}

	bool Prone(const edict_t *pEdict)
	{
		return pEdict->v.iuser3 == kProne || pEdict->v.iuser3 == kProneDeployed;
	}

	bool Deployed(const edict_t *pEdict)
	{
		return pEdict->v.iuser3 == kProneDeployed || pEdict->v.vuser1.x == kSandbagDeployed;
	}

	bool RecoilSkipsDeployed(int id)
	{
		return DOD_IsSandbagGun(id) || id == DOD_FG42;
	}

	int BipodGun(const BotInventory &inv)
	{
		for (int i = 0; i < inv.count; ++i)
		{
			if (DOD_IsSandbagGun(inv.weapons[i].id))
				return inv.weapons[i].id;
		}
		return 0;
	}

	int ActiveId(const BotInventory &inv)
	{
		return (inv.activeIndex >= 0 && inv.activeIndex < inv.count) ? inv.weapons[inv.activeIndex].id : 0;
	}

	bool HasPrimary(const edict_t *pEdict)
	{
		unsigned int weapons = static_cast<unsigned int>(pEdict->v.weapons);
		for (int id = DOD_GARAND; id <= DOD_PIAT; ++id)
		{
			if (DOD_IsPrimary(id) && (weapons & (1u << id)) != 0)
				return true;
		}
		return false;
	}

	void Kick(edict_t *pEdict, Slot &s, int id)
	{
		DODRecoil r = DOD_Recoil(id);
		if (r.pitch <= 0.f)
			return;
		if (Deployed(pEdict) && RecoilSkipsDeployed(id))
			return;

		float scale = 1.f;
		if (Prone(pEdict) && !DOD_NeedsDeploy(id))
			scale = 0.25f;
		else if ((pEdict->v.button & IN_DUCK) != 0)
			scale = 0.5f;

		float yaw = r.yaw * scale * RANDOM_FLOAT(0.8f, 1.1f);
		s.kickPitch = r.pitch * scale;
		s.kickYaw = RANDOM_LONG(0, 1) ? -yaw : yaw;
		s.kickLeft = kRecoilTime;
	}

	void Hold(float &fmove, float &smove, unsigned short &buttons)
	{
		fmove = 0.f;
		smove = 0.f;
		buttons &= ~(IN_FORWARD | IN_BACK | IN_MOVELEFT | IN_MOVERIGHT | IN_JUMP | IN_DUCK | IN_RUN);
	}

	void Toggle(Slot &s, unsigned short &buttons)
	{
		float now = gpGlobals->time;
		if (now < s.nextToggle)
			return;
		s.nextToggle = now + kToggleInterval;
		buttons &= ~IN_ATTACK;
		buttons |= IN_ATTACK2;
	}

	void ToggleProne(edict_t *pEdict, Slot &s)
	{
		float now = gpGlobals->time;
		if (now < s.nextProne)
			return;
		s.nextProne = now + kProneInterval;
		FakeCmd::Run(pEdict, "sprone");
	}

	bool Stop(BotAbility::State &st, const char *why)
	{
		st.task = BotAbility::TASK_NONE;
		st.target.Clear();
		st.why = why;
		return false;
	}

	Vector Eye(const edict_t *ent)
	{
		return ent->v.origin + ent->v.view_ofs;
	}

	void Face(CBot &bot, const Vector &point, bool level)
	{
		float yaw = 0.f, pitch = 0.f;
		if (BotAim::LosAngles(Eye(bot.pEdict), point, yaw, pitch))
			bot.desiredAngles = Vector(level ? 0.f : pitch, yaw, 0.f);
	}

	bool Aimed(const CBot &bot)
	{
		const Vector &want = bot.desiredAngles;
		const Vector &have = bot.pEdict->v.v_angle;
		return fabsf(Bot_NormalizeAngle(have.y - want.y)) < kAimTolerance && fabsf(have.x - want.x) < kAimTolerance;
	}

	float Watching(const edict_t *viewer, const edict_t *ent)
	{
		Vector to = ent->v.origin - viewer->v.origin;
		float dist = to.Length2D();
		if (dist < 1.f)
			return 1.f;
		MAKE_VECTORS(Vector(0.f, viewer->v.v_angle.y, 0.f));
		return DotProduct2D(gpGlobals->v_forward, to * (1.f / dist));
	}

	bool Friend(edict_t *self, edict_t *ent)
	{
		return ent && ent != self && CBotIsLivePlayer(ent) && !g_pGameProfile->IsDead(ent) &&
			XPlay::OwnerOf(ent) == XPlay::OwnerOf(self) && Teams::Of(self, ent) == Teams::REL_FRIEND;
	}

	bool StartDiscard(CBot &bot, const Nav::NavMesh &mesh)
	{
		BotAbility::State &st = bot.ability;
		edict_t *self = bot.pEdict;
		float now = gpGlobals->time;
		if (now < st.nextDiscard || bot.currentAreaId < 0 || bot.sense.target.Get() || BotCombat::Active(bot) ||
			(self->v.flags & FL_ONGROUND) == 0)
			return false;

		edict_t *best = nullptr;
		float bestDist = 0.f;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!Friend(self, ent))
				continue;
			float dist = (ent->v.origin - self->v.origin).Length();
			if (best && dist >= bestDist)
				continue;
			const Nav::Area *area = Nav::FindAreaNearest(mesh, ent->v.origin);
			if (!area || static_cast<int>(area->id) != bot.currentAreaId)
				continue;
			best = ent;
			bestDist = dist;
		}
		if (!best)
			return false;

		st.task = BotAbility::TASK_DISCARD;
		st.target.Set(best);
		st.start = now;
		st.nextDiscard = now + kDiscardRetry;
		st.why = "sharing ammo";
		return true;
	}

	bool RunDiscard(CBot &bot)
	{
		BotAbility::State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *t = st.target.Get();
		float now = gpGlobals->time;
		if (!Friend(self, t))
			return Stop(st, "teammate gone");
		if (bot.sense.target.Get())
			return Stop(st, "enemy");
		if (now - st.start > kDiscardTimeout)
			return Stop(st, "discard timed out");

		Face(bot, Eye(t), true);
		Vector angles = Bot_UpdateFacing(bot);
		if (Aimed(bot))
		{
			FakeCmd::Run(self, "dropammo");
			++st.discards;
			st.nextDiscard = now + kOncePerLife;
			BOT_VLOG("CrossBot: %s drops ammo for %s\n", bot.name, CBotPlayerNameOf(t));
			Stop(st, "shared ammo");
		}
		Bot_RunPlayerMove(bot, angles, 0.f, 0.f, 0);
		return true;
	}

	bool StartStab(CBot &bot)
	{
		BotAbility::State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *t = bot.sense.target.Get();
		if (!BotDiff::Backstab() || !t || !CBotIsLivePlayer(t) || Teams::Of(self, t) != Teams::REL_ENEMY ||
			!DOD_RifleMelee(ActiveId(bot.inventory)))
			return false;
		if ((t->v.origin - self->v.origin).Length() > kStabRange || Watching(t, self) > kStabBehind)
			return false;

		st.task = BotAbility::TASK_STAB;
		st.target.Set(t);
		st.start = gpGlobals->time;
		st.why = "stabbing";
		BOT_VLOG("CrossBot: %s charges %s from behind with its %s\n", bot.name, CBotPlayerNameOf(t),
			ActiveId(bot.inventory) == DOD_KAR ? "bayonet" : "rifle butt");
		return true;
	}

	bool RunStab(CBot &bot)
	{
		BotAbility::State &st = bot.ability;
		edict_t *self = bot.pEdict;
		edict_t *t = st.target.Get();
		if (!t || g_pGameProfile->IsDead(t))
		{
			if (t)
				++st.stabs;
			return Stop(st, "victim down");
		}
		if (gpGlobals->time - st.start > kStabTimeout)
			return Stop(st, "stab timed out");
		if (!DOD_RifleMelee(ActiveId(bot.inventory)))
			return Stop(st, "weapon changed");

		float dist = (t->v.origin - self->v.origin).Length();
		if (dist > kStabLost)
			return Stop(st, "victim lost");
		if (Watching(t, self) > kStabSpotted)
			return Stop(st, "spotted");

		int slot = PlayerSlot(self);
		if (slot)
			g_slots[slot].sprint = true;

		Face(bot, (t->v.absmin + t->v.absmax) * 0.5f, false);
		Vector angles = Bot_UpdateFacing(bot);
		unsigned short buttons = IN_FORWARD | IN_RUN;
		if (dist < kStabReach && Aimed(bot))
			buttons |= IN_ATTACK2;
		Bot_RunPlayerMove(bot, angles, Bot_MaxSpeed(self), 0.f, buttons);
		return true;
	}
}

const char *CDODProfile::GetName() const
{
	return "DOD";
}

bool CDODProfile::Detect(const char *gamedir) const
{
	return gamedir && CB_STRICMP(gamedir, "dod") == 0;
}

void CDODProfile::ApplyAddArg(int, edict_t *pEdict, const char *arg, char *out, unsigned int size) const
{
	int cls = kRandomClass;
	if (arg && arg[0] && strcmp(arg, "*") != 0)
	{
		int found = -1;
		for (int i = 0; i < static_cast<int>(ARRAYSIZE(kClasses)); ++i)
		{
			if (CB_STRICMP(arg, kClasses[i].name) == 0)
				found = i;
		}

		if (found >= 0)
		{
			cls = found;
		}
		else
		{
			char msg[384];
			int len = snprintf(msg, sizeof(msg), "CrossBot: unknown DoD class '%s' -- using random. Classes:", arg);
			for (size_t i = 0; i < ARRAYSIZE(kClasses) && len > 0 && len < static_cast<int>(sizeof(msg)); ++i)
				len += snprintf(msg + len, sizeof(msg) - len, " %s", kClasses[i].name);
			SERVER_PRINT(msg);
			SERVER_PRINT("\n");
		}
	}

	int slot = PlayerSlot(pEdict);
	if (slot)
	{
		g_slots[slot] = Slot();
		g_slots[slot].cls = cls;
		char *info = g_engfuncs.pfnGetInfoKeyBuffer(pEdict);
		if (info)
			g_engfuncs.pfnSetClientKeyValue(slot, info, "cl_autoreload", "0");
	}

	if (out && size)
		snprintf(out, size, "%s", kClasses[cls].name);
}

bool CDODProfile::PreThink(edict_t *pEdict) const
{
	int slot = PlayerSlot(pEdict);
	if (!slot)
		return false;

	int team = pEdict->v.team;
	bool teamed = team == kAllies || team == kAxis;
	int next = -1;
	void *priv = CBotPrivateData(pEdict);
	if (priv && g_offsets.plr_iNextClass >= 0)
		next = GameData::Peek<int>(priv, g_offsets.plr_iNextClass);
	else if (pEdict->v.playerclass != 0)
		next = pEdict->v.playerclass;

	if (teamed && next != -1)
		return false;

	Slot &s = g_slots[slot];
	float now = gpGlobals->time;
	if (now >= s.nextJoin || s.nextJoin - now > kJoinInterval)
	{
		s.nextJoin = now + kJoinInterval;
		bool retry = s.tries++ >= kJoinTries;
		const ClassInfo &info = kClasses[retry ? kRandomClass : s.cls];

		char cmd[32];
		if (!teamed)
			snprintf(cmd, sizeof(cmd), "jointeam %d", info.team ? info.team : kTeamAuto);
		else
			snprintf(cmd, sizeof(cmd), "%s", info.cmd);
		FakeCmd::Run(pEdict, cmd);
	}

	IdleMove(pEdict);
	return true;
}

bool CDODProfile::IsDead(edict_t *pEdict) const
{
	return CHLDMProfile::IsDead(pEdict) || pEdict->v.iuser1 > 0;
}

void CDODProfile::Respawn(edict_t *pEdict) const
{
	IdleMove(pEdict);
}

bool CDODProfile::AbilityRun(CBot &bot, const Nav::NavMesh &mesh) const
{
	BotAbility::State &st = bot.ability;
	edict_t *pEdict = bot.pEdict;
	int slot = PlayerSlot(pEdict);
	if (!slot || Prone(pEdict) || Deployed(pEdict) || g_slots[slot].wantGun)
		return st.task != BotAbility::TASK_NONE && Stop(st, "bipod");

	float now = gpGlobals->time;
	if (st.task == BotAbility::TASK_NONE && (now >= st.nextTry || st.nextTry - now > kDecideInterval))
	{
		st.nextTry = now + kDecideInterval;
		if (!StartStab(bot))
			StartDiscard(bot, mesh);
	}

	switch (st.task)
	{
	case BotAbility::TASK_STAB:    return RunStab(bot);
	case BotAbility::TASK_DISCARD: return RunDiscard(bot);
	default:                       return false;
	}
}

float CDODProfile::EvaluateCombat(edict_t *pEdict, const BotInventory &inv) const
{
	if (!pEdict)
		return 0.f;

	float maxHealth = (pEdict->v.max_health > 0.f) ? pEdict->v.max_health : 100.f;
	float survive = std::clamp(pEdict->v.health / maxHealth, 0.f, 1.f);

	float arms = 0.f;
	for (int i = 0; i < inv.count; ++i)
	{
		const BotWeapon &w = inv.weapons[i];
		const WeaponSpec *spec = GetWeaponSpec(w.id);
		if (spec && spec->value > 0.f && BotFire::WeaponArmed(inv, w))
			arms += spec->value;
	}

	return survive * std::clamp(arms, 0.f, 1.f);
}

PmSim::FallRule CDODProfile::FallDamage() const
{
	return { kFallSafeSpeed, kFallDamage };
}

unsigned short CDODProfile::FireToggle(const CBot &bot, bool firing) const
{
	const BotInventory &inv = bot.inventory;
	if (inv.activeIndex < 0 || inv.activeIndex >= inv.count || g_offsets.wpn_iWeaponState < 0)
		return 0;

	const BotWeapon &w = inv.weapons[inv.activeIndex];
	if (!DOD_IsRocket(w.id) || !w.priv)
		return 0;

	bool raised = (GameData::Peek<int>(w.priv, g_offsets.wpn_iWeaponState) & kStateScoped) != 0;
	bool want = firing && bot.pEdict->v.waterlevel < 2;
	if (raised == want)
		return 0;

	BOT_VLOG("CrossBot: %s %s its launcher\n", bot.name, raised ? "lowers" : "shoulders");
	return IN_ATTACK2;
}

bool CDODProfile::Zoomed(edict_t *pEdict) const
{
	void *priv = CBotPrivateData(pEdict);
	if (!priv || g_offsets.plr_iFOV < 0)
		return CHLDMProfile::Zoomed(pEdict);

	int fov = GameData::Peek<int>(priv, g_offsets.plr_iFOV);
	return fov > 0 && fov < 90;
}

void CDODProfile::AdjustView(CBot &bot, float dt, float &pitch, float &yaw) const
{
	edict_t *pEdict = bot.pEdict;
	int slot = PlayerSlot(pEdict);
	if (!slot)
		return;

	Slot &s = g_slots[slot];
	if (s.kickLeft > 0.f)
	{
		float share = std::min(dt, s.kickLeft) / kRecoilTime;
		pitch -= s.kickPitch * share;
		yaw += s.kickYaw * share;
		s.kickLeft -= dt;
	}

	if (Zoomed(pEdict))
	{
		float amp = kSwayStand;
		if ((pEdict->v.button & IN_DUCK) != 0)
			amp = kSwayDuck;
		else if (pEdict->v.iuser3 >= kProne)
			amp = kSwayProne;

		float t = gpGlobals->time;
		pitch += 2.f * dt * cosf(2.f * t) * amp;
		yaw += dt * cosf(t) * amp;
	}

	if (!Deployed(pEdict))
	{
		if (s.deployed)
			BOT_VLOG("CrossBot: %s folds its gun\n", bot.name);
		s.deployed = false;
		return;
	}

	if (!s.deployed)
	{
		s.deployed = true;
		s.deployYaw = yaw;
		s.deployPitch = std::max(pitch, kDeployPitchUp);
		BOT_VLOG("CrossBot: %s deploys its gun %s, facing yaw %.0f\n", bot.name,
			pEdict->v.iuser3 == kProneDeployed ? "prone" : "on sandbags", s.deployYaw);
	}

	float off = std::clamp(Bot_NormalizeAngle(yaw - s.deployYaw), -kDeployYaw, kDeployYaw);
	yaw = s.deployYaw + off;
	pitch = std::clamp(pitch, s.deployPitch - kDeployPitch, s.deployPitch + kDeployPitch);
}

void CDODProfile::MoveAdjust(CBot &bot, float &fmove, float &smove, unsigned short &buttons) const
{
	edict_t *pEdict = bot.pEdict;
	int slot = PlayerSlot(pEdict);
	if (!slot)
		return;

	Slot &s = g_slots[slot];
	float now = gpGlobals->time;
	const BotInventory &inv = bot.inventory;

	BotCamp::Bipod want = BotCamp::HeldBipod(bot);
	int gun = BipodGun(inv);
	if (want != BotCamp::BIPOD_NONE && (!gun || now - bot.camp.holdStart > kDeployTimeout) && !Deployed(pEdict))
		want = BotCamp::BIPOD_NONE;
	s.wantGun = want != BotCamp::BIPOD_NONE;

	if (s.wantGun)
	{
		s.sprint = false;
		Hold(fmove, smove, buttons);
		if (Deployed(pEdict))
			return;

		if (ActiveId(inv) != gun)
		{
			if (now >= s.nextSelect)
			{
				s.nextSelect = now + kSelectInterval;
				SelectWeapon(pEdict, *DOD_WeaponSpec(gun));
			}
			buttons &= ~(IN_ATTACK | IN_ATTACK2);
			return;
		}

		if (want == BotCamp::BIPOD_SANDBAG)
		{
			if (pEdict->v.vuser1.x == kSandbagZone)
				Toggle(s, buttons);
			return;
		}

		bool facing = bot.sense.target.Get() ||
			fabsf(Bot_NormalizeAngle(pEdict->v.v_angle.y - bot.camp.spot.yaw)) <= kDeployFacing;
		if (pEdict->v.iuser3 == kProne && facing)
			Toggle(s, buttons);
		else if (pEdict->v.iuser3 != kProne)
			ToggleProne(pEdict, s);
		return;
	}

	if (Deployed(pEdict))
	{
		s.sprint = false;
		Hold(fmove, smove, buttons);
		Toggle(s, buttons);
		return;
	}

	if (pEdict->v.iuser3 == kProne)
	{
		s.sprint = false;
		Hold(fmove, smove, buttons);
		ToggleProne(pEdict, s);
		return;
	}

	bool calm = !bot.sense.target.Get() && !BotCombat::Active(bot);
	bool ground = (pEdict->v.flags & FL_ONGROUND) != 0;
	float stamina = pEdict->v.fuser4;
	if (bot.ability.task == BotAbility::TASK_STAB)
		s.sprint = true;
	else if (!calm || fmove <= 0.f || !ground || stamina < kSprintStop)
		s.sprint = false;
	else if (stamina >= kSprintStart)
		s.sprint = true;

	if (s.sprint)
		buttons |= IN_RUN;
}

float CDODProfile::MaxSpeed(edict_t *pEdict, float engineMax) const
{
	int slot = PlayerSlot(pEdict);
	if (!slot || !Own(pEdict))
		return engineMax;

	float stamina = std::clamp(pEdict->v.fuser4, 0.f, kStaminaCap);
	float speed = stamina + (g_slots[slot].sprint ? kSprintBase : kWalkBase);
	return std::min(engineMax, speed);
}

bool CDODProfile::CanDeploy(edict_t *pEdict) const
{
	if (!pEdict || !Own(pEdict))
		return false;

	int weapons = pEdict->v.weapons;
	const int guns[] = { DOD_30CAL, DOD_BAR, DOD_BREN, DOD_MG34, DOD_MG42 };
	for (int id : guns)
	{
		if ((weapons & (1 << id)) != 0)
			return true;
	}
	return false;
}

float CDODProfile::BlindSeconds(float fadeSec, float holdSec, float scale) const
{
	return holdSec + scale * fadeSec;
}

bool CDODProfile::ItemUsable(edict_t *player, edict_t *item) const
{
	if (!player || !item)
		return false;
	if (ClassifyItem(item) == ITEM_WEAPON)
		return !HasPrimary(player);
	if (item->v.owner == player)
		return false;
	return item->v.team == 0 || item->v.team == player->v.team;
}

ItemKind CDODProfile::ClassifyItem(edict_t *ent) const
{
	ItemKind kind = CHLDMProfile::ClassifyItem(ent);
	return (kind == ITEM_AMMO || kind == ITEM_WEAPON) ? kind : ITEM_NONE;
}

const WeaponSpec *CDODProfile::GetWeaponSpec(int id) const
{
	return DOD_WeaponSpec(id);
}

int CDODProfile::WeaponSlotCount() const
{
	return kDODWeaponSlots;
}

const WeaponSpec *CDODProfile::WeaponSlot(int slot) const
{
	return DOD_WeaponSlot(slot);
}

int CDODProfile::ItemSlots() const
{
	return kDODItemSlots;
}

bool CDODProfile::ReadInventory(edict_t *player, BotInventory &out) const
{
	BotWeapons::ReadItems(player, out);

	for (int i = 0; i < out.count; ++i)
	{
		BotWeapon &w = out.weapons[i];
		w.chargeState = 0;
		w.throwStart = 0.f;
		if (!w.priv)
			continue;

		if ((w.id == DOD_ENFIELD || w.id == DOD_FG42) && g_offsets.wpn_iWeaponState >= 0 &&
			(GameData::Peek<int>(w.priv, g_offsets.wpn_iWeaponState) & kStateScoped) != 0)
			w.id = (w.id == DOD_ENFIELD) ? DOD_SCOPEDENFIELD : DOD_SCOPEDFG42;

		if (w.id >= DOD_HANDGRENADE && w.id <= DOD_HANDGRENADE_EX && g_offsets.gren_flStartThrow >= 0)
			w.throwStart = GameData::Peek<float>(w.priv, g_offsets.gren_flStartThrow);
	}

	int slot = PlayerSlot(player);
	if (!slot)
		return true;

	Slot &s = g_slots[slot];
	int id = 0;
	int clip = 0;
	if (out.activeIndex >= 0 && out.activeIndex < out.count)
	{
		const BotWeapon &w = out.weapons[out.activeIndex];
		id = w.id;
		clip = w.clip;
		if (id == s.lastId && clip < s.lastClip && !w.reloading)
			Kick(player, s, id);
	}
	s.lastId = id;
	s.lastClip = clip;
	return true;
}

bool CDODProfile::ModeAvailable(edict_t *player, const WeaponSpec &spec, const WeaponMode &) const
{
	if (DOD_NeedsDeploy(spec.id))
	{
		int slot = PlayerSlot(player);
		return Deployed(player) || (slot && g_slots[slot].wantGun);
	}
	if (DOD_IsRocket(spec.id))
		return player->v.waterlevel < 2;
	return true;
}

const char *CDODProfile::AmmoEntity(int, WeaponAmmo) const
{
	return nullptr;
}

bool CDODProfile::ItemPreferred(edict_t *, ItemKind kind) const
{
	return kind == ITEM_WEAPON;
}

int CDODProfile::MeleeWeapon() const
{
	return DOD_AMERKNIFE;
}

bool CDODProfile::IsMeleeWeapon(int id) const
{
	return id == DOD_AMERKNIFE || id == DOD_GERKNIFE || id == DOD_SPADE;
}

int CDODProfile::AmbushWeapon(AmbushKind kind) const
{
	switch (kind)
	{
	case AMBUSH_GRENADE: return DOD_HANDGRENADE;
	case AMBUSH_SNARK: return DOD_STICKGRENADE;
	default: return 0;
	}
}

int CDODProfile::DeployWeapon(DeployKind) const
{
	return 0;
}

const char *CDODProfile::DeployEntity(DeployKind) const
{
	return nullptr;
}

bool CDODProfile::ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
	float &speed) const
{
	return ThrowArc(mode, viewPitch, kThrowRate, kThrowSpeedMax, elevation, speed);
}
