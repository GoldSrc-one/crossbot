#include <extdll.h>
#include <meta_api.h>
#include <studio.h>

#include <algorithm>

#include "bot_sense.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_camp.h"
#include "bot_difficulty.h"
#include "bot_combat.h"
#include "bot_fire.h"
#include "bot_gauss.h"
#include "bot_move.h"
#include "bot_move_nav.h"
#include "bot_perf.h"
#include "../game/crossplay.h"
#include "../game/game_profile.h"
#include "../game/teams.h"
#include "../nav/nav.h"
#include "../nav/nav_math.h"
#include "../nav/nav_mesh.h"

namespace
{
	using NavMath::kPiF;

	constexpr float kDefaultDecay = 5.f;
	constexpr float kMaxDecay = 60.f;

	constexpr float kViewCone = 90.f;
	constexpr float kViewRange = 4096.f;

	constexpr float kGazeMinDist = 32.f;

	constexpr float kMonsterGaze = 2.f;

	constexpr float kLookPitchLimit = 45.f;

	constexpr float kSoundClipDist = 1000.f;

	constexpr size_t kGateLookahead = 3;

	constexpr size_t kMaxBoneSets = 64;
	constexpr size_t kMaxSampleBones = 32;
	constexpr int kHitGroupHead = 1;

	constexpr float kLeadEpsilonSq = 1.f;

	constexpr float kVantageInterval = 0.1f;
	constexpr float kVantageConeFloor = 0.15f;
	constexpr float kVantagePathSnap = 60.f;
	constexpr float kBackCheckHold = 0.4f;

	struct BoneSetEntry
	{
		int modelIndex = 0;
		const void *hdr = nullptr;
		unsigned char bones[kMaxSampleBones] = {};
		unsigned char count = 0;
		unsigned char headBone = 0;
		bool hasHead = false;
	};

	BoneSetEntry g_boneSets[kMaxBoneSets];
	size_t g_boneSetCount = 0;

	constexpr size_t kMaxMonsters = 64;

	edict_t *g_monsters[kMaxMonsters];
	size_t g_monsterCount = 0;

	float RandomUnit(BotSense::State &sense)
	{
		sense.rngState = sense.rngState * 1664525u + 1013904223u;
		return static_cast<float>((sense.rngState >> 8) & 0xFFFFFFu) / 16777216.f;
	}

	void SeedSense(CBot &bot)
	{
		BotSense::State &sense = bot.sense;
		if (sense.rngSeeded)
			return;

		sense.rngSeeded = true;
		sense.rngState = static_cast<unsigned int>(
			g_engfuncs.pfnIndexOfEdict(bot.pEdict)) * 2246822519u + 7u;
	}

	bool BuildBoneSet(edict_t *ent, BoneSetEntry &entry)
	{
		void *model = g_engfuncs.pfnGetModelPtr(ent);
		if (!model)
			return false;

		studiohdr_t *hdr = static_cast<studiohdr_t *>(model);
		if (hdr->numbones <= 0 || hdr->numhitboxes <= 0)
			return false;

		const mstudiobbox_t *boxes = reinterpret_cast<const mstudiobbox_t *>(
			reinterpret_cast<const byte *>(hdr) + hdr->hitboxindex);

		entry.modelIndex = ent->v.modelindex;
		entry.hdr = model;
		entry.count = 0;
		entry.hasHead = false;

		for (int i = 0; i < hdr->numhitboxes && entry.count < kMaxSampleBones; ++i)
		{
			int bone = boxes[i].bone;
			if (bone < 0 || bone >= hdr->numbones)
				continue;

			if (!entry.hasHead && boxes[i].group == kHitGroupHead)
			{
				entry.hasHead = true;
				entry.headBone = static_cast<unsigned char>(bone);
			}

			bool seen = false;
			for (unsigned char j = 0; j < entry.count; ++j)
			{
				if (entry.bones[j] == static_cast<unsigned char>(bone))
				{
					seen = true;
					break;
				}
			}
			if (seen)
				continue;

			entry.bones[entry.count++] = static_cast<unsigned char>(bone);
		}

		return entry.count > 0;
	}

	const BoneSetEntry *BoneSetFor(edict_t *ent)
	{
		void *model = g_engfuncs.pfnGetModelPtr(ent);
		if (!model)
			return nullptr;

		for (size_t i = 0; i < g_boneSetCount; ++i)
		{
			BoneSetEntry &entry = g_boneSets[i];
			if (entry.modelIndex != ent->v.modelindex)
				continue;
			if (entry.hdr == model)
				return &entry;

			return BuildBoneSet(ent, entry) ? &entry : nullptr;
		}

		size_t slot = g_boneSetCount;
		if (slot >= kMaxBoneSets)
			slot = kMaxBoneSets - 1;

		if (!BuildBoneSet(ent, g_boneSets[slot]))
			return nullptr;

		if (g_boneSetCount < kMaxBoneSets)
			++g_boneSetCount;

		return &g_boneSets[slot];
	}

	Vector SamplePoint(CBot &bot, edict_t *ent, int &boneOut, bool preferHead)
	{
		boneOut = -1;

		if ((ent->v.flags & FL_CLIENT) == 0)
			return BotSense::EntitySoundOrigin(ent);

		const BoneSetEntry *set = BoneSetFor(ent);
		if (!set || set->count == 0)
			return ent->v.origin + ent->v.view_ofs;

		int bone;
		if (preferHead && set->hasHead)
		{
			bone = set->headBone;
		}
		else
		{
			unsigned int pick = static_cast<unsigned int>(RandomUnit(bot.sense) * set->count);
			if (pick >= set->count)
				pick = set->count - 1;
			bone = set->bones[pick];
		}

		Vector origin;
		g_engfuncs.pfnGetBonePosition(ent, bone, origin, nullptr);

		boneOut = bone;
		return origin;
	}

	float RandomBack(BotSense::State &sense)
	{
		sense.backRngState = sense.backRngState * 1664525u + 1013904223u;
		return static_cast<float>((sense.backRngState >> 8) & 0xFFFFFFu) / 16777216.f;
	}

	void SeedBack(CBot &bot)
	{
		BotSense::State &sense = bot.sense;
		if (sense.backRngSeeded)
			return;

		sense.backRngSeeded = true;
		sense.backRngState = static_cast<unsigned int>(
			g_engfuncs.pfnIndexOfEdict(bot.pEdict)) * 2246822519u + 4099u;
	}

	bool BackCheckLook(CBot &bot, const Nav::VantageBlend &blend)
	{
		BotSense::State &sense = bot.sense;

		float lo = 0.f, hi = 0.f;
		if (!BotDiff::BackCheck(lo, hi) || bot.combat.active || Bot_ManeuverActive(bot) ||
			(bot.pEdict->v.flags & FL_ONGROUND) == 0)
			return false;

		SeedBack(bot);

		if (gpGlobals->time >= sense.backNextTime)
		{
			const Nav::VantageWatch *pick = nullptr;
			float acc = 0.f;
			for (int i = 0; i < blend.watchCount; ++i)
			{
				float w = blend.watch[i].amount;
				if (w <= 0.f)
					continue;
				acc += w;
				if (RandomBack(sense) * acc < w)
					pick = &blend.watch[i];
			}

			sense.backNextTime = gpGlobals->time + lo + RandomBack(sense) * (hi - lo);

			if (pick && fabsf(Bot_NormalizeAngle(pick->yaw - bot.desiredAngles.y)) > kVantagePathSnap)
			{
				float pitch = pick->pitch;
				if (pitch > kLookPitchLimit)
					pitch = kLookPitchLimit;
				else if (pitch < -kLookPitchLimit)
					pitch = -kLookPitchLimit;

				sense.backYaw = pick->yaw;
				sense.backPitch = pitch;
				sense.backHoldUntil = gpGlobals->time + kBackCheckHold;
				++sense.backGlances;
			}
		}

		if (gpGlobals->time >= sense.backHoldUntil)
			return false;

		sense.lookSource = BotSense::LOOK_BACK_CHECK;
		bot.desiredAngles = Vector(sense.backPitch, Bot_NormalizeAngle(sense.backYaw), 0.f);
		return true;
	}

	void RefreshVantage(CBot &bot)
	{
		BotSense::State &sense = bot.sense;

		if (!BotSense::VantageEnabled())
		{
			sense.vantage = Nav::VantageBlend();
			return;
		}

		if (gpGlobals->time < sense.nextVantageTime)
			return;

		sense.nextVantageTime = gpGlobals->time + kVantageInterval;

		const Nav::NavMesh *mesh = Nav::Get();
		if (!mesh)
		{
			sense.vantage = Nav::VantageBlend();
			return;
		}

		Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
		Nav::BlendVantage(mesh->vantageField, eye, sense.vantage);
	}

	bool Candidate(const CBot &bot, edict_t *ent)
	{
		if (!ent || ent == bot.pEdict || !CBotIsLivePlayer(ent) || BotSense::NoTarget(ent))
			return false;
		if (g_pGameProfile && g_pGameProfile->IsDead(ent))
			return false;
		if (!Teams::Attackable(bot.pEdict, ent))
			return false;
		return true;
	}

	bool Hunting(const CBot &bot, edict_t *ent)
	{
		if (!ent || ent->free || ent == bot.pEdict || !g_pGameProfile)
			return false;
		if ((ent->v.flags & FL_MONSTER) == 0)
			return false;
		return g_pGameProfile->MonsterEnemy(ent) == bot.pEdict;
	}

	bool MonsterCandidate(const CBot &bot, edict_t *ent)
	{
		if (!ent || ent->free || ent == bot.pEdict || !g_pGameProfile)
			return false;
		if ((ent->v.flags & FL_MONSTER) == 0 || BotSense::NoTarget(ent))
			return false;
		if (g_pGameProfile->IsDead(ent))
			return false;

		edict_t *foe = g_pGameProfile->MonsterEnemy(ent);
		if (!foe)
			return false;

		switch (Teams::Of(bot.pEdict, ent))
		{
		case Teams::REL_ENEMY:
			return true;
		case Teams::REL_FRIEND:
			return false;
		default:
			return foe == bot.pEdict;
		}
	}

	bool Gated(const CBot &bot)
	{
		size_t end = std::min(bot.waypointIndex + kGateLookahead, bot.waypoints.size());
		for (size_t i = bot.waypointIndex; i < end; ++i)
		{
			const Nav::Waypoint &wp = bot.waypoints[i];
			if (wp.requiresLadder || wp.airmoveLaunch || wp.requiresElevatorRide ||
				wp.requiresButtonUse || wp.requiresTriggerTouch)
				return true;
		}
		return false;
	}

	float Weight(const BotSense::Suspicion &entry, float decay)
	{
		float age = gpGlobals->time - entry.time;
		if (age < 0.f)
			age = 0.f;
		if (age >= decay)
			return 0.f;
		return (decay - age) / decay;
	}

	BotSense::SuspicionKind KindOf(const edict_t *ent)
	{
		if ((ent->v.flags & FL_CLIENT) != 0)
			return BotSense::KIND_PLAYER;
		if ((ent->v.flags & FL_MONSTER) != 0)
			return BotSense::KIND_MONSTER;
		return BotSense::KIND_OTHER;
	}

	BotSense::SuspicionKind Tier(const BotSense::Suspicion &entry)
	{
		return entry.hunting ? BotSense::KIND_PLAYER : entry.kind;
	}

	float LiveWeight(const BotSense::Suspicion &entry, float decay)
	{
		edict_t *ent = entry.ent.Get();
		if (!ent || BotSense::NoTarget(ent))
			return 0.f;
		if (entry.kind != BotSense::KIND_OTHER && g_pGameProfile && g_pGameProfile->IsDead(ent))
			return 0.f;
		return Weight(entry, decay);
	}

	void UpdateDamage(CBot &bot)
	{
		edict_t *pEdict = bot.pEdict;
		if (pEdict->v.dmg_take == 0.f)
			return;

		edict_t *inflictor = pEdict->v.dmg_inflictor;
		if (!inflictor || inflictor->free || inflictor == pEdict)
			return;
		if (g_engfuncs.pfnIndexOfEdict(inflictor) == 0)
			return;

		BotSense::Push(bot, BotSense::EntitySoundOrigin(inflictor), BotSense::SUSP_DAMAGE, inflictor);
	}

	bool TraceVisible(CBot &bot, const Vector &eye, edict_t *ent, Vector &pointOut, int &boneOut)
	{
		bool monster = (ent->v.flags & FL_CLIENT) == 0;
		bool head = !monster && bot.sense.headLatched;

		for (int attempt = 0; attempt < 2; ++attempt)
		{
			int bone = -1;
			Vector point;
			{
				BOTPERF_SCOPE(ZONE_SN_BONE);
				point = SamplePoint(bot, ent, bone, head);
			}

			if (monster)
				++bot.sense.monTraces;

			TraceResult tr;
			{
				BOTPERF_SCOPE(ZONE_SN_LOS);
				TRACE_LINE(eye, point, dont_ignore_monsters, bot.pEdict, &tr);
			}
			if (tr.flFraction < 1.f && tr.pHit != ent)
			{
				if (head)
				{
					head = false;
					continue;
				}
				return false;
			}

			if (monster)
				++bot.sense.monHits;

			BotSense::Push(bot, point, BotSense::SUSP_SIGHT, ent);

			pointOut = point;
			boneOut = bone;
			return true;
		}

		return false;
	}

	edict_t *Incumbent(const CBot &bot)
	{
		edict_t *ent = BotCombat::Enemy(bot);
		if (ent)
			return ent;
		return bot.sense.target.Get();
	}

	void UpdateVision(CBot &bot)
	{
		edict_t *pEdict = bot.pEdict;
		Vector eye = pEdict->v.origin + pEdict->v.view_ofs;

		MAKE_VECTORS(pEdict->v.v_angle);
		Vector forward = gpGlobals->v_forward;

		float fov = pEdict->v.fov;
		if (fov <= 0.f)
			fov = kViewCone;
		float minDot = cosf(fov * 0.5f * kPiF / 180.f);

		edict_t *incumbent = Incumbent(bot);
		bool incumbentSeen = false;

		edict_t *pick = nullptr;
		float acc = 0.f;
		int cands = 0;

		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!Candidate(bot, ent))
				continue;

			Vector centre = ent->v.origin + ent->v.view_ofs;
			Vector los = centre - eye;
			float dist = los.Length();
			if (dist > kViewRange || dist < 0.001f)
				continue;

			Vector dir = los * (1.f / dist);
			if (DotProduct(forward, dir) < minDot)
				continue;

			++cands;

			if (ent == incumbent)
			{
				incumbentSeen = true;
				continue;
			}

			Vector theirForward, theirRight, theirUp;
			g_engfuncs.pfnAngleVectors(ent->v.v_angle, theirForward, theirRight, theirUp);

			float gaze = DotProduct(theirForward, dir * -1.f) + 1.f;
			float w = gaze / (dist < kGazeMinDist ? kGazeMinDist : dist);
			if (w <= 0.f)
				continue;

			acc += w;
			if (RandomUnit(bot.sense) * acc < w)
				pick = ent;
		}

		for (size_t m = 0; m < g_monsterCount; ++m)
		{
			edict_t *ent = g_monsters[m];
			if (!ent || ent->free)
				continue;

			++bot.sense.monSeen;

			if (!MonsterCandidate(bot, ent))
				continue;

			++bot.sense.monMine;

			Vector centre = BotSense::EntitySoundOrigin(ent);
			Vector los = centre - eye;
			float dist = los.Length();
			if (dist > kViewRange || dist < 0.001f)
				continue;

			Vector dir = los * (1.f / dist);
			if (DotProduct(forward, dir) < minDot)
				continue;

			++cands;

			if (ent == incumbent)
			{
				incumbentSeen = true;
				continue;
			}

			float w = kMonsterGaze / (dist < kGazeMinDist ? kGazeMinDist : dist);
			if (w <= 0.f)
				continue;

			acc += w;
			if (RandomUnit(bot.sense) * acc < w)
				pick = ent;
		}

		BotSense::State &sense = bot.sense;

		sense.visionCands = cands;
		sense.visionKeep = BotSense::KEEP_NONE;

		edict_t *best = nullptr;
		Vector bestPoint;
		int bestBone = -1;

		if (incumbentSeen && TraceVisible(bot, eye, incumbent, bestPoint, bestBone))
		{
			best = incumbent;
			sense.visionKeep = BotSense::KEEP_INCUMBENT;
		}
		else if (pick && TraceVisible(bot, eye, pick, bestPoint, bestBone))
		{
			best = pick;
			sense.visionKeep = BotSense::KEEP_NEW;
		}

		++sense.keeps[sense.visionKeep];

		if (best && best != sense.target.Get())
		{
			BotAim::ClearDelayQueue(bot);
			BotAim::RollDelay(bot);

			int headPct = BotDiff::HeadPct();
			sense.headLatched = headPct > 0 &&
				BotAim::RandomRoll(bot) * 100.f < static_cast<float>(headPct);
		}

		if (!best && sense.target.Get())
		{
			int predictPct = BotDiff::PredictPct();
			sense.predictLatched = predictPct > 0 &&
				BotAim::RandomRoll(bot) * 100.f < static_cast<float>(predictPct);
		}

		sense.target.Set(best);
		sense.targetBone = bestBone;

		if (!best)
			return;

		sense.targetTime = gpGlobals->time;
		sense.targetPoint = bestPoint;

		sense.enemy.Set(best);
		sense.enemyPos = best->v.origin;
		sense.enemyTime = gpGlobals->time;

		float delay = BotAim::Delay(bot);
		Vector offset = best->v.velocity * delay;
		Vector lead = bestPoint + offset;

		if (offset.LengthSquared() > kLeadEpsilonSq)
		{
			TraceResult leadTrace;
			{
				BOTPERF_SCOPE(ZONE_SN_LOS);
				TRACE_LINE(eye, lead, ignore_monsters, pEdict, &leadTrace);
			}
			if (leadTrace.flFraction < 1.f)
			{
				++sense.leadVetoes;
				return;
			}
		}

		BotAim::PushDelaySlot(bot, lead, best->v.velocity);
	}

	void UpdateEnemy(CBot &bot)
	{
		BotSense::State &sense = bot.sense;

		edict_t *enemy = sense.enemy.Get();
		if (enemy && (g_pGameProfile->IsDead(enemy) || BotSense::NoTarget(enemy) ||
			gpGlobals->time - sense.enemyTime > BotSense::Decay()))
		{
			BotSense::ForgetEnemy(bot);
			enemy = nullptr;
		}
		else if (!enemy && sense.enemyTime != 0.f)
		{
			BotSense::ForgetEnemy(bot);
		}
	}

	bool ApplyVantageLook(CBot &bot)
	{
		float campYaw = 0.f, campPitch = 0.f;
		if (BotCamp::Facing(bot, campYaw, campPitch))
		{
			if (campPitch > kLookPitchLimit)
				campPitch = kLookPitchLimit;
			else if (campPitch < -kLookPitchLimit)
				campPitch = -kLookPitchLimit;

			bot.sense.lookSource = BotSense::LOOK_CAMP;
			bot.desiredAngles = Vector(campPitch, campYaw, 0.f);
			return true;
		}

		const Nav::VantageBlend &blend = bot.sense.vantage;
		if (!blend.valid || blend.watchCount <= 0)
		{
			bot.sense.lookSource = BotSense::LOOK_NO_VANTAGE;
			bot.desiredAngles.x = 0.f;
			return false;
		}

		const Nav::VantageWatch *pick = &blend.watch[0];
		bot.sense.lookSource = BotSense::LOOK_VANTAGE_IDLE;

		if (bot.waypointIndex < bot.waypoints.size())
		{
			bot.sense.lookSource = BotSense::LOOK_VANTAGE_PATH;
			float pathYaw = bot.desiredAngles.y;
			float bestOff = 0.f;
			pick = nullptr;

			for (int i = 0; i < blend.watchCount; ++i)
			{
				float off = fabsf(Bot_NormalizeAngle(blend.watch[i].yaw - pathYaw));
				if (!pick || off < bestOff)
				{
					pick = &blend.watch[i];
					bestOff = off;
				}
			}

			if (bestOff > kVantagePathSnap)
			{
				bot.sense.lookSource = BotSense::LOOK_VANTAGE_OFFPATH;
				bot.desiredAngles.x = 0.f;
				return false;
			}
		}

		float pitch = pick->pitch;
		if (pitch > kLookPitchLimit)
			pitch = kLookPitchLimit;
		else if (pitch < -kLookPitchLimit)
			pitch = -kLookPitchLimit;

		bot.desiredAngles = Vector(pitch, Bot_NormalizeAngle(pick->yaw), 0.f);
		return true;
	}

	bool LookDirection(CBot &bot, Vector &dirOut, float &authorityOut, int &entriesOut,
		BotSense::SuspicionKind &kindOut)
	{
		const BotSense::State &sense = bot.sense;
		float decay = BotSense::Decay();
		Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;

		int tier = BotSense::KIND_COUNT;
		for (size_t i = 0; i < sense.count; ++i)
		{
			const BotSense::Suspicion &entry = sense.slots[i];
			if (LiveWeight(entry, decay) <= 0.f)
				continue;
			if (Tier(entry) < tier)
				tier = Tier(entry);
		}

		if (tier == BotSense::KIND_COUNT)
			return false;

		float bestScore = 0.f;
		float bestWeight = 0.f;
		Vector bestDir(0.f, 0.f, 0.f);
		int live = 0;

		for (size_t i = 0; i < sense.count; ++i)
		{
			const BotSense::Suspicion &entry = sense.slots[i];
			if (Tier(entry) != tier)
				continue;

			float w = LiveWeight(entry, decay);
			if (w <= 0.f)
				continue;

			Vector los = entry.pos - eye;
			float dist = los.Length();
			if (dist < 0.001f)
				continue;

			++live;

			Vector dir = los * (1.f / dist);
			float score = w * BotSense::ConeWeight(sense.vantage, dir);
			if (score <= bestScore)
				continue;

			bestScore = score;
			bestWeight = w;
			bestDir = dir;
		}

		if (live == 0 || bestScore <= 0.f)
			return false;

		authorityOut = bestWeight;
		kindOut = static_cast<BotSense::SuspicionKind>(tier);
		dirOut = bestDir;
		entriesOut = live;
		return true;
	}
}

bool BotSense::Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_sense");
	return cvar && cvar->value != 0.f;
}

bool BotSense::VantageEnabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_vantage");
	return cvar && cvar->value != 0.f;
}

float BotSense::ConeWeight(const Nav::VantageBlend &blend, const Vector &dir)
{
	if (!blend.valid || blend.watchCount <= 0 || blend.watch[0].amount <= 0.f)
		return 1.f;

	float best = 0.f;
	for (int i = 0; i < blend.watchCount; ++i)
	{
		float rel = blend.watch[i].amount / blend.watch[0].amount;
		float d = DotProduct(dir, Nav::VantageDir(blend.watch[i].yaw, blend.watch[i].pitch));
		if (d <= 0.f)
			continue;

		float score = rel * d;
		if (score > best)
			best = score;
	}

	return kVantageConeFloor + (1.f - kVantageConeFloor) * best;
}

float BotSense::Decay()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_sense_decay");
	float decay = cvar ? cvar->value : kDefaultDecay;
	if (decay < 0.1f)
		decay = 0.1f;
	else if (decay > kMaxDecay)
		decay = kMaxDecay;
	return decay;
}

Vector BotSense::EntitySoundOrigin(const edict_t *ent)
{
	return ent->v.origin + (ent->v.mins + ent->v.maxs) * 0.5f;
}

bool BotSense::NoTarget(const edict_t *ent)
{
	if (!ent)
		return false;
	if ((ent->v.flags & FL_NOTARGET) != 0)
		return true;
	if ((ent->v.flags & FL_CLIENT) == 0)
		return false;

	int game = XPlay::OwnerOf(ent);
	if (!XPlay::Supported(game))
		return false;
	XPlay::ScopedGame ctx(game);
	return g_pGameProfile->Disguised(const_cast<edict_t *>(ent));
}

void BotSense::ResetBoneCache()
{
	for (size_t i = 0; i < kMaxBoneSets; ++i)
		g_boneSets[i] = BoneSetEntry();
	g_boneSetCount = 0;

	g_monsterCount = 0;
}

void BotSense::RefreshMonsters()
{
	g_monsterCount = 0;

	for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
	{
		if (g_monsterCount >= kMaxMonsters)
			break;

		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!ent || ent->free || !CBotPrivateData(ent))
			continue;
		if ((ent->v.flags & FL_MONSTER) == 0)
			continue;
		if (ent->v.takedamage == DAMAGE_NO || ent->v.health <= 0.f)
			continue;

		g_monsters[g_monsterCount++] = ent;
	}
}

int BotSense::MonsterCount()
{
	return static_cast<int>(g_monsterCount);
}

const char *BotSense::SourceName(SuspicionSource source)
{
	switch (source)
	{
	case SUSP_SOUND: return "SOUND";
	case SUSP_EVENT: return "EVENT";
	case SUSP_DAMAGE: return "DAMAGE";
	case SUSP_SIGHT: return "SIGHT";
	case SUSP_COUNT: break;
	}
	return "?";
}

const char *BotSense::KindName(SuspicionKind kind)
{
	switch (kind)
	{
	case KIND_PLAYER: return "PLAYER";
	case KIND_MONSTER: return "MONSTER";
	case KIND_OTHER: return "OTHER";
	case KIND_COUNT: break;
	}
	return "?";
}

const char *BotSense::KeepName(VisionKeep keep)
{
	switch (keep)
	{
	case KEEP_NONE: return "none";
	case KEEP_INCUMBENT: return "inc";
	case KEEP_NEW: return "new";
	}
	return "?";
}

void BotSense::Push(CBot &bot, const Vector &pos, SuspicionSource source, edict_t *ent)
{
	if (!ent || ent->free || NoTarget(ent))
		return;

	State &sense = bot.sense;

	if (source < SUSP_COUNT)
		++sense.pushes[source];

	size_t slot = kSuspicionSlots;
	for (size_t i = 0; i < sense.count; ++i)
	{
		if (sense.slots[i].ent.Is(ent))
		{
			slot = i;
			break;
		}
	}

	if (slot == kSuspicionSlots)
	{
		if (sense.count < kSuspicionSlots)
		{
			slot = sense.count++;
		}
		else
		{
			slot = 0;
			for (size_t i = 1; i < kSuspicionSlots; ++i)
			{
				if (sense.slots[i].time < sense.slots[slot].time)
					slot = i;
			}
		}
	}

	Suspicion &entry = sense.slots[slot];
	entry.pos = pos;
	entry.time = gpGlobals->time;
	entry.ent.Set(ent);
	entry.source = source;
	entry.kind = KindOf(ent);
	entry.hunting = entry.kind == KIND_MONSTER && Hunting(bot, ent);
}

void BotSense::Hear(CBot &bot, const edict_t *src, const Vector &pos, float volume,
	float attenuation)
{
	if (attenuation > 0.f)
	{
		float radius = kSoundClipDist * volume / attenuation;
		if ((pos - bot.pEdict->v.origin).Length() > radius)
			return;
	}

	Push(bot, pos, SUSP_SOUND, const_cast<edict_t *>(src));
}

void BotSense::Notice(CBot &bot, const edict_t *invoker)
{
	edict_t *ent = const_cast<edict_t *>(invoker);
	Push(bot, EntitySoundOrigin(ent), SUSP_EVENT, ent);
}

int BotSense::LiveCounts(const CBot &bot, int counts[KIND_COUNT])
{
	const State &sense = bot.sense;
	float decay = Decay();

	for (int i = 0; i < KIND_COUNT; ++i)
		counts[i] = 0;

	int live = 0;
	for (size_t i = 0; i < sense.count; ++i)
	{
		const Suspicion &entry = sense.slots[i];
		if (LiveWeight(entry, decay) <= 0.f)
			continue;

		++counts[entry.kind];
		++live;
	}
	return live;
}

int BotSense::LiveHunting(const CBot &bot)
{
	const State &sense = bot.sense;
	float decay = Decay();

	int hunting = 0;
	for (size_t i = 0; i < sense.count; ++i)
	{
		const Suspicion &entry = sense.slots[i];
		if (!entry.hunting)
			continue;
		if (LiveWeight(entry, decay) <= 0.f)
			continue;
		++hunting;
	}
	return hunting;
}

int BotSense::LiveCount(const CBot &bot)
{
	int counts[KIND_COUNT];
	return LiveCounts(bot, counts);
}

const BotSense::Suspicion *BotSense::Freshest(const CBot &bot)
{
	const State &sense = bot.sense;
	float decay = Decay();

	const Suspicion *best = nullptr;
	for (size_t i = 0; i < sense.count; ++i)
	{
		const Suspicion &entry = sense.slots[i];
		if (LiveWeight(entry, decay) <= 0.f)
			continue;
		if (!best || entry.time > best->time)
			best = &entry;
	}
	return best;
}

const BotSense::Suspicion *BotSense::NearestLive(const CBot &bot, const Vector &point, float radius,
	bool livingOnly, float *distOut)
{
	const State &sense = bot.sense;
	float decay = Decay();
	float bestDist = radius;

	const Suspicion *best = nullptr;
	for (size_t i = 0; i < sense.count; ++i)
	{
		const Suspicion &entry = sense.slots[i];
		if (LiveWeight(entry, decay) <= 0.f)
			continue;
		if (entry.ent.Is(bot.pEdict))
			continue;
		if (livingOnly && entry.kind != KIND_PLAYER && entry.kind != KIND_MONSTER)
			continue;

		edict_t *ent = entry.ent.Get();
		if (ent && ent->v.owner == bot.pEdict)
			continue;
		if (livingOnly && !Teams::Attackable(bot.pEdict, ent))
			continue;

		float dist = (entry.pos - point).Length();
		if (dist > bestDist)
			continue;

		bestDist = dist;
		best = &entry;
	}

	if (best && distOut)
		*distOut = bestDist;
	return best;
}

bool BotSense::Senses(const CBot &bot, const edict_t *ent)
{
	if (!ent)
		return false;

	const State &sense = bot.sense;
	float decay = Decay();

	for (size_t i = 0; i < sense.count; ++i)
	{
		const Suspicion &entry = sense.slots[i];
		if (gpGlobals->time - entry.time > decay)
			continue;
		if (entry.ent.Is(ent))
			return true;
	}

	return false;
}

void BotSense::Update(CBot &bot)
{
	State &sense = bot.sense;

	sense.lookValid = false;
	sense.lookAuthority = 0.f;
	sense.lookEntries = 0;
	sense.lookKind = KIND_OTHER;

	if (!Enabled())
	{
		if (sense.target.Get())
			BotAim::ClearDelayQueue(bot);
		sense.target.Clear();
		sense.targetBone = -1;
		sense.visionCands = 0;
		sense.visionKeep = KEEP_NONE;
		ForgetEnemy(bot);
		return;
	}

	SeedSense(bot);
	{
		BOTPERF_SCOPE(ZONE_SN_VANTAGE);
		RefreshVantage(bot);
	}

	{
		BOTPERF_SCOPE(ZONE_SN_DAMAGE);
		UpdateDamage(bot);
	}
	{
		BOTPERF_SCOPE(ZONE_SN_VISION);
		if (BotDaze::Blind(bot))
		{
			if (sense.target.Get())
				BotAim::ClearDelayQueue(bot);
			sense.target.Clear();
			sense.targetBone = -1;
			sense.visionCands = 0;
			sense.visionKeep = KEEP_NONE;
		}
		else
		{
			UpdateVision(bot);
		}
	}
	{
		BOTPERF_SCOPE(ZONE_SN_ENEMY);
		UpdateEnemy(bot);
	}

	BOTPERF_SCOPE(ZONE_SN_LOOK);
	Vector dir;
	float authority = 0.f;
	int entries = 0;
	SuspicionKind kind = KIND_OTHER;
	if (LookDirection(bot, dir, authority, entries, kind))
	{
		sense.lookValid = true;
		sense.lookDir = dir;
		sense.lookAuthority = authority;
		sense.lookEntries = entries;
		sense.lookKind = kind;
	}
}

void BotSense::ForgetEnemy(CBot &bot)
{
	State &sense = bot.sense;
	sense.enemy.Clear();
	sense.enemyTime = 0.f;
}

void BotSense::OnRespawn(CBot &bot)
{
	State &sense = bot.sense;
	sense.vantage = Nav::VantageBlend();
	sense.nextVantageTime = 0.f;
	sense.backNextTime = 0.f;
	sense.backHoldUntil = 0.f;
	ForgetEnemy(bot);
}

bool BotSense::ApplyLook(CBot &bot)
{
	State &sense = bot.sense;
	bot.aim.servedDelay = 0.f;

	if (!Enabled())
	{
		sense.lookSource = LOOK_DISABLED;
		return false;
	}

	if (Gated(bot))
	{
		sense.lookSource = LOOK_GATED;
		return false;
	}

	float limit = BotAim::PitchLimit();
	Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;

	float delay = BotAim::Delay(bot);
	Vector point;
	float yaw = 0.f, pitch = 0.f;
	bool have = BotFire::AimPoint(bot, point) || BotAim::ReadPredicted(bot, delay, point);
	if (have && BotAim::LosAngles(eye, point, yaw, pitch))
	{
		if (pitch > limit)
			pitch = limit;
		else if (pitch < -limit)
			pitch = -limit;

		bot.aim.servedDelay = delay;
		bot.desiredAngles = Vector(pitch, yaw, 0.f);
		sense.lookSource = bot.aim.predictedNow ? LOOK_PREDICT : LOOK_AIM;
		return true;
	}

	float gaussYaw = 0.f, gaussPitch = 0.f;
	if (BotGauss::Facing(bot, gaussYaw, gaussPitch))
	{
		sense.lookSource = LOOK_GAUSS;
		bot.desiredAngles = Vector(gaussPitch, Bot_NormalizeAngle(gaussYaw), 0.f);
		return true;
	}

	if (sense.vantage.valid && sense.vantage.watchCount > 0 &&
		BackCheckLook(bot, sense.vantage))
		return true;

	if (!sense.lookValid || sense.lookAuthority <= 0.f)
		return ApplyVantageLook(bot);

	float wantYaw = 0.f, wantPitch = 0.f;
	if (!BotAim::LosAngles(eye, eye + sense.lookDir, wantYaw, wantPitch))
		return false;

	if (wantPitch > kLookPitchLimit)
		wantPitch = kLookPitchLimit;
	else if (wantPitch < -kLookPitchLimit)
		wantPitch = -kLookPitchLimit;

	const Vector &base = bot.desiredAngles;
	float authority = sense.lookAuthority;

	float blendYaw = Bot_NormalizeAngle(base.y + Bot_NormalizeAngle(wantYaw - base.y) * authority);
	float blendPitch = base.x + (wantPitch - base.x) * authority;

	if (blendPitch > limit)
		blendPitch = limit;
	else if (blendPitch < -limit)
		blendPitch = -limit;

	bot.desiredAngles = Vector(blendPitch, blendYaw, 0.f);
	sense.lookSource = LOOK_SENSE;
	return true;
}

const char *BotSense::LookSourceName(LookSource source)
{
	switch (source)
	{
	case LOOK_NONE: return "none";
	case LOOK_DISABLED: return "disabled";
	case LOOK_GATED: return "gated";
	case LOOK_AIM: return "aim";
	case LOOK_SENSE: return "sense";
	case LOOK_CAMP: return "camp";
	case LOOK_BACK_CHECK: return "back-check";
	case LOOK_PREDICT: return "predict";
	case LOOK_GAUSS: return "gauss";
	case LOOK_VANTAGE_IDLE: return "vantage-idle";
	case LOOK_VANTAGE_PATH: return "vantage-path";
	case LOOK_VANTAGE_OFFPATH: return "vantage-offpath";
	case LOOK_NO_VANTAGE: return "no-vantage";
	}
	return "?";
}
