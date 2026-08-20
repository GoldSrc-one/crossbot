#include <extdll.h>
#include <meta_api.h>
#include <studio.h>

#include <algorithm>

#include "bot_sense.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_move.h"
#include "../game/game_profile.h"
#include "../nav/nav_math.h"

namespace
{
	using NavMath::kPiF;

	constexpr float kDefaultDecay = 5.f;
	constexpr float kMaxDecay = 60.f;

	constexpr float kViewCone = 90.f;
	constexpr float kViewRange = 4096.f;

	constexpr float kLookPitchLimit = 45.f;

	constexpr float kSoundClipDist = 1000.f;

	constexpr size_t kGateLookahead = 3;

	constexpr size_t kMaxBoneSets = 64;
	constexpr size_t kMaxSampleBones = 32;

	constexpr float kDegenerateMean = 0.05f;

	constexpr float kLeadEpsilonSq = 1.f;

	constexpr int kRefinePasses = 2;

	struct BoneSetEntry
	{
		int modelIndex = 0;
		const void *hdr = nullptr;
		unsigned char bones[kMaxSampleBones] = {};
		unsigned char count = 0;
	};

	BoneSetEntry g_boneSets[kMaxBoneSets];
	size_t g_boneSetCount = 0;

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

		for (int i = 0; i < hdr->numhitboxes && entry.count < kMaxSampleBones; ++i)
		{
			int bone = boxes[i].bone;
			if (bone < 0 || bone >= hdr->numbones)
				continue;

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

	Vector SamplePoint(CBot &bot, edict_t *ent, int &boneOut)
	{
		boneOut = -1;

		const BoneSetEntry *set = BoneSetFor(ent);
		if (!set || set->count == 0)
			return ent->v.origin + ent->v.view_ofs;

		unsigned int pick = static_cast<unsigned int>(RandomUnit(bot.sense) * set->count);
		if (pick >= set->count)
			pick = set->count - 1;

		int bone = set->bones[pick];

		Vector origin;
		g_engfuncs.pfnGetBonePosition(ent, bone, origin, nullptr);

		boneOut = bone;
		return origin;
	}

	bool Candidate(const CBot &bot, edict_t *ent)
	{
		if (!ent || ent == bot.pEdict || !CBotIsLivePlayer(ent))
			return false;
		if (g_pGameProfile && g_pGameProfile->IsDead(ent))
			return false;
		return true;
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

	size_t SlotAt(const BotSense::State &sense, size_t back)
	{
		return (sense.head + BotSense::kSuspicionSlots - 1 - back) % BotSense::kSuspicionSlots;
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

		BotSense::Push(bot, BotSense::EntitySoundOrigin(inflictor), BotSense::SUSP_DAMAGE,
			g_engfuncs.pfnIndexOfEdict(inflictor));
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

		edict_t *best = nullptr;
		Vector bestPoint;
		int bestBone = -1;
		float bestDist = 0.f;

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

			if (DotProduct(forward, los * (1.f / dist)) < minDot)
				continue;

			int bone = -1;
			Vector point = SamplePoint(bot, ent, bone);

			TraceResult tr;
			TRACE_LINE(eye, point, dont_ignore_monsters, pEdict, &tr);
			if (tr.flFraction < 1.f && tr.pHit != ent)
				continue;

			BotSense::Push(bot, point, BotSense::SUSP_SIGHT, i);

			if (best && dist >= bestDist)
				continue;

			best = ent;
			bestPoint = point;
			bestBone = bone;
			bestDist = dist;
		}

		BotSense::State &sense = bot.sense;

		if (best && best != sense.target.Get())
			BotAim::ClearDelayQueue(bot);

		sense.target.Set(best);
		sense.targetBone = bestBone;

		if (!best)
			return;

		sense.targetTime = gpGlobals->time;
		sense.targetPoint = bestPoint;

		float delay = BotAim::Delay();
		Vector offset = best->v.velocity * delay;
		Vector lead = bestPoint + offset;

		if (offset.LengthSquared() > kLeadEpsilonSq)
		{
			TraceResult leadTrace;
			TRACE_LINE(eye, lead, ignore_monsters, pEdict, &leadTrace);
			if (leadTrace.flFraction < 1.f)
			{
				++sense.leadVetoes;
				return;
			}
		}

		BotAim::PushDelaySlot(bot, lead);
	}

	bool LookDirection(CBot &bot, Vector &dirOut, float &authorityOut, int &entriesOut)
	{
		const BotSense::State &sense = bot.sense;
		float decay = BotSense::Decay();
		Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;

		Vector sum(0.f, 0.f, 0.f);
		float totalWeight = 0.f;
		float bestWeight = 0.f;
		Vector bestDir(0.f, 0.f, 0.f);
		int live = 0;

		for (size_t back = 0; back < sense.count; ++back)
		{
			const BotSense::Suspicion &entry = sense.slots[SlotAt(sense, back)];
			float w = Weight(entry, decay);
			if (w <= 0.f)
				break;

			Vector los = entry.pos - eye;
			float dist = los.Length();
			if (dist < 0.001f)
				continue;

			Vector dir = los * (1.f / dist);
			sum = sum + dir * w;
			totalWeight += w;
			++live;

			if (w > bestWeight)
			{
				bestWeight = w;
				bestDir = dir;
			}
		}

		if (live == 0 || totalWeight <= 0.f)
			return false;

		authorityOut = bestWeight;

		float mean = sum.Length();
		if (mean < kDegenerateMean * totalWeight)
		{
			dirOut = bestDir;
			entriesOut = 1;
			return true;
		}

		Vector centre = sum * (1.f / mean);

		float fov = bot.pEdict->v.fov;
		if (fov <= 0.f)
			fov = kViewCone;
		float minDot = cosf(fov * 0.5f * kPiF / 180.f);

		int kept = live;
		for (int pass = 0; pass < kRefinePasses; ++pass)
		{
			Vector refined(0.f, 0.f, 0.f);
			int inCone = 0;

			for (size_t back = 0; back < sense.count; ++back)
			{
				const BotSense::Suspicion &entry = sense.slots[SlotAt(sense, back)];
				float w = Weight(entry, decay);
				if (w <= 0.f)
					break;

				Vector los = entry.pos - eye;
				float dist = los.Length();
				if (dist < 0.001f)
					continue;

				Vector dir = los * (1.f / dist);
				if (DotProduct(dir, centre) < minDot)
					continue;

				refined = refined + dir * w;
				++inCone;
			}

			float len = refined.Length();
			if (inCone == 0 || len < 0.001f)
				break;

			centre = refined * (1.f / len);
			kept = inCone;
		}

		dirOut = centre;
		entriesOut = kept;
		return true;
	}
}

bool BotSense::Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_sense");
	return cvar && cvar->value != 0.f;
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

void BotSense::ResetBoneCache()
{
	for (size_t i = 0; i < kMaxBoneSets; ++i)
		g_boneSets[i] = BoneSetEntry();
	g_boneSetCount = 0;
}

const char *BotSense::SourceName(SuspicionSource source)
{
	switch (source)
	{
	case SUSP_SOUND: return "SOUND";
	case SUSP_EVENT: return "EVENT";
	case SUSP_DAMAGE: return "DAMAGE";
	case SUSP_SIGHT: return "SIGHT";
	}
	return "?";
}

void BotSense::Push(CBot &bot, const Vector &pos, SuspicionSource source, int entIndex)
{
	State &sense = bot.sense;

	if (source == SUSP_SIGHT && entIndex > 0)
	{
		float decay = Decay();
		for (size_t back = 0; back < sense.count; ++back)
		{
			Suspicion &entry = sense.slots[SlotAt(sense, back)];
			if (Weight(entry, decay) <= 0.f)
				break;
			if (entry.source != SUSP_SIGHT || entry.entIndex != entIndex)
				continue;

			entry.pos = pos;
			entry.time = gpGlobals->time;
			return;
		}
	}

	Suspicion &slot = sense.slots[sense.head];
	slot.pos = pos;
	slot.time = gpGlobals->time;
	slot.entIndex = entIndex;
	slot.source = source;

	sense.head = (sense.head + 1) % kSuspicionSlots;
	if (sense.count < kSuspicionSlots)
		++sense.count;
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

	Push(bot, pos, SUSP_SOUND, src ? g_engfuncs.pfnIndexOfEdict(const_cast<edict_t *>(src)) : 0);
}

void BotSense::Notice(CBot &bot, const edict_t *invoker)
{
	edict_t *ent = const_cast<edict_t *>(invoker);
	Push(bot, EntitySoundOrigin(ent), SUSP_EVENT, g_engfuncs.pfnIndexOfEdict(ent));
}

int BotSense::LiveCount(const CBot &bot)
{
	const State &sense = bot.sense;
	float decay = Decay();

	int live = 0;
	for (size_t back = 0; back < sense.count; ++back)
	{
		if (Weight(sense.slots[SlotAt(sense, back)], decay) <= 0.f)
			break;
		++live;
	}
	return live;
}

const BotSense::Suspicion *BotSense::Freshest(const CBot &bot)
{
	const State &sense = bot.sense;
	if (sense.count == 0)
		return nullptr;

	const Suspicion &entry = sense.slots[SlotAt(sense, 0)];
	if (Weight(entry, Decay()) <= 0.f)
		return nullptr;
	return &entry;
}

void BotSense::Update(CBot &bot)
{
	State &sense = bot.sense;

	sense.lookValid = false;
	sense.lookAuthority = 0.f;
	sense.lookEntries = 0;

	if (!Enabled())
	{
		if (sense.target.Get())
			BotAim::ClearDelayQueue(bot);
		sense.target.Clear();
		sense.targetBone = -1;
		return;
	}

	SeedSense(bot);

	UpdateDamage(bot);
	UpdateVision(bot);

	Vector dir;
	float authority = 0.f;
	int entries = 0;
	if (LookDirection(bot, dir, authority, entries))
	{
		sense.lookValid = true;
		sense.lookDir = dir;
		sense.lookAuthority = authority;
		sense.lookEntries = entries;
	}
}

bool BotSense::ApplyLook(CBot &bot)
{
	State &sense = bot.sense;
	bot.aim.servedDelay = 0.f;

	if (!Enabled() || Gated(bot))
		return false;

	float limit = BotAim::PitchLimit();
	Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;

	float delay = BotAim::Delay();
	Vector point;
	float yaw = 0.f, pitch = 0.f;
	if (BotAim::ReadDelayed(bot, delay, point) && BotAim::LosAngles(eye, point, yaw, pitch))
	{
		if (pitch > limit)
			pitch = limit;
		else if (pitch < -limit)
			pitch = -limit;

		bot.aim.servedDelay = delay;
		bot.desiredAngles = Vector(pitch, yaw, 0.f);
		return true;
	}

	if (!sense.lookValid || sense.lookAuthority <= 0.f)
	{
		bot.desiredAngles.x = 0.f;
		return false;
	}

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
	return true;
}
