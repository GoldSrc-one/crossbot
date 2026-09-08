#pragma once

#include <extdll.h>

#include <cstddef>

#include "../game/ent_handle.h"
#include "../nav/nav_vantage.h"

struct CBot;

namespace BotSense
{

constexpr size_t kSuspicionSlots = 256;

enum SuspicionSource : unsigned char
{
	SUSP_SOUND = 0,
	SUSP_EVENT,
	SUSP_DAMAGE,
	SUSP_SIGHT,
	SUSP_COUNT,
};

enum SuspicionKind : unsigned char
{
	KIND_PLAYER = 0,
	KIND_MONSTER,
	KIND_OTHER,
	KIND_COUNT,
};

enum LookSource : unsigned char
{
	LOOK_NONE = 0,
	LOOK_DISABLED,
	LOOK_GATED,
	LOOK_AIM,
	LOOK_SENSE,
	LOOK_CAMP,
	LOOK_VANTAGE_IDLE,
	LOOK_VANTAGE_PATH,
	LOOK_VANTAGE_OFFPATH,
	LOOK_NO_VANTAGE,
};

struct Suspicion
{
	Vector pos;
	float time = 0.f;
	CBotEntHandle ent;
	SuspicionSource source = SUSP_SOUND;
	SuspicionKind kind = KIND_OTHER;
	bool hunting = false;
};

// Which trace won this frame. KEEP_INCUMBENT is the enemy the bot was already fighting; KEEP_NEW
// is a fresh weighted-random pick. During a duel this should read KEEP_INCUMBENT nearly every
// frame -- a run of KEEP_NEW means the lock is broken and the bot is re-rolling its enemy.
enum VisionKeep : unsigned char
{
	KEEP_NONE = 0,
	KEEP_INCUMBENT,
	KEEP_NEW,
};

struct State
{
	Suspicion slots[kSuspicionSlots];
	size_t count = 0;

	unsigned int pushes[SUSP_COUNT] = {};

	CBotEntHandle target;
	float targetTime = 0.f;
	Vector targetPoint;
	int targetBone = -1;
	int leadVetoes = 0;

	// Diagnostics for the two-trace vision rule. visionCands is how many players passed range
	// and viewcone -- i.e. how many tracelines the old every-candidate scan would have spent --
	// and visionKeep says which of the (at most two) traces produced the target.
	int visionCands = 0;
	VisionKeep visionKeep = KEEP_NONE;
	// Cumulative per-outcome tally. visionKeep alone is one frame, and a test that samples it every
	// few seconds reads none almost always -- a bot holds a target on well under half of frames, so
	// sampling proves nothing about whether the incumbent lock works. These cannot miss.
	unsigned int keeps[3] = {};

	unsigned int monSeen = 0;
	unsigned int monMine = 0;
	unsigned int monTraces = 0;
	unsigned int monHits = 0;

	CBotEntHandle enemy;
	Vector enemyPos;
	float enemyTime = 0.f;

	unsigned int rngState = 0;
	bool rngSeeded = false;

	float lookAuthority = 0.f;
	Vector lookDir;
	int lookEntries = 0;
	SuspicionKind lookKind = KIND_OTHER;
	bool lookValid = false;

	Nav::VantageBlend vantage;
	float nextVantageTime = 0.f;
	LookSource lookSource = LOOK_NONE;
};

bool VantageEnabled();

float ConeWeight(const Nav::VantageBlend &blend, const Vector &dir);

bool Enabled();

float Decay();

Vector EntitySoundOrigin(const edict_t *ent);

void Push(CBot &bot, const Vector &pos, SuspicionSource source, edict_t *ent);

void Hear(CBot &bot, const edict_t *src, const Vector &pos, float volume, float attenuation);

void Notice(CBot &bot, const edict_t *invoker);

void ResetBoneCache();

void RefreshMonsters();

int MonsterCount();

void Update(CBot &bot);

void ForgetEnemy(CBot &bot);

void OnRespawn(CBot &bot);

bool ApplyLook(CBot &bot);

int LiveCount(const CBot &bot);

int LiveCounts(const CBot &bot, int counts[KIND_COUNT]);

int LiveHunting(const CBot &bot);

const Suspicion *Freshest(const CBot &bot);

bool Senses(const CBot &bot, const edict_t *ent);

const Suspicion *NearestLive(const CBot &bot, const Vector &point, float radius, bool livingOnly,
	float *distOut = nullptr);

const char *SourceName(SuspicionSource source);

const char *KindName(SuspicionKind kind);

const char *KeepName(VisionKeep keep);

const char *LookSourceName(LookSource source);

}
