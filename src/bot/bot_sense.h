#pragma once

#include <extdll.h>

#include <cstddef>

#include "../game/ent_handle.h"
#include "../nav/nav_vantage.h"

struct CBot;

namespace BotSense
{

constexpr size_t kSuspicionSlots = 256;
constexpr size_t kNoticeSlots = 8;

enum SuspicionSource : unsigned char
{
	SUSP_SOUND = 0,
	SUSP_EVENT,
	SUSP_DAMAGE,
	SUSP_SIGHT,
	SUSP_ALLY,
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
	LOOK_BACK_CHECK,
	LOOK_PREDICT,
	LOOK_GAUSS,
	LOOK_VANTAGE_IDLE,
	LOOK_VANTAGE_PATH,
	LOOK_VANTAGE_OFFPATH,
	LOOK_NO_VANTAGE,
	LOOK_ALERT,
};

struct Suspicion
{
	Vector pos;
	float time = 0.f;
	float sourceTime[SUSP_COUNT] = {};
	CBotEntHandle ent;
	SuspicionSource source = SUSP_SOUND;
	SuspicionKind kind = KIND_OTHER;
	bool hunting = false;
};

struct NoticeSlot
{
	CBotEntHandle ent;
	float progress = 0.f;
	float need = 0.f;
};

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

	int visionCands = 0;
	VisionKeep visionKeep = KEEP_NONE;
	unsigned int keeps[3] = {};

	NoticeSlot notices[kNoticeSlots];

	float lightTarget = 0.f;
	float lightBack = 0.f;
	float lightVis = 1.f;
	unsigned int darkDrops = 0;

	float clarity = 1.f;
	unsigned int covTraces = 0;
	unsigned int covPartial = 0;
	unsigned int covBlocked = 0;

	unsigned int monSeen = 0;
	unsigned int monMine = 0;
	unsigned int monTraces = 0;
	unsigned int monHits = 0;

	unsigned int cloakRolls = 0;
	unsigned int cloakSkips = 0;

	CBotEntHandle enemy;
	Vector enemyPos;
	float enemyTime = 0.f;

	unsigned int rngState = 0;
	bool rngSeeded = false;

	bool headLatched = false;
	bool predictLatched = false;

	float lookAuthority = 0.f;
	Vector lookDir;
	int lookEntries = 0;
	SuspicionKind lookKind = KIND_OTHER;
	bool lookValid = false;

	Nav::VantageBlend vantage;
	float nextVantageTime = 0.f;

	float backNextTime = 0.f;
	float backHoldUntil = 0.f;
	float backYaw = 0.f;
	float backPitch = 0.f;
	unsigned int backGlances = 0;
	unsigned int backRngState = 0;
	bool backRngSeeded = false;

	LookSource lookSource = LOOK_NONE;
};

bool VantageEnabled();

float ConeWeight(const Nav::VantageBlend &blend, const Vector &dir);

bool Enabled();

float Decay();

Vector EntitySoundOrigin(const edict_t *ent);

bool NoTarget(const edict_t *ent);

void Push(CBot &bot, const Vector &pos, SuspicionSource source, edict_t *ent);

void Hear(CBot &bot, const edict_t *src, const Vector &pos, float volume, float attenuation);

void Notice(CBot &bot, const edict_t *invoker);

void ResetBoneCache();

void LoadLight(const char *bspPath);

void ForgetLight();

void LightStyle(int style, const char *pattern);

void RefreshMonsters();

void RefreshAllies();

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

bool HeardEnemy(const CBot &bot, Vector &pos, Vector &vel, edict_t **entOut);

const NoticeSlot *TopNotice(const CBot &bot, int &countOut);

const char *SourceName(SuspicionSource source);

const char *KindName(SuspicionKind kind);

const char *KeepName(VisionKeep keep);

const char *LookSourceName(LookSource source);

}
