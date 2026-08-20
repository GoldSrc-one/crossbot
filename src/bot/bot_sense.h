#pragma once

#include <extdll.h>

#include <cstddef>

#include "../game/ent_handle.h"

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
};

struct Suspicion
{
	Vector pos;
	float time = 0.f;
	int entIndex = 0;
	SuspicionSource source = SUSP_SOUND;
};

struct State
{
	Suspicion slots[kSuspicionSlots];
	size_t head = 0;
	size_t count = 0;

	CBotEntHandle target;
	float targetTime = 0.f;
	Vector targetPoint;
	int targetBone = -1;
	int leadVetoes = 0;

	unsigned int rngState = 0;
	bool rngSeeded = false;

	float lookAuthority = 0.f;
	Vector lookDir;
	int lookEntries = 0;
	bool lookValid = false;
};

bool Enabled();

float Decay();

Vector EntitySoundOrigin(const edict_t *ent);

void Push(CBot &bot, const Vector &pos, SuspicionSource source, int entIndex);

void Hear(CBot &bot, const edict_t *src, const Vector &pos, float volume, float attenuation);

void Notice(CBot &bot, const edict_t *invoker);

void ResetBoneCache();

void Update(CBot &bot);

bool ApplyLook(CBot &bot);

int LiveCount(const CBot &bot);

const Suspicion *Freshest(const CBot &bot);

const char *SourceName(SuspicionSource source);

}
