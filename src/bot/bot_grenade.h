#pragma once

#include <extdll.h>

#include "../game/game_profile.h"

struct CBot;

namespace BotGrenade
{

struct State
{
	int slot = -1;
	bool scared = false;
	float primeTime = 0.f;
	float releaseTime = 0.f;
	float pin = 0.f;
	float fuse = 0.f;
	float minRange = 0.f;
	WeaponMode mode = {};
	Vector look;
	bool haveLook = false;
	float flight = 0.f;
	float nextTry = 0.f;
	const char *why = "-";
	int types[kMaxOffhandSlots] = {};
	int counts[kMaxOffhandSlots] = {};
	unsigned int sightings = 0;
	unsigned int rangeRejects = 0;
	float lastDist = 0.f;
	float minDist = 0.f;

	unsigned int primes = 0;
	unsigned int throws = 0;
	unsigned int forced = 0;
	unsigned int refused = 0;
	unsigned int scaredThrows = 0;
};

bool Enabled();

void Run(CBot &bot, Vector &angles);

void Reset(CBot &bot);

void PrintStatus(const CBot &bot);

}
