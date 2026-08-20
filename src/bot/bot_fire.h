#pragma once

#include <extdll.h>

#include "../game/weapon_spec.h"

struct CBot;
struct BotWeapon;

namespace BotFire
{

enum Mode
{
	MODE_NONE = 0,
	MODE_PRIMARY,
	MODE_SECONDARY,
};

struct State
{
	Mode mode = MODE_NONE;
	int weaponId = 0;
	bool wantReload = false;

	Vector targetSize;
	bool haveSize = false;

	Vector basePoint;
	Vector aimPoint;
	float flightTime = 0.f;
	bool aimValid = false;
	bool aimLead = false;
	bool aimHoming = false;

	int pendingId = 0;
	float nextSelectTime = 0.f;

	unsigned int rngState = 0;
	bool rngSeeded = false;

	float lastMiss = -1.f;
	float lastTol = -1.f;
	float lastPrimaryTimer = 0.f;
	float lastSecondaryTimer = 0.f;
	int lastTimerId = 0;
	bool pressed = false;
	bool pressedLead = false;
	const char *lastReason = "-";
	int shots = 0;
	int leadShots = 0;
	int switches = 0;
	int reloads = 0;
};

bool Enabled();

void Pin(const char *classname);

const char *Pinned();

bool Supported(const WeaponMode &mode);

const WeaponMode *ModeOf(const WeaponSpec &spec, Mode mode);

bool ModeUsable(const CBot &bot, const BotWeapon &w, const WeaponMode &m, float dist);

void Reset(CBot &bot);

void Think(CBot &bot);

bool AimPoint(const CBot &bot, Vector &point);

unsigned short Buttons(CBot &bot, const Vector &angles);

const char *ModeName(Mode mode);

}
