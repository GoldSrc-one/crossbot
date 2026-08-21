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

enum Commit
{
	COMMIT_NONE = 0,
	COMMIT_CHARGE,
	COMMIT_THROW,
	COMMIT_GUIDE,
};

struct State
{
	Mode mode = MODE_NONE;
	int weaponId = 0;
	bool wantReload = false;

	Vector targetSize;
	bool haveSize = false;

	Vector aimPoint;
	Vector targetPoint;
	float flightTime = 0.f;
	bool aimValid = false;
	bool aimLead = false;
	bool aimHoming = false;
	bool aimArc = false;

	bool zoomed = false;

	Commit commit = COMMIT_NONE;
	float commitStart = 0.f;
	bool commitSeen = false;

	int pendingId = 0;
	float nextSelectTime = 0.f;

	unsigned int rngState = 0;
	bool rngSeeded = false;

	float lastMiss = -1.f;
	float lastTol = -1.f;
	float lastPrimaryTimer = 0.f;
	float lastSecondaryTimer = 0.f;
	int lastTimerId = 0;
	int lastChargeState = 0;
	float lastThrowStart = 0.f;
	bool pressed = false;
	bool pressedLead = false;
	bool pressedArc = false;
	const char *lastReason = "-";
	int shots = 0;
	int leadShots = 0;
	int chargeShots = 0;
	int arcShots = 0;
	int throwShots = 0;
	int guides = 0;
	int commitCaps = 0;
	int uphillVetoes = 0;
	int selfVetoes = 0;
	int switches = 0;
	int reloads = 0;
};

bool Enabled();

void Pin(const char *classname, int mode);

const char *Pinned();

const char *PinnedMode();

bool Supported(const WeaponMode &mode);

const WeaponMode *ModeOf(const WeaponSpec &spec, Mode mode, bool zoomed);

bool ModeUsable(CBot &bot, const BotWeapon &w, const WeaponMode &m, float dist, float rise);

void Reset(CBot &bot);

void Think(CBot &bot);

bool AimPoint(const CBot &bot, Vector &point);

unsigned short Buttons(CBot &bot, const Vector &angles);

const char *ModeName(Mode mode);

const char *CommitName(Commit commit);

}
