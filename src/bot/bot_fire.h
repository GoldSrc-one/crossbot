#pragma once

#include <extdll.h>

#include "../game/weapon_spec.h"

struct CBot;
struct BotWeapon;
struct BotInventory;

namespace BotFire
{

enum Mode
{
	MODE_NONE = 0,
	MODE_PRIMARY,
	MODE_SECONDARY,
};

constexpr int kMaxWeaponIds = 32;

struct Carry
{
	float seconds[kMaxWeaponIds] = {};
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
	float lastRange = -1.f;
	bool aimValid = false;
	bool aimLead = false;
	bool aimHoming = false;
	bool aimArc = false;

	bool zoomed = false;

	Commit commit = COMMIT_NONE;
	float commitStart = 0.f;
	bool commitSeen = false;
	bool releaseCharge = false;

	int pendingId = 0;
	Mode pendingMode = MODE_NONE;
	int idleReloadId = 0;
	float nextSelectTime = 0.f;
	float nextIdleTime = 0.f;
	float nextEquipTime = 0.f;

	unsigned int rngState = 0;
	bool rngSeeded = false;

	float lastMiss = -1.f;
	float lastTol = -1.f;
	float lastPrimaryTimer = 0.f;
	float lastSecondaryTimer = 0.f;
	int lastTimerId = 0;
	int lastChargeState = 0;
	float lastThrowStart = 0.f;
	// When a shot was last corroborated by the weapon timer moving. pressed is a one-frame flag
	// that CountShots consumes on the following frame, so it cannot answer "did we just swing?"
	// from outside this file; this can.
	float lastShotTime = 0.f;
	bool pressed = false;
	bool pressedLead = false;
	bool pressedArc = false;
	bool pressedSticky = false;
	const char *lastReason = "-";
	int shots = 0;
	int leadShots = 0;
	int chargeShots = 0;
	int arcShots = 0;
	int throwShots = 0;
	int guides = 0;
	int stickyShots = 0;
	int commitCaps = 0;
	int uphillVetoes = 0;
	int selfVetoes = 0;
	// Shots withheld because the ray missed the target box. Cumulative, because the reason string
	// is a single frame sampled every two seconds and misses most of what it describes -- the same
	// reason selfVetoes exists alongside the "splash-self" reason.
	int aimVetoes = 0;
	// Frames that published an AIM_TARGET aim. For a pinned crossbow that is exactly "primaryZoomed
	// was selected", which the aim= field in the status line can only witness by luck.
	int targetAims = 0;
	int switches = 0;
	int reloads = 0;
	int emptySwitches = 0;
	int idleSwitches = 0;
	int ambushes = 0;
	int duels = 0;
	int idleReloads = 0;
	int gaussJumps = 0;
};

bool Enabled();

void Pin(const char *classname, int mode, int zoom);

const char *Pinned();

const char *PinnedMode();

const char *PinnedZoom();

int PinnedZoomState();

bool Supported(const WeaponMode &mode);

const WeaponMode *ModeOf(const WeaponSpec &spec, Mode mode, bool zoomed);

bool WeaponArmed(const BotInventory &inv, const BotWeapon &w);

bool ModeViable(CBot &bot, const BotWeapon &w, const WeaponMode &m, bool strict);

bool ModeUsable(CBot &bot, const BotWeapon &w, const WeaponMode &m, float dist, float rise,
	bool strict);

float CarryOf(const CBot &bot, int weaponId);

void AccrueCarry(CBot &bot);

void Reset(CBot &bot);

void Think(CBot &bot);

bool AimPoint(const CBot &bot, Vector &point);

unsigned short Buttons(CBot &bot, const Vector &angles);

const char *ModeName(Mode mode);

const char *CommitName(Commit commit);

}
