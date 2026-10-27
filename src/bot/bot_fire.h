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
	COMMIT_SPINUP,
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
	bool cycling = false;
	bool cycleEnd = false;

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
	float lastShotTime = 0.f;
	bool pressed = false;
	bool pressedLead = false;
	bool pressedArc = false;
	bool pressedSticky = false;
	bool pressedWall = false;
	bool rayWall = false;
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
	int aimVetoes = 0;
	int spreadWaits = 0;
	int braceWaits = 0;
	bool braceHolding = false;
	float spreadUntil = 0.f;
	int spreadClip = -1;
	int targetAims = 0;
	int switches = 0;
	int reloads = 0;
	int emptySwitches = 0;
	int idleSwitches = 0;
	int ambushes = 0;
	int duels = 0;
	bool heard = false;
	bool friendCloser = false;
	Vector heardPoint;
	Vector heardVel;
	float nextHeardTime = 0.f;
	int heardThrows = 0;
	int harmVetoes = 0;
	int throwId = 0;
	Vector throwPoint;
	Vector throwVel;
	bool throwReady = false;
	bool throwHeard = false;
	int throwDrops = 0;
	bool throwBan = false;
	float throwBanCombat = 0.f;
	int idleReloads = 0;
	int gaussJumps = 0;
	int wallbangs = 0;
	int spinups = 0;
	int lasers = 0;
	int braces = 0;
	float commitFor = 0.f;
	int orderId = 0;
	Vector orderPoint;
	float orderUntil = 0.f;
	int bracing = 0;
	bool braceDuckClear = false;
	float braceTraceTime = 0.f;
};

bool Enabled();

void Pin(const char *classname, int mode, int zoom);

const char *Pinned();

const char *PinnedMode();

const char *PinnedZoom();

int PinnedZoomState();

bool Supported(const WeaponMode &mode);

const WeaponMode *ModeOf(const WeaponSpec &spec, Mode mode, bool zoomed);

bool SolveOffhand(const WeaponMode &mode, const Vector &src, const Vector &base, const Vector &vel,
	Vector &look, float &flight, edict_t *thrower = nullptr, Vector *impact = nullptr);

bool FriendCloser(edict_t *self, const Vector &point);

bool HarmFavors(edict_t *self, const Vector &impact, const Vector &enemy, float radius);

bool WeaponArmed(const BotInventory &inv, const BotWeapon &w);

bool ModeViable(CBot &bot, const BotWeapon &w, const WeaponMode &m, bool strict);

bool ModeUsable(CBot &bot, const BotWeapon &w, const WeaponMode &m, float dist, float rise,
	bool strict);

float CarryOf(const CBot &bot, int weaponId);

int SnipeWeapon(CBot &bot);

bool CampWeapon(CBot &bot);

void AccrueCarry(CBot &bot);

void Reset(CBot &bot);

void Think(CBot &bot);

bool Order(CBot &bot, int weaponId, const Vector &point, float until, const char *&why);

void CancelOrder(CBot &bot);

int Shots(const CBot &bot);

bool AimPoint(const CBot &bot, Vector &point);

bool ThrowBanned(const CBot &bot);

unsigned short Buttons(CBot &bot, const Vector &angles);

bool Brace(CBot &bot, unsigned short buttons, bool &duck, bool &stanceOnly, float &cap);

bool BraceCreep(const CBot &bot);

const char *ModeName(Mode mode);

const char *CommitName(Commit commit);

}
