#include <extdll.h>
#include <meta_api.h>
#include <commonmacros.h>

#include <stdio.h>
#include <string.h>

#include "../../bot/bot_move.h"
#include "../../bot/bot_timing.h"
#include "../../bot/bot_weapons.h"
#include "../dmc/dmc_profile.h"
#include "../crossplay.h"
#include "../ent_handle.h"
#include "../fake_cmd.h"
#include "../gamedata.h"
#include "tfc_ability.h"
#include "tfc_profile.h"
#include "tfc_weapons.h"

namespace
{
	constexpr float kJoinInterval = 1.2f;
	constexpr float kStationFullValue = 95.f;
	constexpr int kTfStateAiming = 0x800;
	constexpr int kTfStatePrimed = 0x1;

	constexpr float kGrenadePin = 0.8f;
	constexpr float kGrenadePrime = 3.f;
	constexpr float kCaltropPrime = 0.5f;
	constexpr float kGrenadeMaxRange = 650.f;

	constexpr float kFallSafeSpeed = 650.f;
	constexpr float kFallDamage = 100.f / 374.f;

	enum TFCGrenade
	{
		TFC_GREN_NORMAL     = 1,
		TFC_GREN_CONCUSSION = 2,
		TFC_GREN_NAIL       = 3,
		TFC_GREN_MIRV       = 4,
		TFC_GREN_NAPALM     = 5,
		TFC_GREN_GAS        = 7,
		TFC_GREN_EMP        = 8,
		TFC_GREN_CALTROP    = 9,
	};

	const WeaponMode kGrenadeMode = { FIRE_TAP, AIM_LEAD, TOGGLE_NONE, AMMO_NONE, 0, 632.5f, 0.81f, 0.f, 180.f, 0.f, 0.f, WPN_Q_THROW_ARC };
	constexpr int kMaxPlayers = 32;
	constexpr const char *kRandomClass = "randompc";

	const char *const kClasses[] = {
		"scout", "sniper", "soldier", "demoman", "medic", "hwguy", "pyro", "spy", "engineer",
		"civilian", kRandomClass,
	};

	char g_class[kMaxPlayers + 1][16];
	float g_nextJoin[kMaxPlayers + 1];

	int PlayerSlot(edict_t *pEdict)
	{
		int index = g_engfuncs.pfnIndexOfEdict(pEdict);
		return (index >= 1 && index <= kMaxPlayers) ? index : 0;
	}

	const char *KnownClass(const char *arg)
	{
		for (size_t i = 0; i < ARRAYSIZE(kClasses); ++i)
		{
			if (CB_STRICMP(arg, kClasses[i]) == 0)
				return kClasses[i];
		}
		return nullptr;
	}

	void IdleMove(edict_t *pEdict)
	{
		byte msec = Bot_FrameMsec();
		BotTiming::BeginMove(pEdict, msec);
		g_engfuncs.pfnRunPlayerMove(pEdict, pEdict->v.v_angle, 0.f, 0.f, 0.f, 0, 0, msec);
		BotTiming::EndMove(pEdict);
	}
}

const char *CTFCProfile::GetName() const
{
	return "TFC";
}

bool CTFCProfile::Detect(const char *gamedir) const
{
	return gamedir && CB_STRICMP(gamedir, "tfc") == 0;
}

PmSim::FallRule CTFCProfile::FallDamage() const
{
	return { kFallSafeSpeed, kFallDamage };
}

void CTFCProfile::ApplyAddArg(int, edict_t *pEdict, const char *arg, char *out,
	unsigned int size) const
{
	const char *chosen = kRandomClass;
	if (arg && arg[0] && strcmp(arg, "*") != 0)
	{
		const char *known = KnownClass(arg);
		if (known)
		{
			chosen = known;
		}
		else
		{
			char msg[256];
			int len = snprintf(msg, sizeof(msg), "CrossBot: unknown TFC class '%s' -- using %s. Classes:",
				arg, kRandomClass);
			for (size_t i = 0; i < ARRAYSIZE(kClasses) && len > 0 && len < static_cast<int>(sizeof(msg)); ++i)
				len += snprintf(msg + len, sizeof(msg) - len, " %s", kClasses[i]);
			SERVER_PRINT(msg);
			SERVER_PRINT("\n");
		}
	}

	int slot = PlayerSlot(pEdict);
	if (slot)
	{
		snprintf(g_class[slot], sizeof(g_class[slot]), "%s", chosen);
		g_nextJoin[slot] = 0.f;
	}

	if (out && size)
		snprintf(out, size, "%s", chosen);
}

bool CTFCProfile::PreThink(edict_t *pEdict) const
{
	if (TFCAbility::FeignHold(pEdict))
	{
		IdleMove(pEdict);
		return true;
	}

	if (pEdict->v.team != 0 && pEdict->v.playerclass != 0)
		return false;

	int slot = PlayerSlot(pEdict);
	if (!slot)
		return false;

	float now = gpGlobals->time;
	if (now >= g_nextJoin[slot] || g_nextJoin[slot] - now > kJoinInterval)
	{
		g_nextJoin[slot] = now + kJoinInterval;

		char cmd[32];
		if (pEdict->v.team == 0)
			snprintf(cmd, sizeof(cmd), "jointeam 5");
		else
			snprintf(cmd, sizeof(cmd), "%s", g_class[slot][0] ? g_class[slot] : kRandomClass);
		FakeCmd::Run(pEdict, cmd);
	}

	IdleMove(pEdict);
	return true;
}

bool CTFCProfile::OffhandRead(edict_t *pEdict, int slot, OffhandInfo &out) const
{
	out = OffhandInfo();

	int countField = (slot == 0) ? g_offsets.ent_noGrenades1 : g_offsets.ent_noGrenades2;
	int typeField = (slot == 0) ? g_offsets.ent_tpGrenades1 : g_offsets.ent_tpGrenades2;
	if (slot < 0 || slot >= kMaxOffhandSlots || countField < 0 || typeField < 0)
		return false;

	void *priv = CBotPrivateData(pEdict);
	if (!priv)
		return false;

	if (g_offsets.ent_isBuilding >= 0 && GameData::Peek<int>(priv, g_offsets.ent_isBuilding) != 0)
		return false;
	if (g_offsets.ent_isFeigning >= 0 && GameData::Peek<int>(priv, g_offsets.ent_isFeigning) != 0)
		return false;

	out.type = GameData::Peek<int>(priv, typeField);
	out.count = GameData::Peek<int>(priv, countField);
	out.pin = kGrenadePin;
	out.fuse = kGrenadePin + kGrenadePrime;
	out.mode = kGrenadeMode;

	switch (out.type)
	{
	case TFC_GREN_NORMAL:
	case TFC_GREN_NAPALM:
		out.use = OFFHAND_ATTACK;
		out.minRange = 250.f;
		break;
	case TFC_GREN_CONCUSSION:
	case TFC_GREN_GAS:
		out.mode.quirks |= WPN_Q_TEAM_UNSAFE;
		out.use = OFFHAND_ATTACK;
		out.minRange = 300.f;
		break;
	case TFC_GREN_NAIL:
	case TFC_GREN_EMP:
		out.use = OFFHAND_ATTACK;
		out.minRange = 300.f;
		break;
	case TFC_GREN_MIRV:
		out.use = OFFHAND_ATTACK;
		out.minRange = 350.f;
		break;
	case TFC_GREN_CALTROP:
		out.use = OFFHAND_SCARED;
		out.fuse = kGrenadePin + kCaltropPrime;
		break;
	default:
		out.use = OFFHAND_NONE;
		break;
	}
	out.maxRange = kGrenadeMaxRange;
	return true;
}

const char *CTFCProfile::OffhandCommand(int slot, bool prime) const
{
	if (slot == 0)
		return prime ? "+gren1" : "-gren1";
	if (slot == 1)
		return prime ? "+gren2" : "-gren2";
	return nullptr;
}

bool CTFCProfile::OffhandPrimed(edict_t *pEdict) const
{
	void *priv = CBotPrivateData(pEdict);
	if (!priv || g_offsets.ent_tfState < 0)
		return false;
	return (GameData::Peek<int>(priv, g_offsets.ent_tfState) & kTfStatePrimed) != 0;
}

bool CTFCProfile::AbilityRun(CBot &bot, const Nav::NavMesh &mesh) const
{
	return TFCAbility::Run(bot, mesh);
}

void CTFCProfile::RefreshWorld() const
{
	TFCAbility::RefreshTeleporters();
}

bool CTFCProfile::Disguised(edict_t *pEdict) const
{
	return TFCAbility::Disguised(pEdict);
}

bool CTFCProfile::HoldFire(const CBot &bot) const
{
	return TFCAbility::HoldFire(bot);
}

bool CTFCProfile::ItemUsable(edict_t *player, edict_t *item) const
{
	return TFCAbility::ItemUsable(player, item);
}

float CTFCProfile::ItemWorth(edict_t *player, edict_t *item, float ammoNeed) const
{
	return TFCAbility::ItemWorth(player, item, ammoNeed);
}

float CTFCProfile::ItemRespawnIn(edict_t *item, ItemKind kind) const
{
	float wait = TFCAbility::PackWait(item);
	return wait > 0.f ? wait : ItemRespawnDelay(kind);
}

bool CTFCProfile::Fortifies(edict_t *pEdict) const
{
	return TFCAbility::Fortifies(pEdict);
}

bool CTFCProfile::CanDetpack(edict_t *pEdict) const
{
	return TFCAbility::CanDetpack(pEdict);
}

void CTFCProfile::NoteSound(edict_t *src, const char *sample) const
{
	TFCAbility::NoteSound(src, sample);
}

void CTFCProfile::Patients(edict_t *medic, std::vector<edict_t *> &out) const
{
	TFCAbility::Patients(medic, out);
}

ItemKind CTFCProfile::ClassifyItem(edict_t *ent) const
{
	int kind = TFCAbility::ClassifyItem(ent);
	return kind >= 0 ? static_cast<ItemKind>(kind) : CHLDMProfile::ClassifyItem(ent);
}

float CTFCProfile::ChargerFullValue(ItemKind kind) const
{
	return kind == ITEM_STATION ? kStationFullValue : CHLDMProfile::ChargerFullValue(kind);
}

float CTFCProfile::ChargerValue(ItemKind kind, edict_t *pEdict) const
{
	return kind == ITEM_STATION ? TFCAbility::StationValue(pEdict) : CHLDMProfile::ChargerValue(kind, pEdict);
}

const char *CTFCProfile::HazardEntity() const
{
	return "tf_weapon_caltrop";
}

const WeaponSpec *CTFCProfile::GetWeaponSpec(int id) const
{
	return TFC_WeaponSpec(id);
}

int CTFCProfile::WeaponSlotCount() const
{
	return kTFCWeaponSlots;
}

const WeaponSpec *CTFCProfile::WeaponSlot(int slot) const
{
	return TFC_WeaponSlot(slot);
}

int CTFCProfile::ItemSlots() const
{
	return kTFCItemSlots;
}

bool CTFCProfile::ReadInventory(edict_t *player, BotInventory &out) const
{
	BotWeapons::ReadItems(player, out);

	void *priv = CBotPrivateData(player);
	const int fields[] = {
		g_offsets.ent_ammoShells, g_offsets.ent_ammoNails, g_offsets.ent_ammoCells, g_offsets.ent_ammoRockets,
	};

	int tfState = (g_offsets.ent_tfState >= 0) ? GameData::Peek<int>(priv, g_offsets.ent_tfState) : 0;

	for (int i = 0; i < out.count; ++i)
	{
		BotWeapon &w = out.weapons[i];
		if (w.id == TFC_SNIPERRIFLE)
			w.chargeState = (tfState & kTfStateAiming) != 0 ? 1 : 0;

		TFCAmmoField field = TFC_AmmoField(w.id);
		if (field == TFC_FIELD_NONE || fields[field] < 0)
			continue;
		if (w.primaryAmmoType < 0 || w.primaryAmmoType >= MAX_AMMO_SLOTS)
			continue;
		out.ammo[w.primaryAmmoType] = GameData::Peek<int>(priv, fields[field]);
	}

	return true;
}

int CTFCProfile::MeleeWeapon() const
{
	return TFC_AXE;
}

bool CTFCProfile::IsMeleeWeapon(int id) const
{
	return id == TFC_AXE || id == TFC_SPANNER || id == TFC_MEDIKIT || id == TFC_KNIFE;
}

int CTFCProfile::AmbushWeapon(AmbushKind) const
{
	return 0;
}

int CTFCProfile::DeployWeapon(DeployKind) const
{
	return 0;
}

const char *CTFCProfile::DeployEntity(DeployKind) const
{
	return nullptr;
}

bool CTFCProfile::ThrowLaunch(const WeaponMode &mode, float viewPitch, float &elevation,
	float &speed) const
{
	if ((mode.quirks & WPN_Q_THROW_ARC) == 0)
		return false;

	return DMC_ThrowLaunch(viewPitch, elevation, speed);
}
