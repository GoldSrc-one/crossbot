#include <extdll.h>
#include <meta_api.h>
#include <commonmacros.h>

#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#define JSMN_STATIC
#include <jsmn.h>

#include "crossplay.h"
#include "gamedata.h"

GameOffsets g_offsets;

namespace
{

struct Request
{
	const char *cls;
	const char *field;
	size_t slot;
	bool optional;
};

#define CB_OFF(m) offsetof(GameOffsets, m)

const Request kRequests[] = {
	{ "CBaseEntity",       "pev",                     CB_OFF(pev) },
	{ "CBaseEntity",       "m_fInAttack",             CB_OFF(ent_fInAttack), true },
	{ "CBaseEntity",       "m_flStartThrow",          CB_OFF(ent_flStartThrow), true },
	{ "CBaseEntity",       "m_chargeReady",           CB_OFF(ent_chargeReady), true },
	{ "CBaseEntity",       "tfstate",                 CB_OFF(ent_tfState), true },
	{ "CBaseEntity",       "no_grenades_1",           CB_OFF(ent_noGrenades1), true },
	{ "CBaseEntity",       "no_grenades_2",           CB_OFF(ent_noGrenades2), true },
	{ "CBaseEntity",       "tp_grenades_1",           CB_OFF(ent_tpGrenades1), true },
	{ "CBaseEntity",       "tp_grenades_2",           CB_OFF(ent_tpGrenades2), true },
	{ "CBaseEntity",       "is_building",             CB_OFF(ent_isBuilding), true },
	{ "CBaseEntity",       "is_feigning",             CB_OFF(ent_isFeigning), true },
	{ "CBaseEntity",       "is_undercover",           CB_OFF(ent_isUndercover), true },
	{ "CBaseEntity",       "has_sentry",              CB_OFF(ent_hasSentry), true },
	{ "CBaseEntity",       "has_dispenser",           CB_OFF(ent_hasDispenser), true },
	{ "CBaseEntity",       "has_entry_teleporter",    CB_OFF(ent_hasEntryTele), true },
	{ "CBaseEntity",       "has_exit_teleporter",     CB_OFF(ent_hasExitTele), true },
	{ "CBaseEntity",       "real_owner",              CB_OFF(ent_realOwner), true },
	{ "CTFTeleporter",     "m_iType",                 CB_OFF(tele_iType), true },
	{ "CTFTeleporter",     "m_iState",                CB_OFF(tele_iState), true },
	{ "CBaseEntity",       "ammo_shells",             CB_OFF(ent_ammoShells), true },
	{ "CBaseEntity",       "ammo_nails",              CB_OFF(ent_ammoNails), true },
	{ "CBaseEntity",       "ammo_cells",              CB_OFF(ent_ammoCells), true },
	{ "CBaseEntity",       "ammo_rockets",            CB_OFF(ent_ammoRockets), true },
	{ "CBaseEntity",       "team_no",                 CB_OFF(ent_teamNo), true },
	{ "CBaseEntity",       "undercover_team",         CB_OFF(ent_undercoverTeam), true },
	{ "CBaseEntity",       "ammo_detpack",            CB_OFF(ent_ammoDetpack), true },
	{ "CBaseEntity",       "is_detpacking",           CB_OFF(ent_isDetpacking), true },
	{ "CBaseEntity",       "maxammo_shells",          CB_OFF(ent_maxammoShells), true },
	{ "CBaseEntity",       "maxammo_nails",           CB_OFF(ent_maxammoNails), true },
	{ "CBaseEntity",       "maxammo_cells",           CB_OFF(ent_maxammoCells), true },
	{ "CBaseEntity",       "maxammo_rockets",         CB_OFF(ent_maxammoRockets), true },
	{ "CBaseEntity",       "maxarmor",                CB_OFF(ent_maxarmor), true },
	{ "CBaseEntity",       "goal_activation",         CB_OFF(ent_goalActivation), true },
	{ "CBaseEntity",       "goal_state",              CB_OFF(ent_goalState), true },
	{ "CBaseEntity",       "items_allowed",           CB_OFF(ent_itemsAllowed), true },
	{ "CBaseEntity",       "increase_team",           CB_OFF(ent_increaseTeam), true },

	{ "CWeaponBox",        "m_flTouchableAt",         CB_OFF(box_flTouchableAt), true },
	{ "CWeaponBox",        "m_rgpPlayerItems",        CB_OFF(box_rgpPlayerItems), true },
	{ "CWeaponBox",        "m_bIsBomb",               CB_OFF(box_bIsBomb), true },
	{ "CArmoury",          "m_iItem",                 CB_OFF(arm_iItem), true },

	{ "CBasePlayerItem",   "m_pPlayer",               CB_OFF(item_pPlayer) },
	{ "CBasePlayerItem",   "m_pNext",                 CB_OFF(item_pNext) },
	{ "CBasePlayerItem",   "m_iId",                   CB_OFF(item_iId) },

	{ "CBasePlayerWeapon", "m_iClip",                 CB_OFF(wpn_iClip) },
	{ "CBasePlayerWeapon", "m_iPrimaryAmmoType",      CB_OFF(wpn_iPrimaryAmmoType) },
	{ "CBasePlayerWeapon", "m_iSecondaryAmmoType",    CB_OFF(wpn_iSecondaryAmmoType) },
	{ "CBasePlayerWeapon", "m_fInReload",             CB_OFF(wpn_fInReload) },
	{ "CBasePlayerWeapon", "m_flNextPrimaryAttack",   CB_OFF(wpn_flNextPrimaryAttack) },
	{ "CBasePlayerWeapon", "m_flNextSecondaryAttack", CB_OFF(wpn_flNextSecondaryAttack) },
	{ "CBasePlayerWeapon", "m_flTimeWeaponIdle",      CB_OFF(wpn_flTimeWeaponIdle) },
	{ "CBasePlayerWeapon", "m_flAccuracy",            CB_OFF(wpn_flAccuracy), true },
	{ "CBasePlayerWeapon", "m_flLastFire",            CB_OFF(wpn_flLastFire), true },
	{ "CBasePlayerWeapon", "m_iShotsFired",           CB_OFF(wpn_iShotsFired), true },

	{ "CRpg",              "m_cActiveRockets",        CB_OFF(rpg_cActiveRockets), true },
	{ "CEagle",            "m_iLaserActive",          CB_OFF(eagle_iLaserActive), true },

	{ "CBaseMonster",      "m_flNextAttack",          CB_OFF(mon_flNextAttack) },
	{ "CBaseMonster",      "m_hEnemy",                CB_OFF(mon_hEnemy) },

	{ "CBasePlayer",       "m_pActiveItem",           CB_OFF(plr_pActiveItem) },
	{ "CBasePlayer",       "m_rgpPlayerItems",        CB_OFF(plr_rgpPlayerItems) },
	{ "CBasePlayer",       "m_rgAmmo",                CB_OFF(plr_rgAmmo) },
	{ "CBasePlayer",       "m_pTank",                 CB_OFF(plr_pTank) },
	{ "CBasePlayer",       "m_iPowerups",             CB_OFF(plr_iPowerups), true },
	{ "CBasePlayer",       "m_iQuakeItems",           CB_OFF(plr_iQuakeItems), true },
	{ "CBasePlayer",       "m_iQuakeWeapon",          CB_OFF(plr_iQuakeWeapon), true },
	{ "CBasePlayer",       "m_iAmmoShells",           CB_OFF(plr_iAmmoShells), true },
	{ "CBasePlayer",       "m_iAmmoNails",            CB_OFF(plr_iAmmoNails), true },
	{ "CBasePlayer",       "m_iAmmoRockets",          CB_OFF(plr_iAmmoRockets), true },
	{ "CBasePlayer",       "m_iAmmoCells",            CB_OFF(plr_iAmmoCells), true },
	{ "CBasePlayer",       "m_iTeam",                 CB_OFF(plr_iTeam), true },
	{ "CBasePlayer",       "m_iJoiningState",         CB_OFF(plr_iJoiningState), true },
	{ "CBasePlayer",       "m_iMenu",                 CB_OFF(plr_iMenu), true },
	{ "CBasePlayer",       "m_signals",               CB_OFF(plr_signals), true },
	{ "CBasePlayer",       "m_bIsVIP",                CB_OFF(plr_bIsVIP), true },
	{ "CBasePlayer",       "m_bShieldDrawn",          CB_OFF(plr_bShieldDrawn), true },
	{ "CBasePlayer",       "m_bOwnsShield",           CB_OFF(plr_bOwnsShield), true },

	{ "CBaseToggle",       "m_toggle_state",          CB_OFF(tog_toggleState) },
	{ "CBaseToggle",       "m_flWait",                CB_OFF(tog_flWait) },

	{ "CBaseButton",       "m_fStayPushed",           CB_OFF(btn_fStayPushed) },

	{ "CFuncTank",         "m_pController",           CB_OFF(tank_pController) },
	{ "CFuncTank",         "m_vecControllerUsePos",   CB_OFF(tank_vecControllerUsePos) },
	{ "CFuncTank",         "m_yawCenter",             CB_OFF(tank_yawCenter) },
	{ "CFuncTank",         "m_yawRange",              CB_OFF(tank_yawRange) },
	{ "CFuncTank",         "m_yawRate",               CB_OFF(tank_yawRate) },
	{ "CFuncTank",         "m_pitchCenter",           CB_OFF(tank_pitchCenter) },
	{ "CFuncTank",         "m_pitchRange",            CB_OFF(tank_pitchRange) },
	{ "CFuncTank",         "m_pitchRate",             CB_OFF(tank_pitchRate) },
	{ "CFuncTank",         "m_fireRate",              CB_OFF(tank_fireRate) },
	{ "CFuncTank",         "m_iszMaster",             CB_OFF(tank_iszMaster), true },

	{ "CFuncTankControls", "m_pTank",                 CB_OFF(tankctl_pTank) },

	{ "CMultiManager",     "m_cTargets",              CB_OFF(mm_cTargets) },
	{ "CMultiManager",     "m_iTargetName",           CB_OFF(mm_iTargetName) },

	{ "CMultiSource",      "m_rgTriggered",           CB_OFF(ms_rgTriggered) },
	{ "CMultiSource",      "m_iTotal",                CB_OFF(ms_iTotal) },
};

constexpr size_t kRequestCount = ARRAYSIZE(kRequests);

#define CB_MAP(m) offsetof(MapOffsets, m)

const Request kCsRequests[] = {
	{ "CBasePlayer",   "m_bHasC4",                CB_MAP(cs_plrHasC4) },
	{ "CBasePlayer",   "m_bIsVIP",                CB_MAP(cs_plrIsVIP) },
	{ "CBasePlayer",   "m_iTeam",                 CB_MAP(cs_plrTeam) },
	{ "CBaseMonster",  "m_hTargetEnt",            CB_MAP(cs_monTargetEnt) },
	{ "CGrenade",      "m_bIsC4",                 CB_MAP(cs_grnIsC4) },
	{ "CWeaponBox",    "m_bIsBomb",               CB_MAP(cs_boxIsBomb) },
};

const Request kDodRequests[] = {
	{ "CControlPoint", "m_iTeam",                 CB_MAP(dod_cpTeam) },
	{ "CControlPoint", "m_bAlliesCantTouch",      CB_MAP(dod_cpAlliesCantTouch) },
	{ "CControlPoint", "m_bAxisCantTouch",        CB_MAP(dod_cpAxisCantTouch) },
	{ "CAreaCapture",  "m_nOwningTeam",           CB_MAP(dod_acOwningTeam) },
	{ "CAreaCapture",  "m_nAlliesNumCap",         CB_MAP(dod_acAlliesNumCap) },
	{ "CAreaCapture",  "m_nAxisNumCap",           CB_MAP(dod_acAxisNumCap) },
	{ "CAreaCapture",  "m_bAlliesCanCap",         CB_MAP(dod_acAlliesCanCap) },
	{ "CAreaCapture",  "m_bAxisCanCap",           CB_MAP(dod_acAxisCanCap) },
	{ "CAreaCapture",  "m_bRequiresObject",       CB_MAP(dod_acRequiresObject) },
	{ "CAreaCapture",  "sz_RequiredObjectGroup",  CB_MAP(dod_acRequiredGroup) },
	{ "CObject",       "m_iTeam",                 CB_MAP(dod_objTeam) },
	{ "CObject",       "sz_Group",                CB_MAP(dod_objGroup) },
	{ "CObject",       "m_hOwner",                CB_MAP(dod_objOwner) },
	{ "CBreakable",    "sz_Group",                CB_MAP(dod_brkGroup) },
};

const Request kTfcRequests[] = {
	{ "CBaseEntity",   "team_no",                 CB_MAP(tfc_teamNo) },
	{ "CBaseEntity",   "goal_no",                 CB_MAP(tfc_goalNo) },
	{ "CBaseEntity",   "group_no",                CB_MAP(tfc_groupNo) },
	{ "CBaseEntity",   "goal_state",              CB_MAP(tfc_goalState) },
	{ "CBaseEntity",   "owned_by",                CB_MAP(tfc_ownedBy) },
	{ "CBaseEntity",   "teamcheck",               CB_MAP(tfc_teamcheck), true },
	{ "CBaseEntity",   "owned_by_teamcheck",      CB_MAP(tfc_ownedByTeamcheck), true },
	{ "CBaseEntity",   "goal_activation",         CB_MAP(tfc_goalActivation) },
	{ "CBaseEntity",   "items_allowed",           CB_MAP(tfc_itemsAllowed) },
	{ "CBaseEntity",   "has_item_from_group",     CB_MAP(tfc_hasItemFromGroup) },
	{ "CBaseEntity",   "if_goal_is_active",       CB_MAP(tfc_ifGoalActive) },
	{ "CBaseEntity",   "if_goal_is_inactive",     CB_MAP(tfc_ifGoalInactive) },
	{ "CBaseEntity",   "if_goal_is_removed",      CB_MAP(tfc_ifGoalRemoved) },
	{ "CBaseEntity",   "if_group_is_active",      CB_MAP(tfc_ifGroupActive) },
	{ "CBaseEntity",   "if_group_is_inactive",    CB_MAP(tfc_ifGroupInactive) },
	{ "CBaseEntity",   "if_group_is_removed",     CB_MAP(tfc_ifGroupRemoved) },
	{ "CBaseEntity",   "increase_team",           CB_MAP(tfc_increaseTeam) },
};

GameOffsets g_gameOffsets[XPlay::kMaxGames];
bool g_gameLoaded[XPlay::kMaxGames];

char g_why[512];

int *SlotPtr(void *out, size_t slot)
{
	return reinterpret_cast<int *>(reinterpret_cast<char *>(out) + slot);
}

bool TokEq(const char *js, const jsmntok_t &tok, const char *name)
{
	if (tok.type != JSMN_STRING)
		return false;
	int len = tok.end - tok.start;
	return static_cast<int>(strlen(name)) == len && strncmp(js + tok.start, name, len) == 0;
}

int SkipValue(const std::vector<jsmntok_t> &toks, int i)
{
	int children = toks[i].size;
	++i;
	for (int k = 0; k < children; ++k)
		i = SkipValue(toks, i);
	return i;
}

int FindMember(const std::vector<jsmntok_t> &toks, const char *js, int obj, const char *name)
{
	if (obj < 0 || toks[obj].type != JSMN_OBJECT)
		return -1;

	int i = obj + 1;
	for (int c = 0; c < toks[obj].size; ++c)
	{
		int key = i;
		i = SkipValue(toks, key);
		if (TokEq(js, toks[key], name))
			return key + 1;
	}
	return -1;
}

bool TokInt(const char *js, const jsmntok_t &tok, int &out)
{
	int len = tok.end - tok.start;
	if (tok.type != JSMN_PRIMITIVE || len <= 0 || len > 15)
		return false;

	char buf[16];
	memcpy(buf, js + tok.start, len);
	buf[len] = '\0';

	char *end = nullptr;
	long v = strtol(buf, &end, 10);
	if (end != buf + len)
		return false;

	out = static_cast<int>(v);
	return true;
}

bool ReadWholeFile(const char *path, std::vector<char> &out)
{
	FILE *f = fopen(path, "rb");
	if (!f)
		return false;

	fseek(f, 0, SEEK_END);
	long size = ftell(f);
	fseek(f, 0, SEEK_SET);

	if (size <= 0)
	{
		fclose(f);
		return false;
	}

	out.resize(static_cast<size_t>(size));
	bool ok = fread(out.data(), 1, out.size(), f) == out.size();
	fclose(f);
	return ok;
}

const char *Why(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vsnprintf(g_why, sizeof(g_why), fmt, args);
	va_end(args);
	return g_why;
}

const char *LoadOne(const char *path, const Request *requests, size_t count, void *out)
{
	for (size_t r = 0; r < count; ++r)
		*SlotPtr(out, requests[r].slot) = -1;

	std::vector<char> js;
	if (!ReadWholeFile(path, js))
		return Why("cannot read it");

	jsmn_parser parser;
	jsmn_init(&parser);
	int need = jsmn_parse(&parser, js.data(), js.size(), nullptr, 0);
	if (need <= 0)
		return Why("not parseable JSON (jsmn error %d)", need);

	std::vector<jsmntok_t> toks(static_cast<size_t>(need));
	jsmn_init(&parser);
	int got = jsmn_parse(&parser, js.data(), js.size(), toks.data(), static_cast<unsigned int>(need));
	if (got != need)
		return Why("failed the second parse pass (jsmn error %d)", got);

	if (toks[0].type != JSMN_OBJECT)
		return Why("no JSON object at its root");

	const char *base = js.data();

	int i = 1;
	for (int c = 0; c < toks[0].size; ++c)
	{
		int key = i;
		i = SkipValue(toks, key);

		int fields = -1;
		for (size_t r = 0; r < count; ++r)
		{
			if (!TokEq(base, toks[key], requests[r].cls))
				continue;

			if (fields < 0)
			{
				fields = FindMember(toks, base, key + 1, "fields");
				if (fields < 0)
					break;
			}

			int field = FindMember(toks, base, fields, requests[r].field);
			int value = 0;
			if (field < 0 || !TokInt(base, toks[field], value))
				continue;

			*SlotPtr(out, requests[r].slot) = value;
		}
	}

	for (size_t r = 0; r < count; ++r)
	{
		if (!requests[r].optional && *SlotPtr(out, requests[r].slot) < 0)
			return Why("no offset for %s::%s", requests[r].cls, requests[r].field);
	}

	return nullptr;
}

const char *Platform()
{
#if defined(_WIN32)
	return "win";
#else
	return "linux";
#endif
}

}

namespace GameData
{

void Fatal(const char *fmt, ...)
{
	char msg[1024];
	va_list args;
	va_start(args, fmt);
	vsnprintf(msg, sizeof(msg), fmt, args);
	va_end(args);

	char line[1024];
	snprintf(line, sizeof(line), "CrossBot: FATAL: %s\n", msg);
	SERVER_PRINT(line);
	SERVER_PRINT("CrossBot: refusing to run without valid gamedata, shutting the server down.\n");

	g_engfuncs.pfnServerCommand("quit\n");
	g_engfuncs.pfnServerExecute();
	exit(1);
}

void Activate(int iGame)
{
	if (iGame < 0 || iGame >= XPlay::kMaxGames)
		return;

	g_offsets = g_gameOffsets[iGame];
}

bool Loaded(int iGame)
{
	if (iGame < 0 || iGame >= XPlay::kMaxGames)
		return false;

	return g_gameLoaded[iGame];
}

void LoadAll()
{
	for (int i = 0; i < XPlay::kMaxGames; ++i)
	{
		g_gameOffsets[i] = GameOffsets();
		g_gameLoaded[i] = false;
	}

	char hyperdir[256] = { 0 };
	g_engfuncs.pfnGetGameDir(hyperdir);

	const char *platform = Platform();

	char msg[900];
	for (int iGame = 0; iGame < XPlay::NumGames(); ++iGame)
	{
		const char *gamedir = XPlay::GameDir(iGame);

		char path[512];
		snprintf(path, sizeof(path), "%s/gamedata/%s-%s.json", hyperdir, gamedir, platform);

		const char *why = LoadOne(path, kRequests, kRequestCount, &g_gameOffsets[iGame]);
		if (!why && g_gameOffsets[iGame].pev != 4)
			why = Why("CBaseEntity::pev is at %d, but the plugin assumes 4", g_gameOffsets[iGame].pev);
		if (why)
		{
			snprintf(msg, sizeof(msg), "CrossBot: no gamedata for '%s' -- %s (%s).\n", gamedir, why, path);
			SERVER_PRINT(msg);
			continue;
		}

		g_gameLoaded[iGame] = true;
		snprintf(msg, sizeof(msg), "CrossBot: loaded gamedata %s (%d fields).\n",
			path, static_cast<int>(kRequestCount));
		SERVER_PRINT(msg);
	}
}

const char *LoadMap(const char *gamedir, MapOffsets &out)
{
	out = MapOffsets();

	const Request *requests = nullptr;
	size_t count = 0;
	if (CB_STRICMP(gamedir, "cstrike") == 0 || CB_STRICMP(gamedir, "czero") == 0)
	{
		requests = kCsRequests;
		count = ARRAYSIZE(kCsRequests);
	}
	else if (CB_STRICMP(gamedir, "dod") == 0)
	{
		requests = kDodRequests;
		count = ARRAYSIZE(kDodRequests);
	}
	else if (CB_STRICMP(gamedir, "tfc") == 0)
	{
		requests = kTfcRequests;
		count = ARRAYSIZE(kTfcRequests);
	}
	else
	{
		return Why("no objective offsets are defined for this game");
	}

	char hyperdir[256] = { 0 };
	g_engfuncs.pfnGetGameDir(hyperdir);

	char path[512];
	snprintf(path, sizeof(path), "%s/gamedata/%s-%s.json", hyperdir, gamedir, Platform());

	const char *why = LoadOne(path, requests, count, &out);
	if (!why)
		return nullptr;

	char reason[512];
	snprintf(reason, sizeof(reason), "%s", why);
	return Why("%s (%s)", reason, path);
}

}
