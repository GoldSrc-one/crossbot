#include <extdll.h>
#include <meta_api.h>

#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#define JSMN_STATIC
#include <jsmn.h>

#include "gamedata.h"

GameOffsets g_offsets;

namespace
{

struct Request
{
	const char *cls;
	const char *field;
	size_t slot;
};

#define CB_OFF(m) offsetof(GameOffsets, m)

const Request kRequests[] = {
	{ "CBaseEntity",       "pev",                     CB_OFF(pev) },
	{ "CBaseEntity",       "m_fInAttack",             CB_OFF(ent_fInAttack) },
	{ "CBaseEntity",       "m_flStartThrow",          CB_OFF(ent_flStartThrow) },

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

	{ "CRpg",              "m_cActiveRockets",        CB_OFF(rpg_cActiveRockets) },

	{ "CBaseMonster",      "m_flNextAttack",          CB_OFF(mon_flNextAttack) },

	{ "CBasePlayer",       "m_pActiveItem",           CB_OFF(plr_pActiveItem) },
	{ "CBasePlayer",       "m_rgpPlayerItems",        CB_OFF(plr_rgpPlayerItems) },
	{ "CBasePlayer",       "m_rgAmmo",                CB_OFF(plr_rgAmmo) },
};

constexpr size_t kRequestCount = sizeof(kRequests) / sizeof(kRequests[0]);

#ifdef CROSSBOT_DPS
const Request kVirtuals[] = {
	{ "CBaseEntity", "TakeDamage(entvars_t *, entvars_t *, float, int)", CB_OFF(vt_TakeDamage) },
};

constexpr size_t kVirtualCount = sizeof(kVirtuals) / sizeof(kVirtuals[0]);
#endif

int *SlotPtr(size_t slot)
{
	return reinterpret_cast<int *>(reinterpret_cast<char *>(&g_offsets) + slot);
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

	char line[1100];
	snprintf(line, sizeof(line), "CrossBot: FATAL: %s\n", msg);
	SERVER_PRINT(line);
	SERVER_PRINT("CrossBot: refusing to run without valid gamedata, shutting the server down.\n");

	g_engfuncs.pfnServerCommand("quit\n");
	g_engfuncs.pfnServerExecute();
	exit(1);
}

void Load()
{
	static bool loaded = false;
	if (loaded)
		return;
	loaded = true;

	for (size_t r = 0; r < kRequestCount; ++r)
		*SlotPtr(kRequests[r].slot) = -1;

#ifdef CROSSBOT_DPS
	for (size_t r = 0; r < kVirtualCount; ++r)
		*SlotPtr(kVirtuals[r].slot) = -1;
#endif

	char gamedir[256] = { 0 };
	g_engfuncs.pfnGetGameDir(gamedir);

#if defined(_WIN32)
	const char *platform = "win";
#else
	const char *platform = "linux";
#endif

	char path[512];
	snprintf(path, sizeof(path), "%s/addons/crossbot/gamedata/%s-%s.json", gamedir, gamedir, platform);

	std::vector<char> js;
	if (!ReadWholeFile(path, js))
		Fatal("cannot read gamedata file %s", path);

	jsmn_parser parser;
	jsmn_init(&parser);
	int need = jsmn_parse(&parser, js.data(), js.size(), nullptr, 0);
	if (need <= 0)
		Fatal("%s is not parseable JSON (jsmn error %d)", path, need);

	std::vector<jsmntok_t> toks(static_cast<size_t>(need));
	jsmn_init(&parser);
	int got = jsmn_parse(&parser, js.data(), js.size(), toks.data(), static_cast<unsigned int>(need));
	if (got != need)
		Fatal("%s failed the second parse pass (jsmn error %d)", path, got);

	if (toks[0].type != JSMN_OBJECT)
		Fatal("%s does not have a JSON object at its root", path);

	const char *base = js.data();

	int i = 1;
	for (int c = 0; c < toks[0].size; ++c)
	{
		int key = i;
		i = SkipValue(toks, key);

		int fields = -1;
		for (size_t r = 0; r < kRequestCount; ++r)
		{
			if (!TokEq(base, toks[key], kRequests[r].cls))
				continue;

			if (fields < 0)
			{
				fields = FindMember(toks, base, key + 1, "fields");
				if (fields < 0)
					break;
			}

			int field = FindMember(toks, base, fields, kRequests[r].field);
			if (field < 0)
				continue;

			int offset = FindMember(toks, base, field, "offset");
			int value = 0;
			if (offset < 0 || !TokInt(base, toks[offset], value))
				continue;

			*SlotPtr(kRequests[r].slot) = value;
		}

#ifdef CROSSBOT_DPS
		int virtuals = -1;
		for (size_t r = 0; r < kVirtualCount; ++r)
		{
			if (!TokEq(base, toks[key], kVirtuals[r].cls))
				continue;

			if (virtuals < 0)
			{
				virtuals = FindMember(toks, base, key + 1, "virtualFunctions");
				if (virtuals < 0)
					break;
			}

			int entry = FindMember(toks, base, virtuals, kVirtuals[r].field);
			int value = 0;
			if (entry < 0 || !TokInt(base, toks[entry], value))
				continue;

			*SlotPtr(kVirtuals[r].slot) = value;
		}
#endif
	}

	for (size_t r = 0; r < kRequestCount; ++r)
	{
		if (*SlotPtr(kRequests[r].slot) < 0)
			Fatal("%s has no offset for %s::%s", path, kRequests[r].cls, kRequests[r].field);
	}

#ifdef CROSSBOT_DPS
	for (size_t r = 0; r < kVirtualCount; ++r)
	{
		if (*SlotPtr(kVirtuals[r].slot) < 0)
			Fatal("%s has no vtable index for %s::%s", path, kVirtuals[r].cls, kVirtuals[r].field);
	}
#endif

	if (g_offsets.pev != 4)
		Fatal("%s says CBaseEntity::pev is at %d, but the plugin assumes 4", path, g_offsets.pev);

#ifdef CROSSBOT_DPS
	const int virtualCount = static_cast<int>(kVirtualCount);
#else
	const int virtualCount = 0;
#endif

	char msg[600];
	snprintf(msg, sizeof(msg), "CrossBot: loaded gamedata %s (%d fields, %d vtable indices).\n",
		path, static_cast<int>(kRequestCount), virtualCount);
	SERVER_PRINT(msg);
}

}
