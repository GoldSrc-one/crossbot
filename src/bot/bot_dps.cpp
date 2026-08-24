#include <extdll.h>
#include <meta_api.h>

#include "bot_dps.h"

#ifdef CROSSBOT_DPS

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bot.h"
#include "bot_fire.h"
#include "bot_weapons.h"
#include "../game/game_profile.h"
#include "../game/gamedata.h"
#include "../game/take_damage_hook.h"

namespace
{
	constexpr int kSlots = 33;
	constexpr float kFlushInterval = 20.f;

	struct Press
	{
		int shotId;
		float time;
		float range;
		int weaponId;
		int mode;
		int zoomed;
		const char *aim;
		float flight;
		bool valid;
	};

	float g_gaze[kSlots][kSlots];

	Press g_press[kSlots];

	int g_shotSerial = 0;

	FILE *g_file = nullptr;
	char g_path[256] = "";
	bool g_failed = false;

	float g_flushTime = 0.f;
	float g_frameSum = 0.f;
	int g_frameCount = 0;

	int g_rows = 0;
	int g_shotRows = 0;
	int g_dmgRows = 0;
	float g_damageTotal = 0.f;
	float g_gazeTotal = 0.f;
	float g_gazeSpent = 0.f;

	const char *LogPath()
	{
		cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_dps_log");
		if (cvar && cvar->string && cvar->string[0])
			return cvar->string;
		return "valve/addons/crossbot/dps/dps.csv";
	}

	FILE *Out()
	{
		if (g_failed)
			return nullptr;

		const char *want = LogPath();
		if (g_file && strcmp(want, g_path) == 0)
			return g_file;

		if (g_file)
		{
			fclose(g_file);
			g_file = nullptr;
		}

		snprintf(g_path, sizeof(g_path), "%s", want);
		g_file = fopen(g_path, "a");

		if (!g_file)
		{
			char msg[320];
			snprintf(msg, sizeof(msg), "CrossBot: cannot open dps log %s\n", g_path);
			SERVER_PRINT(msg);
			g_failed = true;
		}

		return g_file;
	}

	void Row(const char *fmt, ...)
	{
		FILE *f = Out();
		if (!f)
			return;

		va_list args;
		va_start(args, fmt);
		vfprintf(f, fmt, args);
		va_end(args);

		++g_rows;
	}

	int IndexOf(const edict_t *ent)
	{
		if (!ent || ent->free)
			return 0;
		int index = g_engfuncs.pfnIndexOfEdict(ent);
		if (index < 1 || index >= kSlots || index > gpGlobals->maxClients)
			return 0;
		return index;
	}

	const char *NameOf(int index)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(index);
		return ent ? CBotPlayerNameOf(ent) : "?";
	}

	const char *MapName()
	{
		if (!gpGlobals->pStringBase)
			return "?";
		const char *s = gpGlobals->pStringBase + static_cast<unsigned int>(gpGlobals->mapname);
		return (s && s[0]) ? s : "?";
	}

	const char *AimName(const BotFire::State &fire)
	{
		if (fire.aimArc)
			return "arc";
		if (fire.aimHoming)
			return "homing";
		if (fire.aimLead)
			return "lead";
		return "target";
	}

	Vector EyeOf(const edict_t *ent)
	{
		return ent->v.origin + ent->v.view_ofs;
	}

	void OnDamage(edict_t *victim, edict_t *attacker, edict_t *inflictor, float damage, int bits)
	{
		if (!BotDps::Enabled())
			return;

		int hurt = IndexOf(victim);
		if (hurt == 0)
			return;

		int by = IndexOf(attacker);

		float seconds = 0.f;
		if (by != 0)
		{
			seconds = g_gaze[by][hurt];
			g_gaze[by][hurt] = 0.f;
			g_gazeSpent += seconds;
		}

		float dist = -1.f;
		if (attacker && !attacker->free)
			dist = (EyeOf(victim) - EyeOf(attacker)).Length();

		Row("dmg,%.2f,%s,%s,%.1f,%.3f,%.1f,%d,%s\n", gpGlobals->time, NameOf(hurt),
			by ? NameOf(by) : "?", damage, seconds, dist, bits,
			inflictor ? CBotClassOf(inflictor) : "?");

		++g_dmgRows;
		g_damageTotal += damage;
	}
}

namespace BotDps
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_dps");
	return cvar && cvar->value != 0.f;
}

void Reset()
{
	memset(g_gaze, 0, sizeof(g_gaze));
	memset(g_press, 0, sizeof(g_press));

	g_flushTime = gpGlobals->time;
	g_frameSum = 0.f;
	g_frameCount = 0;
	g_failed = false;

	TakeDamageHook::Install(&OnDamage);
}

void Shutdown()
{
	TakeDamageHook::Remove();

	if (g_file)
	{
		fclose(g_file);
		g_file = nullptr;
	}
	g_path[0] = 0;
}

void NotePress(CBot &bot)
{
	if (!Enabled())
		return;

	int index = IndexOf(bot.pEdict);
	if (index == 0)
		return;

	const BotFire::State &fire = bot.fire;

	Press &p = g_press[index];
	p.shotId = ++g_shotSerial;
	p.time = gpGlobals->time;
	p.range = fire.lastRange;
	p.weaponId = fire.weaponId;
	p.mode = fire.mode;
	p.zoomed = fire.zoomed ? 1 : 0;
	p.aim = AimName(fire);
	p.flight = fire.flightTime;
	p.valid = true;
}

void NoteShot(CBot &bot, int weaponId, int mode)
{
	if (!Enabled())
		return;

	int index = IndexOf(bot.pEdict);
	if (index == 0)
		return;

	const Press &p = g_press[index];
	if (!p.valid)
		return;

	Row("shot,%.2f,%s,%d,%d,%d,%s,%.1f,%.3f,%d\n", gpGlobals->time, NameOf(index),
		weaponId, mode, p.zoomed, p.aim ? p.aim : "target", p.range, p.flight, p.shotId);
	++g_shotRows;
}

void NoteVisible(CBot &bot, edict_t *victim, float dist)
{
	if (!Enabled())
		return;

	(void)dist;

	int attacker = IndexOf(bot.pEdict);
	int target = IndexOf(victim);
	if (attacker == 0 || target == 0)
		return;

	float dt = gpGlobals->frametime;
	g_gaze[attacker][target] += dt;
	g_gazeTotal += dt;
}

void Flush()
{
	if (!Enabled())
		return;

	float now = gpGlobals->time;
	float span = now - g_flushTime;
	float frameMs = (g_frameCount > 0) ? (g_frameSum / g_frameCount * 1000.f) : 0.f;

	int bots = 0;
	for (int i = 1; i <= gpGlobals->maxClients && i < kSlots; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (ent && !ent->free && (ent->v.flags & FL_CLIENT) != 0)
			++bots;
	}

	const char *pin = BotFire::Pinned();

	Row("meta,%.2f,%s,%s,%s,%d,%.1f,%.2f\n", now, MapName(), pin ? pin : "off",
		BotFire::PinnedMode(), bots, span, frameMs);

	g_flushTime = now;
	g_frameSum = 0.f;
	g_frameCount = 0;

	if (g_file)
		fflush(g_file);
}

void PinChange()
{
	Flush();
	memset(g_gaze, 0, sizeof(g_gaze));
}

void Frame()
{
	if (!Enabled())
		return;

	g_frameSum += gpGlobals->frametime;
	++g_frameCount;

	if (gpGlobals->time - g_flushTime >= kFlushInterval)
		Flush();
}

void PrintStatus()
{
	char msg[320];
	snprintf(msg, sizeof(msg),
		"CrossBot: dps=%d log=%s rows=%d shots=%d dmg=%d damage=%.0f gaze=%.1fs spent=%.1fs\n",
		Enabled() ? 1 : 0, g_path[0] ? g_path : LogPath(), g_rows, g_shotRows, g_dmgRows,
		g_damageTotal, g_gazeTotal, g_gazeSpent);
	SERVER_PRINT(msg);
}

}

#endif
