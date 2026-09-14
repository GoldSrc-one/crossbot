#include <extdll.h>
#include <meta_api.h>

#include <math.h>
#include <string.h>

#include "../game/ent_handle.h"
#include "bot.h"
#include "bot_daze.h"
#include "bot_log.h"
#include "bot_manager.h"

namespace
{
	constexpr float kFixedScale = 4096.f;
	constexpr int kFadeModulate = 0x0002;
	constexpr int kFlashAlpha = 200;
	constexpr float kDefaultBlindScale = 0.33f;

	constexpr float kPitchCycle = 1.f;
	constexpr float kYawCycle = 2.f;
	constexpr float kPitchLevel = 0.3f;
	constexpr float kYawLevel = 0.3f;

	constexpr int kMaxFields = 16;

	int g_screenFadeId = 0;
	int g_concussId = 0;

	edict_t *g_capture = nullptr;
	int g_captureType = 0;
	int g_fields[kMaxFields];
	int g_fieldCount = 0;

	float CvarValue(const char *name, float fallback)
	{
		cvar_t *cvar = g_engfuncs.pfnCVarGetPointer(name);
		return cvar ? cvar->value : fallback;
	}

	int DebugLevel()
	{
#if BOT_VLOG_ON
		return static_cast<int>(CvarValue("crossbot_daze_debug", 0.f));
#else
		return 0;
#endif
	}

	float SwayScale(const CBot &bot)
	{
		return static_cast<float>(bot.daze.concuss) * CvarValue("crossbot_conc_gain", 1.f);
	}
}

bool BotDaze::Blind(const CBot &bot)
{
	return bot.daze.blindUntil > gpGlobals->time;
}

void BotDaze::Sway(const CBot &bot, Vector &angles)
{
	if (bot.daze.concuss <= 0)
		return;
	float s = SwayScale(bot);
	angles.x += s * sinf(gpGlobals->time * kPitchCycle) * kPitchLevel;
	angles.y += s * sinf(gpGlobals->time * kYawCycle) * kYawLevel;
}

void BotDaze::OnScreenFade(CBot &bot, int duration, int hold, int flags, int r, int g, int b, int a)
{
	State &st = bot.daze;
	float now = gpGlobals->time;
	float fadeSec = static_cast<float>(duration & 0xFFFF) / kFixedScale;
	float holdSec = static_cast<float>(hold & 0xFFFF) / kFixedScale;

	const char *verdict = "blind";
	if ((flags & kFadeModulate) != 0)
	{
		verdict = "ignored, modulate";
	}
	else if (a < kFlashAlpha)
	{
		verdict = "ignored, alpha below 200";
	}
	else
	{
		float until = now + CvarValue("crossbot_blind_scale", kDefaultBlindScale) * fadeSec;
		if (st.blindUntil <= now)
			st.blindStart = now;
		if (until > st.blindUntil)
			st.blindUntil = until;
		st.fadeSec = fadeSec;
		st.holdSec = holdSec;
		st.alpha = a;
		++st.flashes;
	}

	if (DebugLevel() >= 1)
	{
		BOT_VLOG("CrossBot: %s ScreenFade dur=%d (%.2fs) hold=%d (%.2fs) flags=0x%04x "
			"rgba=%d,%d,%d,%d -> %s, blind for %.2fs\n", bot.name, duration, fadeSec, hold, holdSec,
			flags & 0xFFFF, r, g, b, a, verdict, Blind(bot) ? st.blindUntil - now : 0.f);
	}
}

void BotDaze::OnConcuss(CBot &bot, int scale)
{
	State &st = bot.daze;
	float now = gpGlobals->time;
	int was = st.concuss;
	st.concuss = scale & 0xFF;
	++st.concussUpdates;

	if (was == 0 && st.concuss != 0)
	{
		st.concussStart = now;
		st.concussUpdates = 1;
		++st.concs;
	}

	int level = DebugLevel();
	if (level < 1)
		return;

	if (was == 0 && st.concuss != 0)
		BOT_VLOG("CrossBot: %s Concuss onset scale=%d -> sway up to %.1f deg\n", bot.name,
			st.concuss, SwayScale(bot) * kYawLevel);
	else if (was != 0 && st.concuss == 0)
		BOT_VLOG("CrossBot: %s Concuss cleared after %.2fs, %u updates\n", bot.name,
			now - st.concussStart, st.concussUpdates);
	else if (level >= 2)
		BOT_VLOG("CrossBot: %s Concuss scale=%d at %.2fs\n", bot.name, st.concuss,
			now - st.concussStart);
}

void BotDaze::OnRegUserMsg(const char *name, int id)
{
	if (!name || id <= 0)
		return;
	if (strcmp(name, "ScreenFade") == 0)
		g_screenFadeId = id;
	else if (strcmp(name, "Concuss") == 0)
		g_concussId = id;
}

void BotDaze::OnMessageBegin(int dest, int type, edict_t *ed)
{
	g_capture = nullptr;

	if ((dest != MSG_ONE && dest != MSG_ONE_UNRELIABLE) || !ed || (ed->v.flags & FL_FAKECLIENT) == 0)
		return;

	if (DebugLevel() >= 2)
	{
		const char *name = GET_USER_MSG_NAME(PLID, type, nullptr);
		BOT_VLOG("CrossBot: %s <- message %d %s\n", CBotPlayerNameOf(ed), type,
			name ? name : "(engine)");
	}

	if (type <= 0 || (type != g_screenFadeId && type != g_concussId))
		return;
	if (!BotManager::FindBot(ed))
		return;

	g_capture = ed;
	g_captureType = type;
	g_fieldCount = 0;
}

void BotDaze::OnWrite(int value)
{
	if (!g_capture)
		return;
	if (g_fieldCount < kMaxFields)
		g_fields[g_fieldCount++] = value;
}

void BotDaze::OnMessageEnd()
{
	edict_t *ed = g_capture;
	g_capture = nullptr;
	if (!ed)
		return;

	CBot *bot = BotManager::FindBot(ed);
	if (!bot)
		return;

	if (g_captureType == g_screenFadeId && g_fieldCount >= 7)
		OnScreenFade(*bot, g_fields[0], g_fields[1], g_fields[2], g_fields[3], g_fields[4],
			g_fields[5], g_fields[6]);
	else if (g_captureType == g_concussId && g_fieldCount >= 1)
		OnConcuss(*bot, g_fields[0]);
}

void BotDaze::PrintStatus(const CBot &bot)
{
	const State &st = bot.daze;
	float now = gpGlobals->time;
	float s = SwayScale(bot);

	char line[320];
	snprintf(line, sizeof(line),
		"CrossBot: %s blind=%.2fs alpha=%d fade=%.2fs hold=%.2fs flashes=%u conc=%d for %.1fs "
		"updates=%u concs=%u sway=%+.1f,%+.1f\n",
		bot.name, Blind(bot) ? st.blindUntil - now : 0.f, st.alpha, st.fadeSec, st.holdSec,
		st.flashes, st.concuss, st.concuss ? now - st.concussStart : 0.f, st.concussUpdates,
		st.concs, s * sinf(now * kPitchCycle) * kPitchLevel, s * sinf(now * kYawCycle) * kYawLevel);
	SERVER_PRINT(line);
}

void BotDaze::PrintUserMsgs()
{
	char line[160];
	int found = 0;
	for (int id = 64; id < 256; ++id)
	{
		int size = 0;
		const char *name = GET_USER_MSG_NAME(PLID, id, &size);
		if (!name)
			continue;
		++found;
		snprintf(line, sizeof(line), "CrossBot: usermsg %3d %-24s size %d%s\n", id, name, size,
			(id == g_screenFadeId || id == g_concussId) ? " <- daze" : "");
		SERVER_PRINT(line);
	}

	snprintf(line, sizeof(line), "CrossBot: %d user messages, ScreenFade=%d Concuss=%d\n", found,
		g_screenFadeId, g_concussId);
	SERVER_PRINT(line);
}
