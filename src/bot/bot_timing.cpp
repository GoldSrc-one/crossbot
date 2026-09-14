#include <extdll.h>
#include <meta_api.h>

#include <stdio.h>
#include <string.h>

#include "bot_timing.h"
#include "bot.h"

namespace
{
	constexpr int kMaxPlayers = 33;
	constexpr float kReportInterval = 1.f;

	struct Slot
	{
		bool active = false;

		int postThinks = 0;
		int postThinksAtMoveStart = 0;
		byte pendingMsec = 0;
		int movesThisFrame = 0;

		int frames = 0;
		int framesNoMove = 0;
		int framesMultiMove = 0;
		int moves = 0;
		int issuedMsec = 0;
		int droppedMsec = 0;
		int droppedCalls = 0;
		float distance = 0.f;
		float peakSpeed = 0.f;
		int deadFrames = 0;
		int shortFrames = 0;
		float worstRatio = 1.f;
		float frameMsec = 0.f;
		Vector lastOrigin;
		bool hasLastOrigin = false;

		double cmdSeconds = 0.0;
		float clockStart = 0.f;
		bool clockStarted = false;
	};

	Slot g_slots[kMaxPlayers];

	float g_windowStart = 0.f;
	double g_windowHostTime = 0.0;
	float g_frameHostTime = 0.f;
	int g_windowFrames = 0;
	bool g_windowOpen = false;
	int g_anomaliesThisWindow = 0;

	bool g_dumpFrame = false;
	float g_lastDump = 0.f;
	char g_dump[1024] = {};

	int SlotIndex(edict_t *pEdict)
	{
		if (!pEdict)
			return -1;
		int idx = g_engfuncs.pfnIndexOfEdict(pEdict);
		if (idx < 1 || idx >= kMaxPlayers)
			return -1;
		return idx;
	}

	float RealTime()
	{
		return g_engfuncs.pfnTime();
	}
}

namespace BotTiming
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_time_debug");
	return cvar && cvar->value != 0.f;
}

void Reset(edict_t *pEdict)
{
	int idx = SlotIndex(pEdict);
	if (idx >= 0)
		g_slots[idx] = Slot();
}

void BeginFrame()
{
	if (!Enabled())
		return;

	g_frameHostTime = gpGlobals->frametime;

	if (!g_windowOpen)
	{
		g_windowOpen = true;
		g_windowStart = RealTime();
		g_windowHostTime = 0.0;
		g_windowFrames = 0;
	}

	g_windowHostTime += g_frameHostTime;
	++g_windowFrames;

	float now = RealTime();
	g_dumpFrame = (now - g_lastDump >= kReportInterval);
	if (g_dumpFrame)
	{
		g_lastDump = now;
		snprintf(g_dump, sizeof(g_dump), "CrossBot TIME chain (frame %.2fms):", g_frameHostTime * 1000.f);
	}

	for (Slot &slot : g_slots)
	{
		slot.movesThisFrame = 0;
		slot.frameMsec = 0.f;
		slot.active = false;
	}
}

void BeginMove(edict_t *pEdict, byte msec)
{
	if (!Enabled())
		return;
	int idx = SlotIndex(pEdict);
	if (idx < 0)
		return;

	Slot &slot = g_slots[idx];
	slot.pendingMsec = msec;
	slot.postThinksAtMoveStart = slot.postThinks;

	if (g_dumpFrame)
	{
		edict_t *e = pEdict;
		bool dead = e->v.deadflag != DEAD_NO || e->v.health <= 0.f;
		size_t len = strlen(g_dump);
		snprintf(g_dump + len, sizeof(g_dump) - len, " %s%s[saw %.0fms]=%dms",
			STRING(e->v.netname), dead ? "(dead)" : "", gpGlobals->frametime * 1000.f, msec);
	}
}

void NotePostThink(edict_t *pEdict)
{
	if (!Enabled())
		return;
	int idx = SlotIndex(pEdict);
	if (idx < 0)
		return;
	++g_slots[idx].postThinks;
}

void EndMove(edict_t *pEdict)
{
	if (!Enabled())
		return;
	int idx = SlotIndex(pEdict);
	if (idx < 0)
		return;

	Slot &slot = g_slots[idx];
	++slot.moves;
	++slot.movesThisFrame;
	slot.issuedMsec += slot.pendingMsec;
	slot.frameMsec += slot.pendingMsec;

	if (!slot.clockStarted)
	{
		slot.clockStarted = true;
		slot.clockStart = RealTime();
		slot.cmdSeconds = 0.0;
	}
	slot.cmdSeconds += slot.pendingMsec / 1000.0;

	if (slot.postThinks == slot.postThinksAtMoveStart)
	{
		slot.droppedMsec += slot.pendingMsec;
		++slot.droppedCalls;
	}

	slot.pendingMsec = 0;
}

void EndBot(CBot &bot)
{
	if (!Enabled())
		return;
	int idx = SlotIndex(bot.pEdict);
	if (idx < 0)
		return;

	Slot &slot = g_slots[idx];
	slot.active = true;
	++slot.frames;
	if (slot.movesThisFrame == 0)
		++slot.framesNoMove;
	else if (slot.movesThisFrame > 1)
		++slot.framesMultiMove;

	const Vector &org = bot.pEdict->v.origin;
	if (slot.hasLastOrigin)
	{
		float step = (org - slot.lastOrigin).Length();
		if (step < 512.f)
			slot.distance += step;
	}
	slot.lastOrigin = org;
	slot.hasLastOrigin = true;

	const Vector &vel = bot.pEdict->v.velocity;
	float speed = vel.Length();
	if (speed > slot.peakSpeed)
		slot.peakSpeed = speed;

	bool dead = bot.pEdict->v.deadflag != DEAD_NO || bot.pEdict->v.health <= 0.f;
	if (dead)
	{
		++slot.deadFrames;
		return;
	}

	float budget = g_frameHostTime * 1000.f;
	if (budget < 0.5f)
		return;

	float ratio = slot.frameMsec / budget;
	if (ratio < slot.worstRatio)
		slot.worstRatio = ratio;

	if (ratio >= 0.6f)
		return;

	++slot.shortFrames;
	if (g_anomaliesThisWindow++ >= 4)
		return;

	char msg[320];
	snprintf(msg, sizeof(msg),
		"CrossBot TIME SHORT %s: frame %.2fms but issued %.0fms (%.0f%%) over %d moves | "
		"gpGlobals->frametime now %.2fms | onground %d water %d ducking %d spd %.0f\n",
		bot.name, budget, slot.frameMsec, ratio * 100.f, slot.movesThisFrame,
		gpGlobals->frametime * 1000.f, (bot.pEdict->v.flags & FL_ONGROUND) != 0 ? 1 : 0,
		bot.pEdict->v.waterlevel, (bot.pEdict->v.flags & FL_DUCKING) != 0 ? 1 : 0, speed);
	SERVER_PRINT(msg);
}

void EndFrame()
{
	if (!Enabled() || !g_windowOpen)
		return;

	if (g_dumpFrame)
	{
		size_t len = strlen(g_dump);
		snprintf(g_dump + len, sizeof(g_dump) - len, "\n");
		SERVER_PRINT(g_dump);
		g_dumpFrame = false;
	}

	float now = RealTime();
	float wall = now - g_windowStart;
	if (wall < kReportInterval)
		return;

	char msg[384];
	snprintf(msg, sizeof(msg),
		"CrossBot TIME frame: wall %.0fms host %.0fms over %d frames (%.1f fps, %.2fms/frame)\n",
		wall * 1000.f, g_windowHostTime * 1000.0, g_windowFrames,
		g_windowFrames / wall, g_windowFrames ? (wall * 1000.f / g_windowFrames) : 0.f);
	SERVER_PRINT(msg);

	for (int i = 1; i < kMaxPlayers; ++i)
	{
		Slot &slot = g_slots[i];
		if (slot.frames == 0)
			continue;

		edict_t *pEdict = g_engfuncs.pfnPEntityOfEntIndex(i);
		const char *name = pEdict ? STRING(pEdict->v.netname) : "?";
		int ranMsec = slot.issuedMsec - slot.droppedMsec;
		float drift = slot.clockStarted
			? static_cast<float>(slot.cmdSeconds) - (now - slot.clockStart) : 0.f;

		snprintf(msg, sizeof(msg),
			"CrossBot TIME %s: budget %.0fms issued %dms (%d moves / %d frames, none=%d multi=%d) "
			"ran %dms drop %dms/%d | sim %.0f%% of real | drift %+.2fs | dead %d short %d worst %.0f%% "
			"| dist %.0fu peak %.0fu/s grav %.2f\n",
			name, g_windowHostTime * 1000.0, slot.issuedMsec, slot.moves, slot.frames,
			slot.framesNoMove, slot.framesMultiMove, ranMsec, slot.droppedMsec, slot.droppedCalls,
			wall > 0.f ? (ranMsec / (wall * 1000.f)) * 100.f : 0.f, drift, slot.deadFrames,
			slot.shortFrames, slot.worstRatio * 100.f, slot.distance, slot.peakSpeed,
			pEdict ? pEdict->v.gravity : 0.f);
		SERVER_PRINT(msg);

		slot.frames = 0;
		slot.framesNoMove = 0;
		slot.framesMultiMove = 0;
		slot.moves = 0;
		slot.issuedMsec = 0;
		slot.droppedMsec = 0;
		slot.droppedCalls = 0;
		slot.distance = 0.f;
		slot.peakSpeed = 0.f;
		slot.deadFrames = 0;
		slot.shortFrames = 0;
		slot.worstRatio = 1.f;
	}

	g_anomaliesThisWindow = 0;

	g_windowOpen = false;
}

}
