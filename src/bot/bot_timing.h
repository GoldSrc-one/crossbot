#pragma once

#include <extdll.h>

struct CBot;

// Movement-clock diagnostics.
//
// A fake client gets no engine physics of its own: SV_Physics skips every edict
// with index <= maxclients, so a bot advances only by the msec we hand to
// pfnRunPlayerMove. That makes the bot's clock entirely ours to get wrong, and a
// bot that receives less sim time per second than the wall clock spends looks
// like it is wading through honey -- slower running AND slower gravity.
//
// This module measures, per bot per second:
//   * host frame time actually elapsed (the budget)
//   * msec we issued, and over how many pfnRunPlayerMove calls
//   * how much of that the engine actually executed -- SV_RunCmd silently drops
//     commands while host_client->ignorecmdtime > realtime (the "clockwindow"
//     speedhack guard), which is invisible from the game DLL unless you watch
//     for PlayerPostThink not firing
//   * the resulting drift between bot sim time and real time, which is the same
//     quantity SV_CheckCmdTimes compares against clockwindow
namespace BotTiming
{
	bool Enabled();

	// Top of StartFrame, before any bot has moved: gpGlobals->frametime is still
	// host_frametime here. Every pfnRunPlayerMove clobbers it afterwards.
	void BeginFrame();

	// Around each pfnRunPlayerMove call, so a dropped command can be spotted.
	void BeginMove(edict_t *pEdict, byte msec);
	void EndMove(edict_t *pEdict);

	// The engine ran the command for this player (called from PlayerPostThink).
	void NotePostThink(edict_t *pEdict);

	// After Bot_MoveNav for one bot.
	void EndBot(CBot &bot);

	// Bottom of StartFrame: emits the once-a-second report.
	void EndFrame();

	void Reset(edict_t *pEdict);
}
