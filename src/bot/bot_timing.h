#pragma once

#include <extdll.h>

struct CBot;

namespace BotTiming
{
	bool Enabled();

	void BeginFrame();

	void BeginMove(edict_t *pEdict, byte msec);
	void EndMove(edict_t *pEdict);

	void NotePostThink(edict_t *pEdict);

	void EndBot(CBot &bot);

	void EndFrame();

	void Reset(edict_t *pEdict);
}
