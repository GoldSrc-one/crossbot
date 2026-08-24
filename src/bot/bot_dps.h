#pragma once

#include <extdll.h>

struct CBot;

#ifdef CROSSBOT_DPS

namespace BotDps
{

bool Enabled();

void Reset();

void Shutdown();

void NotePress(CBot &bot);

void NoteShot(CBot &bot, int weaponId, int mode);

void NoteVisible(CBot &bot, edict_t *victim, float dist);

void Frame();

void Flush();

void PinChange();

void PrintStatus();

}

#else

namespace BotDps
{

inline bool Enabled() { return false; }

inline void Reset() {}

inline void Shutdown() {}

inline void NotePress(CBot &) {}

inline void NoteShot(CBot &, int, int) {}

inline void NoteVisible(CBot &, edict_t *, float) {}

inline void Frame() {}

inline void Flush() {}

inline void PinChange() {}

inline void PrintStatus() {}

}

#endif
