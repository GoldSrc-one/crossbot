#pragma once

#include <extdll.h>

namespace BotSight
{

void Build(const char *mapName, const char *bspPath);
void Clear();
void Poll();
void RefreshBodies();
bool Enabled();
bool Ready();
int Count();

float Clarity(const Vector &from, const Vector &to, int monsters, edict_t *skip, edict_t *target,
	bool *covered = nullptr);

void Print();

}
