#pragma once

#include <extdll.h>

namespace Teams
{

enum Relation : unsigned char
{
	REL_UNKNOWN = 0,
	REL_FRIEND,
	REL_ENEMY,
};

void DetectMap();

void ForgetMap();

bool Enabled();

bool MapTeamplay();

const char *MapGameDir();

int TeamOf(const edict_t *ent);

bool Active(const edict_t *self);

Relation Of(const edict_t *self, const edict_t *ent);

bool Attackable(const edict_t *self, const edict_t *ent);

const char *RelationName(Relation rel);

}
