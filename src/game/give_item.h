#pragma once

#include <extdll.h>

namespace GiveItem
{

bool Give(edict_t *player, const char *classname);

edict_t *Spawn(const char *classname, const Vector &origin);

}
