#pragma once

#include <extdll.h>

#include "../nav/nav_mesh.h"
#include "bot.h"

namespace BotOppo
{

bool Enabled();

bool TryJump(CBot &bot, const Nav::NavMesh &mesh);

void OnRepath(CBot &bot);

}
