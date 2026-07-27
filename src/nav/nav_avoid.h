#pragma once

#include <extdll.h>

struct CBot;

namespace NavAvoid
{

Vector Deflect(CBot &bot, const Vector &desiredDir);

void SetEnabled(bool enabled);
bool IsEnabled();

}

