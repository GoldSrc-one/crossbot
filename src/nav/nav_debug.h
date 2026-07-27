#pragma once

#include <extdll.h>

namespace NavDebug
{

void Precache();

void Draw();

void SetPathBeamsEnabled(bool enabled);
bool PathBeamsEnabled();

void SetAreaBeamsEnabled(bool enabled);
bool AreaBeamsEnabled();

}

