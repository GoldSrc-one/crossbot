#pragma once

#include <extdll.h>

#include <vector>

struct CBot;

namespace Nav
{
struct NavMesh;
}

namespace BotTactics
{

bool Enabled();

bool DebugEnabled();

bool Pick(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot);

bool Closest(const CBot &bot);

void ForgetMap();

}
