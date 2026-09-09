#pragma once

struct CBot;

namespace Nav
{
	struct NavMesh;
}

namespace BotGauss
{

bool Enabled();

void Update(CBot &bot, const Nav::NavMesh &mesh);

bool Facing(const CBot &bot, float &yawOut, float &pitchOut);

void Reset(CBot &bot);

}
