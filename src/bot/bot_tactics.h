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

void EnemyField(const CBot &bot, const Nav::NavMesh &mesh, std::vector<float> &out);

struct OverlookSpot
{
	int campIndex = -1;
	Vector seen;
	int clearEnt = 0;
};

const std::vector<OverlookSpot> &Overlooks(const Nav::NavMesh &mesh, edict_t *goal, const Vector &point);

void ForgetMap();

}
