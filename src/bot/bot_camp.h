#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

struct CBot;

namespace Nav
{
struct NavMesh;
}

namespace BotCamp
{

struct RouteSpot
{
	int campIndex = -1;
	int areaId = -1;
	Vector pos;
	float yaw = 0.f;
	float pitch = 0.f;
	float toGoal = 0.f;
	uint32_t flags = 0;
	bool duck = false;
	bool zoom = false;
};

struct State
{
	std::vector<RouteSpot> route;

	bool holding = false;
	float holdStart = 0.f;
	RouteSpot spot;

	bool scared = false;
	float scareDist = 0.f;
	float scareTime = 0.f;
	bool friendInView = false;
	float nextFriendCheck = 0.f;

	bool unzoom = false;
	bool checkTravel = false;

	float nextCampTime = 0.f;

	unsigned int camps = 0;
};

bool Enabled();

bool DebugEnabled();

float Budget();

float ScareRadius();

float ReachRadius();

float ZoomShare();

float ZoomDist();

void BuildRoute(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot);

void Track(CBot &bot);

bool Consider(CBot &bot);

bool OnArrived(CBot &bot);

bool Run(CBot &bot);

bool CheckTravel(CBot &bot, float seconds);

void Leave(CBot &bot, const char *why);

void Abandon(CBot &bot, const char *why);

void Reset(CBot &bot);

bool Holding(const CBot &bot);

bool Facing(const CBot &bot, float &yaw, float &pitch);

bool WantZoom(const CBot &bot, bool &want);

void OnRespawn(CBot &bot);

}
