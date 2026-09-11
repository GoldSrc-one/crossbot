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

// A camp spot that survived the route filter, flattened out of Nav::CampSpot so neither the trigger
// nor the hold has to touch the mesh again. duck and zoom are decided here, at build time, from the
// spot's scores and spread -- they are properties of the spot, not of the moment.
struct RouteSpot
{
	int campIndex = -1;
	int areaId = -1;
	Vector pos;
	float yaw = 0.f;
	float pitch = 0.f;
	// Cost-to-target of this camp's area. Fixed for the life of the target, which is what lets the
	// trigger re-test "still on the way" without running a search.
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

	// The scare tracker: one running minimum serving both the entry rule ("something got closer")
	// and the exit rule ("nothing has got closer for a second").
	bool scared = false;
	float scareDist = 0.f;
	float scareTime = 0.f;
	bool friendInView = false;
	float nextFriendCheck = 0.f;

	// Set while we owe the weapon a zoom-out for a camp we have left.
	bool unzoom = false;
	// One-shot armed when a camp leg is committed: the next planned route gets its length checked.
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

// Called from BotGoal::Select once the cost-to-target field exists. One linear pass over
// mesh.camps; no search of its own.
void BuildRoute(CBot &bot, const Nav::NavMesh &mesh, const std::vector<float> &fromBot);

// Every frame, before anything can return early.
void Track(CBot &bot);

// Trigger and commit. Returns true when the goal changed and the caller should idle out the frame.
bool Consider(CBot &bot);

// The camp leg arrived; start holding.
bool OnArrived(CBot &bot);

// The hold itself.
bool Run(CBot &bot);

// The route to a just-committed camp turned out to be a long walk. Consumes the one-shot; returns
// true when the caller should abandon the camp and re-pick a stop.
bool CheckTravel(CBot &bot, float seconds);

// Leave and Abandon only reset camp state -- picking the next stop is the caller's job, because
// BotGoal::OnRepathFailed and OnBlocked already do it themselves.
void Leave(CBot &bot, const char *why);

void Abandon(CBot &bot, const char *why);

// Drop the route and any hold, for when the goal that produced them is gone.
void Reset(CBot &bot);

bool Holding(const CBot &bot);

// The spot's measured facing, while holding. Used as the base the sense look blends away from.
bool Facing(const CBot &bot, float &yaw, float &pitch);

// True when camping has an opinion about the zoom state; BotFire still owns the toggle.
bool WantZoom(const CBot &bot, bool &want);

void OnRespawn(CBot &bot);

}
