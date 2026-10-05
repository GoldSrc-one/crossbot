#pragma once

#include <extdll.h>

#include <cstdint>
#include <vector>

struct CBot;

namespace Nav
{
struct NavMesh;
struct CampSpot;
}

namespace BotCamp
{

enum Bipod : unsigned char
{
	BIPOD_NONE = 0,
	BIPOD_SANDBAG,
	BIPOD_PRONE,
};

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
	Bipod bipod = BIPOD_NONE;
	float arc = 0.f;
	bool timed = false;
	float holdFor = 0.f;
	float raceRunner = 0.f;
	float raceOut = 0.f;
	float raceLead = 0.f;
	int snipeId = 0;
};

struct SpotMemo
{
	int campIndex = -1;
	float readyTime = 0.f;
};

struct State
{
	std::vector<RouteSpot> route;

	bool holding = false;
	float holdStart = 0.f;
	RouteSpot spot;

	bool scared = false;
	float scareDist = 0.f;
	Vector scarePos;
	float scareTime = 0.f;
	bool friendInView = false;
	float nextFriendCheck = 0.f;
	bool charging = false;
	bool rolled = false;

	bool unzoom = false;
	bool checkTravel = false;

	float nextCampTime = 0.f;
	float nextSnipeCheck = 0.f;

	unsigned int camps = 0;

	std::vector<SpotMemo> memo;
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

void CommitTimed(CBot &bot, const Nav::NavMesh &mesh, int campIndex, float holdFor, float runner, float out,
	float lead);

void CommitSnipe(CBot &bot, const Nav::NavMesh &mesh, int campIndex, float holdFor, int weaponId,
	const Vector &watch);

int SnipeId(const CBot &bot);

void NoteSpot(CBot &bot, int campIndex, float seconds);

bool SpotReady(const CBot &bot, int campIndex);

float SpotDist(const Nav::CampSpot &camp);

Vector SpotEye(const Nav::CampSpot &camp);

void AddSandbags();

bool SpotAllowed(const CBot &bot, const Nav::CampSpot &camp);

bool FacesThreat(const Nav::CampSpot &camp, const Vector &threat);

Bipod HeldBipod(const CBot &bot);

}
