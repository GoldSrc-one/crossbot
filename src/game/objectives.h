#pragma once

#include <extdll.h>

#include <vector>

#include "ent_handle.h"

namespace Objectives
{

constexpr int kAnyGroup = -1;

struct Site
{
	CBotEntHandle ent;
	CBotEntHandle aim;
	Vector point;
	bool volume = false;
	bool urgent = false;
	unsigned short buttons = 0;
	int attackTeams = 0;
	int defendTeams = 0;
	int needId = 0;
	int needGroup = 0;
	int needPlayers = 0;
};

struct Payload
{
	CBotEntHandle ent;
	Vector point;
	int id = 0;
	int group = 0;
	int carryTeams = 0;
	int defendTeams = 0;
	unsigned short buttons = 0;
	bool shared = false;
};

struct Carrier
{
	CBotEntHandle player;
	int team = 0;
	int id = 0;
	int group = 0;
	bool known = true;
	bool shared = false;
};

struct Spawn
{
	CBotEntHandle ent;
	int team = 0;
};

struct Tables
{
	std::vector<Site> sites;
	std::vector<Payload> payloads;
	std::vector<Carrier> carriers;
	std::vector<Spawn> spawns;
};

inline int TeamBit(int team)
{
	return (team > 0 && team < 31) ? (1 << team) : 0;
}

void DetectMap();

void ForgetMap();

bool Enabled();

bool Active();

void Refresh(bool force = false);

const Tables &Get();

void PrintStatus();

}
