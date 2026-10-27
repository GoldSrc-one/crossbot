#pragma once

#include <extdll.h>

#include <functional>
#include <string>
#include <vector>

#include "brain_config.h"
#include "brain_feed.h"

struct CBot;

namespace BrainContext
{

struct Input
{
	int team = 0;
	bool ffa = false;
	const BrainFeed::Event *trigger = nullptr;
	std::string recheck;
	float recheckAge = 0.f;
	std::function<bool(int userid)> ignored;
	std::string standIn;
};

struct Output
{
	std::string jevState;
	std::string jevQuestions;
	std::string context;
	std::vector<std::string> bots;
};

const char *TeamName(int game, int team);

bool OnTeam(const edict_t *ent, int team, bool ffa);

void Build(const Input &in, const BrainConfig::Config &config, Output &out);

}
