#pragma once

#include <deque>
#include <string>
#include <vector>

#include "game_profile.h"

enum ThrowStyle
{
	THROW_HL = 0,
	THROW_QUAKE,
};

enum RespawnPress
{
	RESPAWN_ATTACK = 0,
	RESPAWN_IDLE,
};

constexpr int kItemKindCount = ITEM_STATION + 1;

struct GameConfig
{
	GameConfig() = default;
	GameConfig(const GameConfig &) = delete;
	GameConfig &operator=(const GameConfig &) = delete;
	GameConfig(GameConfig &&) = default;
	GameConfig &operator=(GameConfig &&) = default;

	std::string profile;
	const char *name = "";
	bool deathmatchOnly = false;
	int itemSlots = 6;
	std::vector<int> melee;
	int ambush[AMBUSH_COUNT] = {};
	int deployWeapon[DEPLOY_COUNT] = {};
	const char *deployEntity[DEPLOY_COUNT] = {};
	std::vector<const char *> preferred;
	float respawn[kItemKindCount] = { 30.f, 20.f, 20.f, 30.f, 60.f, 30.f, 30.f, 30.f };
	float fullTimeout = 120.f;
	float unreachableTimeout = 30.f;
	PmSim::FallRule fall = PmSim::kHlFall;
	const char *fallCvar = nullptr;
	ThrowStyle throwStyle = THROW_HL;
	float throwRate = 4.f;
	float throwSpeedMax = 500.f;
	RespawnPress respawnPress = RESPAWN_ATTACK;
	bool absoluteTimers = false;
	bool braceDuck = false;
	bool weaponSetup = false;
	std::vector<WeaponSpec> weapons;
	std::vector<RadioInfo> radio;
	std::deque<std::string> strings;
};

void GameConfig_Path(const char *gamedir, char *out, size_t size);

bool GameConfig_Load(const char *gamedir, GameConfig &out, std::string &why);
