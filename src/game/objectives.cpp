#include <extdll.h>
#include <meta_api.h>
#include <in_buttons.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "crossplay.h"
#include "ent_handle.h"
#include "gamedata.h"
#include "objectives.h"
#include "teams.h"

namespace
{

using Objectives::TeamBit;

enum Reader
{
	READER_NONE = 0,
	READER_CS,
	READER_DOD,
	READER_TFC,
};

constexpr float kRefreshInterval = 1.f;
constexpr size_t kGroupChars = 256;

constexpr int kCsTerrorist = 2;
constexpr int kCsCT = 1;
constexpr int kCsC4 = 1;
constexpr int kCsHostage = 2;
constexpr int kCsVip = 3;

constexpr int kDodAllies = 1;
constexpr int kDodAxis = 2;
constexpr int kDodSfTnt = 64;

constexpr int kTfcStateActive = 1;
constexpr int kTfcStateInactive = 2;
constexpr int kTfcStateRemoved = 3;
constexpr int kTfcActivateTouch = 1;
constexpr int kTfcActivateDetpack = 2;
constexpr int kTfcTeams = 4;

Reader g_reader = READER_NONE;
int g_owner = -1;
MapOffsets g_off;
Objectives::Tables g_tables;
float g_nextRefresh = 0.f;
std::vector<std::string> g_groups;
char g_status[768] = "CrossBot: objectives: no map\n";

void *Priv(const edict_t *ent)
{
	return XPlay::PrivateData(ent, g_owner);
}

int Int(const edict_t *ent, int off)
{
	void *base = Priv(ent);
	return (base && off > 0) ? GameData::Peek<int>(base, off) : 0;
}

bool Flag(const edict_t *ent, int off)
{
	void *base = Priv(ent);
	return base && off > 0 && GameData::Peek<unsigned char>(base, off) != 0;
}

std::string Chars(const edict_t *ent, int off)
{
	void *base = Priv(ent);
	if (!base || off <= 0)
		return std::string();

	const char *s = reinterpret_cast<const char *>(base) + off;
	return std::string(s, strnlen(s, kGroupChars));
}

edict_t *Handle(const edict_t *ent, int off)
{
	void *base = Priv(ent);
	if (!base || off <= 0)
		return nullptr;

	edict_t *pent = GameData::Peek<edict_t *>(base, off);
	int serial = GameData::Peek<int>(base, off + static_cast<int>(sizeof(edict_t *)));
	if (!pent || pent->free || pent->serialnumber != serial)
		return nullptr;
	return pent;
}

template <typename Fn>
void ForEach(const char *cls, Fn fn)
{
	edict_t *ent = nullptr;
	while (true)
	{
		ent = FIND_ENTITY_BY_STRING(ent, "classname", cls);
		if (!ent || g_engfuncs.pfnIndexOfEdict(ent) == 0)
			break;
		if (!ent->free)
			fn(ent);
	}
}

template <typename Fn>
void ForEachPlayer(Fn fn)
{
	for (int i = 1; i <= gpGlobals->maxClients; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (ent && CBotIsLivePlayer(ent))
			fn(ent);
	}
}

bool IsBrush(const edict_t *ent)
{
	if (!ent->v.model)
		return false;
	const char *model = g_engfuncs.pfnSzFromIndex(ent->v.model);
	return model && model[0] == '*';
}

Vector CenterOf(const edict_t *ent)
{
	return IsBrush(ent) ? (ent->v.absmin + ent->v.absmax) * 0.5f : ent->v.origin;
}

bool Gone(const edict_t *ent)
{
	return ent->v.solid == SOLID_NOT || (ent->v.effects & EF_NODRAW) != 0;
}

int GroupId(const std::string &name)
{
	if (name.empty())
		return 0;

	for (size_t i = 0; i < g_groups.size(); ++i)
	{
		if (g_groups[i] == name)
			return static_cast<int>(i) + 1;
	}

	g_groups.push_back(name);
	return static_cast<int>(g_groups.size());
}

Objectives::Site &AddSite(edict_t *ent, int attack, int defend, int needId, int needGroup)
{
	Objectives::Site site;
	site.ent.Set(ent);
	site.point = CenterOf(ent);
	site.volume = IsBrush(ent);
	site.attackTeams = attack;
	site.defendTeams = defend;
	site.needId = needId;
	site.needGroup = needGroup;
	g_tables.sites.push_back(site);
	return g_tables.sites.back();
}

Objectives::Payload &AddPayload(edict_t *ent, int id, int group, int carry, int defend)
{
	Objectives::Payload payload;
	payload.ent.Set(ent);
	payload.point = CenterOf(ent);
	payload.id = id;
	payload.group = group;
	payload.carryTeams = carry;
	payload.defendTeams = defend;
	g_tables.payloads.push_back(payload);
	return g_tables.payloads.back();
}

void AddCarrier(edict_t *player, int id, int group, bool known, bool shared)
{
	if (!player || !CBotIsLivePlayer(player))
		return;

	Objectives::Carrier carrier;
	carrier.player.Set(player);
	carrier.team = Teams::TeamOf(player);
	carrier.id = id;
	carrier.group = group;
	carrier.known = known;
	carrier.shared = shared;
	g_tables.carriers.push_back(carrier);
}

void AddSpawn(edict_t *ent, int team)
{
	Objectives::Spawn spawn;
	spawn.ent.Set(ent);
	spawn.team = team;
	g_tables.spawns.push_back(spawn);
}

void ReadCs()
{
	int t = TeamBit(kCsTerrorist);
	int ct = TeamBit(kCsCT);

	auto bombSite = [&](edict_t *ent) { AddSite(ent, t, ct, kCsC4, 0); };
	ForEach("func_bomb_target", bombSite);
	ForEach("info_bomb_target", bombSite);

	size_t rescueFrom = g_tables.sites.size();
	auto rescueSite = [&](edict_t *ent) { AddSite(ent, ct, t, kCsHostage, 0); };
	ForEach("func_hostage_rescue", rescueSite);
	ForEach("info_hostage_rescue", rescueSite);
	bool rescueZones = g_tables.sites.size() > rescueFrom;

	ForEach("func_vip_safetyzone", [&](edict_t *ent) { AddSite(ent, ct, t, kCsVip, 0); });
	ForEach("func_escapezone", [&](edict_t *ent) { AddSite(ent, t, ct, 0, 0); });

	bool hostages = false;
	auto hostage = [&](edict_t *ent) {
		if (ent->v.deadflag != DEAD_NO || Gone(ent))
			return;

		hostages = true;
		edict_t *leader = Handle(ent, g_off.cs_monTargetEnt);
		if (leader && CBotIsLivePlayer(leader))
		{
			AddCarrier(leader, kCsHostage, 0, true, true);
			return;
		}

		Objectives::Payload &payload = AddPayload(ent, kCsHostage, 0, ct, t);
		payload.buttons = IN_USE;
		payload.shared = true;
	};
	ForEach("hostage_entity", hostage);
	ForEach("monster_scientist", hostage);

	if (hostages && !rescueZones)
		ForEach("info_player_start", rescueSite);

	ForEach("weaponbox", [&](edict_t *ent) {
		if (!Gone(ent) && Flag(ent, g_off.cs_boxIsBomb))
			AddPayload(ent, kCsC4, 0, t, ct);
	});

	ForEach("grenade", [&](edict_t *ent) {
		if (!Flag(ent, g_off.cs_grnIsC4))
			return;

		Objectives::Site &site = AddSite(ent, ct, t, 0, 0);
		site.urgent = true;
		site.buttons = IN_USE | IN_DUCK;
		site.aim.Set(ent);
	});

	ForEachPlayer([&](edict_t *player) {
		if (Flag(player, g_off.cs_plrHasC4))
			AddCarrier(player, kCsC4, 0, false, false);
		if (Flag(player, g_off.cs_plrIsVIP))
			AddCarrier(player, kCsVip, 0, true, false);
	});

	ForEach("info_player_deathmatch", [](edict_t *ent) { AddSpawn(ent, kCsTerrorist); });
	ForEach("info_player_start", [](edict_t *ent) { AddSpawn(ent, kCsCT); });
}

void ReadDod()
{
	const int teams[2] = { kDodAllies, kDodAxis };
	const int both = TeamBit(kDodAllies) | TeamBit(kDodAxis);
	const int cantTouch[2] = { g_off.dod_cpAlliesCantTouch, g_off.dod_cpAxisCantTouch };
	const int canCap[2] = { g_off.dod_acAlliesCanCap, g_off.dod_acAxisCanCap };
	const int numCap[2] = { g_off.dod_acAlliesNumCap, g_off.dod_acAxisNumCap };

	ForEach("dod_control_point", [&](edict_t *ent) {
		int owner = Int(ent, g_off.dod_cpTeam);
		int attack = 0;
		for (int i = 0; i < 2; ++i)
		{
			if (owner != teams[i] && !Flag(ent, cantTouch[i]))
				attack |= TeamBit(teams[i]);
		}

		int defend = (attack & ~TeamBit(owner)) ? TeamBit(owner) : 0;
		if (attack || defend)
			AddSite(ent, attack, defend, 0, 0);
	});

	ForEach("dod_capture_area", [&](edict_t *ent) {
		int owner = Int(ent, g_off.dod_acOwningTeam);
		int attack = 0;
		int players = 0;
		for (int i = 0; i < 2; ++i)
		{
			if (owner == teams[i] || !Flag(ent, canCap[i]))
				continue;
			attack |= TeamBit(teams[i]);
			players = std::max(players, Int(ent, numCap[i]));
		}

		if (!attack)
			return;

		int group = 0;
		if (Flag(ent, g_off.dod_acRequiresObject))
		{
			group = GroupId(Chars(ent, g_off.dod_acRequiredGroup));
			if (group == 0)
				group = Objectives::kAnyGroup;
		}

		Objectives::Site &site = AddSite(ent, attack, TeamBit(owner), 0, group);
		site.needPlayers = players;
	});

	ForEach("dod_object", [&](edict_t *ent) {
		int team = Int(ent, g_off.dod_objTeam);
		int group = GroupId(Chars(ent, g_off.dod_objGroup));

		edict_t *holder = Handle(ent, g_off.dod_objOwner);
		if (holder && CBotIsLivePlayer(holder))
		{
			AddCarrier(holder, group, group, true, false);
			return;
		}

		if ((ent->v.effects & EF_NODRAW) != 0)
			return;

		int carry = (team == kDodAllies || team == kDodAxis) ? TeamBit(3 - team) : both;
		AddPayload(ent, group, group, carry, TeamBit(team));
	});

	ForEach("func_breakable", [&](edict_t *ent) {
		int flags = static_cast<int>(ent->v.spawnflags);
		if (ent->v.solid == SOLID_NOT || (flags & kDodSfTnt) == 0)
			return;

		int attack = 0;
		for (int i = 0; i < 2; ++i)
		{
			if ((flags & (8 << teams[i])) != 0)
				attack |= TeamBit(teams[i]);
		}

		std::string name = Chars(ent, g_off.dod_brkGroup);
		int group = GroupId(name);
		if (!attack && group)
		{
			for (const Objectives::Payload &payload : g_tables.payloads)
			{
				if (payload.group == group)
					attack |= payload.carryTeams;
			}
			for (const Objectives::Carrier &carrier : g_tables.carriers)
			{
				if (carrier.group == group)
					attack |= TeamBit(carrier.team);
			}
		}
		if (!attack)
			attack = both;

		AddSite(ent, attack, both & ~attack, 0, group ? group : Objectives::kAnyGroup);
	});

	ForEach("info_player_allies", [](edict_t *ent) { AddSpawn(ent, kDodAllies); });
	ForEach("info_player_axis", [](edict_t *ent) { AddSpawn(ent, kDodAxis); });
}

struct TfcGoal
{
	edict_t *ent = nullptr;
	int no = 0;
	int group = 0;
	int state = 0;
};

int TfcGoalState(const std::vector<TfcGoal> &goals, int no)
{
	for (const TfcGoal &goal : goals)
	{
		if (goal.no == no)
			return goal.state;
	}
	return 0;
}

bool TfcGroupIs(const std::vector<TfcGoal> &goals, int group, int state)
{
	for (const TfcGoal &goal : goals)
	{
		if (goal.group == group && goal.state != state)
			return false;
	}
	return true;
}

int TfcTeamCheck(const edict_t *ent, int off, int fallback)
{
	int name = Int(ent, off);
	if (!name)
		return fallback;
	edict_t *check = FIND_ENTITY_BY_STRING(nullptr, "targetname", STRING(name));
	if (!check || g_engfuncs.pfnIndexOfEdict(check) == 0 || strcmp(CBotClassOf(check), "info_tf_teamcheck") != 0)
		return fallback;
	return Int(check, g_off.tfc_teamNo);
}

bool TfcCriteria(const std::vector<TfcGoal> &goals, const edict_t *ent)
{
	const int goalFields[3] = { g_off.tfc_ifGoalActive, g_off.tfc_ifGoalInactive, g_off.tfc_ifGoalRemoved };
	const int groupFields[3] = { g_off.tfc_ifGroupActive, g_off.tfc_ifGroupInactive, g_off.tfc_ifGroupRemoved };
	const int states[3] = { kTfcStateActive, kTfcStateInactive, kTfcStateRemoved };

	for (int i = 0; i < 3; ++i)
	{
		int no = Int(ent, goalFields[i]);
		if (no && TfcGoalState(goals, no) != states[i])
			return false;

		int group = Int(ent, groupFields[i]);
		if (group && !TfcGroupIs(goals, group, states[i]))
			return false;
	}
	return true;
}

void ReadTfc()
{
	int allTeams = 0;
	auto spawn = [&](edict_t *ent) {
		int team = Int(ent, g_off.tfc_teamNo);
		allTeams |= TeamBit(team);
		AddSpawn(ent, team);
	};
	ForEach("info_player_teamspawn", spawn);
	ForEach("i_p_t", spawn);
	if (!allTeams)
		allTeams = TeamBit(1) | TeamBit(2);

	std::vector<TfcGoal> goals;
	auto note = [&](edict_t *ent) {
		TfcGoal goal;
		goal.ent = ent;
		goal.no = Int(ent, g_off.tfc_goalNo);
		goal.group = Int(ent, g_off.tfc_groupNo);
		goal.state = Int(ent, g_off.tfc_goalState);
		goals.push_back(goal);
	};
	ForEach("info_tfgoal", note);
	ForEach("i_t_g", note);

	auto item = [&](edict_t *ent) {
		int no = Int(ent, g_off.tfc_goalNo);
		int group = Int(ent, g_off.tfc_groupNo);

		edict_t *holder = ent->v.owner ? ent->v.owner : ent->v.aiment;
		if (holder && CBotIsLivePlayer(holder))
		{
			AddCarrier(holder, no, group, true, false);
			return;
		}

		if (Gone(ent) || Int(ent, g_off.tfc_goalState) == kTfcStateRemoved)
			return;

		int team = TfcTeamCheck(ent, g_off.tfc_teamcheck, Int(ent, g_off.tfc_teamNo));
		int carry = team ? TeamBit(team) : allTeams;
		int owner = TfcTeamCheck(ent, g_off.tfc_ownedByTeamcheck, Int(ent, g_off.tfc_ownedBy));
		AddPayload(ent, no, group, carry, TeamBit(owner) & ~carry);
	};
	ForEach("item_tfgoal", item);

	for (const TfcGoal &goal : goals)
	{
		edict_t *ent = goal.ent;
		int activation = Int(ent, g_off.tfc_goalActivation);
		if ((activation & kTfcActivateDetpack) != 0)
		{
			g_tables.detpackMap = true;
			if (goal.state != kTfcStateActive && goal.state != kTfcStateRemoved && TfcCriteria(goals, ent))
			{
				int team = TfcTeamCheck(ent, g_off.tfc_teamcheck, Int(ent, g_off.tfc_teamNo));
				AddSite(ent, team ? TeamBit(team) : allTeams, team ? allTeams & ~TeamBit(team) : 0, 0, 0).detpack = true;
			}
		}
		if ((activation & kTfcActivateTouch) == 0)
			continue;
		if (ent->v.solid == SOLID_NOT || goal.state == kTfcStateActive || goal.state == kTfcStateRemoved)
			continue;

		bool scores = ent->v.frags > 0.f;
		for (int i = 0; i < kTfcTeams; ++i)
		{
			if (Int(ent, g_off.tfc_increaseTeam + i * 4) > 0)
				scores = true;
		}

		if (!scores || !TfcCriteria(goals, ent))
			continue;

		int team = TfcTeamCheck(ent, g_off.tfc_teamcheck, Int(ent, g_off.tfc_teamNo));
		AddSite(ent, team ? TeamBit(team) : allTeams, 0, Int(ent, g_off.tfc_itemsAllowed),
			Int(ent, g_off.tfc_hasItemFromGroup));
	}
}

void Rebuild()
{
	g_tables.sites.clear();
	g_tables.payloads.clear();
	g_tables.carriers.clear();
	g_tables.spawns.clear();
	g_tables.detpackMap = false;

	switch (g_reader)
	{
	case READER_CS: ReadCs(); break;
	case READER_DOD: ReadDod(); break;
	case READER_TFC: ReadTfc(); break;
	default: break;
	}
}

void TeamList(int mask, char *out, size_t size)
{
	size_t len = 0;
	out[0] = '\0';
	for (int team = 1; team < 31 && len + 4 < size; ++team)
	{
		if ((mask & (1 << team)) != 0)
			len += snprintf(out + len, size - len, len ? ",%d" : "%d", team);
	}
	if (len == 0)
		snprintf(out, size, "-");
}

}

namespace Objectives
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_objectives");
	return !cvar || cvar->value != 0.f;
}

bool Active()
{
	return g_reader != READER_NONE && Enabled();
}

void DetectMap()
{
	ForgetMap();

	g_owner = XPlay::MapOwner();
	const char *dir = XPlay::GameDir(g_owner);
	if (!dir)
		dir = "?";

	if (CB_STRICMP(dir, "cstrike") == 0 || CB_STRICMP(dir, "czero") == 0)
		g_reader = READER_CS;
	else if (CB_STRICMP(dir, "dod") == 0)
		g_reader = READER_DOD;
	else if (CB_STRICMP(dir, "tfc") == 0)
		g_reader = READER_TFC;

	if (g_reader == READER_NONE)
	{
		snprintf(g_status, sizeof(g_status), "CrossBot: objectives: map belongs to '%s', which has none.\n", dir);
	}
	else if (const char *why = GameData::LoadMap(dir, g_off))
	{
		g_reader = READER_NONE;
		snprintf(g_status, sizeof(g_status), "CrossBot: objectives: off for '%s' -- %s.\n", dir, why);
	}
	else
	{
		snprintf(g_status, sizeof(g_status), "CrossBot: objectives: reading '%s' objectives.\n", dir);
	}

	SERVER_PRINT(g_status);
	Refresh(true);
}

void ForgetMap()
{
	g_reader = READER_NONE;
	g_owner = -1;
	g_off = MapOffsets();
	g_groups.clear();
	g_nextRefresh = 0.f;
	g_tables = Tables();
	snprintf(g_status, sizeof(g_status), "CrossBot: objectives: no map\n");
}

void Refresh(bool force)
{
	if (g_reader == READER_NONE)
		return;

	float now = gpGlobals->time;
	if (!force && now < g_nextRefresh && now > g_nextRefresh - 2.f * kRefreshInterval)
		return;

	g_nextRefresh = now + kRefreshInterval;
	Rebuild();
}

const Tables &Get()
{
	return g_tables;
}

void PrintStatus()
{
	SERVER_PRINT(g_status);
	if (g_reader == READER_NONE)
		return;

	Refresh(true);

	char msg[512];
	char attack[32];
	char defend[32];

	for (const Site &site : g_tables.sites)
	{
		edict_t *ent = site.ent.Get();
		if (!ent)
			continue;

		TeamList(site.attackTeams, attack, sizeof(attack));
		TeamList(site.defendTeams, defend, sizeof(defend));
		snprintf(msg, sizeof(msg),
			"CrossBot: site %s #%d at (%.0f,%.0f,%.0f)%s attack %s defend %s need id %d group %d players %d%s%s%s%s\n",
			CBotClassOf(ent), g_engfuncs.pfnIndexOfEdict(ent), site.point.x, site.point.y, site.point.z,
			site.volume ? " volume" : "", attack, defend, site.needId, site.needGroup, site.needPlayers,
			site.urgent ? " urgent" : "", (site.buttons & IN_USE) ? " USE" : "", (site.buttons & IN_DUCK) ? " DUCK" : "",
			site.detpack ? " detpack" : "");
		SERVER_PRINT(msg);
	}

	for (const Payload &payload : g_tables.payloads)
	{
		edict_t *ent = payload.ent.Get();
		if (!ent)
			continue;

		TeamList(payload.carryTeams, attack, sizeof(attack));
		TeamList(payload.defendTeams, defend, sizeof(defend));
		snprintf(msg, sizeof(msg), "CrossBot: payload %s #%d at (%.0f,%.0f,%.0f) id %d group %d carry %s defend %s%s\n",
			CBotClassOf(ent), g_engfuncs.pfnIndexOfEdict(ent), payload.point.x, payload.point.y, payload.point.z,
			payload.id, payload.group, attack, defend, payload.shared ? " shared" : "");
		SERVER_PRINT(msg);
	}

	for (const Carrier &carrier : g_tables.carriers)
	{
		edict_t *ent = carrier.player.Get();
		if (!ent)
			continue;

		snprintf(msg, sizeof(msg), "CrossBot: carrier %s team %d id %d group %d%s%s\n", CBotPlayerNameOf(ent),
			carrier.team, carrier.id, carrier.group, carrier.known ? "" : " hidden", carrier.shared ? " shared" : "");
		SERVER_PRINT(msg);
	}

	for (size_t i = 0; i < g_groups.size(); ++i)
	{
		snprintf(msg, sizeof(msg), "CrossBot: object group %d is '%s'\n", static_cast<int>(i) + 1, g_groups[i].c_str());
		SERVER_PRINT(msg);
	}

	snprintf(msg, sizeof(msg), "CrossBot: %d sites, %d payloads, %d carriers, %d spawns\n",
		static_cast<int>(g_tables.sites.size()), static_cast<int>(g_tables.payloads.size()),
		static_cast<int>(g_tables.carriers.size()), static_cast<int>(g_tables.spawns.size()));
	SERVER_PRINT(msg);

	ForEachPlayer([&](edict_t *player) {
		if (g_reader == READER_CS)
			snprintf(msg, sizeof(msg), "CrossBot: player %s pev->team %d m_iTeam %d\n", CBotPlayerNameOf(player),
				player->v.team, Int(player, g_off.cs_plrTeam));
		else
			snprintf(msg, sizeof(msg), "CrossBot: player %s pev->team %d\n", CBotPlayerNameOf(player), player->v.team);
		SERVER_PRINT(msg);
	});
}

}
