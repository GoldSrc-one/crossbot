#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../bot/bot.h"
#include "../bot/bot_combat.h"
#include "../bot/bot_directive.h"
#include "../bot/bot_goal.h"
#include "../bot/bot_manager.h"
#include "../bot/bot_sight.h"
#include "../game/crossplay.h"
#include "../game/ent_handle.h"
#include "../game/game_profile.h"
#include "../game/objectives.h"
#include "../game/teams.h"
#include "brain_context.h"
#include "brain_memory.h"
#include "brain_sites.h"
#include "brain_tools.h"
#include "json.h"

namespace
{
	constexpr float kAtSite = 400.f;
	constexpr float kNearSite = 1000.f;
	constexpr float kNearSpawn = 700.f;
	constexpr float kEnemyMemory = 20.f;
	constexpr int kJevComms = 6;

	std::string Fmt(const char *fmt, ...)
	{
		char buf[512];
		va_list args;
		va_start(args, fmt);
		vsnprintf(buf, sizeof(buf), fmt, args);
		va_end(args);
		return buf;
	}

	std::string Ago(float t)
	{
		return Fmt("[-%.0fs]", gpGlobals->time - t);
	}

	std::string Where(const Vector &p)
	{
		std::string s = Fmt("(%.0f %.0f %.0f)", p.x, p.y, p.z);
		std::string place = BrainSites::PlaceAt(p);
		if (!place.empty())
			s += " in " + place + ",";
		const Objectives::Tables &tables = Objectives::Get();
		int bestSite = -1;
		float bestDist = kNearSite;
		for (size_t i = 0; i < tables.sites.size(); ++i)
		{
			float d = (tables.sites[i].point - p).Length();
			if (d < bestDist)
			{
				bestDist = d;
				bestSite = static_cast<int>(i);
			}
		}
		if (bestSite >= 0)
		{
			std::string label = BrainSites::Label(bestSite);
			return s + (bestDist < kAtSite ? " at site " + label : Fmt(" %.0fu from site ", bestDist) + label);
		}

		const Objectives::Spawn *spawn = nullptr;
		float spawnDist = kNearSpawn;
		for (const Objectives::Spawn &sp : tables.spawns)
		{
			edict_t *ent = sp.ent.Get();
			if (!ent)
				continue;
			float d = (ent->v.origin - p).Length();
			if (d < spawnDist)
			{
				spawnDist = d;
				spawn = &sp;
			}
		}
		if (spawn)
			return s + " near the " + BrainContext::TeamName(XPlay::MapOwner(), spawn->team) + " spawn";
		if (!place.empty())
			s.pop_back();
		return s;
	}

	std::string ShortWeapon(const char *classname)
	{
		if (!classname)
			return "?";
		if (strncmp(classname, "weapon_", 7) == 0)
			classname += 7;
		return classname;
	}

	std::string HeldModel(const edict_t *ent)
	{
		const char *model = STRING(ent->v.weaponmodel);
		if (!model || !model[0])
			return "nothing";
		const char *slash = strrchr(model, '/');
		std::string m = slash ? slash + 1 : model;
		if (m.compare(0, 2, "p_") == 0)
			m = m.substr(2);
		size_t dot = m.rfind('.');
		if (dot != std::string::npos)
			m.resize(dot);
		return m;
	}

	std::string WeaponList(const CBot &bot)
	{
		const IGameProfile *profile = XPlay::ProfileFor(bot.gameDll);
		std::string out;
		for (int i = 0; i < bot.inventory.count; ++i)
		{
			const WeaponSpec *spec = profile ? profile->GetWeaponSpec(bot.inventory.weapons[i].id) : nullptr;
			if (!spec || !spec->classname)
				continue;
			if (!out.empty())
				out += ", ";
			if (i == bot.inventory.activeIndex)
				out += "*";
			out += spec->classname;
		}
		return out.empty() ? "none" : out;
	}

	std::string ClassOf(const CBot &bot)
	{
		static const char *const kTfc[] = { "", "scout", "sniper", "soldier", "demoman", "medic", "hwguy", "pyro",
			"spy", "engineer" };
		const char *dir = XPlay::GameDir(bot.gameDll);
		int cls = bot.pEdict->v.playerclass;
		if (!dir || CB_STRICMP(dir, "tfc") != 0 || cls <= 0 || cls >= static_cast<int>(sizeof(kTfc) / sizeof(kTfc[0])))
			return std::string();
		return std::string(", ") + kTfc[cls];
	}

	bool Carries(const edict_t *ent)
	{
		for (const Objectives::Carrier &c : Objectives::Get().carriers)
		{
			if (c.player.Get() == ent)
				return true;
		}
		return false;
	}

	bool Sees(const edict_t *from, const edict_t *to)
	{
		return BotSight::Clarity(from->v.origin + from->v.view_ofs, to->v.origin + to->v.view_ofs, ignore_monsters,
			const_cast<edict_t *>(from), nullptr) > 0.f;
	}

	bool Dead(const edict_t *ent)
	{
		return ent->v.deadflag != DEAD_NO || ent->v.health <= 0.f;
	}

	std::string BotDoing(const CBot &bot)
	{
		char desc[384];
		BotDirective::Describe(bot, desc, sizeof(desc));
		std::string s = BotDirective::Active(bot) ? std::string("orders: ") + desc : std::string(desc);
		if (BotCombat::Active(bot))
			s += "; fighting";
		else if (BotGoal::StopTier(bot) != BotGoal::GOAL_NONE && !BotDirective::Active(bot))
			s += std::string("; own plan: ") + BotGoal::TierName(BotGoal::StopTier(bot)) + " goal";
		return s;
	}

	edict_t *Speaker(const BrainContext::Input &in)
	{
		if (!in.trigger || in.trigger->entIndex <= 0)
			return nullptr;
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(in.trigger->entIndex);
		return CBotIsLivePlayer(ent) ? ent : nullptr;
	}

	std::string CommsLine(const BrainFeed::Event &e)
	{
		const char *channel = e.kind == BrainFeed::EV_RADIO ? "radio" :
			(e.kind == BrainFeed::EV_VOICE ? (e.teamOnly ? "team voice" : "voice, everyone hears") : (e.teamOnly ? "team chat" : "all chat"));
		return Ago(e.time) + " (" + channel + ") " + e.who + (e.dead ? " *DEAD*" : "") + ": " + e.text;
	}

	bool Audible(const BrainContext::Input &in, const BrainFeed::Event &e)
	{
		if (!BrainFeed::IsComms(e.kind))
			return false;
		if (in.ignored && e.userid && in.ignored(e.userid))
			return false;
		return !e.teamOnly || in.ffa || e.team == in.team;
	}

	std::string TriggerText(const BrainContext::Input &in)
	{
		if (in.trigger)
			return CommsLine(*in.trigger) + (in.trigger->human ? "" : " (sent by a bot standing in for a human)");
		return Fmt("Re-check, nothing new was said. Armed %.0fs ago to look again at: ", in.recheckAge) + in.recheck;
	}

	std::string Header(const BrainContext::Input &in)
	{
		std::string map = STRING(gpGlobals->mapname);
		std::string s = "Map " + map;
		int round = BrainFeed::Round();
		if (round > 0)
			s += Fmt(", round %d, %.0fs in", round, gpGlobals->time - BrainFeed::RoundTime());
		if (in.ffa)
			s += ". Free-for-all: everyone is an enemy.";
		else
			s += ". You are team " + Fmt("%d", in.team) + " (" + BrainContext::TeamName(XPlay::MapOwner(), in.team) + ").";
		std::string places = BrainSites::PlaceNames();
		if (!places.empty())
			s += "\nNamed places (usable as targets): " + places + ".";
		return s;
	}

	std::string Capabilities(const std::vector<int> &games)
	{
		std::string s;
		for (int g : games)
		{
			const IGameProfile *profile = XPlay::ProfileFor(g);
			if (!profile)
				continue;
			s += Fmt("- %s bots:", XPlay::GameDir(g));
			const RadioInfo *radio = nullptr;
			int radios = profile->RadioCommands(radio);
			std::string list;
			for (int i = 0; i < radios; ++i)
			{
				if (i > 0 && strcmp(radio[i].command, radio[i - 1].command) == 0)
					continue;
				list += (list.empty() ? "" : ", ") + std::string(radio[i].command) + " \"" + radio[i].text + "\"";
			}
			s += " radio: " + (list.empty() ? std::string("none (use chat)") : list) + ".";
			const AbilityInfo *abilities = nullptr;
			int count = profile->Abilities(abilities);
			std::string al;
			for (int i = 0; i < count; ++i)
				al += (al.empty() ? "" : "; ") + std::string(abilities[i].name) + " (" + abilities[i].desc +
					(abilities[i].args ? std::string("; arg: ") + abilities[i].args : std::string()) + ")";
			s += " abilities: " + (al.empty() ? std::string("none") : al) + ".";
			std::string place;
			for (int k = 0; k < DEPLOY_COUNT; ++k)
			{
				int id = profile->DeployWeapon(static_cast<DeployKind>(k));
				const WeaponSpec *spec = id ? profile->GetWeaponSpec(id) : nullptr;
				if (spec && spec->classname)
					place += (place.empty() ? "" : ", ") + std::string(spec->classname);
			}
			s += " place: " + (place.empty() ? std::string("nothing") : place) + ".";
			if (profile->OffhandCommand(0, true))
				s += " fire also takes gren1 / gren2 (the class's two offhand grenade types).";
			if (CB_STRICMP(XPlay::GameDir(g), "dod") == 0)
				s += " hold can use stance prone; a machine gunner deploys the gun when holding prone or at sandbags.";
			s += "\n";
		}
		return s;
	}

	std::string Fill(std::string s, const char *key, const std::string &value)
	{
		for (size_t at = s.find(key); at != std::string::npos; at = s.find(key, at + value.size()))
			s.replace(at, strlen(key), value);
		return s;
	}

	std::string NoulQuestion(const char *id, const std::string &instructions, const std::string &yes,
		const std::string &no)
	{
		return Json::Quote(id) + ":{\"type\":\"noul\",\"instructions\":" + Json::Quote(instructions.c_str()) +
			",\"criteria\":{\"true\":" + Json::Quote(yes.c_str()) + ",\"false\":" + Json::Quote(no.c_str()) + "}}";
	}

	std::string Duration(float s)
	{
		if (s >= 3600.f)
			return Fmt("%.0f hour%s", s / 3600.f, s >= 7200.f ? "s" : "");
		if (s >= 120.f)
			return Fmt("%.0f minutes", s / 60.f);
		return Fmt("%.0f seconds", s);
	}

	std::string ScoreQuestion(const char *id, const std::string &instructions, const std::vector<std::string> &levels)
	{
		std::string s = Json::Quote(id) + ":{\"type\":\"score\",\"instructions\":" + Json::Quote(instructions.c_str()) +
			",\"criteria\":[";
		for (size_t i = 0; i < levels.size(); ++i)
			s += (i ? "," : "") + Json::Quote(levels[i].c_str());
		return s + "]}";
	}

	std::string IgnoreLevel(const BrainConfig::Config &config, size_t i)
	{
		const std::vector<float> &levels = config.ignoreLevels;
		const std::vector<std::string> &why = config.ignoreWhy;
		if (i == 0)
			return config.ignoreNone;
		size_t k = (i - 1) * why.size() / (levels.size() > 1 ? levels.size() - 1 : 1);
		if (k >= why.size())
			k = why.size() - 1;
		return Fill(Fill(config.ignoreLevel, "{duration}", Duration(levels[i])), "{why}", why[k]);
	}
}

namespace BrainContext
{

const char *TeamName(int game, int team)
{
	const char *dir = XPlay::GameDir(game);
	if (dir && (CB_STRICMP(dir, "cstrike") == 0 || CB_STRICMP(dir, "czero") == 0))
		return team == 1 ? "CT" : team == 2 ? "T" : "spectator";
	if (dir && CB_STRICMP(dir, "dod") == 0)
		return team == 1 ? "Allies" : team == 2 ? "Axis" : "spectator";
	if (dir && CB_STRICMP(dir, "tfc") == 0)
	{
		static const char *const kTfc[] = { "spectator", "Blue", "Red", "Yellow", "Green" };
		return team >= 0 && team <= 4 ? kTfc[team] : "?";
	}
	static char buf[16];
	snprintf(buf, sizeof(buf), "team %d", team);
	return buf;
}

bool OnTeam(const edict_t *ent, int team, bool ffa)
{
	return ffa || Teams::TeamOf(ent) == team;
}

void Build(const Input &in, const BrainConfig::Config &config, Output &out)
{
	edict_t *speaker = Speaker(in);
	std::string header = Header(in);

	std::vector<CBot *> bots;
	std::vector<int> games;
	for (int i = 0; i < BotManager::Slots(); ++i)
	{
		CBot *bot = BotManager::Slot(i);
		if (!bot || !CBotIsLivePlayer(bot->pEdict) || !OnTeam(bot->pEdict, in.team, in.ffa))
			continue;
		if (!in.standIn.empty() && CB_STRICMP(bot->name, in.standIn.c_str()) == 0)
			continue;
		bots.push_back(bot);
		if (std::find(games.begin(), games.end(), bot->gameDll) == games.end())
			games.push_back(bot->gameDll);
	}

	std::string roster;
	std::string jevBots;
	out.bots.clear();
	for (CBot *bot : bots)
	{
		edict_t *e = bot->pEdict;
		bool dead = Dead(e);
		std::string rel;
		if (speaker && speaker != e)
			rel = Fmt("; %.0fu from %s%s", (e->v.origin - speaker->v.origin).Length(), CBotPlayerNameOf(speaker),
				!dead && Sees(e, speaker) ? ", can see them" : "");
		std::string state = dead ? "dead" : Fmt("alive, %.0f hp, %.0f armor", e->v.health, e->v.armorvalue);
		roster += Fmt("- %s [bot, %s%s, %s] weapons: ", bot->name, XPlay::GameDir(bot->gameDll), ClassOf(*bot).c_str(),
			state.c_str()) +
			WeaponList(*bot) + (Carries(e) ? "; carries the objective item" : "") + "; " +
			(dead ? std::string("") : "at " + Where(e->v.origin)) + rel + "; " + BotDoing(*bot) + "\n";
		jevBots += Fmt("%zu. %s - %s", out.bots.size(), bot->name, dead ? "dead" : "alive") + rel + "; " +
			BotDoing(*bot) + "\n";
		out.bots.push_back(bot->name);
	}

	for (int i = 1; i <= gpGlobals->maxClients; ++i)
	{
		edict_t *e = g_engfuncs.pfnPEntityOfEntIndex(i);
		bool standIn = !in.standIn.empty() && CB_STRICMP(CBotPlayerNameOf(e), in.standIn.c_str()) == 0;
		if (!CBotIsLivePlayer(e) || (BotManager::FindBot(e) && !standIn) || !OnTeam(e, in.team, in.ffa))
			continue;
		bool dead = Dead(e);
		roster += Fmt("- %s [human, %s] holding %s", CBotPlayerNameOf(e),
			dead ? "dead" : Fmt("alive, %.0f hp", e->v.health).c_str(), HeldModel(e).c_str()) +
			(Carries(e) ? "; carries the objective item" : "") + (dead ? std::string("") : "; at " + Where(e->v.origin)) +
			"\n";
	}

	std::string enemies;
	{
		struct Seen
		{
			edict_t *ent;
			Vector pos;
			float time;
			const char *by;
		};
		std::vector<Seen> seen;
		for (CBot *bot : bots)
		{
			edict_t *enemy = bot->sense.enemy.Get();
			if (!enemy || gpGlobals->time - bot->sense.enemyTime > kEnemyMemory)
				continue;
			auto it = std::find_if(seen.begin(), seen.end(), [&](const Seen &s) { return s.ent == enemy; });
			if (it == seen.end())
				seen.push_back({ enemy, bot->sense.enemyPos, bot->sense.enemyTime, bot->name });
			else if (bot->sense.enemyTime > it->time)
				*it = { enemy, bot->sense.enemyPos, bot->sense.enemyTime, bot->name };
		}
		for (const Seen &s : seen)
		{
			const char *name = (s.ent->v.flags & FL_CLIENT) ? CBotPlayerNameOf(s.ent) : CBotClassOf(s.ent);
			enemies += "- " + Ago(s.time) + " " + name + " seen by " + s.by + " at " + Where(s.pos) + "\n";
		}
	}

	std::string objectives;
	const Objectives::Tables &tables = Objectives::Get();
	for (size_t i = 0; i < tables.sites.size(); ++i)
	{
		const Objectives::Site &site = tables.sites[i];
		objectives += "- site " + BrainSites::Label(static_cast<int>(i)) + " (" + CBotClassOf(site.ent.Get()) + ") at " +
			Fmt("(%.0f %.0f %.0f)", site.point.x, site.point.y, site.point.z) +
			((site.attackTeams & Objectives::TeamBit(in.team)) ? ", we attack it" : "") +
			((site.defendTeams & Objectives::TeamBit(in.team)) ? ", we defend it" : "") +
			(site.urgent ? ", ACTIVE (bomb planted / under attack)" : "") + "\n";
	}
	for (const Objectives::Payload &p : tables.payloads)
		objectives += "- item " + std::string(CBotClassOf(p.ent.Get())) + " at " + Where(p.point) + "\n";
	for (const Objectives::Carrier &c : tables.carriers)
	{
		edict_t *ent = c.player.Get();
		if (!ent || (!c.known && !OnTeam(ent, in.team, in.ffa)))
			continue;
		objectives += std::string("- ") + CBotPlayerNameOf(ent) + " carries an objective item\n";
	}
	if (!tables.sites.empty() && !BrainSites::Labeled(0))
		objectives += "(site letters are unknown on this map; '#n' is the site index)\n";

	std::string focus;
	if (speaker)
	{
		Vector eye = speaker->v.origin + speaker->v.view_ofs;
		Vector fwd, right, up;
		g_engfuncs.pfnAngleVectors(speaker->v.v_angle, fwd, right, up);
		TraceResult tr;
		TRACE_LINE(eye, eye + fwd * 8192.f, ignore_monsters, speaker, &tr);
		if (tr.flFraction < 1.f)
		{
			edict_t *hit = tr.pHit ? tr.pHit : g_engfuncs.pfnPEntityOfEntIndex(0);
			const char *tex = g_engfuncs.pfnTraceTexture(hit, eye, eye + fwd * 8192.f);
			focus = Fmt("%s stands at ", CBotPlayerNameOf(speaker)) + Where(speaker->v.origin) +
				" and is looking at " + Where(tr.vecEndPos) + Fmt(", %.0fu away, surface '%s' of %s", (tr.vecEndPos - eye).Length(),
					tex ? tex : "?", CBotClassOf(hit)) + Fmt(", %s", tr.vecPlaneNormal.z > 0.7f ? "a floor or top" :
						tr.vecPlaneNormal.z < -0.7f ? "a ceiling" : "a wall") + ".\n";
		}
	}

	std::vector<const BrainFeed::Event *> events;
	std::vector<const BrainFeed::Event *> comms;
	const std::deque<BrainFeed::Event> &feed = BrainFeed::Events();
	unsigned int roundSerial = BrainFeed::RoundSerial();
	for (auto it = feed.rbegin(); it != feed.rend(); ++it)
	{
		const BrainFeed::Event &e = *it;
		if (Audible(in, e))
		{
			if (static_cast<int>(comms.size()) < config.comms && gpGlobals->time - e.time <= config.commsSeconds)
				comms.push_back(&e);
		}
		else if (!BrainFeed::IsComms(e.kind) && e.serial >= roundSerial &&
			static_cast<int>(events.size()) < config.events)
		{
			events.push_back(&e);
		}
	}

	std::string eventText;
	for (auto it = events.rbegin(); it != events.rend(); ++it)
		eventText += Ago((*it)->time) + " " + (*it)->text + "\n";
	std::string commsText;
	std::string jevComms;
	for (size_t i = comms.size(); i-- > 0;)
	{
		commsText += CommsLine(*comms[i]) + "\n";
		if (i < static_cast<size_t>(kJevComms))
			jevComms += CommsLine(*comms[i]) + "\n";
	}

	std::vector<std::string> memories = BrainMemory::Select(in.trigger ? in.trigger->who : nullptr,
		STRING(gpGlobals->mapname), config.memories);
	std::string memoryText;
	for (const std::string &m : memories)
		memoryText += "- " + m + "\n";

	std::string trigger = TriggerText(in);

	out.context = "# Situation\n" + header + "\n\n## Your team\n" + roster +
		"\n## Enemies your bots know about\n" + (enemies.empty() ? "none recently\n" : enemies) +
		"\n## Objectives\n" + (objectives.empty() ? "none\n" : objectives) +
		(focus.empty() ? "" : "\n## Speaker focus\n" + focus) +
		"\n## Events this round\n" + (eventText.empty() ? "none\n" : eventText) +
		"\n## Recent comms (oldest first)\n" + (commsText.empty() ? "none\n" : commsText) +
		"\n## Memories\n" + (memoryText.empty() ? "none\n" : memoryText) +
		"\n## Capabilities\n" + Capabilities(games) +
		"\n## Trigger\n" + trigger + "\n";

	out.jevState = header + "\nMessage to judge: " + trigger + "\nRecent comms:\n" +
		(jevComms.empty() ? "none\n" : jevComms) + "Bots on this team:\n" + (jevBots.empty() ? "none\n" : jevBots);

	std::string q = "{";
	q += NoulQuestion("act", config.actAsk, config.actYes, config.actNo);
	for (size_t i = 0; i < out.bots.size(); ++i)
	{
		char id[24];
		snprintf(id, sizeof(id), "involve_%zu", i);
		q += "," + NoulQuestion(id, Fill(config.involveAsk, "{bot}", out.bots[i]), config.involveYes, config.involveNo);
	}
	std::vector<std::string> ignoreLevels;
	for (size_t i = 0; i < config.ignoreLevels.size(); ++i)
		ignoreLevels.push_back(IgnoreLevel(config, i));
	q += "," + ScoreQuestion("ignore", config.ignoreAsk, ignoreLevels);
	q += "," + NoulQuestion("remember", config.rememberAsk, config.rememberYes, config.rememberNo);
	std::vector<std::string> recheckLevels;
	for (size_t i = 0; i < config.recheckLevels.size(); ++i)
		recheckLevels.push_back(i == 0 ? config.recheckNone : Fill(config.recheckLevel, "{duration}", Duration(config.recheckLevels[i])));
	q += "," + ScoreQuestion("recheck", config.recheckAsk, recheckLevels);
	q += "}";
	out.jevQuestions = q;
}

}
