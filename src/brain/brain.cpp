#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cstdio>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "../bot/bot.h"
#include "../game/cvars.h"
#include "../bot/bot_manager.h"
#include "../game/crossplay.h"
#include "../game/ent_handle.h"
#include "../game/teams.h"
#include "brain.h"
#include "brain_config.h"
#include "brain_context.h"
#include "brain_feed.h"
#include "brain_memory.h"
#include "brain_sites.h"
#include "brain_tools.h"
#include "brain_voice.h"
#include "brain_worker.h"
#include "json.h"

namespace
{
	constexpr int kBrains = 5;
	constexpr float kRateWindow = 60.f;

	cvar_t g_brainCvar = { "crossbot_brain", "1", FCVAR_SERVER };
	cvar_t g_debugCvar = { "crossbot_brain_debug", "0", FCVAR_SERVER };
	cvar_t g_testHumanCvar = { "crossbot_brain_test_human", "", FCVAR_SERVER };
	cvar_t g_traceCvar = { "crossbot_brain_trace", "0", FCVAR_SERVER };

	struct TeamBrain
	{
		bool running = false;
		bool inflight = false;
		bool pending = false;
		unsigned int trigger = 0;
		unsigned int jobId = 0;

		int triggerUserid = 0;
		int triggerEnt = 0;
		std::string triggerText;

		float holdUntil = 0.f;
		bool hasHeld = false;
		BrainWorker::Result held;

		bool recheckArmed = false;
		float recheckAt = 0.f;
		float recheckArmedAt = 0.f;
		std::string recheckWhy;
		int chain = 0;

		unsigned int jobs = 0;
		unsigned int llmCalls = 0;
		unsigned int calls = 0;
		unsigned int errors = 0;
		int jevTokens = 0;
		int llmIn = 0;
		int llmOut = 0;
		float lastJevSeconds = 0.f;
		float lastLlmSeconds = 0.f;
		std::string lastVerdict;
		std::string lastCalls;
		std::string lastError;
	};

	struct Ignore
	{
		int userid = 0;
		float until = 0.f;
		std::string name;
	};

	TeamBrain g_brains[kBrains];
	unsigned int g_epoch = 1;
	unsigned int g_nextJob = 0;
	unsigned int g_seen = 0;
	bool g_dormant = true;
	int g_humans = 0;
	std::vector<Ignore> g_ignores;
	std::map<int, std::deque<float>> g_rate;

	float CvarValue(const char *name, float def)
	{
		cvar_t *cvar = CBotCvar(name);
		return cvar ? cvar->value : def;
	}

	void PrintLong(const char *title, const std::string &text)
	{
		std::string head = std::string("CrossBot: brain ") + title + " ----\n";
		SERVER_PRINT(head.c_str());
		constexpr size_t kChunk = 400;
		for (size_t at = 0; at < text.size(); at += kChunk)
		{
			std::string part = text.substr(at, kChunk);
			SERVER_PRINT(part.c_str());
		}
		SERVER_PRINT("\n");
	}

	bool Debug()
	{
		return CvarValue("crossbot_brain_debug", 0.f) != 0.f;
	}

	const char *TestHuman()
	{
		cvar_t *cvar = CBotCvar("crossbot_brain_test_human");
		return cvar && cvar->string ? cvar->string : "";
	}

	bool StandIn(const char *name)
	{
		const char *who = TestHuman();
		return who[0] && name && CB_STRICMP(who, name) == 0;
	}

	bool Ffa()
	{
		return !Teams::Enabled();
	}

	int SlotFor(int team)
	{
		if (Ffa())
			return 0;
		return team >= 1 && team < kBrains ? team : -1;
	}

	int TeamOfSlot(int slot)
	{
		return Ffa() ? 0 : slot;
	}

	bool Ignored(int userid)
	{
		for (const Ignore &ig : g_ignores)
		{
			if (ig.userid == userid && ig.until > gpGlobals->time)
				return true;
		}
		return false;
	}

	bool RateOk(int userid, int perMinute)
	{
		std::deque<float> &times = g_rate[userid];
		float now = gpGlobals->time;
		while (!times.empty() && (now - times.front() > kRateWindow || times.front() > now))
			times.pop_front();
		if (static_cast<int>(times.size()) >= perMinute)
			return false;
		times.push_back(now);
		return true;
	}

	edict_t *PlayerByUserid(int userid)
	{
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (CBotIsLivePlayer(ent) && g_engfuncs.pfnGetPlayerUserId(ent) == userid)
				return ent;
		}
		return nullptr;
	}

	int CountHumans()
	{
		int humans = 0;
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!CBotIsLivePlayer(ent))
				continue;
			if (BrainFeed::IsHuman(ent) || StandIn(CBotPlayerNameOf(ent)))
				++humans;
		}
		return humans;
	}

	void UpdateRunning()
	{
		bool ffa = Ffa();
		for (int s = 0; s < kBrains; ++s)
			g_brains[s].running = false;
		for (int i = 0; i < BotManager::Slots(); ++i)
		{
			CBot *bot = BotManager::Slot(i);
			if (!bot || !CBotIsLivePlayer(bot->pEdict))
				continue;
			int slot = ffa ? 0 : SlotFor(Teams::TeamOf(bot->pEdict));
			if (slot >= 0)
				g_brains[slot].running = true;
		}
		for (TeamBrain &b : g_brains)
		{
			if (b.running)
				continue;
			b.pending = false;
			b.recheckArmed = false;
		}
	}

	void GoDormant()
	{
		g_dormant = true;
		++g_epoch;
		for (TeamBrain &b : g_brains)
		{
			b.pending = false;
			b.inflight = false;
			b.recheckArmed = false;
			b.chain = 0;
		}
		g_seen = BrainFeed::LastSerial();
	}

	void Scan(const BrainConfig::Config &config)
	{
		const std::deque<BrainFeed::Event> &feed = BrainFeed::Events();
		for (const BrainFeed::Event &e : feed)
		{
			if (e.serial <= g_seen)
				continue;
			g_seen = e.serial;
			if (!BrainFeed::IsComms(e.kind) || e.automatic)
				continue;
			if (!e.human && !StandIn(e.who))
				continue;
			if (Ignored(e.userid) || !RateOk(e.userid, config.linesPerMinute))
				continue;

			for (int s = 0; s < kBrains; ++s)
			{
				TeamBrain &b = g_brains[s];
				if (!b.running)
					continue;
				if (e.teamOnly && !Ffa() && SlotFor(e.team) != s)
					continue;
				b.pending = true;
				b.trigger = e.serial;
				b.recheckArmed = false;
				b.chain = 0;
			}
		}
	}

	const BrainFeed::Event *FindEvent(unsigned int serial)
	{
		for (const BrainFeed::Event &e : BrainFeed::Events())
		{
			if (e.serial == serial)
				return &e;
		}
		return nullptr;
	}

	void Submit(int slot, const BrainConfig::Config &config, const BrainFeed::Event *trigger, const std::string &recheck,
		float recheckAge)
	{
		TeamBrain &b = g_brains[slot];
		BrainContext::Input in;
		in.team = TeamOfSlot(slot);
		in.ffa = Ffa();
		in.trigger = trigger;
		in.recheck = recheck;
		in.recheckAge = recheckAge;
		in.ignored = [](int userid) { return Ignored(userid); };
		in.standIn = TestHuman();

		BrainContext::Output out;
		BrainContext::Build(in, config, out);

		BrainWorker::Job job;
		job.brain = slot;
		job.id = ++g_nextJob;
		job.epoch = g_epoch;
		job.config = config;
		job.bots = out.bots;
		job.context = out.context;
		std::string dir = Brain::TraceDir();
		if (!dir.empty())
			job.tracePath = dir + "/brain_" + STRING(gpGlobals->mapname) + ".jsonl";
		job.jevBody = "{\"model\":" + Json::Quote(config.jevModel.c_str()) + ",\"state\":" +
			Json::Quote(out.jevState.c_str()) + ",\"questions\":" + out.jevQuestions + "}";

		if (Debug())
		{
			PrintLong("JEV state", out.jevState);
			PrintLong("LLM context", out.context);
		}

		b.inflight = true;
		b.pending = false;
		b.jobId = job.id;
		b.holdUntil = trigger && trigger->kind == BrainFeed::EV_RADIO ? trigger->time + config.radioHold : 0.f;
		++b.jobs;
		if (trigger)
		{
			b.triggerUserid = trigger->userid;
			b.triggerEnt = trigger->entIndex;
			b.triggerText = std::string(trigger->who) + ": " + trigger->text;
		}
		else
		{
			b.triggerUserid = 0;
			b.triggerEnt = 0;
			b.triggerText = recheck;
		}

		char msg[320];
		snprintf(msg, sizeof(msg), "CrossBot: brain %s job %u: %s\n",
			BrainContext::TeamName(XPlay::MapOwner(), TeamOfSlot(slot)), job.id, trigger ? "message" : "re-check");
		SERVER_PRINT(msg);
		BrainWorker::Submit(std::move(job));
	}

	CBot *TeamBot(int slot, const std::string &name)
	{
		for (int i = 0; i < BotManager::Slots(); ++i)
		{
			CBot *bot = BotManager::Slot(i);
			if (bot && CB_STRICMP(bot->name, name.c_str()) == 0 && !StandIn(bot->name) &&
				BrainContext::OnTeam(bot->pEdict, TeamOfSlot(slot), Ffa()))
				return bot;
		}
		return nullptr;
	}

	std::string RunCall(int slot, const BrainWorker::Call &call, edict_t *speaker)
	{
		Json::Doc doc;
		if (!doc.Parse(call.args) || doc.TypeOf(doc.Root()) != Json::T_OBJECT)
			return call.name + ": bad arguments";
		int root = doc.Root();

		if (call.name == "remember")
		{
			std::string text = doc.GetStr(root, "text");
			BrainMemory::Add(text.c_str(), doc.GetStr(root, "players").c_str(), STRING(gpGlobals->mapname),
				BrainFeed::Round());
			return "remember: " + text;
		}

		std::string name = doc.GetStr(root, "bot");
		CBot *bot = TeamBot(slot, name);
		if (!bot)
			return call.name + ": no bot '" + name + "' on this team";

		std::string result;
		XPlay::ScopedGame ctx(bot->gameDll);
		XPlay::InGame(bot->gameDll, [&] { BrainTools::Execute(*bot, call.name.c_str(), doc, root, speaker, result); });
		return call.name + " " + name + " -> " + result;
	}

	void AddIgnore(edict_t *who, int userid, float seconds)
	{
		Ignore ig;
		ig.userid = userid;
		ig.until = gpGlobals->time + seconds;
		ig.name = who ? CBotPlayerNameOf(who) : "?";
		g_ignores.push_back(ig);
		bool human = who && BrainFeed::IsHuman(who);
		char line[192];
		snprintf(line, sizeof(line), "CrossBot: brain ignoring %s (userid %d) for %.0fs, found %d, human %d, notice %s\n",
			ig.name.c_str(), userid, seconds, who ? 1 : 0, human ? 1 : 0, human ? "sent" : "not sent");
		SERVER_PRINT(line);
		if (human)
		{
			char note[128];
			snprintf(note, sizeof(note), "[CrossBot] The bots will ignore you for %.0f seconds.\n", seconds);
			g_engfuncs.pfnClientPrintf(who, print_chat, note);
		}
	}

	void Apply(const BrainWorker::Result &res, const BrainConfig::Config &config)
	{
		if (res.brain < 0 || res.brain >= kBrains)
			return;
		TeamBrain &b = g_brains[res.brain];
		if (res.epoch != g_epoch || res.id != b.jobId)
			return;
		b.inflight = false;
		b.lastJevSeconds = res.jevSeconds;
		b.lastLlmSeconds = res.llmSeconds;
		b.jevTokens += res.jevTokens;
		b.llmIn += res.llmIn;
		b.llmOut += res.llmOut;
		if (res.llmCalled)
			++b.llmCalls;

		if (Debug())
		{
			PrintLong("JEV raw", res.jevRaw);
			if (res.llmCalled)
				PrintLong("LLM raw", res.llmRaw);
		}

		const char *team = BrainContext::TeamName(XPlay::MapOwner(), TeamOfSlot(res.brain));
		char line[512];
		if (!res.error.empty())
		{
			++b.errors;
			b.lastError = res.error;
			snprintf(line, sizeof(line), "CrossBot: brain %s job %u failed: %s\n", team, res.id, res.error.c_str());
			SERVER_PRINT(line);
			if (!res.jevOk)
				return;
		}

		std::string involved;
		for (size_t i = 0; i < res.involve.size(); ++i)
		{
			char part[32];
			snprintf(part, sizeof(part), "%s%.2f", i ? "," : "", res.involve[i]);
			involved += part;
		}
		char llmTime[32] = "";
		if (res.llmCalled)
			snprintf(llmTime, sizeof(llmTime), ", llm %.2fs", res.llmSeconds);
		char verdict[256];
		snprintf(verdict, sizeof(verdict), "act %.2f involve [%s] ignore %.0fs remember %.2f recheck %.0fs (jev %.2fs%s)",
			res.act, involved.c_str(), res.ignoreSeconds, res.remember, res.recheckSeconds, res.jevSeconds, llmTime);
		b.lastVerdict = verdict;

		if (res.ignoreSeconds >= config.ignoreMin && b.triggerUserid)
		{
			AddIgnore(PlayerByUserid(b.triggerUserid), b.triggerUserid, res.ignoreSeconds);
		}

		if (res.recheckSeconds >= config.recheckMin && b.chain < config.recheckChain && b.running && !res.calls.empty())
		{
			b.recheckArmed = true;
			b.recheckAt = gpGlobals->time + res.recheckSeconds;
			b.recheckArmedAt = gpGlobals->time;
			if (b.triggerUserid)
				b.recheckWhy = b.triggerText;
		}

		edict_t *speaker = nullptr;
		if (b.triggerEnt > 0)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(b.triggerEnt);
			if (CBotIsLivePlayer(ent) && g_engfuncs.pfnGetPlayerUserId(ent) == b.triggerUserid)
				speaker = ent;
		}

		std::string calls;
		for (const BrainWorker::Call &call : res.calls)
		{
			std::string r = RunCall(res.brain, call, speaker);
			calls += (calls.empty() ? "" : " | ") + r;
			++b.calls;
		}
		b.lastCalls = res.llmCalled ? (calls.empty() ? std::string("(no tool calls)") : calls) : std::string("(gate said no)");

		snprintf(line, sizeof(line), "CrossBot: brain %s job %u: %s%s%s\n", team, res.id, verdict,
			res.llmCalled ? "; llm: " : "", res.llmCalled ? b.lastCalls.substr(0, 250).c_str() : "");
		SERVER_PRINT(line);
	}
}

namespace Brain
{

void RegisterCvars()
{
	g_engfuncs.pfnCvar_RegisterVariable(&g_brainCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_debugCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_testHumanCvar);
	g_engfuncs.pfnCvar_RegisterVariable(&g_traceCvar);
	BrainVoice::Attach();
}

void OnMapStart()
{
	BrainConfig::Load();
	BrainSites::Forget();
	++g_epoch;
	for (TeamBrain &b : g_brains)
		b = TeamBrain();
	g_seen = BrainFeed::LastSerial();
	g_dormant = true;
	g_ignores.clear();
	g_rate.clear();
	if (!BrainConfig::Get().enabled)
	{
		std::string msg = "CrossBot: brain off: " + BrainConfig::Get().why + "\n";
		SERVER_PRINT(msg.c_str());
	}
}

void OnMapEnd()
{
	BrainVoice::OnMapEnd();
	++g_epoch;
	for (TeamBrain &b : g_brains)
	{
		b.pending = false;
		b.inflight = false;
		b.recheckArmed = false;
	}
	BrainSites::Forget();
}

void Frame()
{
	BrainVoice::Frame();
	const BrainConfig::Config &config = BrainConfig::Get();
	if (!config.enabled || CvarValue("crossbot_brain", 1.f) == 0.f)
	{
		g_seen = BrainFeed::LastSerial();
		BrainWorker::Result drop;
		while (BrainWorker::Poll(drop))
		{
		}
		return;
	}

	g_humans = CountHumans();
	if (g_humans == 0)
	{
		if (!g_dormant)
		{
			GoDormant();
			SERVER_PRINT("CrossBot: brain dormant, no humans\n");
		}
		BrainWorker::Result drop;
		while (BrainWorker::Poll(drop))
		{
		}
		g_seen = BrainFeed::LastSerial();
		return;
	}
	g_dormant = false;

	float now = gpGlobals->time;
	BrainWorker::Result res;
	while (BrainWorker::Poll(res))
	{
		if (res.brain >= 0 && res.brain < kBrains && now < g_brains[res.brain].holdUntil)
		{
			g_brains[res.brain].held = std::move(res);
			g_brains[res.brain].hasHeld = true;
		}
		else
			Apply(res, config);
	}
	for (TeamBrain &b : g_brains)
	{
		if (b.hasHeld && now >= b.holdUntil)
		{
			b.hasHeld = false;
			Apply(b.held, config);
		}
	}

	UpdateRunning();
	Scan(config);

	for (int s = 0; s < kBrains; ++s)
	{
		TeamBrain &b = g_brains[s];
		if (!b.running || b.inflight)
			continue;
		if (b.pending)
		{
			const BrainFeed::Event *trigger = FindEvent(b.trigger);
			if (trigger)
				Submit(s, config, trigger, std::string(), 0.f);
			else
				b.pending = false;
		}
		else if (b.recheckArmed && now >= b.recheckAt)
		{
			b.recheckArmed = false;
			++b.chain;
			Submit(s, config, nullptr, b.recheckWhy, now - b.recheckArmedAt);
		}
	}

	g_ignores.erase(std::remove_if(g_ignores.begin(), g_ignores.end(),
		[now](const Ignore &ig) { return ig.until <= now || ig.until - now > 7200.f; }), g_ignores.end());
}

void Shutdown()
{
	BrainVoice::Detach();
	BrainWorker::Stop();
}

void PrintStatus()
{
	BrainConfig::Print();
	char line[512];
	snprintf(line, sizeof(line), "CrossBot: brain %s, %d human(s)%s, worker busy %d, feed #%u, %d memory note(s)\n",
		g_dormant ? "dormant" : "awake", g_humans, TestHuman()[0] ? " (stand-in counted)" : "", BrainWorker::Busy(),
		BrainFeed::LastSerial(), BrainMemory::Count());
	SERVER_PRINT(line);

	float now = gpGlobals->time;
	for (int s = 0; s < kBrains; ++s)
	{
		const TeamBrain &b = g_brains[s];
		if (!b.running && b.jobs == 0)
			continue;
		snprintf(line, sizeof(line),
			"CrossBot: brain %s: %s%s%s, jobs %u, llm %u, calls %u, errors %u, chain %d%s, tokens jev %d llm %d/%d, "
			"last jev %.2fs llm %.2fs\n",
			BrainContext::TeamName(XPlay::MapOwner(), TeamOfSlot(s)), b.running ? "running" : "idle",
			b.inflight ? ", in flight" : "", b.pending ? ", pending" : "", b.jobs, b.llmCalls, b.calls, b.errors, b.chain,
			b.recheckArmed ? (" (recheck in " + std::to_string(static_cast<int>(b.recheckAt - now)) + "s)").c_str() : "",
			b.jevTokens, b.llmIn, b.llmOut, b.lastJevSeconds, b.lastLlmSeconds);
		SERVER_PRINT(line);
		if (!b.lastVerdict.empty())
		{
			std::string l = "CrossBot:   last verdict: " + b.lastVerdict + "\n";
			SERVER_PRINT(l.c_str());
		}
		if (!b.lastCalls.empty())
		{
			std::string l = "CrossBot:   last calls: " + b.lastCalls.substr(0, 400) + "\n";
			SERVER_PRINT(l.c_str());
		}
		if (!b.lastError.empty())
		{
			std::string l = "CrossBot:   last error: " + b.lastError + "\n";
			SERVER_PRINT(l.c_str());
		}
	}

	BrainVoice::PrintStatus();

	for (const Ignore &ig : g_ignores)
	{
		if (ig.until <= now)
			continue;
		snprintf(line, sizeof(line), "CrossBot: brain ignores %s (userid %d) for %.0fs more\n", ig.name.c_str(),
			ig.userid, ig.until - now);
		SERVER_PRINT(line);
	}
}

void Reload()
{
	BrainConfig::Load();
	BrainConfig::Print();
}

void Unignore(const char *name)
{
	size_t before = g_ignores.size();
	g_ignores.erase(std::remove_if(g_ignores.begin(), g_ignores.end(),
		[name](const Ignore &ig) { return !name || !name[0] || CB_STRICMP(ig.name.c_str(), name) == 0; }),
		g_ignores.end());
	char line[96];
	snprintf(line, sizeof(line), "CrossBot: brain removed %zu ignore(s)\n", before - g_ignores.size());
	SERVER_PRINT(line);
}

void IgnorePlayer(const char *name, float seconds)
{
	for (int i = 1; i <= gpGlobals->maxClients; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (CBotIsLivePlayer(ent) && CB_STRICMP(CBotPlayerNameOf(ent), name) == 0)
		{
			AddIgnore(ent, g_engfuncs.pfnGetPlayerUserId(ent), seconds);
			return;
		}
	}
	SERVER_PRINT("CrossBot: brain ignore: no such player\n");
}

bool Hears(int userid)
{
	const BrainConfig::Config &config = BrainConfig::Get();
	return config.enabled && CvarValue("crossbot_brain", 1.f) != 0.f && !Ignored(userid);
}

std::string TraceDir()
{
	if (CvarValue("crossbot_brain_trace", 0.f) == 0.f)
		return std::string();
	return std::string(XPlay::HyperDir()) + "/addons/crossbot/trace";
}

}
