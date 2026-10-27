#include <extdll.h>
#include <meta_api.h>

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../bot/bot.h"
#include "../bot/bot_directive.h"
#include "../game/crossplay.h"
#include "../game/ent_handle.h"
#include "../game/fake_cmd.h"
#include "../game/game_profile.h"
#include "../game/objectives.h"
#include "brain_sites.h"
#include "brain_tools.h"
#include "brain_voice.h"

extern IGameProfile *g_pGameProfile;

namespace
{
	constexpr float kAimRange = 8192.f;
	constexpr float kAimPullback = 8.f;
	constexpr float kAlertSeconds = 10.f;
	constexpr float kMaxSeconds = 600.f;

	bool ContainsNoCase(const char *hay, const char *needle)
	{
		size_t n = strlen(needle);
		for (; *hay; ++hay)
		{
			if (CB_STRNICMP(hay, needle, n) == 0)
				return true;
		}
		return false;
	}

	bool ParseTriple(const char *ref, Vector &out)
	{
		float v[3];
		const char *p = ref;
		for (int i = 0; i < 3; ++i)
		{
			while (*p == ' ' || *p == ',')
				++p;
			char *end = nullptr;
			v[i] = static_cast<float>(strtod(p, &end));
			if (end == p)
				return false;
			p = end;
		}
		while (*p == ' ')
			++p;
		if (*p)
			return false;
		out = Vector(v[0], v[1], v[2]);
		return true;
	}

	bool ParseSite(const char *label, Vector &out)
	{
		const std::vector<Objectives::Site> &sites = Objectives::Get().sites;
		int index = BrainSites::Find(label);
		if (index < 0 || index >= static_cast<int>(sites.size()))
			return false;
		out = sites[static_cast<size_t>(index)].point;
		return true;
	}

	std::string SiteLabel(const char *ref)
	{
		std::string s = ref;
		for (char &c : s)
			c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
		static const char *const kPrefixes[] = { "bombsite", "bomb site", "site" };
		for (const char *prefix : kPrefixes)
		{
			size_t n = strlen(prefix);
			if (s.compare(0, n, prefix) == 0)
			{
				std::string rest = s.substr(n);
				size_t at = rest.find_first_not_of(" :#_-");
				return at == std::string::npos ? std::string() : rest.substr(at);
			}
		}
		size_t space = s.find(' ');
		if (space != std::string::npos && (s.substr(space + 1) == "site" || s.substr(space + 1) == "bombsite"))
			return s.substr(0, space);
		if (s.size() > 1 && s[0] == '#')
			return s.substr(1);
		return std::string();
	}

	float Seconds(const Json::Doc &doc, int obj, const char *name, float def)
	{
		float v = doc.GetNum(obj, name, def);
		if (v < 0.f)
			v = 0.f;
		return v > kMaxSeconds ? kMaxSeconds : v;
	}

	bool ReadStep(const Json::Doc &doc, int obj, edict_t *speaker, BotDirective::Step &step, std::string &err)
	{
		std::string kind = doc.GetStr(obj, "do");
		if (!BotDirective::ParseStep(kind.c_str(), step.kind))
		{
			err = "unknown step '" + kind + "'";
			return false;
		}

		std::string target = doc.GetStr(obj, "target");
		std::string watch = doc.GetStr(obj, "watch");
		std::string item = doc.GetStr(obj, "item");
		std::string ability = doc.GetStr(obj, "ability");
		std::string stance = doc.GetStr(obj, "stance");
		step.within = doc.GetNum(obj, "within", 0.f);
		step.seconds = Seconds(doc, obj, "seconds", 0.f);
		step.count = static_cast<int>(doc.GetNum(obj, "count", 0.f));
		if (!stance.empty() && !BotDirective::ParseStance(stance.c_str(), step.stance))
		{
			err = "unknown stance '" + stance + "'";
			return false;
		}
		snprintf(step.value, sizeof(step.value), "%s", BrainTools::Sanitize(doc.GetStr(obj, "arg").c_str(), sizeof(step.value) - 1).c_str());

		bool needsTarget = step.kind == BotDirective::STEP_GO || step.kind == BotDirective::STEP_FOLLOW ||
			step.kind == BotDirective::STEP_HOLD || step.kind == BotDirective::STEP_LOOK ||
			step.kind == BotDirective::STEP_FIRE || step.kind == BotDirective::STEP_PLACE ||
			(step.kind == BotDirective::STEP_ABILITY && !target.empty());
		if (needsTarget && !BrainTools::ResolveTarget(target.c_str(), speaker, step.target, err))
			return false;
		if (step.kind == BotDirective::STEP_FOLLOW && !step.target.ent.ent)
		{
			err = "follow needs a player target";
			return false;
		}
		if (!watch.empty() && !BrainTools::ResolveTarget(watch.c_str(), speaker, step.watch, err))
			return false;

		const std::string &arg = step.kind == BotDirective::STEP_ABILITY ? ability : item;
		if (step.kind == BotDirective::STEP_ABILITY && arg.empty())
		{
			err = "ability step needs 'ability'";
			return false;
		}
		if (step.kind == BotDirective::STEP_SELECT && arg.empty())
		{
			err = "select step needs 'item'";
			return false;
		}
		snprintf(step.arg, sizeof(step.arg), "%s", arg.c_str());
		return true;
	}

	bool Say(CBot &bot, const char *verb, const Json::Doc &doc, int args, std::string &result)
	{
		size_t room = FakeCmd::kMaxLength - strlen(verb) - 2;
		std::string text = BrainTools::Sanitize(doc.GetStr(args, "text").c_str(), room);
		if (text.empty())
		{
			result = "error: empty text";
			return false;
		}
		std::string cmd = std::string(verb) + " " + text;
		if (!BotDirective::Queue(bot, cmd.c_str()))
		{
			result = "error: too many queued commands";
			return false;
		}
		result = "ok: queued " + cmd;
		return true;
	}
}

namespace BrainTools
{

bool AimPoint(edict_t *player, Vector &out)
{
	if (!player)
		return false;
	Vector eye = player->v.origin + player->v.view_ofs;
	Vector forward, right, up;
	g_engfuncs.pfnAngleVectors(player->v.v_angle, forward, right, up);

	TraceResult tr;
	TRACE_LINE(eye, eye + forward * kAimRange, ignore_monsters, player, &tr);
	if (tr.flFraction >= 1.f)
		return false;
	out = tr.vecEndPos - forward * kAimPullback;
	return true;
}

edict_t *FindPlayer(const char *name)
{
	if (!name || !name[0])
		return nullptr;

	edict_t *partial = nullptr;
	int partials = 0;
	for (int i = 1; i <= gpGlobals->maxClients; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (!CBotIsLivePlayer(ent))
			continue;
		const char *n = CBotPlayerNameOf(ent);
		if (CB_STRICMP(n, name) == 0)
			return ent;
		if (ContainsNoCase(n, name))
		{
			partial = ent;
			++partials;
		}
	}
	return partials == 1 ? partial : nullptr;
}

bool ResolveTarget(const char *ref, edict_t *speaker, BotDirective::Target &out, std::string &err)
{
	out = BotDirective::Target();
	if (!ref || !ref[0])
	{
		err = "missing target";
		return false;
	}

	if (CB_STRICMP(ref, "me") == 0 || CB_STRICMP(ref, "speaker") == 0)
	{
		if (!speaker)
		{
			err = "no speaker to resolve 'me'";
			return false;
		}
		out.ent.Set(speaker);
		out.valid = true;
		return true;
	}

	if (CB_STRICMP(ref, "aim") == 0 || CB_STRNICMP(ref, "aim:", 4) == 0)
	{
		edict_t *who = ref[3] == ':' ? FindPlayer(ref + 4) : speaker;
		if (!who)
		{
			err = std::string("no player for '") + ref + "'";
			return false;
		}
		if (!AimPoint(who, out.point))
		{
			err = "aim hits nothing";
			return false;
		}
		out.valid = true;
		return true;
	}

	std::string site = SiteLabel(ref);
	if (!site.empty())
	{
		if (!ParseSite(site.c_str(), out.point))
		{
			err = "no objective site '" + site + "'";
			return false;
		}
		out.valid = true;
		return true;
	}

	if (ParseTriple(ref, out.point))
	{
		out.valid = true;
		return true;
	}

	std::string place;
	bool exactPlace = BrainSites::FindPlace(ref, out.point, place) && CB_STRICMP(place.c_str(), ref) == 0;
	edict_t *ent = exactPlace ? nullptr : FindPlayer(ref);
	if (!ent)
	{
		if (!place.empty() || BrainSites::FindPlace(ref, out.point, place))
		{
			out.valid = true;
			return true;
		}
		err = std::string("no player or place '") + ref + "'";
		return false;
	}
	out.ent.Set(ent);
	out.valid = true;
	return true;
}

std::string Sanitize(const char *text, size_t maxLength)
{
	std::string out;
	if (!text)
		return out;
	for (const unsigned char *p = reinterpret_cast<const unsigned char *>(text); *p && out.size() < maxLength; ++p)
	{
		unsigned char c = *p;
		if (c == '"' || c == ';' || c == '\\')
			continue;
		if (c < 0x20)
			c = ' ';
		if (c == ' ' && (out.empty() || out.back() == ' '))
			continue;
		out += static_cast<char>(c);
	}
	while (!out.empty() && out.back() == ' ')
		out.pop_back();
	return out;
}

bool Execute(CBot &bot, const char *tool, const Json::Doc &doc, int args, edict_t *speaker, std::string &result)
{
	std::string err;

	if (CB_STRICMP(tool, "act") == 0)
	{
		int steps = doc.Member(args, "steps");
		int count = doc.Count(steps);
		if (doc.TypeOf(steps) != Json::T_ARRAY || count <= 0)
		{
			result = "error: act needs a non-empty 'steps' array";
			return false;
		}
		if (count > BotDirective::kMaxSteps)
			count = BotDirective::kMaxSteps;

		BotDirective::Step list[BotDirective::kMaxSteps];
		for (int i = 0; i < count; ++i)
		{
			if (!ReadStep(doc, doc.Element(steps, i), speaker, list[i], err))
			{
				char where[32];
				snprintf(where, sizeof(where), "error: step %d: ", i + 1);
				result = where + err;
				return false;
			}
		}
		BotDirective::Set(bot, list, count, Seconds(doc, args, "seconds", 0.f));
		char msg[64];
		snprintf(msg, sizeof(msg), "ok: %d step(s) set", count);
		result = msg;
		return true;
	}

	if (CB_STRICMP(tool, "rules") == 0)
	{
		BotDirective::SetRules(bot, doc.GetBool(args, "melee_only", false), doc.GetBool(args, "hold_fire", false),
			doc.GetBool(args, "no_auto_objective", false), Seconds(doc, args, "seconds", 0.f));
		result = "ok: rules set";
		return true;
	}

	if (CB_STRICMP(tool, "alert") == 0)
	{
		BotDirective::Target target;
		if (!ResolveTarget(doc.GetStr(args, "target").c_str(), speaker, target, err))
		{
			result = "error: " + err;
			return false;
		}
		edict_t *ent = target.ent.Get();
		BotDirective::Alert(bot, ent ? ent->v.origin + ent->v.view_ofs : target.point,
			Seconds(doc, args, "seconds", kAlertSeconds));
		result = "ok: alerted";
		return true;
	}

	if (CB_STRICMP(tool, "release") == 0)
	{
		BotDirective::Release(bot, "released by order");
		result = "ok: released";
		return true;
	}

	if (CB_STRICMP(tool, "say") == 0 || CB_STRICMP(tool, "say_team") == 0)
		return Say(bot, CB_STRICMP(tool, "say") == 0 ? "say" : "say_team", doc, args, result);

	if (CB_STRICMP(tool, "speak") == 0)
	{
		std::string text = doc.GetStr(args, "text");
		if (BrainVoice::Speak(bot, text.c_str(), false, result))
			return true;
		std::string why = result;
		bool ok = Say(bot, "say_team", doc, args, result);
		result = why + ", wrote it in team chat instead: " + result;
		return ok;
	}

	if (CB_STRICMP(tool, "radio") == 0)
	{
		std::string command = doc.GetStr(args, "command");
		const RadioInfo *list = nullptr;
		int count = g_pGameProfile ? g_pGameProfile->RadioCommands(list) : 0;
		for (int i = 0; i < count; ++i)
		{
			if (CB_STRICMP(list[i].command, command.c_str()) != 0)
				continue;
			if (!BotDirective::Queue(bot, list[i].command))
			{
				result = "error: too many queued commands";
				return false;
			}
			result = std::string("ok: radio ") + list[i].command;
			return true;
		}
		result = "error: '" + command + "' is not a radio command in this bot's game";
		return false;
	}

	result = std::string("error: unknown tool '") + tool + "'";
	return false;
}

}
