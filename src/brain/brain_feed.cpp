#include <extdll.h>
#include <meta_api.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "../bot/bot_manager.h"
#include "../game/crossplay.h"
#include "../game/ent_handle.h"
#include "../game/fake_cmd.h"
#include "../game/game_profile.h"
#include "../game/teams.h"
#include "brain_feed.h"

namespace
{
	constexpr size_t kMaxEvents = 256;
	constexpr float kRadioDedupe = 0.5f;
	constexpr float kRoundStartRadio = 2.f;
	constexpr int kMaxCaptureBytes = 4;
	constexpr int kCaptureString = 64;

	const char *const kKindNames[BrainFeed::EV_COUNT] = {
		"chat", "radio", "kill", "objective", "round", "score", "presence", "voice",
	};

	std::deque<BrainFeed::Event> g_events;
	unsigned int g_serial = 0;
	bool g_roundStart = false;
	int g_round = 0;
	float g_roundTime = 0.f;
	unsigned int g_roundSerial = 0;


	int g_sendAudioId = 0;
	int g_handSignalId = 0;

	int g_captureType = 0;
	int g_bytes[kMaxCaptureBytes] = {};
	int g_byteCount = 0;
	char g_string[kCaptureString] = {};

	int g_lastRadioSender = 0;
	int g_lastRadioCode = 0;
	char g_lastRadioSound[kCaptureString] = {};
	float g_lastRadioTime = 0.f;

	unsigned short g_usVoiceIndex = 0;
	unsigned short g_gerVoiceIndex = 0;

	void Copy(char *dst, size_t size, const char *src)
	{
		snprintf(dst, size, "%s", src ? src : "");
	}

	void FillActor(BrainFeed::Event &e, edict_t *ent)
	{
		if (!ent || ent->free)
			return;
		e.entIndex = g_engfuncs.pfnIndexOfEdict(ent);
		e.userid = g_engfuncs.pfnGetPlayerUserId(ent);
		e.team = Teams::TeamOf(ent);
		e.game = XPlay::OwnerOf(ent);
		e.bot = BotManager::FindBot(ent) != nullptr;
		e.human = BrainFeed::IsHuman(ent);
		e.dead = ent->v.deadflag != DEAD_NO || ent->v.health <= 0.f;
		Copy(e.who, sizeof(e.who), CBotPlayerNameOf(ent));
	}

	void Add(BrainFeed::Event &e)
	{
		e.serial = ++g_serial;
		e.time = gpGlobals->time;
		g_events.push_back(e);
		while (g_events.size() > kMaxEvents)
			g_events.pop_front();
	}

	edict_t *PlayerByUserId(int userid)
	{
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (CBotIsLivePlayer(ent) && g_engfuncs.pfnGetPlayerUserId(ent) == userid)
				return ent;
		}
		return nullptr;
	}

	const RadioInfo *FindRadio(int game, const char *sound, int code)
	{
		const IGameProfile *profile = XPlay::ProfileFor(game);
		if (!profile)
			return nullptr;
		const RadioInfo *list = nullptr;
		int count = profile->RadioCommands(list);
		for (int i = 0; i < count; ++i)
		{
			if (sound && list[i].sound && strcmp(list[i].sound, sound) == 0)
				return &list[i];
			if (!sound && code && list[i].code == code)
				return &list[i];
		}
		return nullptr;
	}

	void AddRadio(edict_t *sender, const char *sound, int code)
	{
		if (!sender || !CBotIsLivePlayer(sender))
			return;

		int index = g_engfuncs.pfnIndexOfEdict(sender);
		float now = gpGlobals->time;
		bool same = index == g_lastRadioSender && code == g_lastRadioCode &&
			strcmp(sound ? sound : "", g_lastRadioSound) == 0;
		if (same && now - g_lastRadioTime < kRadioDedupe && now >= g_lastRadioTime)
			return;
		g_lastRadioSender = index;
		g_lastRadioCode = code;
		Copy(g_lastRadioSound, sizeof(g_lastRadioSound), sound);
		g_lastRadioTime = now;

		BrainFeed::Event e;
		FillActor(e, sender);
		const RadioInfo *info = FindRadio(e.game, sound, code);
		if (!info)
			return;
		e.kind = BrainFeed::EV_RADIO;
		e.teamOnly = true;
		e.automatic = g_roundTime > 0.f && now - g_roundTime >= 0.f && now - g_roundTime < kRoundStartRadio;
		Copy(e.text, sizeof(e.text), info->text);
		Add(e);
	}

	bool ParsePlayerToken(const std::string &seg, std::string &name, int &uid, std::string &team)
	{
		if (seg.size() < 7 || seg.back() != '>')
			return false;
		size_t teamOpen = seg.rfind('<');
		if (teamOpen == std::string::npos || teamOpen == 0 || seg[teamOpen - 1] != '>')
			return false;
		size_t authOpen = seg.rfind('<', teamOpen - 1);
		if (authOpen == std::string::npos || authOpen == 0 || seg[authOpen - 1] != '>')
			return false;
		size_t uidOpen = seg.rfind('<', authOpen - 1);
		if (uidOpen == std::string::npos)
			return false;

		std::string uidText = seg.substr(uidOpen + 1, authOpen - uidOpen - 2);
		char *end = nullptr;
		long v = strtol(uidText.c_str(), &end, 10);
		if (uidText.empty() || !end || *end)
			return false;

		uid = static_cast<int>(v);
		name = seg.substr(0, uidOpen);
		team = seg.substr(teamOpen + 1, seg.size() - teamOpen - 2);
		return true;
	}

	std::string Humanize(const char *line, int &actorUid)
	{
		std::string out;
		actorUid = 0;
		const char *p = line;
		while (*p)
		{
			if (*p != '"')
			{
				if (*p != '\n' && *p != '\r')
					out += *p;
				++p;
				continue;
			}

			const char *close = strchr(p + 1, '"');
			if (!close)
			{
				out += p + 1;
				break;
			}

			std::string seg(p + 1, close);
			std::string name, team;
			int uid = 0;
			if (ParsePlayerToken(seg, name, uid, team))
			{
				if (uid < 0 || name.empty())
				{
					out += "world";
				}
				else
				{
					out += name;
					if (!team.empty())
						out += "[" + team + "]";
					if (!actorUid)
						actorUid = uid;
				}
			}
			else
			{
				out += seg;
			}
			p = close + 1;
		}
		return out;
	}

	bool Has(const char *line, const char *what)
	{
		return strstr(line, what) != nullptr;
	}
}

namespace BrainFeed
{

int Round()
{
	return g_round;
}

float RoundTime()
{
	return g_roundTime;
}

unsigned int RoundSerial()
{
	return g_roundSerial;
}

void Reset()
{
	g_events.clear();
	g_roundStart = false;
	g_round = 0;
	g_roundTime = 0.f;
	g_roundSerial = 0;
	g_captureType = 0;
	g_lastRadioSender = 0;
	g_lastRadioTime = 0.f;
}

void OnRegUserMsg(const char *name, int id)
{
	if (!name || id <= 0)
		return;
	if (strcmp(name, "SendAudio") == 0)
		g_sendAudioId = id;
	else if (strcmp(name, "HandSignal") == 0)
		g_handSignalId = id;
}

void OnMessageBegin(int dest, int type, edict_t *ed)
{
	(void)dest;
	(void)ed;
	g_captureType = 0;
	if (type <= 0 || (type != g_sendAudioId && type != g_handSignalId))
		return;
	g_captureType = type;
	g_byteCount = 0;
	g_string[0] = '\0';
}

void OnWriteByte(int value)
{
	if (g_captureType && g_byteCount < kMaxCaptureBytes)
		g_bytes[g_byteCount++] = value;
}

void OnWriteShort(int value)
{
	(void)value;
}

void OnWriteString(const char *value)
{
	if (g_captureType && !g_string[0])
		Copy(g_string, sizeof(g_string), value);
}

void OnMessageEnd()
{
	int type = g_captureType;
	g_captureType = 0;
	if (!type || g_byteCount < 1)
		return;

	edict_t *sender = g_bytes[0] > 0 ? g_engfuncs.pfnPEntityOfEntIndex(g_bytes[0]) : nullptr;
	if (type == g_sendAudioId)
	{
		if (strncmp(g_string, "%!MRAD_", 7) == 0 && strcmp(g_string, "%!MRAD_FIREINHOLE") != 0)
			AddRadio(sender, g_string, 0);
	}
	else if (type == g_handSignalId && g_byteCount >= 2)
	{
		AddRadio(sender, nullptr, kRadioSignalBase + g_bytes[1]);
	}
}

void OnPrecacheEvent(const char *name, unsigned short index)
{
	if (!name)
		return;
	if (strcmp(name, "events/misc/usvoice.sc") == 0)
		g_usVoiceIndex = index;
	else if (strcmp(name, "events/misc/gervoice.sc") == 0)
		g_gerVoiceIndex = index;
}

void OnPlaybackEvent(const edict_t *invoker, unsigned short eventindex, int iparam1)
{
	if (!eventindex || (eventindex != g_usVoiceIndex && eventindex != g_gerVoiceIndex) || iparam1 <= 0)
		return;
	AddRadio(const_cast<edict_t *>(invoker), nullptr, iparam1);
}

bool IsHuman(const edict_t *ent)
{
	if (!ent || ent->free || (ent->v.flags & FL_FAKECLIENT) != 0)
		return false;
	if (BotManager::FindBot(const_cast<edict_t *>(ent)))
		return false;
	const char *auth = g_engfuncs.pfnGetPlayerAuthId(const_cast<edict_t *>(ent));
	return !auth || strcmp(auth, "BOT") != 0;
}

void OnClientCommand(edict_t *ent)
{
	if (!ent || !CBotIsLivePlayer(ent))
		return;
	const char *cmd = CMD_ARGV(0);
	if (!cmd)
		return;
	bool team = strcmp(cmd, "say_team") == 0;
	if (!team && strcmp(cmd, "say") != 0)
		return;
	NoteChat(ent, team, CMD_ARGS());
}

void NoteChat(edict_t *ent, bool team, const char *args, Kind kind)
{
	if (!ent || !CBotIsLivePlayer(ent))
		return;
	std::string text = args ? args : "";
	if (text.size() >= 2 && text.front() == '"' && text.back() == '"')
		text = text.substr(1, text.size() - 2);
	while (!text.empty() && (text.back() == ' ' || text.back() == '\n'))
		text.pop_back();
	if (text.empty())
		return;

	Event e;
	FillActor(e, ent);
	e.kind = kind;
	e.teamOnly = team;
	Copy(e.text, sizeof(e.text), text.c_str());
	Add(e);
}

void OnLog(const char *line)
{
	if (!line || !line[0])
		return;
	if (Has(line, "\" say \"") || Has(line, "\" say_team \"") || Has(line, "Rcon:") ||
		Has(line, "Server cvar") || Has(line, "Log file") || Has(line, "Server say"))
		return;

	Kind kind = EV_COUNT;
	if (Has(line, "World triggered \"Round_Start\""))
	{
		kind = EV_ROUND;
		g_roundStart = true;
		++g_round;
		g_roundTime = gpGlobals->time;
		g_roundSerial = g_serial + 1;
	}
	else if (strncmp(line, "World triggered", 15) == 0)
		kind = EV_ROUND;
	else if (Has(line, "\" killed \"") || Has(line, "\" committed suicide"))
		kind = EV_KILL;
	else if (Has(line, "\" scored \""))
		kind = EV_SCORE;
	else if (Has(line, " triggered "))
		kind = EV_OBJECTIVE;
	else if (Has(line, "\" joined team \"") || Has(line, "\" entered the game") ||
		Has(line, "\" disconnected") || Has(line, "\" changed role to \""))
		kind = EV_PRESENCE;

	if (kind == EV_COUNT)
		return;

	int uid = 0;
	std::string text = Humanize(line, uid);
	Event e;
	edict_t *actor = uid > 0 ? PlayerByUserId(uid) : nullptr;
	if (actor)
		FillActor(e, actor);
	e.kind = kind;
	Copy(e.text, sizeof(e.text), text.c_str());
	Add(e);
}

bool TakeRoundStart()
{
	bool started = g_roundStart;
	g_roundStart = false;
	return started;
}

const std::deque<Event> &Events()
{
	return g_events;
}

unsigned int LastSerial()
{
	return g_serial;
}

bool IsComms(Kind kind)
{
	return kind == EV_CHAT || kind == EV_RADIO || kind == EV_VOICE;
}

const char *KindName(Kind kind)
{
	return kind < EV_COUNT ? kKindNames[kind] : "?";
}

void Print(int count)
{
	if (count <= 0)
		count = 30;
	size_t first = g_events.size() > static_cast<size_t>(count) ? g_events.size() - count : 0;
	float now = gpGlobals->time;
	for (size_t i = first; i < g_events.size(); ++i)
	{
		const Event &e = g_events[i];
		char line[320];
		snprintf(line, sizeof(line), "CrossBot: feed #%u %6.1fs %-9s team=%d%s %s%s%s%s\n", e.serial, e.time - now,
			KindName(e.kind), e.team, e.teamOnly ? "(team)" : "", e.who[0] ? e.who : "",
			e.who[0] ? (e.human ? " (human)" : " (bot)") : "", e.who[0] ? ": " : "", e.text);
		SERVER_PRINT(line);
	}
	char sum[160];
	snprintf(sum, sizeof(sum),
		"CrossBot: feed has %zu event(s), last #%u, SendAudio=%d HandSignal=%d usvoice=%u gervoice=%u\n",
		g_events.size(), g_serial, g_sendAudioId, g_handSignalId, g_usVoiceIndex, g_gerVoiceIndex);
	SERVER_PRINT(sum);
}

}
