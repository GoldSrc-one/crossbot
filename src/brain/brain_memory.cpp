#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include "../game/crossplay.h"
#include "brain_memory.h"
#include "json.h"

namespace
{
	constexpr size_t kMaxNotes = 200;
	constexpr size_t kMaxText = 200;

	struct Note
	{
		long long when = 0;
		std::string map;
		int round = 0;
		std::string players;
		std::string text;
	};

	bool g_loaded = false;
	std::vector<Note> g_notes;

	std::string Path()
	{
		char path[512];
		snprintf(path, sizeof(path), "%s/addons/crossbot/brain_memory.jsonl", XPlay::HyperDir());
		return path;
	}

	std::string Line(const Note &n)
	{
		char head[64];
		snprintf(head, sizeof(head), "{\"when\":%lld,\"round\":%d,", n.when, n.round);
		return std::string(head) + "\"map\":" + Json::Quote(n.map.c_str()) + ",\"players\":" +
			Json::Quote(n.players.c_str()) + ",\"text\":" + Json::Quote(n.text.c_str()) + "}";
	}

	void Load()
	{
		if (g_loaded)
			return;
		g_loaded = true;
		FILE *f = fopen(Path().c_str(), "rb");
		if (!f)
			return;
		char buf[2048];
		while (fgets(buf, sizeof(buf), f))
		{
			Json::Doc doc;
			if (!doc.Parse(buf, strlen(buf)) || doc.TypeOf(doc.Root()) != Json::T_OBJECT)
				continue;
			int root = doc.Root();
			Note n;
			n.when = atoll(doc.GetStr(root, "when", "0").c_str());
			n.round = static_cast<int>(doc.GetNum(root, "round", 0.f));
			n.map = doc.GetStr(root, "map");
			n.players = doc.GetStr(root, "players");
			n.text = doc.GetStr(root, "text");
			if (!n.text.empty())
				g_notes.push_back(n);
		}
		fclose(f);
		if (g_notes.size() > kMaxNotes)
			g_notes.erase(g_notes.begin(), g_notes.end() - kMaxNotes);
	}

	void Rewrite()
	{
		FILE *f = fopen(Path().c_str(), "wb");
		if (!f)
			return;
		for (const Note &n : g_notes)
			fprintf(f, "%s\n", Line(n).c_str());
		fclose(f);
	}

	std::string Age(long long when)
	{
		long long d = static_cast<long long>(time(nullptr)) - when;
		char buf[32];
		if (d < 120)
			snprintf(buf, sizeof(buf), "%llds ago", d);
		else if (d < 7200)
			snprintf(buf, sizeof(buf), "%lldmin ago", d / 60);
		else if (d < 172800)
			snprintf(buf, sizeof(buf), "%lldh ago", d / 3600);
		else
			snprintf(buf, sizeof(buf), "%lldd ago", d / 86400);
		return buf;
	}

	bool Mentions(const Note &n, const char *name)
	{
		if (!name || !name[0])
			return false;
		std::string lower = n.players + " " + n.text;
		std::string want = name;
		std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
		std::transform(want.begin(), want.end(), want.begin(), ::tolower);
		return lower.find(want) != std::string::npos;
	}
}

namespace BrainMemory
{

void Add(const char *text, const char *players, const char *map, int round)
{
	Load();
	Note n;
	n.when = static_cast<long long>(time(nullptr));
	n.map = map ? map : "";
	n.round = round;
	n.players = players ? players : "";
	n.text = text ? text : "";
	if (n.text.size() > kMaxText)
		n.text.resize(kMaxText);
	if (n.text.empty())
		return;
	g_notes.push_back(n);

	if (g_notes.size() > kMaxNotes)
	{
		g_notes.erase(g_notes.begin(), g_notes.end() - kMaxNotes);
		Rewrite();
		return;
	}
	if (FILE *f = fopen(Path().c_str(), "ab"))
	{
		fprintf(f, "%s\n", Line(n).c_str());
		fclose(f);
	}
}

std::vector<std::string> Select(const char *speaker, const char *map, int max)
{
	Load();
	std::vector<const Note *> picked;
	auto pick = [&](auto pred) {
		for (size_t i = g_notes.size(); i-- > 0 && static_cast<int>(picked.size()) < max;)
		{
			const Note *n = &g_notes[i];
			if (std::find(picked.begin(), picked.end(), n) == picked.end() && pred(*n))
				picked.push_back(n);
		}
	};
	pick([&](const Note &n) { return Mentions(n, speaker); });
	pick([&](const Note &n) { return map && n.map == map; });
	pick([](const Note &) { return true; });

	std::vector<std::string> out;
	for (const Note *n : picked)
	{
		std::string line = "[" + Age(n->when) + ", " + n->map + "]";
		if (!n->players.empty())
			line += " (" + n->players + ")";
		out.push_back(line + " " + n->text);
	}
	return out;
}

int Count()
{
	Load();
	return static_cast<int>(g_notes.size());
}

void Print(int count)
{
	std::vector<std::string> lines = Select(nullptr, nullptr, count > 0 ? count : 20);
	for (const std::string &l : lines)
	{
		std::string msg = "CrossBot: memory " + l + "\n";
		SERVER_PRINT(msg.c_str());
	}
	char sum[64];
	snprintf(sum, sizeof(sum), "CrossBot: %d memory note(s)\n", Count());
	SERVER_PRINT(sum);
}

}
