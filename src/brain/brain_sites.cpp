#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../game/crossplay.h"
#include "../game/objectives.h"
#include "brain_sites.h"

namespace
{
	constexpr float kMatchRadius = 1500.f;
	constexpr float kPlaceRadius = 900.f;
	constexpr float kHeightWeight = 2.f;
	constexpr int kPlaceTries = 4;

	struct Mark
	{
		std::string name;
		Vector point;
	};

	bool g_loaded = false;
	std::vector<std::string> g_labels;
	std::vector<bool> g_known;
	std::vector<Mark> g_marks;
	std::vector<std::string> g_names;
	std::vector<Vector> g_centers;

	std::string SiteLetter(const std::string &name)
	{
		static const char *const kPrefixes[] = { "Bombsite ", "Site " };
		for (const char *prefix : kPrefixes)
		{
			size_t n = strlen(prefix);
			if (name.size() > n && CB_STRNICMP(name.c_str(), prefix, n) == 0 && name.find(' ', n) == std::string::npos)
				return name.substr(n);
		}
		return std::string();
	}

	void Load()
	{
		g_marks.clear();
		char path[512];
		snprintf(path, sizeof(path), "%s/addons/crossbot/places.txt", XPlay::HyperDir());
		const char *map = STRING(gpGlobals->mapname);
		FILE *f = fopen(path, "rb");
		if (!f)
			return;
		char line[256];
		while (fgets(line, sizeof(line), f))
		{
			char name[64];
			float x = 0.f, y = 0.f, z = 0.f;
			int used = 0;
			if (line[0] == '#' || sscanf(line, "%63s %f %f %f %n", name, &x, &y, &z, &used) != 4 ||
				CB_STRICMP(name, map) != 0)
				continue;
			std::string place = line + used;
			while (!place.empty() && isspace(static_cast<unsigned char>(place.back())))
				place.pop_back();
			if (!place.empty())
				g_marks.push_back({ place, Vector(x, y, z) });
		}
		fclose(f);
	}

	void BuildPlaces()
	{
		g_names.clear();
		g_centers.clear();
		for (const Mark &m : g_marks)
		{
			if (std::find(g_names.begin(), g_names.end(), m.name) == g_names.end())
				g_names.push_back(m.name);
		}
		for (const std::string &name : g_names)
		{
			Vector sum(0, 0, 0);
			int count = 0;
			for (const Mark &m : g_marks)
			{
				if (m.name == name)
				{
					sum = sum + m.point;
					++count;
				}
			}
			Vector mean = sum / static_cast<float>(count);
			Vector best = mean;
			float bestDist = 1e30f;
			for (const Mark &m : g_marks)
			{
				float d = (m.point - mean).Length();
				if (m.name == name && d < bestDist)
				{
					bestDist = d;
					best = m.point;
				}
			}
			g_centers.push_back(best);
		}
	}

	void BuildSites()
	{
		const std::vector<Objectives::Site> &sites = Objectives::Get().sites;
		g_labels.assign(sites.size(), std::string());
		g_known.assign(sites.size(), false);

		std::vector<Mark> letters;
		for (size_t i = 0; i < g_names.size(); ++i)
		{
			std::string letter = SiteLetter(g_names[i]);
			if (!letter.empty())
				letters.push_back({ letter, g_centers[i] });
		}

		std::vector<bool> taken(letters.size(), false);
		for (size_t s = 0; s < sites.size(); ++s)
		{
			int best = -1;
			float bestDist = kMatchRadius;
			for (size_t m = 0; m < letters.size(); ++m)
			{
				float d = (letters[m].point - sites[s].point).Length();
				if (!taken[m] && d < bestDist)
				{
					best = static_cast<int>(m);
					bestDist = d;
				}
			}
			if (best >= 0)
			{
				taken[static_cast<size_t>(best)] = true;
				g_labels[s] = letters[static_cast<size_t>(best)].name;
				g_known[s] = true;
			}
		}

		char fallback[16];
		for (size_t s = 0; s < sites.size(); ++s)
		{
			if (g_known[s])
				continue;
			snprintf(fallback, sizeof(fallback), "#%zu", s);
			g_labels[s] = fallback;
		}
	}

	void Ensure()
	{
		if (!g_loaded)
		{
			g_loaded = true;
			Load();
			BuildPlaces();
			BuildSites();
		}
		else if (g_labels.size() != Objectives::Get().sites.size())
		{
			BuildSites();
		}
	}

	float Metric(const Vector &a, const Vector &b)
	{
		Vector d = a - b;
		d.z *= kHeightWeight;
		return d.Length();
	}

	bool Clear(const Vector &a, const Vector &b)
	{
		TraceResult tr;
		Vector lift(0, 0, 16);
		TRACE_LINE(a + lift, b + lift, ignore_monsters, nullptr, &tr);
		return tr.flFraction >= 1.f && !tr.fStartSolid;
	}
}

namespace BrainSites
{

void Forget()
{
	g_loaded = false;
	g_labels.clear();
	g_known.clear();
	g_marks.clear();
	g_names.clear();
	g_centers.clear();
}

std::string Label(int site)
{
	Ensure();
	if (site < 0 || site >= static_cast<int>(g_labels.size()))
		return "?";
	return g_labels[static_cast<size_t>(site)];
}

bool Labeled(int site)
{
	Ensure();
	return site >= 0 && site < static_cast<int>(g_known.size()) && g_known[static_cast<size_t>(site)];
}

int Find(const char *label)
{
	Ensure();
	if (!label || !label[0])
		return -1;
	if (label[0] == '#')
		label++;
	for (size_t s = 0; s < g_labels.size(); ++s)
	{
		const char *l = g_labels[s].c_str();
		if (l[0] == '#')
			l++;
		if (CB_STRICMP(l, label) == 0)
			return static_cast<int>(s);
	}
	if (isdigit(static_cast<unsigned char>(label[0])))
	{
		int index = atoi(label);
		if (index >= 0 && index < static_cast<int>(g_labels.size()))
			return index;
	}
	return -1;
}

std::string PlaceAt(const Vector &p)
{
	Ensure();
	std::vector<std::pair<float, size_t>> hits;
	for (size_t i = 0; i < g_marks.size(); ++i)
	{
		float d = Metric(g_marks[i].point, p);
		if (d < kPlaceRadius)
			hits.push_back({ d, i });
	}
	if (hits.empty())
		return std::string();
	std::sort(hits.begin(), hits.end());
	for (size_t i = 0; i < hits.size() && i < static_cast<size_t>(kPlaceTries); ++i)
	{
		if (Clear(p, g_marks[hits[i].second].point))
			return g_marks[hits[i].second].name;
	}
	return hits[0].first < kPlaceRadius * 0.5f ? g_marks[hits[0].second].name : std::string();
}

bool FindPlace(const char *name, Vector &point, std::string &matched)
{
	Ensure();
	if (!name || !name[0])
		return false;
	if (CB_STRNICMP(name, "place:", 6) == 0)
		name += 6;
	while (*name == ' ')
		++name;
	int found = -1;
	for (size_t i = 0; i < g_names.size(); ++i)
	{
		if (CB_STRICMP(g_names[i].c_str(), name) == 0)
			found = static_cast<int>(i);
	}
	if (found < 0)
	{
		std::string want = name;
		std::transform(want.begin(), want.end(), want.begin(), [](unsigned char c) { return static_cast<char>(tolower(c)); });
		for (size_t i = 0; i < g_names.size(); ++i)
		{
			std::string have = g_names[i];
			std::transform(have.begin(), have.end(), have.begin(), [](unsigned char c) { return static_cast<char>(tolower(c)); });
			if (have.find(want) == std::string::npos)
				continue;
			if (found >= 0)
				return false;
			found = static_cast<int>(i);
		}
	}
	if (found < 0)
		return false;
	point = g_centers[static_cast<size_t>(found)];
	matched = g_names[static_cast<size_t>(found)];
	return true;
}

std::string PlaceNames()
{
	Ensure();
	std::string out;
	for (const std::string &name : g_names)
		out += (out.empty() ? "" : ", ") + name;
	return out;
}

}
