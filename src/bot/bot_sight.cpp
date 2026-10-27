#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef _WIN32
#include <io.h>
#else
#include <sys/file.h>
#include <unistd.h>
#endif

#include "bot_sight.h"
#include "../game/cvars.h"
#include "bot_perf.h"
#include "../game/crossplay.h"
#include "../game/ent_handle.h"
#include "../nav/sight_coverage.h"

namespace
{

constexpr char kCacheMagic[4] = { 'C', 'B', 'C', 'V' };
constexpr int kMaxPanes = 4;

std::vector<edict_t *> g_bodies;

bool BodySolid(const edict_t *ent)
{
	return ent && !ent->free && (ent->v.solid == SOLID_BBOX || ent->v.solid == SOLID_SLIDEBOX);
}

bool SegmentHitsBox(const Vector &a, const Vector &b, const Vector &mins, const Vector &maxs)
{
	float enter = 0.f;
	float leave = 1.f;
	for (int axis = 0; axis < 3; ++axis)
	{
		float d = b[axis] - a[axis];
		float lo = mins[axis];
		float hi = maxs[axis];
		if (std::fabs(d) < 1e-6f)
		{
			if (a[axis] < lo || a[axis] > hi)
				return false;
			continue;
		}
		float t0 = (lo - a[axis]) / d;
		float t1 = (hi - a[axis]) / d;
		if (t0 > t1)
			std::swap(t0, t1);
		if (t0 > enter)
			enter = t0;
		if (t1 < leave)
			leave = t1;
		if (enter > leave)
			return false;
	}
	return true;
}

bool BodyBetween(const Vector &from, const Vector &to, const edict_t *skip, const edict_t *target)
{
	for (int i = 1; i <= gpGlobals->maxClients; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (ent == skip || ent == target || !BodySolid(ent))
			continue;
		if (SegmentHitsBox(from, to, ent->v.absmin, ent->v.absmax))
			return true;
	}
	for (edict_t *ent : g_bodies)
	{
		if (ent == skip || ent == target || !BodySolid(ent))
			continue;
		if (SegmentHitsBox(from, to, ent->v.absmin, ent->v.absmax))
			return true;
	}
	return false;
}
constexpr float kZeroSize = 1.f;

struct Request
{
	std::string key;
	bool studio = false;
	int submodel = 0;
	std::string model;
	int sequence = 0;
	int body = 0;
	int skin = 0;
	int rendermode = 0;
	float renderamt = 0.f;
};

struct Pending
{
	CBotEntHandle ent;
	std::string key;
	bool studio = false;
};

struct Built
{
	std::unordered_map<std::string, std::vector<SightCoverage::Model>> models;
	std::vector<std::string> notes;
	int hits = 0;
	int misses = 0;
	double seconds = 0.0;
	bool wrote = false;
};

struct Occluder
{
	CBotEntHandle ent;
	const std::vector<SightCoverage::Model> *chunks = nullptr;
	SightCoverage::Placed placed;
	bool studio = false;
};

std::unique_ptr<Built> g_built;
std::thread g_worker;
std::atomic<bool> g_done(false);
std::atomic<bool> g_cancel(false);
std::vector<Pending> g_pending;
std::vector<Occluder> g_occluders;
std::vector<int> g_byIndex;
bool g_ready = false;

bool Describe(edict_t *ent, Request &req, bool &studio)
{
	if (!ent || ent->free || ent->v.modelindex <= 0)
		return false;
	if ((ent->v.flags & (FL_CLIENT | FL_MONSTER)) != 0)
		return false;
	if (ent->v.movetype == MOVETYPE_FOLLOW || (ent->v.effects & EF_NODRAW) != 0)
		return false;

	const char *model = STRING(ent->v.model);
	if (!model || !*model)
		return false;

	req.rendermode = ent->v.rendermode;
	req.renderamt = ent->v.renderamt;
	if (SightCoverage::RenderOpacity(req.rendermode, req.renderamt) <= 0.f)
		return false;

	char tail[64];
	snprintf(tail, sizeof(tail), "|%d|%d", req.rendermode, static_cast<int>(req.renderamt));

	if (model[0] == '*')
	{
		if (ent->v.solid != SOLID_NOT && req.rendermode == SightCoverage::kRenderNormal)
			return false;
		req.submodel = atoi(model + 1);
		req.key = std::string("b|") + model + tail;
		studio = false;
		return req.submodel > 0;
	}

	size_t len = strlen(model);
	if (len < 4 || (strcmp(model + len - 4, ".mdl") != 0 && strcmp(model + len - 4, ".MDL") != 0))
		return false;
	if (ent->v.movetype != MOVETYPE_NONE)
		return false;
	Vector size = ent->v.maxs - ent->v.mins;
	if (ent->v.solid != SOLID_NOT && size.Length() >= kZeroSize)
		return false;

	req.studio = true;
	req.model = model;
	req.sequence = ent->v.sequence;
	req.body = ent->v.body;
	req.skin = ent->v.skin;
	char mid[64];
	snprintf(mid, sizeof(mid), "|%d|%d|%d", req.sequence, req.body, req.skin);
	req.key = std::string("s|") + model + mid + tail;
	studio = true;
	return true;
}

typedef std::unordered_map<std::string, std::vector<SightCoverage::Model>> ModelMap;

bool LockCache(FILE *f, bool exclusive)
{
#ifdef _WIN32
	OVERLAPPED ov = {};
	return LockFileEx(reinterpret_cast<HANDLE>(_get_osfhandle(_fileno(f))), exclusive ? LOCKFILE_EXCLUSIVE_LOCK : 0, 0,
		MAXDWORD, MAXDWORD, &ov) != 0;
#else
	return flock(fileno(f), exclusive ? LOCK_EX : LOCK_SH) == 0;
#endif
}

void UnlockCache(FILE *f)
{
#ifdef _WIN32
	OVERLAPPED ov = {};
	UnlockFileEx(reinterpret_cast<HANDLE>(_get_osfhandle(_fileno(f))), 0, MAXDWORD, MAXDWORD, &ov);
#else
	flock(fileno(f), LOCK_UN);
#endif
}

bool HeaderValid(FILE *f)
{
	char magic[4] = {};
	uint32_t version = 0;
	std::fseek(f, 0, SEEK_SET);
	return std::fread(magic, 1, 4, f) == 4 && std::memcmp(magic, kCacheMagic, 4) == 0 &&
		std::fread(&version, sizeof(version), 1, f) == 1 && version == SightCoverage::kFormatVersion;
}

void ReadCache(const std::string &path, const std::unordered_set<std::string> &wanted, ModelMap &out)
{
	FILE *f = std::fopen(path.c_str(), "rb");
	if (!f)
		return;
	if (!LockCache(f, false))
	{
		std::fclose(f);
		return;
	}

	if (HeaderValid(f))
	{
		std::vector<uint8_t> body;
		for (;;)
		{
			uint32_t size = 0;
			uint16_t len = 0;
			if (std::fread(&size, sizeof(size), 1, f) != 1 || size < sizeof(len) ||
				std::fread(&len, sizeof(len), 1, f) != 1 || len > size - sizeof(len))
				break;
			std::string key(len, '\0');
			if (len > 0 && std::fread(&key[0], 1, len, f) != len)
				break;

			size_t rest = size - sizeof(len) - len;
			if (!wanted.count(key) || out.count(key))
			{
				if (std::fseek(f, static_cast<long>(rest), SEEK_CUR) != 0)
					break;
				continue;
			}

			body.resize(rest);
			if (rest > 0 && std::fread(body.data(), 1, rest, f) != rest)
				break;
			std::vector<SightCoverage::Model> chunks;
			if (SightCoverage::ParseChunks(body.data(), body.data() + body.size(), chunks))
				out[key] = std::move(chunks);
		}
	}

	UnlockCache(f);
	std::fclose(f);
}

void AppendCache(const std::string &path, const std::vector<std::pair<std::string, const std::vector<SightCoverage::Model> *>> &records)
{
	std::error_code ec;
	std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);

	FILE *f = std::fopen(path.c_str(), "a+b");
	if (!f)
		return;
	if (!LockCache(f, true))
	{
		std::fclose(f);
		return;
	}

	std::fseek(f, 0, SEEK_END);
	bool fresh = std::ftell(f) == 0 || !HeaderValid(f);
	std::fseek(f, 0, SEEK_END);
	if (fresh)
	{
#ifdef _WIN32
		_chsize(_fileno(f), 0);
#else
		ftruncate(fileno(f), 0);
#endif
		std::fseek(f, 0, SEEK_SET);
		uint32_t version = SightCoverage::kFormatVersion;
		std::fwrite(kCacheMagic, 1, 4, f);
		std::fwrite(&version, sizeof(version), 1, f);
	}

	std::vector<uint8_t> buffer;
	for (const auto &record : records)
	{
		buffer.clear();
		uint16_t len = static_cast<uint16_t>(record.first.size());
		buffer.insert(buffer.end(), reinterpret_cast<const uint8_t *>(&len),
			reinterpret_cast<const uint8_t *>(&len) + sizeof(len));
		buffer.insert(buffer.end(), record.first.begin(), record.first.end());
		SightCoverage::AppendChunks(buffer, *record.second);

		uint32_t size = static_cast<uint32_t>(buffer.size());
		std::fwrite(&size, sizeof(size), 1, f);
		std::fwrite(buffer.data(), 1, buffer.size(), f);
	}
	std::fflush(f);

	UnlockCache(f);
	std::fclose(f);
}

std::string Hex(uint32_t value)
{
	char buf[16];
	snprintf(buf, sizeof(buf), "%08x", value);
	return buf;
}

void Work(std::vector<Request> requests, std::string bspPath, std::string cachePath, Built *out)
{
	auto start = std::chrono::steady_clock::now();

	SightCoverage::Bsp bsp;
	if (!bsp.Open(bspPath))
	{
		out->notes.push_back("cannot read " + bspPath);
		g_done = true;
		return;
	}

	std::string root = SightCoverage::AssetRoot(bspPath);
	std::string game = std::filesystem::path(root).filename().string();
	std::string bspSum = Hex(bsp.Checksum());
	std::unordered_map<std::string, uint32_t> modelSums;

	std::vector<std::string> fileKeys(requests.size());
	std::unordered_set<std::string> wanted;
	for (size_t i = 0; i < requests.size(); ++i)
	{
		const Request &req = requests[i];
		if (req.studio)
		{
			auto it = modelSums.find(req.model);
			if (it == modelSums.end())
				it = modelSums.emplace(req.model, SightCoverage::StudioChecksum(root, req.model)).first;
			if (it->second == 0)
				continue;
			fileKeys[i] = "s|" + game + "|" + Hex(it->second) + req.key.substr(1);
		}
		else
		{
			fileKeys[i] = "b|" + bspSum + req.key.substr(1);
		}
		wanted.insert(fileKeys[i]);
	}

	ModelMap cached;
	ReadCache(cachePath, wanted, cached);

	SightCoverage::Stats stats;
	std::vector<std::pair<std::string, const std::vector<SightCoverage::Model> *>> fresh;
	for (size_t i = 0; i < requests.size(); ++i)
	{
		if (g_cancel)
		{
			g_done = true;
			return;
		}

		const Request &req = requests[i];
		auto hit = fileKeys[i].empty() ? cached.end() : cached.find(fileKeys[i]);
		if (hit != cached.end())
		{
			++out->hits;
			out->models[req.key] = std::move(hit->second);
			continue;
		}

		++out->misses;
		std::vector<SightCoverage::Model> chunks;
		if (req.studio)
		{
			std::string why;
			if (!SightCoverage::BuildStudio(root, req.model, req.sequence, nullptr, req.body, req.skin,
				req.rendermode, req.renderamt, chunks, stats, why))
				out->notes.push_back(req.model + ": " + why);
		}
		else
		{
			bsp.BuildBrush(req.submodel, req.rendermode, req.renderamt, chunks, stats);
		}

		std::vector<SightCoverage::Model> &stored = out->models[req.key];
		stored = std::move(chunks);
		if (!fileKeys[i].empty())
			fresh.emplace_back(fileKeys[i], &stored);
	}

	for (const std::string &name : bsp.Missing())
		out->notes.push_back("missing wad or texture " + name + " (treated as opaque)");

	if (!fresh.empty() && !g_cancel)
	{
		AppendCache(cachePath, fresh);
		out->wrote = true;
	}

	out->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
	g_done = true;
}
Occluder *Find(edict_t *ent)
{
	if (!ent)
		return nullptr;
	int index = g_engfuncs.pfnIndexOfEdict(ent);
	if (index <= 0 || index >= static_cast<int>(g_byIndex.size()) || g_byIndex[index] < 0)
		return nullptr;
	Occluder &occ = g_occluders[g_byIndex[index]];
	return occ.ent.Get() == ent ? &occ : nullptr;
}

float OccluderAlpha(Occluder &occ, edict_t *ent, const Vector &from, const Vector &to)
{
	if (std::memcmp(occ.placed.origin, &ent->v.origin.x, sizeof(float) * 3) != 0 ||
		std::memcmp(occ.placed.angles, &ent->v.angles.x, sizeof(float) * 3) != 0)
	{
		float origin[3] = { ent->v.origin.x, ent->v.origin.y, ent->v.origin.z };
		float angles[3] = { ent->v.angles.x, ent->v.angles.y, ent->v.angles.z };
		SightCoverage::Place(occ.placed, *occ.chunks, origin, angles, occ.studio);
	}

	float a[3] = { from.x, from.y, from.z };
	float b[3] = { to.x, to.y, to.z };
	return SightCoverage::Alpha(occ.placed, a, b);
}

}

void BotSight::Build(const char *mapName, const char *bspPath)
{
	Clear();
	if (!mapName || !bspPath || !*bspPath)
		return;

	std::vector<Request> unique;
	std::unordered_set<std::string> keys;
	for (int i = 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		Request req;
		bool studio = false;
		if (!Describe(ent, req, studio))
			continue;

		Pending pending;
		pending.ent.Set(ent);
		pending.key = req.key;
		pending.studio = studio;
		g_pending.push_back(pending);
		if (keys.insert(req.key).second)
			unique.push_back(req);
	}

	if (g_pending.empty())
		return;

	std::string cachePath = std::string(XPlay::HyperDir()) + "/addons/crossbot/coverage.cov";
	g_built.reset(new Built);
	g_done = false;
	g_cancel = false;
	g_worker = std::thread(Work, unique, std::string(bspPath), cachePath, g_built.get());
}

void BotSight::Clear()
{
	g_cancel = true;
	if (g_worker.joinable())
		g_worker.join();
	g_cancel = false;
	g_done = false;
	g_pending.clear();
	g_occluders.clear();
	g_byIndex.clear();
	g_built.reset();
	g_ready = false;
}

void BotSight::Poll()
{
	if (g_ready || !g_worker.joinable() || !g_done)
		return;
	g_worker.join();

	for (const Pending &pending : g_pending)
	{
		edict_t *ent = pending.ent.Get();
		if (!ent)
			continue;
		auto it = g_built->models.find(pending.key);
		if (it == g_built->models.end() || it->second.empty())
			continue;

		Occluder occ;
		occ.ent = pending.ent;
		occ.chunks = &it->second;
		occ.studio = pending.studio;
		float origin[3] = { ent->v.origin.x, ent->v.origin.y, ent->v.origin.z };
		float angles[3] = { ent->v.angles.x, ent->v.angles.y, ent->v.angles.z };
		SightCoverage::Place(occ.placed, it->second, origin, angles, pending.studio);
		g_occluders.push_back(std::move(occ));
	}

	g_byIndex.assign(static_cast<size_t>(gpGlobals->maxEntities), -1);
	for (size_t i = 0; i < g_occluders.size(); ++i)
	{
		int index = g_engfuncs.pfnIndexOfEdict(g_occluders[i].ent.ent);
		if (index > 0 && index < gpGlobals->maxEntities)
			g_byIndex[index] = static_cast<int>(i);
	}
	g_ready = true;

	char line[256];
	snprintf(line, sizeof(line),
		"CrossBot: sight coverage: %d occluder(s) from %d model(s), cache %d hit / %d built%s, %.2fs\n",
		static_cast<int>(g_occluders.size()), static_cast<int>(g_built->models.size()), g_built->hits,
		g_built->misses, g_built->wrote ? ", cache written" : "", g_built->seconds);
	SERVER_PRINT(line);
	for (const std::string &note : g_built->notes)
	{
		snprintf(line, sizeof(line), "CrossBot:   coverage note: %s\n", note.c_str());
		SERVER_PRINT(line);
	}
}

void BotSight::RefreshBodies()
{
	g_bodies.clear();
	for (int i = gpGlobals->maxClients + 1; i < gpGlobals->maxEntities; ++i)
	{
		edict_t *ent = g_engfuncs.pfnPEntityOfEntIndex(i);
		if (BodySolid(ent))
			g_bodies.push_back(ent);
	}
}

bool BotSight::Enabled()
{
	cvar_t *cvar = CBotCvar("crossbot_coverage");
	return !cvar || cvar->value != 0.f;
}

bool BotSight::Ready()
{
	return g_ready;
}

int BotSight::Count()
{
	return static_cast<int>(g_occluders.size());
}

float BotSight::Clarity(const Vector &from, const Vector &to, int monsters, edict_t *skip, edict_t *target,
	bool *covered)
{
	bool coverage = g_ready && Enabled();
	float clarity = 1.f;
	if (monsters == dont_ignore_monsters)
	{
		if (BodyBetween(from, to, skip, target))
			return 0.f;
		monsters = ignore_monsters;
	}
	Vector start = from;
	edict_t *pass = skip;

	for (int pane = 0;; ++pane)
	{
		TraceResult tr;
		TRACE_LINE(start, to, monsters, pass, &tr);
		if (tr.flFraction >= 1.f || (target && tr.pHit == target))
			break;
		if (!coverage || pane + 1 >= kMaxPanes)
			return 0.f;

		Occluder *occ = Find(tr.pHit);
		if (!occ)
			return 0.f;

		BOTPERF_SCOPE(ZONE_SN_COV);
		clarity *= 1.f - OccluderAlpha(*occ, tr.pHit, from, to);
		if (covered)
			*covered = true;
		if (clarity <= 0.f)
			return 0.f;

		start = tr.vecEndPos;
		pass = tr.pHit;
	}

	if (!coverage)
		return clarity;

	BOTPERF_SCOPE(ZONE_SN_COV);
	for (Occluder &occ : g_occluders)
	{
		edict_t *ent = occ.ent.Get();
		if (!ent || ent->v.solid != SOLID_NOT || (ent->v.effects & EF_NODRAW) != 0 || ent == target)
			continue;

		float alpha = OccluderAlpha(occ, ent, from, to);
		if (alpha <= 0.f)
			continue;
		if (covered)
			*covered = true;
		clarity *= 1.f - alpha;
		if (clarity <= 0.f)
			return 0.f;
	}

	return clarity;
}

void BotSight::Print()
{
	char line[320];
	for (size_t i = 0; i < g_occluders.size(); ++i)
	{
		const Occluder &occ = g_occluders[i];
		edict_t *ent = occ.ent.Get();
		if (!ent)
			continue;
		snprintf(line, sizeof(line),
			"CrossBot: occluder #%d %s %s solid=%d chunks=%d render=%d/%.0f box=(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)\n",
			g_engfuncs.pfnIndexOfEdict(ent), CBotClassOf(ent), STRING(ent->v.model), ent->v.solid,
			static_cast<int>(occ.chunks->size()), ent->v.rendermode, ent->v.renderamt,
			occ.placed.absmin[0], occ.placed.absmin[1], occ.placed.absmin[2],
			occ.placed.absmax[0], occ.placed.absmax[1], occ.placed.absmax[2]);
		SERVER_PRINT(line);
	}

	snprintf(line, sizeof(line), "CrossBot: %d occluder(s), coverage=%d ready=%d\n", Count(), Enabled() ? 1 : 0,
		g_ready ? 1 : 0);
	SERVER_PRINT(line);
}
