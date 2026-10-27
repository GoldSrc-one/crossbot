#include "blocking_entities.h"

#include "baked_blockers.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <unordered_map>

#include "clipnode_tree.h"
#include "hull_defs.h"
#include "polytope.h"

namespace navgen
{

namespace
{

constexpr double kOverlapEpsilon = 0.5;
constexpr double kMinPieceVolume = 1.0;
constexpr double kNullCarveTolerance = 1e-6;
constexpr size_t kMaxNullEvents = 20;
constexpr double kInflationReportRatio = 2.0;

constexpr int kSfBreakTriggerOnly = 1;
constexpr int kSfBreakDodExplosiveOnly = 8 | 64 | 256 | 512;
constexpr int kSfDoorStartOpen = 1;
constexpr int kSfDoorPassable = 8;
constexpr int kSfDoorUseOnly = 256;
constexpr int kSfPushOnce = 1;
constexpr int kSfPushStartOff = 2;
constexpr int kSfHurtStartOff = 2;
constexpr int kSfHurtNoClients = 8;
constexpr int kSfHurtOneTeam = 64 | 128;
constexpr double kLethalHurtDamage = 100.0;

constexpr int kSfEnvModelDropToFloor = 2;
constexpr double kDropToFloorDistance = 256.0;
constexpr double kCoverageSvgCell = 6.0;

constexpr double kDefaultPushSpeed = 100.0;

constexpr double kDefaultConveyorSpeed = 100.0;
constexpr double kGravityEpsilon = 0.005;
constexpr size_t kMaxGravityMultipliers = 4;

constexpr double kDefaultDoorSpeed = 100.0;
constexpr double kDefaultRotateDistance = 90.0;
constexpr double kDefaultBreakableHealth = 1.0;
constexpr double kBreakSecondsPerHealth = 0.1;

bool IsBarrierWall(const Entity &ent)
{
	return crossbot_nav::IsBarrierWall(ent.Get("classname").c_str(), !ent.Get("targetname").empty());
}

bool IsBlockingEntity(const Entity &ent)
{
	std::string classname = ent.Get("classname");
	return crossbot_nav::IsBakedBlocker(classname.c_str(), ParseInt(ent.Get("spawnflags"), 0),
		!ent.Get("targetname").empty());
}

bool IsDoorClassname(const std::string &classname)
{
	return classname == "func_door" || classname == "func_door_rotating";
}

bool IsPassableClassname(const std::string &classname)
{
	return IsDoorClassname(classname) || classname == "func_breakable";
}

void PolytopeBounds(const Polytope &poly, Vec3 &mins, Vec3 &maxs)
{
	mins = Vec3(1e30, 1e30, 1e30);
	maxs = Vec3(-1e30, -1e30, -1e30);
	for (const Face &f : poly.faces)
	{
		for (const Vec3 &v : f.winding)
		{
			mins.x = std::min(mins.x, v.x);
			mins.y = std::min(mins.y, v.y);
			mins.z = std::min(mins.z, v.z);
			maxs.x = std::max(maxs.x, v.x);
			maxs.y = std::max(maxs.y, v.y);
			maxs.z = std::max(maxs.z, v.z);
		}
	}
}

bool BoundsOverlap(const Vec3 &aMins, const Vec3 &aMaxs, const Vec3 &bMins, const Vec3 &bMaxs, double epsilon)
{
	return aMins.x <= bMaxs.x + epsilon && aMaxs.x >= bMins.x - epsilon &&
		aMins.y <= bMaxs.y + epsilon && aMaxs.y >= bMins.y - epsilon &&
		aMins.z <= bMaxs.z + epsilon && aMaxs.z >= bMins.z - epsilon;
}

uint8_t ActivatorKindFor(const std::string &classname)
{
	if (classname == "func_button" || classname == "func_rot_button")
		return crossbot_nav::ACT_BUTTON;
	if (classname == "trigger_multiple" || classname == "trigger_once")
		return crossbot_nav::ACT_TRIGGER;
	return crossbot_nav::ACT_NONE;
}

bool TargetReaches(const std::vector<Entity> &entities, const std::string &target,
	const std::string &targetname)
{
	if (target.empty())
		return false;
	if (target == targetname)
		return true;

	for (const Entity &mid : entities)
	{
		std::string midClass = mid.Get("classname");
		if (mid.Get("targetname") != target)
			continue;

		if (midClass == "trigger_relay")
		{
			if (mid.Get("target") == targetname)
				return true;
		}
		else if (midClass == "multi_manager")
		{
			if (mid.Find(targetname) != nullptr)
				return true;
		}
	}
	return false;
}

}

double ParseDouble(const std::string &s, double def)
{
	if (s.empty())
		return def;
	char *end = nullptr;
	double v = std::strtod(s.c_str(), &end);
	return (end == s.c_str()) ? def : v;
}

int ParseInt(const std::string &s, int def)
{
	if (s.empty())
		return def;
	char *end = nullptr;
	long v = std::strtol(s.c_str(), &end, 10);
	return (end == s.c_str()) ? def : static_cast<int>(v);
}

Vec3 MoveDirFromAngles(const Vec3 &angles)
{
	if (angles.x == 0.0 && angles.z == 0.0)
	{
		if (angles.y == -1.0)
			return Vec3(0, 0, 1);
		if (angles.y == -2.0)
			return Vec3(0, 0, -1);
	}

	double pitch = angles.x * 3.14159265358979323846 / 180.0;
	double yaw = angles.y * 3.14159265358979323846 / 180.0;
	return Vec3(std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), -std::sin(pitch));
}

int ParseBrushModelIndex(const std::string &model)
{
	if (model.size() < 2 || model[0] != '*')
		return -1;
	char *end = nullptr;
	long idx = std::strtol(model.c_str() + 1, &end, 10);
	if (end == model.c_str() + 1 || *end != '\0' || idx <= 0)
		return -1;
	return static_cast<int>(idx);
}

Vec3 EntityAngles(const Entity &ent)
{
	if (const std::string *angles = ent.Find("angles"))
		return ParseEntityOrigin(*angles);
	if (const std::string *angle = ent.Find("angle"))
		return Vec3(0.0, ParseDouble(*angle, 0.0), 0.0);
	return Vec3(0.0, 0.0, 0.0);
}

Vec3 ParseEntityOrigin(const std::string &origin)
{
	double x = 0, y = 0, z = 0;
	std::sscanf(origin.c_str(), "%lf %lf %lf", &x, &y, &z);
	return Vec3(x, y, z);
}

Vec3 ModelWorldCenter(const dmodel_t &model, const Vec3 &origin)
{
	return Vec3(origin.x + (model.mins[0] + model.maxs[0]) * 0.5,
		origin.y + (model.mins[1] + model.maxs[1]) * 0.5,
		origin.z + (model.mins[2] + model.maxs[2]) * 0.5);
}

double DoorSpeed(const Entity &ent)
{
	double speed = ParseDouble(ent.Get("speed"), 0.0);
	return speed <= 0.0 ? kDefaultDoorSpeed : speed;
}

double DoorTravelDistance(const Entity &ent, const dmodel_t &model)
{
	Vec3 movedir = MoveDirFromAngles(EntityAngles(ent));
	Vec3 size(model.maxs[0] - model.mins[0], model.maxs[1] - model.mins[1], model.maxs[2] - model.mins[2]);

	double travel = std::fabs(movedir.x * (size.x - 2.0)) + std::fabs(movedir.y * (size.y - 2.0)) +
		std::fabs(movedir.z * (size.z - 2.0)) - ParseDouble(ent.Get("lip"), 0.0);

	return travel < 0.0 ? 0.0 : travel;
}

void CollectActivatorsFor(const BspFile &bsp, const std::vector<Entity> &entities,
	const std::string &targetname, std::vector<ActivatorVolume> &out)
{
	for (size_t i = 0; i < entities.size(); ++i)
	{
		const Entity &ent = entities[i];

		uint8_t kind = ActivatorKindFor(ent.Get("classname"));
		if (kind == crossbot_nav::ACT_NONE)
			continue;

		if (!TargetReaches(entities, ent.Get("target"), targetname))
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));

		ActivatorVolume act;
		act.kind = kind;
		act.mins = Vec3(model.mins[0], model.mins[1], model.mins[2]) + origin;
		act.maxs = Vec3(model.maxs[0], model.maxs[1], model.maxs[2]) + origin;
		act.center = ModelWorldCenter(model, origin);
		act.entityIndex = static_cast<int>(i);
		out.push_back(act);
	}
}

namespace
{

double DoorOpenSeconds(const Entity &ent, const dmodel_t &model)
{
	double speed = DoorSpeed(ent);

	if (ent.Get("classname") == "func_door_rotating")
	{
		double distance = ParseDouble(ent.Get("distance"), 0.0);
		if (distance <= 0.0)
			distance = kDefaultRotateDistance;
		return distance / speed;
	}

	return DoorTravelDistance(ent, model) / speed;
}

}

std::vector<BlockingVolume> ExtractBlockingVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	std::vector<BlockingVolume> result;

	for (size_t entIndex = 0; entIndex < entities.size(); ++entIndex)
	{
		const Entity &ent = entities[entIndex];
		if (!IsBlockingEntity(ent))
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[hullIndex] < 0)
			continue;

		BlockingVolume vol;
		vol.headnode = model.headnode[hullIndex];
		vol.origin = ParseEntityOrigin(ent.Get("origin"));
		vol.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
			model.mins[2] + hullMins.z) + vol.origin;
		vol.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
			model.maxs[2] + hullMaxs.z) + vol.origin;
		vol.entityIndex = static_cast<int>(entIndex);
		result.push_back(vol);
	}

	return result;
}

std::vector<BlockingVolume> ExtractVisBlockers(const BspFile &bsp, const std::vector<Entity> &entities,
	const SightCoverageSet *coverage)
{
	std::vector<BlockingVolume> result;

	if (coverage && !coverage->grid.Empty())
	{
		BlockingVolume vol;
		vol.coverage = &coverage->grid;
		vol.mins = Vec3(coverage->grid.Mins()[0], coverage->grid.Mins()[1], coverage->grid.Mins()[2]);
		vol.maxs = Vec3(coverage->grid.Maxs()[0], coverage->grid.Maxs()[1], coverage->grid.Maxs()[2]);
		result.push_back(vol);
	}

	for (size_t entIndex = 0; entIndex < entities.size(); ++entIndex)
	{
		const Entity &ent = entities[entIndex];
		if (!IsBlockingEntity(ent))
			continue;
		if (coverage && ParseInt(ent.Get("rendermode"), SightCoverage::kRenderNormal) != SightCoverage::kRenderNormal)
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex <= 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[kHullPoint] < 0)
			continue;

		BlockingVolume vol;
		vol.headnode = model.headnode[kHullPoint];
		vol.origin = ParseEntityOrigin(ent.Get("origin"));
		vol.mins = Vec3(model.mins[0], model.mins[1], model.mins[2]) + vol.origin;
		vol.maxs = Vec3(model.maxs[0], model.maxs[1], model.maxs[2]) + vol.origin;
		vol.entityIndex = static_cast<int>(entIndex);
		result.push_back(vol);
	}

	return result;
}

void ExtractSightCoverage(const std::string &bspPath, const std::vector<Entity> &entities, const HullTree &hull,
	int worldHeadnode, SightCoverageSet &out)
{
	auto start = std::chrono::steady_clock::now();

	SightCoverage::Bsp source;
	if (!source.Open(bspPath))
	{
		out.notes.push_back("cannot open " + bspPath + " for sight coverage");
		return;
	}
	std::string root = SightCoverage::AssetRoot(bspPath);
	std::unordered_map<std::string, size_t> studioModels;

	for (size_t entIndex = 0; entIndex < entities.size(); ++entIndex)
	{
		const Entity &ent = entities[entIndex];
		std::string classname = ent.Get("classname");
		std::string model = ent.Get("model");
		int rendermode = ParseInt(ent.Get("rendermode"), SightCoverage::kRenderNormal);
		float renderamt = static_cast<float>(ParseDouble(ent.Get("renderamt"), 0.0));
		Vec3 origin = ParseEntityOrigin(ent.Get("origin"));
		Vec3 angles(0.0, 0.0, 0.0);
		size_t modelIndex = SIZE_MAX;

		int submodel = ParseBrushModelIndex(model);
		bool studio = submodel <= 0;
		if (!studio)
		{
			if (classname != "func_illusionary" &&
				!(IsBlockingEntity(ent) && rendermode != SightCoverage::kRenderNormal))
				continue;

			std::vector<SightCoverage::Model> chunks;
			source.BuildBrush(submodel, rendermode, renderamt, chunks, out.stats);
			if (chunks.empty())
				continue;
			out.models.push_back(std::move(chunks));
			out.modelNames.push_back(classname + " " + model);
			modelIndex = out.models.size() - 1;
		}
		else
		{
			if (classname != "env_model" && classname != "cycler_sprite")
				continue;
			if (model.size() < 4 || (model.compare(model.size() - 4, 4, ".mdl") != 0 &&
				model.compare(model.size() - 4, 4, ".MDL") != 0))
				continue;

			int sequence = ParseInt(ent.Get("sequence"), 0);
			std::string label = classname == "env_model" ? ent.Get("m_iszSequence_On") : std::string();
			int body = ParseInt(ent.Get("body"), 0);
			int skin = ParseInt(ent.Get("skin"), 0);
			angles = EntityAngles(ent);

			if (classname == "env_model" && (ParseInt(ent.Get("spawnflags"), 0) & kSfEnvModelDropToFloor) != 0)
			{
				TraceResult tr = TraceHull(hull, worldHeadnode, origin,
					origin - Vec3(0.0, 0.0, kDropToFloorDistance));
				if (!tr.allsolid && tr.fraction < 1.f)
					origin = tr.endpos;
			}

			std::string key = model + "|" + std::to_string(sequence) + "|" + label + "|" + std::to_string(body) +
				"|" + std::to_string(skin) + "|" + std::to_string(rendermode) + "|" + std::to_string(renderamt);
			auto found = studioModels.find(key);
			if (found == studioModels.end())
			{
				std::vector<SightCoverage::Model> chunks;
				std::string why;
				size_t index = SIZE_MAX;
				if (SightCoverage::BuildStudio(root, model, sequence, label.c_str(), body, skin, rendermode,
					renderamt, chunks, out.stats, why))
				{
					out.models.push_back(std::move(chunks));
					out.modelNames.push_back(classname + " " + model);
					index = out.models.size() - 1;
				}
				else
				{
					out.notes.push_back(classname + " " + model + ": " + why);
				}
				found = studioModels.emplace(key, index).first;
			}
			modelIndex = found->second;
			if (modelIndex == SIZE_MAX)
				continue;
		}

		float o[3] = { static_cast<float>(origin.x), static_cast<float>(origin.y), static_cast<float>(origin.z) };
		float a[3] = { static_cast<float>(angles.x), static_cast<float>(angles.y), static_cast<float>(angles.z) };

		SightCoverageItem item;
		item.entityIndex = static_cast<int>(entIndex);
		item.modelIndex = modelIndex;
		SightCoverage::Place(item.placed, out.models[modelIndex], o, a, studio);
		out.items.push_back(std::move(item));
	}

	for (const std::string &name : source.Missing())
		out.notes.push_back("missing wad or texture " + name + " (treated as opaque)");

	std::vector<const SightCoverage::Placed *> placed;
	for (const SightCoverageItem &item : out.items)
		placed.push_back(&item.placed);
	out.grid.Build(placed);

	out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

void DumpSightCoverage(const std::string &bspPath, const std::vector<Entity> &entities,
	const SightCoverageSet &set, const std::string &dir)
{
	size_t chunkCount = 0;
	for (const std::vector<SightCoverage::Model> &chunks : set.models)
		chunkCount += chunks.size();

	std::printf("%s: sight coverage: %zu instances of %zu models (%zu chunks), %d triangles, "
		"winding %d/%d mismatched, %.2fs\n",
		bspPath.c_str(), set.items.size(), set.models.size(), chunkCount, set.stats.triangles,
		set.stats.windingMismatch, set.stats.windingChecked, set.seconds);
	for (const std::string &note : set.notes)
		std::printf("  note: %s\n", note.c_str());

	for (const SightCoverageItem &item : set.items)
	{
		const SightCoverage::Placed &in = item.placed;
		const Entity &ent = entities[static_cast<size_t>(item.entityIndex)];
		std::printf("  ent %d %s %s model %zu origin (%.0f %.0f %.0f) angles (%.0f %.0f %.0f) "
			"box (%.0f %.0f %.0f)-(%.0f %.0f %.0f)\n",
			item.entityIndex, ent.Get("classname").c_str(), ent.Get("model").c_str(), item.modelIndex,
			in.origin[0], in.origin[1], in.origin[2], in.angles[0], in.angles[1], in.angles[2],
			in.absmin[0], in.absmin[1], in.absmin[2], in.absmax[0], in.absmax[1], in.absmax[2]);
	}

	std::error_code ec;
	std::filesystem::create_directories(dir, ec);

	for (size_t m = 0; m < set.models.size(); ++m)
	{
		const std::vector<SightCoverage::Model> &chunks = set.models[m];

		int ringOf[SightCoverage::kViewCount];
		double viewU0[SightCoverage::kViewCount], viewV0[SightCoverage::kViewCount];
		double viewU1[SightCoverage::kViewCount], viewV1[SightCoverage::kViewCount];
		double ringSum[5] = {}, ringCells[5] = {};
		for (int v = 0; v < SightCoverage::kViewCount; ++v)
		{
			float d[3], u[3], w[3];
			SightCoverage::ViewBasis(v, d, u, w);
			ringOf[v] = static_cast<int>(std::lround(std::asin(d[2]) / (3.14159265358979 / 4.0))) + 2;
			viewU0[v] = viewV0[v] = 1e30;
			viewU1[v] = viewV1[v] = -1e30;
			for (const SightCoverage::Model &chunk : chunks)
			{
				const SightCoverage::View &view = chunk.views[v];
				viewU0[v] = std::min(viewU0[v], static_cast<double>(view.u0));
				viewV0[v] = std::min(viewV0[v], static_cast<double>(view.v0));
				viewU1[v] = std::max(viewU1[v], view.u0 + view.w * static_cast<double>(SightCoverage::kCellSize));
				viewV1[v] = std::max(viewV1[v], view.v0 + view.h * static_cast<double>(SightCoverage::kCellSize));
				for (uint8_t c : view.cells)
					ringSum[ringOf[v]] += c / 255.0;
				ringCells[ringOf[v]] += static_cast<double>(view.cells.size());
			}
		}

		float mins[3] = { 1e30f, 1e30f, 1e30f }, maxs[3] = { -1e30f, -1e30f, -1e30f };
		for (const SightCoverage::Model &chunk : chunks)
		{
			for (int k = 0; k < 3; ++k)
			{
				mins[k] = std::min(mins[k], chunk.mins[k]);
				maxs[k] = std::max(maxs[k], chunk.maxs[k]);
			}
		}

		std::printf("  model %zu %s: %zu chunks, box (%.0f %.0f %.0f)-(%.0f %.0f %.0f), cell alpha down %.2f / "
			"-45 %.2f / level %.2f / +45 %.2f / up %.2f\n",
			m, set.modelNames[m].c_str(), chunks.size(), mins[0], mins[1], mins[2], maxs[0], maxs[1], maxs[2],
			ringSum[0] / std::max(1.0, ringCells[0]), ringSum[1] / std::max(1.0, ringCells[1]),
			ringSum[2] / std::max(1.0, ringCells[2]), ringSum[3] / std::max(1.0, ringCells[3]),
			ringSum[4] / std::max(1.0, ringCells[4]));

		double scale = kCoverageSvgCell / SightCoverage::kCellSize;
		double rowHeight[5] = {}, rowWidth[5] = {};
		for (int v = 0; v < SightCoverage::kViewCount; ++v)
		{
			rowWidth[ringOf[v]] += (viewU1[v] - viewU0[v]) * scale + kCoverageSvgCell;
			rowHeight[ringOf[v]] = std::max(rowHeight[ringOf[v]], (viewV1[v] - viewV0[v]) * scale);
		}
		double width = 0.0, height = 24.0;
		for (int r = 0; r < 5; ++r)
		{
			width = std::max(width, rowWidth[r]);
			height += rowHeight[r] + kCoverageSvgCell;
		}

		std::string path = dir + "/coverage_" + std::to_string(m) + ".svg";
		FILE *f = std::fopen(path.c_str(), "w");
		if (!f)
			continue;

		std::fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\">\n",
			width + kCoverageSvgCell, height);
		std::fprintf(f, "<rect width=\"100%%\" height=\"100%%\" fill=\"#f4f1ea\"/>\n");
		std::fprintf(f, "<text x=\"4\" y=\"16\" font-family=\"monospace\" font-size=\"12\">%zu %s</text>\n", m,
			set.modelNames[m].c_str());

		double y = 24.0;
		for (int r = 4; r >= 0; --r)
		{
			double x = kCoverageSvgCell;
			for (int v = 0; v < SightCoverage::kViewCount; ++v)
			{
				if (ringOf[v] != r)
					continue;

				double viewH = (viewV1[v] - viewV0[v]) * scale;
				std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" fill=\"none\" "
					"stroke=\"#bbb\"/>\n", x, y, (viewU1[v] - viewU0[v]) * scale, viewH);
				for (const SightCoverage::Model &chunk : chunks)
				{
					const SightCoverage::View &view = chunk.views[v];
					for (int cy = 0; cy < view.h; ++cy)
					{
						for (int cx = 0; cx < view.w; ++cx)
						{
							uint8_t a = view.cells[static_cast<size_t>(cy) * view.w + cx];
							if (a == 0)
								continue;
							double px = x + (view.u0 - viewU0[v] + cx * SightCoverage::kCellSize) * scale;
							double py = y + viewH - (view.v0 - viewV0[v] + (cy + 1) * SightCoverage::kCellSize) * scale;
							std::fprintf(f, "<rect x=\"%.1f\" y=\"%.1f\" width=\"%.1f\" height=\"%.1f\" "
								"fill=\"#1d3b2a\" fill-opacity=\"%.2f\"/>\n",
								px, py, kCoverageSvgCell, kCoverageSvgCell, a / 255.0);
						}
					}
				}
				x += (viewU1[v] - viewU0[v]) * scale + kCoverageSvgCell;
			}
			y += rowHeight[r] + kCoverageSvgCell;
		}
		std::fprintf(f, "</svg>\n");
		std::fclose(f);
	}
}

std::vector<BlockingVolume> ExtractLadderVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	std::vector<BlockingVolume> result;

	for (const Entity &ent : entities)
	{
		if (ent.Get("classname") != "func_ladder")
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[hullIndex] < 0)
			continue;

		BlockingVolume vol;
		vol.headnode = model.headnode[hullIndex];
		vol.origin = ParseEntityOrigin(ent.Get("origin"));
		vol.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
			model.mins[2] + hullMins.z) + vol.origin;
		vol.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
			model.maxs[2] + hullMaxs.z) + vol.origin;
		result.push_back(vol);
	}

	return result;
}

DriftSet ExtractDriftVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	DriftSet result;

	for (size_t i = 0; i < entities.size(); ++i)
	{
		const Entity &ent = entities[i];
		const std::string &classname = ent.Get("classname");

		bool conveyor = classname == "func_conveyor";
		bool push = classname == "trigger_push";
		if (!conveyor && !push)
			continue;

		DriftReport report;
		report.entityIndex = static_cast<int>(i);
		report.classname = classname;
		report.spawnflags = ParseInt(ent.Get("spawnflags"), 0);
		report.modelIndex = ParseBrushModelIndex(ent.Get("model"));
		report.movedir = MoveDirFromAngles(EntityAngles(ent));
		report.speed = ParseDouble(ent.Get("speed"), 0.0);
		if (report.speed == 0.0)
			report.speed = conveyor ? kDefaultConveyorSpeed : kDefaultPushSpeed;

		if (conveyor && (report.spawnflags & crossbot_nav::kSfConveyorVisual) != 0)
			report.reason = "visual only, no FL_CONVEYOR";
		else if (conveyor && (report.spawnflags & crossbot_nav::kSfConveyorNotSolid) != 0)
			report.reason = "not solid, nothing stands on it";
		else if (push && (report.spawnflags & kSfPushOnce) != 0)
			report.reason = "one-shot impulse, not a steady field";
		else if (push && (report.spawnflags & kSfPushStartOff) != 0)
			report.reason = "starts off";
		else if (report.modelIndex < 0 ||
			static_cast<size_t>(report.modelIndex) >= bsp.Models().size())
			report.reason = "no brush model";

		if (report.reason.empty())
		{
			const dmodel_t &model = bsp.Models()[static_cast<size_t>(report.modelIndex)];
			if (model.headnode[hullIndex] < 0)
			{
				report.reason = "no clip tree in this hull";
			}
			else
			{
				DriftVolume drift;
				drift.velocity = Vec3(report.movedir.x * report.speed,
					report.movedir.y * report.speed, 0.0);
				drift.standOn = conveyor;
				drift.volume.headnode = model.headnode[hullIndex];
				drift.volume.origin = ParseEntityOrigin(ent.Get("origin"));
				drift.volume.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
					model.mins[2] + hullMins.z) + drift.volume.origin;
				drift.volume.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
					model.maxs[2] + hullMaxs.z) + drift.volume.origin;
				drift.volume.entityIndex = static_cast<int>(i);

				report.mins = drift.volume.mins;
				report.maxs = drift.volume.maxs;

				double horizontal = std::sqrt(drift.velocity.x * drift.velocity.x +
					drift.velocity.y * drift.velocity.y);
				if (horizontal < crossbot_nav::kMinDriftSpeed)
					report.reason = "no meaningful horizontal component";
				else
				{
					report.taken = true;
					result.volumes.push_back(drift);
				}
			}
		}

		result.reports.push_back(report);
	}

	return result;
}

std::vector<PushVolume> ExtractPushVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	std::vector<PushVolume> result;

	for (size_t i = 0; i < entities.size(); ++i)
	{
		const Entity &ent = entities[i];
		if (ent.Get("classname") != "trigger_push")
			continue;

		int spawnflags = ParseInt(ent.Get("spawnflags"), 0);
		if ((spawnflags & (kSfPushOnce | kSfPushStartOff)) != 0)
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[hullIndex] < 0)
			continue;

		double speed = ParseDouble(ent.Get("speed"), kDefaultPushSpeed);
		if (speed == 0.0)
			speed = kDefaultPushSpeed;

		PushVolume push;
		push.push = MoveDirFromAngles(EntityAngles(ent)) * speed;
		push.volume.headnode = model.headnode[hullIndex];
		push.volume.origin = ParseEntityOrigin(ent.Get("origin"));
		push.volume.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
			model.mins[2] + hullMins.z) + push.volume.origin;
		push.volume.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
			model.maxs[2] + hullMaxs.z) + push.volume.origin;
		push.volume.entityIndex = static_cast<int>(i);
		result.push_back(push);
	}

	return result;
}


std::vector<BlockingVolume> ExtractHurtVolumes(const BspFile &bsp, const std::vector<Entity> &entities,
	int hullIndex)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	std::vector<BlockingVolume> result;

	for (size_t i = 0; i < entities.size(); ++i)
	{
		const Entity &ent = entities[i];
		if (ent.Get("classname") != "trigger_hurt")
			continue;

		if ((ParseInt(ent.Get("spawnflags"), 0) & (kSfHurtStartOff | kSfHurtNoClients | kSfHurtOneTeam)) != 0)
			continue;

		if (ParseDouble(ent.Get("dmg"), 0.0) < kLethalHurtDamage)
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[hullIndex] < 0)
			continue;

		BlockingVolume vol;
		vol.headnode = model.headnode[hullIndex];
		vol.origin = ParseEntityOrigin(ent.Get("origin"));
		vol.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
			model.mins[2] + hullMins.z) + vol.origin;
		vol.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
			model.maxs[2] + hullMaxs.z) + vol.origin;
		vol.entityIndex = static_cast<int>(i);
		result.push_back(vol);
	}

	return result;
}

std::vector<double> CollectGravityMultipliers(const std::vector<Entity> &entities)
{
	std::vector<double> result{1.0};

	for (const Entity &ent : entities)
	{
		if (ent.Get("classname") != "trigger_gravity")
			continue;

		double gravity = ParseDouble(ent.Get("gravity"), 0.0);
		if (gravity <= 0.0)
			continue;

		bool seen = false;
		for (double have : result)
			if (std::fabs(have - gravity) < kGravityEpsilon)
				seen = true;

		if (!seen)
			result.push_back(gravity);
	}

	std::sort(result.begin(), result.end());

	if (result.size() > kMaxGravityMultipliers)
	{
		std::printf("warning: %zu distinct trigger_gravity values, sampling only the first %zu\n",
			result.size(), kMaxGravityMultipliers);
		result.resize(kMaxGravityMultipliers);
	}

	return result;
}

PassableSet ExtractPassableVolumes(const BspFile &bsp, const std::vector<Entity> &entities, int hullIndex,
	const std::vector<int> &skipEntities)
{
	Vec3 hullMins(kHumanHullMinX, kHumanHullMinY, kHumanHullMinZ);
	Vec3 hullMaxs(kHumanHullMaxX, kHumanHullMaxY, kHumanHullMaxZ);
	if (hullIndex == kHullDuck)
	{
		hullMins = Vec3(kDuckClipHullMinX, kDuckClipHullMinY, kDuckClipHullMinZ);
		hullMaxs = Vec3(kDuckClipHullMaxX, kDuckClipHullMaxY, kDuckClipHullMaxZ);
	}

	PassableSet result;

	bool dodMap = std::any_of(entities.begin(), entities.end(), [](const Entity &ent) {
		const std::string &classname = ent.Get("classname");
		return classname == "info_player_allies" || classname == "info_player_axis";
	});

	for (size_t entIndex = 0; entIndex < entities.size(); ++entIndex)
	{
		const Entity &ent = entities[entIndex];

		std::string classname = ent.Get("classname");
		bool barrierWall = IsBarrierWall(ent);
		if (!IsPassableClassname(classname) && !barrierWall)
			continue;

		if (std::find(skipEntities.begin(), skipEntities.end(), static_cast<int>(entIndex)) !=
			skipEntities.end())
			continue;

		int modelIndex = ParseBrushModelIndex(ent.Get("model"));
		if (modelIndex < 0 || static_cast<size_t>(modelIndex) >= bsp.Models().size())
			continue;

		const dmodel_t &model = bsp.Models()[static_cast<size_t>(modelIndex)];
		if (model.headnode[hullIndex] < 0)
			continue;

		bool isDoor = IsDoorClassname(classname);
		int spawnflags = ParseInt(ent.Get("spawnflags"), 0);

		if (isDoor && ((spawnflags & kSfDoorStartOpen) || (spawnflags & kSfDoorPassable)))
			continue;

		BlockingVolume vol;
		vol.headnode = model.headnode[hullIndex];
		vol.origin = ParseEntityOrigin(ent.Get("origin"));
		vol.mins = Vec3(model.mins[0] + hullMins.x, model.mins[1] + hullMins.y,
			model.mins[2] + hullMins.z) + vol.origin;
		vol.maxs = Vec3(model.maxs[0] + hullMaxs.x, model.maxs[1] + hullMaxs.y,
			model.maxs[2] + hullMaxs.z) + vol.origin;
		vol.entityIndex = static_cast<int>(entIndex);

		PassableVolume pass;
		pass.volume = vol;
		pass.entityIndex = static_cast<int>(entIndex);

		auto pushBarrier = [&]() {
			pass.areaFlag = crossbot_nav::NAV_AREA_BARRIER;
			pass.activation = crossbot_nav::ACT_NONE;
			pass.activators.clear();
			pass.openSeconds = 0.0;
			pass.modelIndex = modelIndex;
			pass.modelMins = Vec3(model.mins[0], model.mins[1], model.mins[2]) + vol.origin;
			pass.modelMaxs = Vec3(model.maxs[0], model.maxs[1], model.maxs[2]) + vol.origin;
			result.passable.push_back(pass);
		};

		if (barrierWall || (dodMap && !isDoor && (spawnflags & kSfBreakDodExplosiveOnly) != 0))
		{
			pushBarrier();
			continue;
		}

		pass.areaFlag = isDoor ? crossbot_nav::NAV_AREA_DOOR : crossbot_nav::NAV_AREA_BREAKABLE;
		pass.openSeconds = isDoor ? DoorOpenSeconds(ent, model)
			: std::max(ParseDouble(ent.Get("health"), kDefaultBreakableHealth), kDefaultBreakableHealth) *
				kBreakSecondsPerHealth;

		std::string targetname = ent.Get("targetname");
		bool triggerOnly = isDoor ? ((spawnflags & kSfDoorUseOnly) != 0)
			: ((spawnflags & kSfBreakTriggerOnly) != 0);

		if (targetname.empty() && !triggerOnly)
		{
			pass.activation = crossbot_nav::ACT_NONE;
			result.passable.push_back(pass);
			continue;
		}

		if (targetname.empty() && isDoor)
		{
			ActivatorVolume self;
			self.kind = crossbot_nav::ACT_BUTTON;
			self.mins = Vec3(model.mins[0], model.mins[1], model.mins[2]) + vol.origin;
			self.maxs = Vec3(model.maxs[0], model.maxs[1], model.maxs[2]) + vol.origin;
			self.center = ModelWorldCenter(model, vol.origin);
			self.entityIndex = static_cast<int>(entIndex);

			pass.activation = crossbot_nav::ACT_BUTTON;
			pass.activators.push_back(self);
			result.passable.push_back(pass);
			continue;
		}

		CollectActivatorsFor(bsp, entities, targetname, pass.activators);

		if (pass.activators.empty())
		{
			if (isDoor)
				result.impassable.push_back(vol);
			else if (triggerOnly)
				pushBarrier();
			else
				result.passable.push_back(pass);
			continue;
		}

		pass.activation = pass.activators.front().kind;
		for (const ActivatorVolume &act : pass.activators)
		{
			if (act.kind == crossbot_nav::ACT_BUTTON)
			{
				pass.activation = crossbot_nav::ACT_BUTTON;
				break;
			}
		}

		result.passable.push_back(pass);
	}

	return result;
}

bool OverlapsAnyBlockingVolume(const ExtractedLeaf &leaf, const std::vector<BlockingVolume> &blockers)
{
	Vec3 mins, maxs;
	PolytopeBounds(leaf.polytope, mins, maxs);

	for (const BlockingVolume &blocker : blockers)
		if (BoundsOverlap(mins, maxs, blocker.mins, blocker.maxs, kOverlapEpsilon))
			return true;

	return false;
}

bool OverlapsBlockingVolume(const Polytope &poly, const BlockingVolume &blocker)
{
	Vec3 mins, maxs;
	PolytopeBounds(poly, mins, maxs);
	return BoundsOverlap(mins, maxs, blocker.mins, blocker.maxs, kOverlapEpsilon);
}

void SplitByPassableVolume(const BspFile &bsp, const Polytope &poly, const BlockingVolume &blocker,
	double mapRadius, std::vector<Polytope> &inside, std::vector<Polytope> &outside, double epsilon,
	CarveStats *stats)
{
	if (stats)
		++stats->tests;

	if (!OverlapsBlockingVolume(poly, blocker))
	{
		outside.push_back(poly);
		return;
	}

	Polytope local = TranslatePolytope(poly, blocker.origin * -1.0);

	std::vector<Polytope> insidePieces = CarvePolytopeAgainstTree(bsp, blocker.headnode, local,
		CONTENTS_EMPTY, mapRadius, kMinPieceVolume, epsilon);

	std::vector<Polytope> outsidePieces;
	if (!insidePieces.empty() || stats)
		outsidePieces = CarvePolytopeAgainstTree(bsp, blocker.headnode, local, CONTENTS_SOLID,
			mapRadius, kMinPieceVolume, epsilon);

	if (stats)
	{
		++stats->boundsHits;
		stats->carves += 2;
		stats->piecesOut += insidePieces.size() + outsidePieces.size();
		if (insidePieces.empty())
		{
			++stats->nullCarves;
			if (outsidePieces.size() > 1)
			{
				stats->nullCarveExtraPieces += outsidePieces.size() - 1;
				if (stats->nullEvents.size() < kMaxNullEvents)
				{
					NullCarveEvent ev;
					ev.entityIndex = blocker.entityIndex;
					ev.pieces = outsidePieces.size();
					ev.volume = poly.Volume();
					ev.centroid = poly.Centroid();
					stats->nullEvents.push_back(ev);
				}
			}
		}
		else
		{
			++stats->solidCarves;
			size_t total = insidePieces.size() + outsidePieces.size();
			if (total > 1)
				stats->solidCarveExtraPieces += total - 1;
		}
	}

	if (insidePieces.empty())
	{
		outside.push_back(poly);
		return;
	}

	for (Polytope &piece : outsidePieces)
		outside.push_back(TranslatePolytope(piece, blocker.origin));

	for (Polytope &piece : insidePieces)
		inside.push_back(TranslatePolytope(piece, blocker.origin));
}

std::vector<Polytope> SubtractBoxVolumes(const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &boxes, double mapRadius, double epsilon)
{
	std::vector<Polytope> pieces{leafPolytope};

	for (const BlockingVolume &box : boxes)
	{
		const Plane slabs[6] = {
			{Vec3(1.0, 0.0, 0.0), box.mins.x}, {Vec3(-1.0, 0.0, 0.0), -box.maxs.x},
			{Vec3(0.0, 1.0, 0.0), box.mins.y}, {Vec3(0.0, -1.0, 0.0), -box.maxs.y},
			{Vec3(0.0, 0.0, 1.0), box.mins.z}, {Vec3(0.0, 0.0, -1.0), -box.maxs.z}};

		std::vector<Polytope> next;
		for (const Polytope &piece : pieces)
		{
			Vec3 mins, maxs;
			PolytopeBounds(piece, mins, maxs);
			if (!BoundsOverlap(mins, maxs, box.mins, box.maxs, -kOverlapEpsilon))
			{
				next.push_back(piece);
				continue;
			}

			Polytope remain = piece;
			for (const Plane &slab : slabs)
			{
				Polytope outside = ClipPolytope(remain, slab, false, mapRadius, epsilon);
				if (outside.Volume() > kMinPieceVolume)
					next.push_back(std::move(outside));

				remain = ClipPolytope(remain, slab, true, mapRadius, epsilon);
				if (remain.Empty())
					break;
			}
		}
		pieces = std::move(next);
	}

	return pieces;
}

std::vector<Polytope> SubtractBlockingVolumes(const BspFile &bsp, const Polytope &leafPolytope,
	const std::vector<BlockingVolume> &blockers, double mapRadius, double epsilon, CarveStats *stats)
{
	std::vector<Polytope> pieces{leafPolytope};

	if (stats)
	{
		++stats->inputs;
		stats->perBlockerPieces.resize(blockers.size());
		stats->perBlockerNullCarves.resize(blockers.size());
		stats->perBlockerBoundsHits.resize(blockers.size());
	}

	for (size_t bi = 0; bi < blockers.size(); ++bi)
	{
		const BlockingVolume &blocker = blockers[bi];
		std::vector<Polytope> next;
		for (Polytope &piece : pieces)
		{
			if (stats)
				++stats->tests;

			Vec3 mins, maxs;
			PolytopeBounds(piece, mins, maxs);
			if (!BoundsOverlap(mins, maxs, blocker.mins, blocker.maxs, kOverlapEpsilon))
			{
				next.push_back(std::move(piece));
				continue;
			}

			double before = stats ? piece.Volume() : 0.0;

			bool removedAny = false;
			Polytope local = TranslatePolytope(piece, blocker.origin * -1.0);
			std::vector<Polytope> carved = CarvePolytopeAgainstTree(bsp, blocker.headnode, local,
				CONTENTS_SOLID, mapRadius, kMinPieceVolume, epsilon, &removedAny);

			if (stats)
			{
				++stats->boundsHits;
				++stats->carves;
				++stats->perBlockerBoundsHits[bi];
				stats->perBlockerPieces[bi] += carved.size();

				double after = 0.0;
				for (const Polytope &c : carved)
					after += c.Volume();

				if (after > before + kNullCarveTolerance * std::max(1.0, before))
				{
					++stats->inflatedCarves;
					if (after / std::max(1.0, before) > stats->worstInflation)
						stats->worstInflation = after / std::max(1.0, before);
					if (after > before * kInflationReportRatio && stats->nullEvents.size() < kMaxNullEvents)
					{
						NullCarveEvent ev;
						ev.entityIndex = blocker.entityIndex;
						ev.pieces = piece.faces.size();
						ev.volume = before;
						ev.centroid = piece.Centroid();
						ev.after = after;
						ev.outFaces = carved.empty() ? 0 : carved[0].faces.size();
						stats->nullEvents.push_back(ev);
					}
				}

				if (after > before - kNullCarveTolerance * std::max(1.0, before))
				{
					++stats->nullCarves;
					++stats->perBlockerNullCarves[bi];
					if (carved.size() > 1)
						stats->nullCarveExtraPieces += carved.size() - 1;
				}
				else
				{
					++stats->solidCarves;
					if (carved.size() > 1)
						stats->solidCarveExtraPieces += carved.size() - 1;
				}
			}

			if (!removedAny)
			{
				next.push_back(std::move(piece));
				continue;
			}

			for (const Polytope &c : carved)
				next.push_back(TranslatePolytope(c, blocker.origin));
		}
		pieces = std::move(next);

		if (stats && pieces.size() > stats->peakPieces)
			stats->peakPieces = pieces.size();
	}

	if (stats)
		stats->piecesOut += pieces.size();

	return pieces;
}

}

