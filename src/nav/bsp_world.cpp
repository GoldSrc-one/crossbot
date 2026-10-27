#include "bsp_world.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "crc32.h"

namespace Nav
{

namespace
{

template <typename T>
bool ReadLump(const std::vector<unsigned char> &data, const lump_t &lump, std::vector<T> &out)
{
	if (lump.filelen < 0 || lump.fileofs < 0 ||
		static_cast<size_t>(lump.fileofs) + static_cast<size_t>(lump.filelen) > data.size())
		return false;
	if (static_cast<size_t>(lump.filelen) % sizeof(T) != 0)
		return false;
	out.resize(static_cast<size_t>(lump.filelen) / sizeof(T));
	if (!out.empty())
		std::memcpy(out.data(), data.data() + lump.fileofs, static_cast<size_t>(lump.filelen));
	return true;
}

bool ValidTree(const BspWorld &world)
{
	int nodes = static_cast<int>(world.clipnodes.size());
	for (const dclipnode_t &node : world.clipnodes)
	{
		if (node.planenum < 0 || static_cast<size_t>(node.planenum) >= world.planes.size())
			return false;
		if (node.children[0] >= nodes || node.children[1] >= nodes)
			return false;
	}
	for (const dmodel_t &model : world.models)
	{
		for (int hull = 1; hull < MAX_MAP_HULLS; ++hull)
		{
			if (model.headnode[hull] >= nodes)
				return false;
		}
	}
	return !world.models.empty();
}

const Vector kHullMins[MAX_MAP_HULLS] = { Vector(0.f, 0.f, 0.f), Vector(-16.f, -16.f, -36.f),
	Vector(-32.f, -32.f, -32.f), Vector(-16.f, -16.f, -18.f) };
const Vector kHullMaxs[MAX_MAP_HULLS] = { Vector(0.f, 0.f, 0.f), Vector(16.f, 16.f, 36.f),
	Vector(32.f, 32.f, 32.f), Vector(16.f, 16.f, 18.f) };

struct SegResult
{
	Vector endpos;
	Vector planeNormal;
	float fraction = 1.f;
	bool allsolid = true;
	bool startsolid = false;
	int contents = CONTENTS_EMPTY;
};

struct BoxHull
{
	dplane_t planes[6];
	dclipnode_t nodes[6];
	HullTrace::HullTree tree;

	BoxHull(const Vector &mins, const Vector &maxs)
	{
		float dist[6] = { maxs.x, mins.x, maxs.y, mins.y, maxs.z, mins.z };
		for (int i = 0; i < 6; ++i)
		{
			int side = i & 1;
			nodes[i].planenum = i;
			nodes[i].children[side] = CONTENTS_EMPTY;
			nodes[i].children[side ^ 1] = static_cast<short>((i != 5) ? i + 1 : CONTENTS_SOLID);
			planes[i].normal[0] = planes[i].normal[1] = planes[i].normal[2] = 0.f;
			planes[i].normal[i >> 1] = 1.f;
			planes[i].dist = dist[i];
			planes[i].type = i >> 1;
		}
		tree.clipnodes = nodes;
		tree.clipnodeCount = 6;
		tree.planes = planes;
		tree.planeCount = 6;
	}
};

SegResult TraceTree(const HullTrace::HullTree &tree, int headnode, const Vector &start, const Vector &end)
{
	SegResult r;
	r.endpos = end;
	HullTrace::HullCheck<false>(tree, headnode, headnode, 0.f, 1.f, start, end, r);
	return r;
}

Vector RotateIn(const WorldSolid &solid, const Vector &v)
{
	return Vector(DotProduct(solid.forward, v), -DotProduct(solid.right, v), DotProduct(solid.up, v));
}

Vector RotateOut(const WorldSolid &solid, const Vector &v)
{
	return solid.forward * v.x - solid.right * v.y + solid.up * v.z;
}

Vector ToLocal(const WorldSolid &solid, const Vector &p)
{
	Vector local = p - solid.origin;
	return solid.rotated ? RotateIn(solid, local) : local;
}

SegResult TraceSolid(const BspWorld &world, const WorldSolid &solid, int hull, const Vector &start,
	const Vector &end)
{
	Vector a = ToLocal(solid, start);
	Vector b = ToLocal(solid, end);

	SegResult r;
	if (solid.model >= 0)
		r = TraceTree(world.Hull(), world.models[static_cast<size_t>(solid.model)].headnode[hull], a, b);
	else
	{
		BoxHull box(solid.boxMins - kHullMaxs[hull], solid.boxMaxs - kHullMins[hull]);
		r = TraceTree(box.tree, 0, a, b);
	}

	if (r.fraction != 1.f && solid.rotated)
		r.planeNormal = RotateOut(solid, r.planeNormal);
	return r;
}

}

HullTrace::HullTree BspWorld::Hull() const
{
	HullTrace::HullTree tree;
	tree.clipnodes = clipnodes.data();
	tree.clipnodeCount = clipnodes.size();
	tree.planes = planes.data();
	tree.planeCount = planes.size();
	return tree;
}

std::shared_ptr<const BspWorld> LoadBspWorld(const char *bspPath)
{
	FILE *f = std::fopen(bspPath, "rb");
	if (!f)
		return nullptr;

	std::fseek(f, 0, SEEK_END);
	long size = std::ftell(f);
	std::fseek(f, 0, SEEK_SET);
	if (size < static_cast<long>(sizeof(dheader_t)))
	{
		std::fclose(f);
		return nullptr;
	}

	std::vector<unsigned char> data(static_cast<size_t>(size));
	size_t readBytes = std::fread(data.data(), 1, data.size(), f);
	std::fclose(f);
	if (readBytes != data.size())
		return nullptr;

	dheader_t header;
	std::memcpy(&header, data.data(), sizeof(header));
	if (header.version != HLBSP_VERSION)
		return nullptr;

	auto world = std::make_shared<BspWorld>();
	if (!ReadLump(data, header.lumps[LUMP_PLANES], world->planes) ||
		!ReadLump(data, header.lumps[LUMP_CLIPNODES], world->clipnodes) ||
		!ReadLump(data, header.lumps[LUMP_MODELS], world->models) ||
		!ValidTree(*world))
		return nullptr;

	world->checksum = navgen::BspChecksumLumps(
		world->planes.data(), world->planes.size() * sizeof(dplane_t),
		world->clipnodes.data(), world->clipnodes.size() * sizeof(dclipnode_t),
		world->models.data(), world->models.size() * sizeof(dmodel_t));
	return world;
}

void SetSolidAngles(WorldSolid &solid, const Vector &angles)
{
	solid.rotated = angles.x != 0.f || angles.y != 0.f || angles.z != 0.f;
	if (!solid.rotated)
		return;

	float rad = static_cast<float>(M_PI) / 180.f;
	float sy = sinf(angles.y * rad), cy = cosf(angles.y * rad);
	float sp = sinf(angles.x * rad), cp = cosf(angles.x * rad);
	float sr = sinf(angles.z * rad), cr = cosf(angles.z * rad);

	solid.forward = Vector(cp * cy, cp * sy, -sp);
	solid.right = Vector(-sr * sp * cy + cr * sy, -sr * sp * sy - cr * cy, -sr * cp);
	solid.up = Vector(cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp);
}

namespace
{

bool Skips(const WorldSolid &solid, const TraceFilter &filter, const Vector &mins, const Vector &maxs)
{
	if (solid.index == filter.ignore || (filter.ignore != 0 && solid.owner == filter.ignore))
		return true;
	if (filter.phaseBreakables && solid.breakable)
		return true;
	return mins.x > solid.absmax.x || mins.y > solid.absmax.y || mins.z > solid.absmax.z ||
		maxs.x < solid.absmin.x || maxs.y < solid.absmin.y || maxs.z < solid.absmin.z;
}

}

PmSim::PmTrace TraceWorld(const WorldSnapshot &snap, const TraceFilter &filter, const Vector &start,
	const Vector &end)
{
	const BspWorld &world = *snap.world;
	int hull = filter.hull;
	SegResult best = TraceTree(world.Hull(), world.models[0].headnode[hull], start, end);
	bool hitEntity = false;

	Vector boxMins(std::min(start.x, end.x) + kHullMins[hull].x - 1.f,
		std::min(start.y, end.y) + kHullMins[hull].y - 1.f, std::min(start.z, end.z) + kHullMins[hull].z - 1.f);
	Vector boxMaxs(std::max(start.x, end.x) + kHullMaxs[hull].x + 1.f,
		std::max(start.y, end.y) + kHullMaxs[hull].y + 1.f, std::max(start.z, end.z) + kHullMaxs[hull].z + 1.f);

	for (const WorldSolid &solid : snap.solids)
	{
		if (best.allsolid)
			break;
		if (Skips(solid, filter, boxMins, boxMaxs))
			continue;

		SegResult tr = TraceSolid(world, solid, hull, start, end);
		if (tr.allsolid || tr.startsolid || tr.fraction < best.fraction)
		{
			bool startsolid = best.startsolid;
			best = tr;
			best.startsolid = best.startsolid || startsolid;
			hitEntity = solid.counts;
		}
	}

	PmSim::PmTrace out;
	out.fraction = best.fraction;
	out.endpos = (best.fraction != 1.f) ? start + (end - start) * best.fraction : end;
	out.planeNormal = best.planeNormal;
	out.allsolid = best.allsolid;
	out.startsolid = best.startsolid;
	out.hitEntity = hitEntity;
	return out;
}

namespace
{

bool ArcIntoSolid(const BspWorld &world, const WorldSolid &ent, int hull, const Vector &start, const Vector &vel,
	const Vector &accel, float maxTime, float &time)
{
	Vector localStart = ToLocal(ent, start);
	Vector localVel = ent.rotated ? RotateIn(ent, vel) : vel;
	Vector localAccel = ent.rotated ? RotateIn(ent, accel) : accel;

	if (ent.model >= 0)
	{
		HullTrace::ArcTrace<Vector, float> hit = HullTrace::TraceArc(world.Hull(),
			world.models[static_cast<size_t>(ent.model)].headnode[hull], localStart, localVel, localAccel, maxTime);
		time = hit.time;
		return hit.allsolid || hit.startsolid;
	}

	BoxHull box(ent.boxMins - kHullMaxs[hull], ent.boxMaxs - kHullMins[hull]);
	HullTrace::ArcTrace<Vector, float> hit = HullTrace::TraceArc(box.tree, 0, localStart, localVel, localAccel,
		maxTime);
	time = hit.time;
	return hit.allsolid || hit.startsolid;
}

float ArcHitTime(const WorldSnapshot &snap, const TraceFilter &filter, const Vector &start, const Vector &vel,
	float gravity, float maxTime, bool &solid)
{
	const BspWorld &world = *snap.world;
	int hull = filter.hull;
	Vector accel(0.f, 0.f, -gravity);
	HullTrace::ArcTrace<Vector, float> arc = HullTrace::TraceArc(world.Hull(), world.models[0].headnode[hull],
		start, vel, accel, maxTime);
	if (arc.allsolid || arc.startsolid)
	{
		solid = true;
		return 0.f;
	}

	float best = arc.time;
	Vector mins, maxs;
	ArcMath::ArcBounds(start, vel, gravity, best, mins, maxs);
	mins = mins + kHullMins[hull] - Vector(1.f, 1.f, 1.f);
	maxs = maxs + kHullMaxs[hull] + Vector(1.f, 1.f, 1.f);

	for (const WorldSolid &ent : snap.solids)
	{
		if (Skips(ent, filter, mins, maxs))
			continue;

		float time = best;
		if (ArcIntoSolid(world, ent, hull, start, vel, accel, best, time))
		{
			solid = true;
			return 0.f;
		}
		best = std::min(best, time);
	}

	return best;
}

}

float SafeArcTime(const WorldSnapshot &snap, const TraceFilter &filter, const Vector &start, const Vector &vel,
	float gravity, float maxTime)
{
	bool solid = false;
	float safe = ArcHitTime(snap, filter, start, vel, gravity, maxTime, solid);
	if (solid)
		return 0.f;

	Vector probe(start.x, start.y, start.z - PmSim::kGroundTraceDist);
	safe = ArcHitTime(snap, filter, probe, vel, gravity, safe, solid);
	if (solid)
		return 0.f;

	for (const WorldSolid &push : snap.pushes)
		safe = ArcMath::ArcEnterBox(start, vel, gravity, push.absmin - kHullMaxs[filter.hull],
			push.absmax - kHullMins[filter.hull], safe);

	return safe;
}

Vector PushAt(const WorldSnapshot &snap, int hull, const Vector &origin)
{
	const BspWorld &world = *snap.world;
	Vector total(0.f, 0.f, 0.f);

	for (const WorldSolid &push : snap.pushes)
	{
		if (origin.x < push.absmin.x - kHullMaxs[hull].x || origin.x > push.absmax.x - kHullMins[hull].x ||
			origin.y < push.absmin.y - kHullMaxs[hull].y || origin.y > push.absmax.y - kHullMins[hull].y ||
			origin.z < push.absmin.z - kHullMaxs[hull].z || origin.z > push.absmax.z - kHullMins[hull].z)
			continue;

		int headnode = world.models[static_cast<size_t>(push.model)].headnode[hull];
		if (HullTrace::ClassifyAt(world.Hull(), headnode, ToLocal(push, origin)) == CONTENTS_SOLID)
			total = total + push.push;
	}

	return total;
}

}
