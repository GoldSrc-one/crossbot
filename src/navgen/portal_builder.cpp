#include "portal_builder.h"

#include <cmath>
#include <unordered_map>

namespace navgen
{

namespace
{

std::pair<Plane, int> CanonicalizePlane(const Plane &p)
{
	bool flip = false;
	if (p.normal.x < -1e-6)
		flip = true;
	else if (std::fabs(p.normal.x) <= 1e-6 && p.normal.y < -1e-6)
		flip = true;
	else if (std::fabs(p.normal.x) <= 1e-6 && std::fabs(p.normal.y) <= 1e-6 && p.normal.z < -1e-6)
		flip = true;

	if (!flip)
		return {p, +1};

	Plane f;
	f.normal = p.normal * -1.0;
	f.dist = -p.dist;
	return {f, -1};
}

struct PlaneKey
{
	long long nx, ny, nz, d;
	bool operator==(const PlaneKey &o) const
	{
		return nx == o.nx && ny == o.ny && nz == o.nz && d == o.d;
	}
};

struct PlaneKeyHash
{
	size_t operator()(const PlaneKey &k) const
	{
		auto mix = [](size_t h, long long v)
		{
			size_t hv = std::hash<long long>()(v);
			return h ^ (hv + static_cast<size_t>(0x9e3779b9UL) + (h << 6) + (h >> 2));
		};
		size_t h = 0;
		h = mix(h, k.nx);
		h = mix(h, k.ny);
		h = mix(h, k.nz);
		h = mix(h, k.d);
		return h;
	}
};

PlaneKey QuantizePlane(const Plane &p)
{
	auto q = [](double v) { return static_cast<long long>(std::llround(v * 100.0)); };
	return {q(p.normal.x), q(p.normal.y), q(p.normal.z), q(p.dist)};
}

struct FaceRef
{
	int leafIndex;
	int faceIndex;
	int sign;
};

std::pair<Vec3, Vec3> PolygonDiameter(const Winding &w)
{
	Vec3 a, b;
	double best = -1.0;
	for (size_t i = 0; i < w.size(); ++i)
	{
		for (size_t j = i + 1; j < w.size(); ++j)
		{
			double d = (w[i] - w[j]).Length();
			if (d > best)
			{
				best = d;
				a = w[i];
				b = w[j];
			}
		}
	}
	return {a, b};
}

constexpr double kMinPortalArea = 1.0;

}

std::vector<Portal> BuildPortals(const std::vector<ExtractedLeaf> &leaves)
{
	std::unordered_map<PlaneKey, std::vector<FaceRef>, PlaneKeyHash> buckets;

	for (const ExtractedLeaf &leaf : leaves)
	{
		for (size_t fi = 0; fi < leaf.polytope.faces.size(); ++fi)
		{
			const Face &face = leaf.polytope.faces[fi];
			auto [canon, sign] = CanonicalizePlane(face.plane);
			buckets[QuantizePlane(canon)].push_back({leaf.id, static_cast<int>(fi), sign});
		}
	}

	std::vector<Portal> portals;

	for (const auto &entry : buckets)
	{
		const std::vector<FaceRef> &refs = entry.second;

		std::vector<const FaceRef *> pos, neg;
		for (const FaceRef &r : refs)
			(r.sign > 0 ? pos : neg).push_back(&r);

		for (const FaceRef *pr : pos)
		{
			for (const FaceRef *nr : neg)
			{
				if (pr->leafIndex == nr->leafIndex)
					continue;

				const Face &fa = leaves[static_cast<size_t>(pr->leafIndex)].polytope.faces[static_cast<size_t>(pr->faceIndex)];
				const Face &fb = leaves[static_cast<size_t>(nr->leafIndex)].polytope.faces[static_cast<size_t>(nr->faceIndex)];

				Winding overlap = IntersectCoplanarPolygons(fa.winding, fb.winding, fa.plane.normal);
				if (overlap.size() < 3)
					continue;

				double area = WindingArea(overlap, fa.plane.normal);
				if (area < kMinPortalArea)
					continue;

				auto [pointA, pointB] = PolygonDiameter(overlap);

				Portal portal;
				portal.leafA = pr->leafIndex;
				portal.leafB = nr->leafIndex;
				portal.faceIndexA = pr->faceIndex;
				portal.faceIndexB = nr->faceIndex;
				portal.pointA = pointA;
				portal.pointB = pointB;
				portal.winding = std::move(overlap);
				portal.area = area;
				portals.push_back(portal);
			}
		}
	}

	return portals;
}

}

