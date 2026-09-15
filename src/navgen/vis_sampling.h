#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "blocking_entities.h"
#include "clipnode_tree.h"
#include "polytope.h"

namespace navgen
{

constexpr double kVisDeg = 3.14159265358979323846 / 180.0;

struct TraceCounters
{
	size_t worldTraces = 0;
	size_t blockerTraces = 0;
};

struct VisBlockerBox
{
	int headnode = -1;
	Vec3 origin;
	Vec3 mins;
	Vec3 maxs;
};

inline Vec3 DirFromAngles(double yawDeg, double pitchDeg)
{
	double e = -pitchDeg * kVisDeg;
	double y = yawDeg * kVisDeg;
	double ce = std::cos(e);
	return Vec3(std::cos(y) * ce, std::sin(y) * ce, std::sin(e));
}

inline void AnglesFromDir(const Vec3 &dir, double &yawDeg, double &pitchDeg)
{
	double flat = std::sqrt(dir.x * dir.x + dir.y * dir.y);
	yawDeg = (flat < 1e-9 && std::fabs(dir.z) < 1e-9) ? 0.0 : std::atan2(dir.y, dir.x) / kVisDeg;
	pitchDeg = -std::atan2(dir.z, flat) / kVisDeg;
}

inline std::vector<VisBlockerBox> BuildVisBlockerBoxes(const std::vector<BlockingVolume> &blockers)
{
	std::vector<VisBlockerBox> boxes;
	boxes.reserve(blockers.size());
	for (const BlockingVolume &vol : blockers)
	{
		VisBlockerBox box;
		box.headnode = vol.headnode;
		box.origin = vol.origin;
		box.mins = vol.mins;
		box.maxs = vol.maxs;
		boxes.push_back(box);
	}

	return boxes;
}

// The blocker half of Occluded, for callers that traced the world already (SegmentsBlockedBatch).
inline bool BlockersOccluded(const HullTree &hull, const std::vector<VisBlockerBox> &blockers,
	const Vec3 &a, const Vec3 &b, TraceCounters &counters)
{
	for (const VisBlockerBox &blocker : blockers)
	{
		if (!SegmentHitsBox(a, b, blocker.mins, blocker.maxs))
			continue;

		++counters.blockerTraces;
		if (SegmentBlocked(hull, blocker.headnode, a - blocker.origin, b - blocker.origin))
			return true;
	}

	return false;
}

inline bool Occluded(const HullTree &hull, int worldHeadnode, const std::vector<VisBlockerBox> &blockers,
	const Vec3 &a, const Vec3 &b, TraceCounters &counters)
{
	++counters.worldTraces;
	if (SegmentBlocked(hull, worldHeadnode, a, b))
		return true;

	return BlockersOccluded(hull, blockers, a, b, counters);
}

inline bool Occluded(const HullTree &hull, int worldHeadnode, const std::vector<VisBlockerBox> &blockers,
	const Vec3 &a, const Vec3 &b, TraceCounters &counters, HullOriginChain &chain)
{
	++counters.worldTraces;
	if (SegmentBlocked(hull, worldHeadnode, a, b, chain))
		return true;

	return BlockersOccluded(hull, blockers, a, b, counters);
}

class BinLayout
{
public:

	BinLayout(int yawBins, int pitchBins, double pitchRange)
		: m_yawBins(yawBins), m_pitchBins(pitchBins), m_pitchRange(pitchRange)
	{
		m_dirs.resize(static_cast<size_t>(m_yawBins) * static_cast<size_t>(m_pitchBins));
		for (int p = 0; p < m_pitchBins; ++p)
		{
			double pitch = -m_pitchRange + (p + 0.5) * (2.0 * m_pitchRange) / m_pitchBins;
			for (int y = 0; y < m_yawBins; ++y)
			{
				double yaw = -180.0 + (y + 0.5) * 360.0 / m_yawBins;
				m_dirs[Index(y, p)] = DirFromAngles(yaw, pitch);
			}
		}
	}

	size_t Count() const { return m_dirs.size(); }

	const Vec3 &Dir(size_t bin) const { return m_dirs[bin]; }

	size_t Index(int yawBin, int pitchBin) const
	{
		return static_cast<size_t>(pitchBin) * static_cast<size_t>(m_yawBins) +
			static_cast<size_t>(yawBin);
	}

	size_t BinFor(double yawDeg, double pitchDeg) const
	{
		double yn = (yawDeg + 180.0) / 360.0 * m_yawBins;
		int yb = static_cast<int>(std::floor(yn));
		yb %= m_yawBins;
		if (yb < 0)
			yb += m_yawBins;

		double clamped = std::max(-m_pitchRange, std::min(m_pitchRange, pitchDeg));
		double pn = (clamped + m_pitchRange) / (2.0 * m_pitchRange) * m_pitchBins;
		int pb = static_cast<int>(std::floor(pn));
		pb = std::max(0, std::min(m_pitchBins - 1, pb));

		return Index(yb, pb);
	}

private:
	int m_yawBins;
	int m_pitchBins;
	double m_pitchRange;
	std::vector<Vec3> m_dirs;
};

}
