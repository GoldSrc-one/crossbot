#include "sight_coverage.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>
#include <unordered_map>

#include "archtypes.h"
#include "mathlib.h"
#include "const.h"
#include "bspfile.h"
#include "studio.h"

#include "crc32.h"

namespace SightCoverage
{

namespace
{

constexpr float kPi = 3.14159265358979f;
constexpr float kMaxRasterExtent = 2048.f;
constexpr float kWindingSign = -1.f;
constexpr int kRingCount = 5;
constexpr int kRingViews[kRingCount] = { 1, 8, 16, 8, 1 };
constexpr int kRingFirst[kRingCount] = { 0, 1, 9, 25, 33 };
constexpr float kRingStepDeg = 45.f;

struct Tri
{
	float p[3][3];
	float st[3][2];
	float n[3];
	const uint8_t *pixels;
	int width;
	int height;
	float opacity;
	bool masked;
};

float Dot(const float a[3], const float b[3])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Cross(const float a[3], const float b[3], float out[3])
{
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}

bool Normalize(float v[3])
{
	float len = std::sqrt(Dot(v, v));
	if (len < 1e-8f)
		return false;
	v[0] /= len;
	v[1] /= len;
	v[2] /= len;
	return true;
}

bool WindingNormal(const float a[3], const float b[3], const float c[3], float n[3])
{
	float e1[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
	float e2[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
	Cross(e1, e2, n);
	n[0] *= kWindingSign;
	n[1] *= kWindingSign;
	n[2] *= kWindingSign;
	return Normalize(n);
}

void AngleVectors(const float angles[3], float forward[3], float right[3], float up[3])
{
	float deg = kPi / 180.f;
	float sy = std::sin(angles[1] * deg), cy = std::cos(angles[1] * deg);
	float sp = std::sin(angles[0] * deg), cp = std::cos(angles[0] * deg);
	float sr = std::sin(angles[2] * deg), cr = std::cos(angles[2] * deg);

	forward[0] = cp * cy;
	forward[1] = cp * sy;
	forward[2] = -sp;
	right[0] = -sr * sp * cy + cr * sy;
	right[1] = -sr * sp * sy - cr * cy;
	right[2] = -sr * cp;
	up[0] = cr * sp * cy + sr * sy;
	up[1] = cr * sp * sy - sr * cy;
	up[2] = cr * cp;
}

bool SegmentHitsBox(const float a[3], const float b[3], const float mins[3], const float maxs[3])
{
	float t0 = 0.f, t1 = 1.f;
	for (int i = 0; i < 3; ++i)
	{
		float d = b[i] - a[i];
		if (std::fabs(d) < 1e-8f)
		{
			if (a[i] < mins[i] || a[i] > maxs[i])
				return false;
			continue;
		}
		float n = (mins[i] - a[i]) / d;
		float f = (maxs[i] - a[i]) / d;
		if (n > f)
			std::swap(n, f);
		t0 = std::max(t0, n);
		t1 = std::min(t1, f);
		if (t0 > t1)
			return false;
	}
	return true;
}

void RasterTri(const Tri &tri, const float d[3], const float u[3], const float v[3], float u0, float v0, float step,
	int width, int height, int cellPx, std::vector<float> &trans, std::vector<uint8_t> &touched)
{
	if (tri.opacity <= 0.f || Dot(tri.n, d) >= 0.f)
		return;

	float x[3], y[3];
	for (int i = 0; i < 3; ++i)
	{
		x[i] = (Dot(tri.p[i], u) - u0) / step;
		y[i] = (Dot(tri.p[i], v) - v0) / step;
	}

	float area = (x[1] - x[0]) * (y[2] - y[0]) - (y[1] - y[0]) * (x[2] - x[0]);
	if (std::fabs(area) < 1e-6f)
		return;
	float inv = 1.f / area;

	int minX = std::max(0, static_cast<int>(std::floor(std::min({ x[0], x[1], x[2] }))));
	int maxX = std::min(width - 1, static_cast<int>(std::ceil(std::max({ x[0], x[1], x[2] }))));
	int minY = std::max(0, static_cast<int>(std::floor(std::min({ y[0], y[1], y[2] }))));
	int maxY = std::min(height - 1, static_cast<int>(std::ceil(std::max({ y[0], y[1], y[2] }))));
	if (minX > maxX || minY > maxY)
		return;

	int cellsW = width / cellPx;
	for (int cy = minY / cellPx; cy <= maxY / cellPx; ++cy)
	{
		for (int cx = minX / cellPx; cx <= maxX / cellPx; ++cx)
		{
			uint8_t &mark = touched[static_cast<size_t>(cy) * cellsW + cx];
			if (mark)
				continue;
			mark = 1;
			for (int py = cy * cellPx; py < (cy + 1) * cellPx; ++py)
				std::fill_n(trans.begin() + static_cast<size_t>(py) * width + cx * cellPx, cellPx, 1.f);
		}
	}

	float ax[3], by[3], c0[3];
	ax[0] = -(y[2] - y[1]) * inv;
	by[0] = (x[2] - x[1]) * inv;
	c0[0] = (-(x[2] - x[1]) * y[1] + (y[2] - y[1]) * x[1]) * inv;
	ax[1] = -(y[0] - y[2]) * inv;
	by[1] = (x[0] - x[2]) * inv;
	c0[1] = (-(x[0] - x[2]) * y[2] + (y[0] - y[2]) * x[2]) * inv;
	ax[2] = -ax[0] - ax[1];
	by[2] = -by[0] - by[1];
	c0[2] = 1.f - c0[0] - c0[1];

	bool sample = tri.masked && tri.pixels;
	float keep = 1.f - tri.opacity;
	float sBias = static_cast<float>(tri.width) * 4096.f;
	float tBias = static_cast<float>(tri.height) * 4096.f;
	float ds0 = tri.st[0][0] - tri.st[2][0], ds1 = tri.st[1][0] - tri.st[2][0];
	float dt0 = tri.st[0][1] - tri.st[2][1], dt1 = tri.st[1][1] - tri.st[2][1];

	for (int py = minY; py <= maxY; ++py)
	{
		float cy = py + 0.5f;
		float lo = static_cast<float>(minX), hi = static_cast<float>(maxX);
		bool empty = false;
		for (int e = 0; e < 3 && !empty; ++e)
		{
			float b = by[e] * cy + c0[e];
			if (std::fabs(ax[e]) < 1e-9f)
			{
				empty = b < 0.f;
				continue;
			}
			float root = -b / ax[e] - 0.5f;
			if (ax[e] > 0.f)
				lo = std::max(lo, std::ceil(root));
			else
				hi = std::min(hi, std::floor(root));
			empty = lo > hi;
		}
		if (empty)
			continue;

		int x0 = static_cast<int>(lo), x1 = static_cast<int>(hi);
		float *row = trans.data() + static_cast<size_t>(py) * width;
		if (!sample)
		{
			for (int px = x0; px <= x1; ++px)
				row[px] *= keep;
			continue;
		}

		float cx = x0 + 0.5f;
		float w0 = ax[0] * cx + by[0] * cy + c0[0];
		float w1 = ax[1] * cx + by[1] * cy + c0[1];
		float s = tri.st[2][0] + w0 * ds0 + w1 * ds1 + sBias;
		float t = tri.st[2][1] + w0 * dt0 + w1 * dt1 + tBias;
		float sStep = ax[0] * ds0 + ax[1] * ds1;
		float tStep = ax[0] * dt0 + ax[1] * dt1;
		for (int px = x0; px <= x1; ++px, s += sStep, t += tStep)
		{
			int tx = static_cast<int>(s) % tri.width;
			int ty = static_cast<int>(t) % tri.height;
			if (tx < 0 || ty < 0 || tri.pixels[ty * tri.width + tx] == 255)
				continue;
			row[px] *= keep;
		}
	}
}
bool BuildViews(const std::vector<Tri> &tris, Model &model)
{
	if (tris.empty())
		return false;

	for (int i = 0; i < 3; ++i)
	{
		model.mins[i] = tris[0].p[0][i];
		model.maxs[i] = tris[0].p[0][i];
	}
	for (const Tri &tri : tris)
	{
		for (int k = 0; k < 3; ++k)
		{
			for (int i = 0; i < 3; ++i)
			{
				model.mins[i] = std::min(model.mins[i], tri.p[k][i]);
				model.maxs[i] = std::max(model.maxs[i], tri.p[k][i]);
			}
		}
	}

	std::vector<float> trans;
	std::vector<uint8_t> touched;
	for (int view = 0; view < kViewCount; ++view)
	{
		float d[3], u[3], v[3];
		ViewBasis(view, d, u, v);

		float umin = 1e30f, umax = -1e30f, vmin = 1e30f, vmax = -1e30f;
		for (int c = 0; c < 8; ++c)
		{
			float corner[3] = {
				(c & 1) ? model.maxs[0] : model.mins[0],
				(c & 2) ? model.maxs[1] : model.mins[1],
				(c & 4) ? model.maxs[2] : model.mins[2] };
			float pu = Dot(corner, u), pv = Dot(corner, v);
			umin = std::min(umin, pu);
			umax = std::max(umax, pu);
			vmin = std::min(vmin, pv);
			vmax = std::max(vmax, pv);
		}

		View &out = model.views[view];
		out.u0 = std::floor(umin);
		out.v0 = std::floor(vmin);
		int cellsW = std::max(1, static_cast<int>(std::ceil((umax - out.u0) / kCellSize)));
		int cellsH = std::max(1, static_cast<int>(std::ceil((vmax - out.v0) / kCellSize)));
		out.w = static_cast<uint16_t>(cellsW);
		out.h = static_cast<uint16_t>(cellsH);

		float step = 1.f;
		while (std::max(cellsW, cellsH) * kCellSize / step > kMaxRasterExtent && step < kCellSize)
			step *= 2.f;
		int cellPx = static_cast<int>(kCellSize / step);
		int width = cellsW * cellPx;
		int height = cellsH * cellPx;

		if (trans.size() < static_cast<size_t>(width) * height)
			trans.resize(static_cast<size_t>(width) * height);
		touched.assign(static_cast<size_t>(cellsW) * cellsH, 0);
		for (const Tri &tri : tris)
			RasterTri(tri, d, u, v, out.u0, out.v0, step, width, height, cellPx, trans, touched);

		out.cells.assign(static_cast<size_t>(cellsW) * cellsH, 0);
		float norm = 1.f / static_cast<float>(cellPx * cellPx);
		for (int cy = 0; cy < cellsH; ++cy)
		{
			for (int cx = 0; cx < cellsW; ++cx)
			{
				if (!touched[static_cast<size_t>(cy) * cellsW + cx])
					continue;
				float sum = 0.f;
				for (int py = cy * cellPx; py < (cy + 1) * cellPx; ++py)
				{
					const float *row = trans.data() + static_cast<size_t>(py) * width + cx * cellPx;
					for (int px = 0; px < cellPx; ++px)
						sum += row[px];
				}
				float alpha = 1.f - sum * norm;
				int value = static_cast<int>(std::lround(alpha * 255.f));
				out.cells[static_cast<size_t>(cy) * cellsW + cx] = static_cast<uint8_t>(std::max(0, std::min(255, value)));
			}
		}
	}

	return true;
}

std::string JoinPath(const std::string &root, const std::string &rel)
{
	std::string path = root;
	if (!path.empty() && path.back() != '/' && path.back() != '\\')
		path += '/';
	for (char c : rel)
		path += c == '\\' ? '/' : c;
	return path;
}

std::string BaseName(const std::string &path)
{
	size_t slash = path.find_last_of("/\\");
	return slash == std::string::npos ? path : path.substr(slash + 1);
}

bool SameName(const char *a, const char *b)
{
	for (int i = 0; i < 16; ++i)
	{
		char ca = static_cast<char>(std::tolower(static_cast<unsigned char>(a[i])));
		char cb = static_cast<char>(std::tolower(static_cast<unsigned char>(b[i])));
		if (ca != cb)
			return false;
		if (ca == 0)
			return true;
	}
	return true;
}

std::string WorldspawnValue(const std::string &text, const char *key)
{
	size_t end = text.find('}');
	std::string needle = std::string("\"") + key + "\"";
	size_t pos = text.find(needle);
	if (pos == std::string::npos || pos > end)
		return std::string();
	size_t open = text.find('"', pos + needle.size());
	if (open == std::string::npos)
		return std::string();
	size_t close = text.find('"', open + 1);
	if (close == std::string::npos)
		return std::string();
	return text.substr(open + 1, close - open - 1);
}

bool SameLabel(const char *a, const char *b)
{
	for (; *a || *b; ++a, ++b)
	{
		if (std::tolower(static_cast<unsigned char>(*a)) != std::tolower(static_cast<unsigned char>(*b)))
			return false;
	}
	return true;
}

struct ClipVert
{
	float p[3];
	float st[2];
};

void ClipPolygon(std::vector<ClipVert> &poly, int axis, float bound, bool keepBelow)
{
	std::vector<ClipVert> out;
	size_t n = poly.size();
	for (size_t i = 0; i < n; ++i)
	{
		const ClipVert &a = poly[i];
		const ClipVert &b = poly[(i + 1) % n];
		float da = keepBelow ? bound - a.p[axis] : a.p[axis] - bound;
		float db = keepBelow ? bound - b.p[axis] : b.p[axis] - bound;
		if (da >= 0.f)
			out.push_back(a);
		if ((da >= 0.f) != (db >= 0.f))
		{
			float f = da / (da - db);
			ClipVert c;
			for (int k = 0; k < 3; ++k)
				c.p[k] = a.p[k] + (b.p[k] - a.p[k]) * f;
			for (int k = 0; k < 2; ++k)
				c.st[k] = a.st[k] + (b.st[k] - a.st[k]) * f;
			out.push_back(c);
		}
	}
	poly.swap(out);
}

void BuildChunks(const std::vector<Tri> &tris, std::vector<Model> &chunks)
{
	std::unordered_map<uint64_t, std::vector<Tri>> cells;
	std::vector<uint64_t> order;
	if (tris.empty())
		return;

	float base[3];
	for (int k = 0; k < 3; ++k)
	{
		base[k] = tris[0].p[0][k];
		for (const Tri &tri : tris)
			base[k] = std::min({ base[k], tri.p[0][k], tri.p[1][k], tri.p[2][k] });
	}

	for (const Tri &tri : tris)
	{
		int lo[3], hi[3];
		for (int k = 0; k < 3; ++k)
		{
			float mn = std::min({ tri.p[0][k], tri.p[1][k], tri.p[2][k] }) - base[k];
			float mx = std::max({ tri.p[0][k], tri.p[1][k], tri.p[2][k] }) - base[k];
			lo[k] = static_cast<int>(std::floor(mn / kChunkSize));
			hi[k] = std::max(lo[k], static_cast<int>(std::ceil(mx / kChunkSize)) - 1);
		}

		for (int cz = lo[2]; cz <= hi[2]; ++cz)
		{
			for (int cy = lo[1]; cy <= hi[1]; ++cy)
			{
				for (int cx = lo[0]; cx <= hi[0]; ++cx)
				{
					int c[3] = { cx, cy, cz };
					std::vector<ClipVert> poly(3);
					for (int i = 0; i < 3; ++i)
					{
						for (int k = 0; k < 3; ++k)
							poly[i].p[k] = tri.p[i][k];
						poly[i].st[0] = tri.st[i][0];
						poly[i].st[1] = tri.st[i][1];
					}
					for (int k = 0; k < 3 && poly.size() >= 3; ++k)
					{
						if (lo[k] != hi[k])
						{
							ClipPolygon(poly, k, base[k] + c[k] * kChunkSize, false);
							if (poly.size() >= 3)
								ClipPolygon(poly, k, base[k] + (c[k] + 1) * kChunkSize, true);
						}
					}
					if (poly.size() < 3)
						continue;

					uint64_t key = (static_cast<uint64_t>(cx + 32768) << 32) |
						(static_cast<uint64_t>(cy + 32768) << 16) | static_cast<uint64_t>(cz + 32768);
					auto it = cells.find(key);
					if (it == cells.end())
					{
						it = cells.emplace(key, std::vector<Tri>()).first;
						order.push_back(key);
					}

					for (size_t i = 1; i + 1 < poly.size(); ++i)
					{
						Tri piece = tri;
						const ClipVert *src[3] = { &poly[0], &poly[i], &poly[i + 1] };
						for (int v = 0; v < 3; ++v)
						{
							for (int k = 0; k < 3; ++k)
								piece.p[v][k] = src[v]->p[k];
							piece.st[v][0] = src[v]->st[0];
							piece.st[v][1] = src[v]->st[1];
						}
						it->second.push_back(piece);
					}
				}
			}
		}
	}

	std::vector<Model> built(order.size());
	std::vector<char> ok(order.size(), 0);
	std::atomic<size_t> next(0);
	auto work = [&]()
	{
		for (size_t i = next++; i < order.size(); i = next++)
			ok[i] = BuildViews(cells.find(order[i])->second, built[i]) ? 1 : 0;
	};

	size_t threads = std::min<size_t>(order.size(), std::max(1u, std::thread::hardware_concurrency()));
	std::vector<std::thread> pool;
	for (size_t i = 1; i < threads; ++i)
		pool.emplace_back(work);
	work();
	for (std::thread &thread : pool)
		thread.join();

	for (size_t i = 0; i < order.size(); ++i)
	{
		if (ok[i])
			chunks.push_back(std::move(built[i]));
	}
}

bool ReadAsset(const std::string &root, const std::string &rel, std::vector<uint8_t> &out)
{
	return ReadFile(JoinPath(root, rel), out) || ReadFile(JoinPath(root + "/../valve", rel), out);
}

float AnimValue(const mstudioanim_t *anim, int channel)
{
	const mstudioanimvalue_t *value = reinterpret_cast<const mstudioanimvalue_t *>(
		reinterpret_cast<const uint8_t *>(anim) + anim->offset[channel]);
	if (value->num.valid > 0)
		return value[1].value;
	return value[0].value;
}

void AngleQuaternion(const float angles[3], float q[4])
{
	float sy = std::sin(angles[2] * 0.5f), cy = std::cos(angles[2] * 0.5f);
	float sp = std::sin(angles[1] * 0.5f), cp = std::cos(angles[1] * 0.5f);
	float sr = std::sin(angles[0] * 0.5f), cr = std::cos(angles[0] * 0.5f);

	q[0] = sr * cp * cy - cr * sp * sy;
	q[1] = cr * sp * cy + sr * cp * sy;
	q[2] = cr * cp * sy - sr * sp * cy;
	q[3] = cr * cp * cy + sr * sp * sy;
}

void QuaternionMatrix(const float q[4], const float pos[3], float m[3][4])
{
	m[0][0] = 1.f - 2.f * q[1] * q[1] - 2.f * q[2] * q[2];
	m[1][0] = 2.f * q[0] * q[1] + 2.f * q[3] * q[2];
	m[2][0] = 2.f * q[0] * q[2] - 2.f * q[3] * q[1];
	m[0][1] = 2.f * q[0] * q[1] - 2.f * q[3] * q[2];
	m[1][1] = 1.f - 2.f * q[0] * q[0] - 2.f * q[2] * q[2];
	m[2][1] = 2.f * q[1] * q[2] + 2.f * q[3] * q[0];
	m[0][2] = 2.f * q[0] * q[2] + 2.f * q[3] * q[1];
	m[1][2] = 2.f * q[1] * q[2] - 2.f * q[3] * q[0];
	m[2][2] = 1.f - 2.f * q[0] * q[0] - 2.f * q[1] * q[1];
	m[0][3] = pos[0];
	m[1][3] = pos[1];
	m[2][3] = pos[2];
}

void ConcatTransforms(const float a[3][4], const float b[3][4], float out[3][4])
{
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
			if (c == 3)
				out[r][c] += a[r][3];
		}
	}
}

void Transform(const float m[3][4], const float in[3], float out[3])
{
	for (int r = 0; r < 3; ++r)
		out[r] = m[r][0] * in[0] + m[r][1] * in[1] + m[r][2] * in[2] + m[r][3];
}

bool InFile(const std::vector<uint8_t> &data, size_t offset, size_t bytes)
{
	return offset <= data.size() && bytes <= data.size() - offset;
}

}

float RenderOpacity(int rendermode, float renderamt)
{
	switch (rendermode)
	{
	case kRenderNormal:
		return 1.f;
	case kRenderTransColor:
	case kRenderTransTexture:
	case kRenderTransAlpha:
		return std::max(0.f, std::min(1.f, renderamt / 255.f));
	default:
		return 0.f;
	}
}

void ViewBasis(int view, float d[3], float u[3], float v[3])
{
	int ring = 0;
	while (ring + 1 < kRingCount && view >= kRingFirst[ring + 1])
		++ring;

	float lat = (ring - 2) * kRingStepDeg * kPi / 180.f;
	float az = (view - kRingFirst[ring]) * 2.f * kPi / kRingViews[ring];

	d[0] = std::cos(lat) * std::cos(az);
	d[1] = std::cos(lat) * std::sin(az);
	d[2] = std::sin(lat);

	if (kRingViews[ring] == 1)
	{
		d[0] = 0.f;
		d[1] = 0.f;
		d[2] = ring == 0 ? -1.f : 1.f;
		u[0] = 1.f;
		u[1] = 0.f;
		u[2] = 0.f;
	}
	else
	{
		u[0] = -d[1];
		u[1] = d[0];
		u[2] = 0.f;
		Normalize(u);
	}

	Cross(d, u, v);
}

int NearestView(const float dir[3])
{
	float z = std::max(-1.f, std::min(1.f, dir[2]));
	float lat = std::asin(z) * 180.f / kPi;
	int ring = static_cast<int>(std::lround(lat / kRingStepDeg)) + 2;
	ring = std::max(0, std::min(kRingCount - 1, ring));

	int count = kRingViews[ring];
	if (count == 1)
		return kRingFirst[ring];

	float az = std::atan2(dir[1], dir[0]);
	int k = static_cast<int>(std::lround(az / (2.f * kPi / count)));
	k %= count;
	if (k < 0)
		k += count;
	return kRingFirst[ring] + k;
}

float MeanAlpha(const View &view)
{
	if (view.cells.empty())
		return 0.f;
	float sum = 0.f;
	for (uint8_t c : view.cells)
		sum += c;
	return sum / (255.f * view.cells.size());
}

void Place(Placed &placed, const std::vector<Model> &chunks, const float origin[3], const float angles[3],
	bool studio)
{
	for (int i = 0; i < 3; ++i)
	{
		placed.origin[i] = origin[i];
		placed.angles[i] = angles[i];
		placed.absmin[i] = 1e30f;
		placed.absmax[i] = -1e30f;
	}

	float turned[3] = { studio ? -angles[0] : angles[0], angles[1], angles[2] };
	float f[3], r[3], u[3];
	AngleVectors(turned, f, r, u);

	placed.chunks.assign(chunks.size(), Instance());
	for (size_t n = 0; n < chunks.size(); ++n)
	{
		Instance &instance = placed.chunks[n];
		const Model &model = chunks[n];
		instance.model = &model;
		for (int i = 0; i < 3; ++i)
		{
			instance.origin[i] = origin[i];
			instance.axes[0][i] = f[i];
			instance.axes[1][i] = -r[i];
			instance.axes[2][i] = u[i];
			instance.absmin[i] = 1e30f;
			instance.absmax[i] = -1e30f;
		}

		for (int c = 0; c < 8; ++c)
		{
			float corner[3] = {
				(c & 1) ? model.maxs[0] : model.mins[0],
				(c & 2) ? model.maxs[1] : model.mins[1],
				(c & 4) ? model.maxs[2] : model.mins[2] };
			for (int i = 0; i < 3; ++i)
			{
				float w = origin[i] + instance.axes[0][i] * corner[0] + instance.axes[1][i] * corner[1] +
					instance.axes[2][i] * corner[2];
				instance.absmin[i] = std::min(instance.absmin[i], w);
				instance.absmax[i] = std::max(instance.absmax[i], w);
			}
		}

		for (int i = 0; i < 3; ++i)
		{
			placed.absmin[i] = std::min(placed.absmin[i], instance.absmin[i]);
			placed.absmax[i] = std::max(placed.absmax[i], instance.absmax[i]);
		}
	}
}

float Alpha(const Placed &placed, const float from[3], const float to[3])
{
	if (placed.chunks.empty() || !SegmentHitsBox(from, to, placed.absmin, placed.absmax))
		return 0.f;

	float clear = 1.f;
	for (const Instance &instance : placed.chunks)
		clear *= 1.f - Alpha(instance, from, to);
	return 1.f - clear;
}

float Alpha(const Instance &instance, const float from[3], const float to[3])
{
	const Model *model = instance.model;
	if (!model || !SegmentHitsBox(from, to, instance.absmin, instance.absmax))
		return 0.f;

	float a[3], b[3];
	for (int k = 0; k < 3; ++k)
	{
		float wa[3] = { from[0] - instance.origin[0], from[1] - instance.origin[1], from[2] - instance.origin[2] };
		float wb[3] = { to[0] - instance.origin[0], to[1] - instance.origin[1], to[2] - instance.origin[2] };
		a[k] = Dot(wa, instance.axes[k]);
		b[k] = Dot(wb, instance.axes[k]);
	}
	if (!SegmentHitsBox(a, b, model->mins, model->maxs))
		return 0.f;

	float r[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
	float len = std::sqrt(Dot(r, r));
	if (len < 1e-4f)
		return 0.f;
	r[0] /= len;
	r[1] /= len;
	r[2] /= len;

	int index = NearestView(r);
	float d[3], u[3], v[3];
	ViewBasis(index, d, u, v);
	float along = Dot(r, d);
	if (along < 1e-3f)
		return 0.f;

	float c[3] = {
		(model->mins[0] + model->maxs[0]) * 0.5f,
		(model->mins[1] + model->maxs[1]) * 0.5f,
		(model->mins[2] + model->maxs[2]) * 0.5f };
	float toCentre[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
	float t = std::max(0.f, std::min(len, Dot(toCentre, d) / along));
	float p[3] = { a[0] + r[0] * t, a[1] + r[1] * t, a[2] + r[2] * t };

	const View &view = model->views[index];
	int cx = static_cast<int>(std::floor((Dot(p, u) - view.u0) / kCellSize));
	int cy = static_cast<int>(std::floor((Dot(p, v) - view.v0) / kCellSize));
	if (cx < 0 || cy < 0 || cx >= view.w || cy >= view.h)
		return 0.f;
	return view.cells[static_cast<size_t>(cy) * view.w + cx] / 255.f;
}

void Grid::Build(const std::vector<const Placed *> &placed)
{
	m_chunks.clear();
	for (int k = 0; k < 3; ++k)
	{
		m_mins[k] = 1e30f;
		m_maxs[k] = -1e30f;
	}
	for (const Placed *p : placed)
	{
		for (const Instance &chunk : p->chunks)
		{
			m_chunks.push_back(&chunk);
			for (int k = 0; k < 3; ++k)
			{
				m_mins[k] = std::min(m_mins[k], chunk.absmin[k]);
				m_maxs[k] = std::max(m_maxs[k], chunk.absmax[k]);
			}
		}
	}
	if (m_chunks.empty())
		return;

	for (int k = 0; k < 3; ++k)
		m_dims[k] = std::max(1, static_cast<int>(std::ceil((m_maxs[k] - m_mins[k]) / kChunkSize)));

	std::vector<std::vector<uint32_t>> cells(static_cast<size_t>(m_dims[0]) * m_dims[1] * m_dims[2]);
	for (uint32_t i = 0; i < m_chunks.size(); ++i)
	{
		const Instance &chunk = *m_chunks[i];
		int lo[3], hi[3];
		for (int k = 0; k < 3; ++k)
		{
			lo[k] = std::max(0, static_cast<int>((chunk.absmin[k] - m_mins[k]) / kChunkSize));
			hi[k] = std::min(m_dims[k] - 1, static_cast<int>((chunk.absmax[k] - m_mins[k]) / kChunkSize));
		}
		for (int z = lo[2]; z <= hi[2]; ++z)
			for (int y = lo[1]; y <= hi[1]; ++y)
				for (int x = lo[0]; x <= hi[0]; ++x)
					cells[(static_cast<size_t>(z) * m_dims[1] + y) * m_dims[0] + x].push_back(i);
	}

	m_cellStart.assign(cells.size() + 1, 0);
	m_items.clear();
	for (size_t c = 0; c < cells.size(); ++c)
	{
		m_cellStart[c] = static_cast<uint32_t>(m_items.size());
		m_items.insert(m_items.end(), cells[c].begin(), cells[c].end());
	}
	m_cellStart[cells.size()] = static_cast<uint32_t>(m_items.size());
}

float Grid::Clear(const float from[3], const float to[3], float stopBelow) const
{
	if (m_chunks.empty() || !SegmentHitsBox(from, to, m_mins, m_maxs))
		return 1.f;

	thread_local std::vector<uint32_t> stamps;
	thread_local uint32_t tick = 0;
	if (stamps.size() < m_chunks.size())
		stamps.assign(m_chunks.size(), 0);
	if (++tick == 0)
	{
		std::fill(stamps.begin(), stamps.end(), 0);
		tick = 1;
	}

	float p0[3], d[3];
	float t0 = 0.f, t1 = 1.f;
	for (int k = 0; k < 3; ++k)
	{
		p0[k] = (from[k] - m_mins[k]) / kChunkSize;
		d[k] = (to[k] - from[k]) / kChunkSize;
		float hi = static_cast<float>(m_dims[k]);
		if (std::fabs(d[k]) < 1e-9f)
		{
			if (p0[k] < 0.f || p0[k] > hi)
				return 1.f;
			continue;
		}
		float a = -p0[k] / d[k];
		float b = (hi - p0[k]) / d[k];
		if (a > b)
			std::swap(a, b);
		t0 = std::max(t0, a);
		t1 = std::min(t1, b);
	}
	if (t0 > t1)
		return 1.f;

	int cell[3], step[3];
	float tMax[3], tDelta[3];
	for (int k = 0; k < 3; ++k)
	{
		float s = p0[k] + d[k] * t0;
		cell[k] = std::max(0, std::min(m_dims[k] - 1, static_cast<int>(std::floor(s))));
		step[k] = d[k] > 0.f ? 1 : -1;
		if (std::fabs(d[k]) < 1e-9f)
		{
			tMax[k] = 1e30f;
			tDelta[k] = 1e30f;
		}
		else
		{
			float edge = static_cast<float>(cell[k] + (d[k] > 0.f ? 1 : 0));
			tMax[k] = (edge - p0[k]) / d[k];
			tDelta[k] = 1.f / std::fabs(d[k]);
		}
	}

	float clear = 1.f;
	for (;;)
	{
		size_t c = (static_cast<size_t>(cell[2]) * m_dims[1] + cell[1]) * m_dims[0] + cell[0];
		for (uint32_t i = m_cellStart[c]; i < m_cellStart[c + 1]; ++i)
		{
			uint32_t item = m_items[i];
			if (stamps[item] == tick)
				continue;
			stamps[item] = tick;
			clear *= 1.f - Alpha(*m_chunks[item], from, to);
			if (clear < stopBelow)
				return clear;
		}

		int axis = tMax[0] < tMax[1] ? (tMax[0] < tMax[2] ? 0 : 2) : (tMax[1] < tMax[2] ? 1 : 2);
		if (tMax[axis] > t1)
			break;
		cell[axis] += step[axis];
		tMax[axis] += tDelta[axis];
		if (cell[axis] < 0 || cell[axis] >= m_dims[axis])
			break;
	}
	return clear;
}
bool ReadFile(const std::string &path, std::vector<uint8_t> &out)
{
	FILE *f = std::fopen(path.c_str(), "rb");
	if (!f)
		return false;
	std::fseek(f, 0, SEEK_END);
	long size = std::ftell(f);
	std::fseek(f, 0, SEEK_SET);
	if (size <= 0)
	{
		std::fclose(f);
		return false;
	}
	out.resize(static_cast<size_t>(size));
	bool ok = std::fread(out.data(), 1, out.size(), f) == out.size();
	std::fclose(f);
	return ok;
}

std::string AssetRoot(const std::string &bspPath)
{
	size_t slash = bspPath.find_last_of("/\\");
	if (slash == std::string::npos)
		return "..";
	std::string dir = bspPath.substr(0, slash);
	size_t up = dir.find_last_of("/\\");
	if (up == std::string::npos)
		return ".";
	return dir.substr(0, up);
}

uint32_t StudioChecksum(const std::string &root, const std::string &modelPath)
{
	std::vector<uint8_t> data;
	if (!ReadAsset(root, modelPath, data))
		return 0;
	uint32_t sum = navgen::Crc32(data.data(), data.size());

	std::string texPath = modelPath;
	if (texPath.size() > 4)
		texPath.insert(texPath.size() - 4, "T");
	std::vector<uint8_t> tex;
	if (ReadAsset(root, texPath, tex))
		sum ^= navgen::Crc32(tex.data(), tex.size()) * 2654435761u;
	return sum == 0 ? 1u : sum;
}
const uint8_t *Bsp::Lump(int index, size_t elemSize, int &count) const
{
	count = 0;
	const dheader_t *header = reinterpret_cast<const dheader_t *>(m_data.data());
	const lump_t &lump = header->lumps[index];
	if (lump.filelen < 0 || lump.fileofs < 0 ||
		!InFile(m_data, static_cast<size_t>(lump.fileofs), static_cast<size_t>(lump.filelen)))
		return nullptr;
	count = static_cast<int>(static_cast<size_t>(lump.filelen) / elemSize);
	return m_data.data() + lump.fileofs;
}

bool Bsp::Open(const std::string &bspPath)
{
	if (!ReadFile(bspPath, m_data) || m_data.size() < sizeof(dheader_t))
		return false;

	const dheader_t *header = reinterpret_cast<const dheader_t *>(m_data.data());
	if (header->version != HLBSP_VERSION)
		return false;

	m_root = AssetRoot(bspPath);

	int entBytes = 0;
	const uint8_t *ents = Lump(LUMP_ENTITIES, 1, entBytes);
	if (ents)
		m_entityText.assign(reinterpret_cast<const char *>(ents), static_cast<size_t>(entBytes));

	int planes = 0, clipnodes = 0;
	const uint8_t *planeData = Lump(LUMP_PLANES, sizeof(dplane_t), planes);
	const uint8_t *clipData = Lump(LUMP_CLIPNODES, sizeof(dclipnode_t), clipnodes);
	const uint8_t *modelData = Lump(LUMP_MODELS, sizeof(dmodel_t), m_modelCount);
	if (!planeData || !clipData || !modelData)
		return false;
	m_checksum = navgen::BspChecksumLumps(planeData, planes * sizeof(dplane_t), clipData,
		clipnodes * sizeof(dclipnode_t), modelData, m_modelCount * sizeof(dmodel_t));

	int texBytes = 0;
	const uint8_t *texLump = Lump(LUMP_TEXTURES, 1, texBytes);
	int texCount = 0;
	if (texLump && texBytes >= 4)
		std::memcpy(&texCount, texLump, sizeof(int));
	m_textures.assign(static_cast<size_t>(std::max(0, texCount)), Texture());

	std::string wads = WorldspawnValue(m_entityText, "wad");
	size_t start = 0;
	while (start < wads.size())
	{
		size_t semi = wads.find(';', start);
		std::string entry = wads.substr(start, semi == std::string::npos ? std::string::npos : semi - start);
		if (!entry.empty())
			m_wadNames.push_back(BaseName(entry));
		if (semi == std::string::npos)
			break;
		start = semi + 1;
	}

	return true;
}

void Bsp::LoadWads()
{
	m_wadsLoaded = true;
	for (const std::string &name : m_wadNames)
	{
		std::vector<uint8_t> data;
		if (ReadAsset(m_root, name, data) && data.size() >= 12)
			m_wads.push_back(std::move(data));
		else
			m_missing.push_back(name);
	}
}

const Bsp::Texture &Bsp::TextureAt(int miptex)
{
	static Texture none;
	if (miptex < 0 || static_cast<size_t>(miptex) >= m_textures.size())
		return none;

	Texture &tex = m_textures[miptex];
	if (tex.resolved)
		return tex;
	tex.resolved = true;

	int texBytes = 0;
	const uint8_t *texLump = Lump(LUMP_TEXTURES, 1, texBytes);
	int offset = 0;
	std::memcpy(&offset, texLump + 4 + miptex * 4, sizeof(int));
	if (offset < 0 || static_cast<size_t>(offset) + sizeof(miptex_t) > static_cast<size_t>(texBytes))
		return tex;

	const miptex_t *mip = reinterpret_cast<const miptex_t *>(texLump + offset);
	tex.name.assign(mip->name, strnlen(mip->name, sizeof(mip->name)));
	tex.width = static_cast<int>(mip->width);
	tex.height = static_cast<int>(mip->height);
	if (tex.width <= 0 || tex.height <= 0)
		return tex;

	size_t pixelBytes = static_cast<size_t>(tex.width) * tex.height;
	if (mip->offsets[0] != 0 && static_cast<size_t>(offset) + mip->offsets[0] + pixelBytes <= static_cast<size_t>(texBytes))
	{
		tex.pixels = texLump + offset + mip->offsets[0];
		return tex;
	}

	if (!m_wadsLoaded)
		LoadWads();

	for (const std::vector<uint8_t> &wad : m_wads)
	{
		int lumps = 0, info = 0;
		std::memcpy(&lumps, wad.data() + 4, sizeof(int));
		std::memcpy(&info, wad.data() + 8, sizeof(int));
		for (int i = 0; i < lumps; ++i)
		{
			size_t at = static_cast<size_t>(info) + static_cast<size_t>(i) * 32;
			if (!InFile(wad, at, 32))
				break;
			const char *name = reinterpret_cast<const char *>(wad.data() + at + 16);
			if (!SameName(name, mip->name))
				continue;

			int filepos = 0;
			std::memcpy(&filepos, wad.data() + at, sizeof(int));
			if (filepos < 0 || !InFile(wad, static_cast<size_t>(filepos), sizeof(miptex_t)))
				continue;
			const miptex_t *wmip = reinterpret_cast<const miptex_t *>(wad.data() + filepos);
			if (static_cast<int>(wmip->width) != tex.width || static_cast<int>(wmip->height) != tex.height)
				continue;
			if (!InFile(wad, static_cast<size_t>(filepos) + wmip->offsets[0], pixelBytes))
				continue;
			tex.pixels = wad.data() + filepos + wmip->offsets[0];
			return tex;
		}
	}

	m_missing.push_back(tex.name);
	return tex;
}

void Bsp::BuildBrush(int submodel, int rendermode, float renderamt, std::vector<Model> &chunks, Stats &stats)
{
	float opacity = RenderOpacity(rendermode, renderamt);
	if (opacity <= 0.f || submodel <= 0 || submodel >= m_modelCount)
		return;

	int count = 0;
	const dmodel_t *model = reinterpret_cast<const dmodel_t *>(Lump(LUMP_MODELS, sizeof(dmodel_t), count)) + submodel;
	int faceCount = 0, edgeCount = 0, surfCount = 0, vertCount = 0, texinfoCount = 0, planeCount = 0;
	const dface_t *faces = reinterpret_cast<const dface_t *>(Lump(LUMP_FACES, sizeof(dface_t), faceCount));
	const dedge_t *edges = reinterpret_cast<const dedge_t *>(Lump(LUMP_EDGES, sizeof(dedge_t), edgeCount));
	const int *surfedges = reinterpret_cast<const int *>(Lump(LUMP_SURFEDGES, sizeof(int), surfCount));
	const dvertex_t *verts = reinterpret_cast<const dvertex_t *>(Lump(LUMP_VERTEXES, sizeof(dvertex_t), vertCount));
	const texinfo_t *texinfos = reinterpret_cast<const texinfo_t *>(Lump(LUMP_TEXINFO, sizeof(texinfo_t), texinfoCount));
	const dplane_t *planes = reinterpret_cast<const dplane_t *>(Lump(LUMP_PLANES, sizeof(dplane_t), planeCount));
	if (!faces || !edges || !surfedges || !verts || !texinfos || !planes)
		return;

	std::vector<Tri> tris;

	for (int f = model->firstface; f < model->firstface + model->numfaces && f < faceCount; ++f)
	{
		const dface_t &face = faces[f];
		if (face.texinfo < 0 || face.texinfo >= texinfoCount || face.planenum < 0 || face.planenum >= planeCount)
			continue;

		const texinfo_t &ti = texinfos[face.texinfo];
		const Texture &tex = TextureAt(ti._miptex);
		bool masked = rendermode == kRenderTransAlpha && !tex.name.empty() && tex.name[0] == '{';

		float n[3] = { planes[face.planenum].normal[0], planes[face.planenum].normal[1], planes[face.planenum].normal[2] };
		if (face.side)
		{
			n[0] = -n[0];
			n[1] = -n[1];
			n[2] = -n[2];
		}

		std::vector<const float *> ring;
		for (int e = 0; e < face.numedges; ++e)
		{
			int index = face.firstedge + e;
			if (index < 0 || index >= surfCount)
				break;
			int se = surfedges[index];
			int edge = se >= 0 ? se : -se;
			if (edge >= edgeCount)
				break;
			int vi = se >= 0 ? edges[edge].v[0] : edges[edge].v[1];
			if (vi >= vertCount)
				break;
			ring.push_back(verts[vi].point);
		}
		if (ring.size() < 3)
			continue;

		for (size_t k = 1; k + 1 < ring.size(); ++k)
		{
			Tri tri;
			const float *src[3] = { ring[0], ring[k], ring[k + 1] };
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
					tri.p[i][j] = src[i][j];
				tri.st[i][0] = Dot(src[i], ti.vecs[0]) + ti.vecs[0][3];
				tri.st[i][1] = Dot(src[i], ti.vecs[1]) + ti.vecs[1][3];
			}
			tri.n[0] = n[0];
			tri.n[1] = n[1];
			tri.n[2] = n[2];
			tri.pixels = tex.pixels;
			tri.width = tex.width;
			tri.height = tex.height;
			tri.opacity = opacity;
			tri.masked = masked;

			float wn[3];
			if (WindingNormal(tri.p[0], tri.p[1], tri.p[2], wn))
			{
				++stats.windingChecked;
				if (Dot(wn, n) < 0.f)
					++stats.windingMismatch;
			}

			++stats.triangles;
			tris.push_back(tri);
		}
	}

	BuildChunks(tris, chunks);
}

bool BuildStudio(const std::string &root, const std::string &modelPath, int sequence, const char *sequenceLabel,
	int body, int skin, int rendermode, float renderamt, std::vector<Model> &chunks, Stats &stats,
	std::string &why)
{
	float opacity = RenderOpacity(rendermode, renderamt);
	if (opacity <= 0.f)
	{
		why = "render mode is see-through";
		return false;
	}

	std::vector<uint8_t> data;
	if (!ReadAsset(root, modelPath, data) || data.size() < sizeof(studiohdr_t))
	{
		why = "cannot read " + JoinPath(root, modelPath);
		return false;
	}

	const studiohdr_t *hdr = reinterpret_cast<const studiohdr_t *>(data.data());
	if (std::memcmp(&hdr->id, "IDST", 4) != 0 || hdr->version != 10)
	{
		why = "not a studio v10 model";
		return false;
	}

	std::vector<uint8_t> texData;
	const std::vector<uint8_t> *texFile = &data;
	if (hdr->numtextures == 0)
	{
		std::string texPath = modelPath;
		if (texPath.size() > 4)
			texPath.insert(texPath.size() - 4, "T");
		if (ReadAsset(root, texPath, texData) && texData.size() >= sizeof(studiohdr_t))
			texFile = &texData;
	}
	const studiohdr_t *texHdr = reinterpret_cast<const studiohdr_t *>(texFile->data());

	if (hdr->numbones <= 0 || hdr->numbones > MAXSTUDIOBONES ||
		!InFile(data, static_cast<size_t>(hdr->boneindex), sizeof(mstudiobone_t) * hdr->numbones) ||
		!InFile(data, static_cast<size_t>(hdr->bodypartindex), sizeof(mstudiobodyparts_t) * hdr->numbodyparts))
	{
		why = "bad bone or bodypart table";
		return false;
	}

	if (hdr->numseq > 0 && InFile(data, static_cast<size_t>(hdr->seqindex), sizeof(mstudioseqdesc_t) * hdr->numseq))
	{
		const mstudioseqdesc_t *seqs = reinterpret_cast<const mstudioseqdesc_t *>(data.data() + hdr->seqindex);
		if (sequenceLabel && *sequenceLabel)
		{
			for (int i = 0; i < hdr->numseq; ++i)
			{
				if (SameLabel(seqs[i].label, sequenceLabel))
				{
					sequence = i;
					break;
				}
			}
		}
	}
	if (sequence < 0 || sequence >= hdr->numseq)
		sequence = 0;

	const mstudiobone_t *bones = reinterpret_cast<const mstudiobone_t *>(data.data() + hdr->boneindex);
	const mstudioanim_t *anim = nullptr;
	if (hdr->numseq > 0 && InFile(data, static_cast<size_t>(hdr->seqindex), sizeof(mstudioseqdesc_t) * hdr->numseq))
	{
		const mstudioseqdesc_t &seq = reinterpret_cast<const mstudioseqdesc_t *>(data.data() + hdr->seqindex)[sequence];
		if (seq.seqgroup == 0 && hdr->numseqgroups > 0 &&
			InFile(data, static_cast<size_t>(hdr->seqgroupindex), sizeof(mstudioseqgroup_t)))
		{
			const mstudioseqgroup_t *group = reinterpret_cast<const mstudioseqgroup_t *>(data.data() + hdr->seqgroupindex);
			size_t at = static_cast<size_t>(group->unused2) + static_cast<size_t>(seq.animindex);
			if (InFile(data, at, sizeof(mstudioanim_t) * hdr->numbones))
				anim = reinterpret_cast<const mstudioanim_t *>(data.data() + at);
		}
	}

	float boneMats[MAXSTUDIOBONES][3][4];
	for (int b = 0; b < hdr->numbones; ++b)
	{
		const mstudiobone_t &bone = bones[b];
		float pos[3], ang[3];
		for (int j = 0; j < 3; ++j)
		{
			pos[j] = bone.value[j];
			ang[j] = bone.value[j + 3];
			if (anim && anim[b].offset[j] && InFile(data,
				static_cast<size_t>(reinterpret_cast<const uint8_t *>(&anim[b]) - data.data()) + anim[b].offset[j], 4))
				pos[j] += AnimValue(&anim[b], j) * bone.scale[j];
			if (anim && anim[b].offset[j + 3] && InFile(data,
				static_cast<size_t>(reinterpret_cast<const uint8_t *>(&anim[b]) - data.data()) + anim[b].offset[j + 3], 4))
				ang[j] += AnimValue(&anim[b], j + 3) * bone.scale[j + 3];
		}

		float q[4];
		float local[3][4];
		AngleQuaternion(ang, q);
		QuaternionMatrix(q, pos, local);

		if (bone.parent >= 0 && bone.parent < b)
			ConcatTransforms(boneMats[bone.parent], local, boneMats[b]);
		else
			std::memcpy(boneMats[b], local, sizeof(local));
	}

	const mstudiotexture_t *textures = nullptr;
	const short *skinTable = nullptr;
	if (texHdr->numtextures > 0 &&
		InFile(*texFile, static_cast<size_t>(texHdr->textureindex), sizeof(mstudiotexture_t) * texHdr->numtextures) &&
		InFile(*texFile, static_cast<size_t>(texHdr->skinindex),
			sizeof(short) * std::max(1, texHdr->numskinfamilies) * texHdr->numskinref))
	{
		textures = reinterpret_cast<const mstudiotexture_t *>(texFile->data() + texHdr->textureindex);
		skinTable = reinterpret_cast<const short *>(texFile->data() + texHdr->skinindex);
	}
	if (skin < 0 || skin >= texHdr->numskinfamilies)
		skin = 0;

	std::vector<Tri> tris;
	const mstudiobodyparts_t *parts = reinterpret_cast<const mstudiobodyparts_t *>(data.data() + hdr->bodypartindex);
	for (int bp = 0; bp < hdr->numbodyparts; ++bp)
	{
		const mstudiobodyparts_t &part = parts[bp];
		if (part.nummodels <= 0 || part.base <= 0)
			continue;
		int index = (body / part.base) % part.nummodels;
		size_t modelAt = static_cast<size_t>(part.modelindex) + sizeof(mstudiomodel_t) * index;
		if (!InFile(data, modelAt, sizeof(mstudiomodel_t)))
			continue;
		const mstudiomodel_t &sub = *reinterpret_cast<const mstudiomodel_t *>(data.data() + modelAt);
		if (sub.numverts <= 0 ||
			!InFile(data, static_cast<size_t>(sub.vertindex), sizeof(vec3_t) * sub.numverts) ||
			!InFile(data, static_cast<size_t>(sub.vertinfoindex), static_cast<size_t>(sub.numverts)) ||
			!InFile(data, static_cast<size_t>(sub.meshindex), sizeof(mstudiomesh_t) * sub.nummesh))
			continue;

		const float *rawVerts = reinterpret_cast<const float *>(data.data() + sub.vertindex);
		const uint8_t *vertBone = data.data() + sub.vertinfoindex;
		std::vector<float> xv(static_cast<size_t>(sub.numverts) * 3);
		for (int v = 0; v < sub.numverts; ++v)
		{
			int b = vertBone[v] < hdr->numbones ? vertBone[v] : 0;
			Transform(boneMats[b], rawVerts + v * 3, &xv[static_cast<size_t>(v) * 3]);
		}

		const mstudiomesh_t *meshes = reinterpret_cast<const mstudiomesh_t *>(data.data() + sub.meshindex);
		for (int mi = 0; mi < sub.nummesh; ++mi)
		{
			const mstudiomesh_t &mesh = meshes[mi];
			const uint8_t *pixels = nullptr;
			int width = 0, height = 0;
			bool masked = false;
			if (textures && mesh.skinref >= 0 && mesh.skinref < texHdr->numskinref)
			{
				int texIndex = skinTable[skin * texHdr->numskinref + mesh.skinref];
				if (texIndex >= 0 && texIndex < texHdr->numtextures)
				{
					const mstudiotexture_t &tex = textures[texIndex];
					if (tex.flags & STUDIO_NF_ADDITIVE)
						continue;
					masked = (tex.flags & STUDIO_NF_MASKED) != 0;
					if (tex.width > 0 && tex.height > 0 &&
						InFile(*texFile, static_cast<size_t>(tex.index), static_cast<size_t>(tex.width) * tex.height))
					{
						pixels = texFile->data() + tex.index;
						width = tex.width;
						height = tex.height;
					}
				}
			}

			size_t cmdAt = static_cast<size_t>(mesh.triindex);
			std::vector<int> fanVerts;
			std::vector<float> fanST;
			while (InFile(data, cmdAt, sizeof(short)))
			{
				short cmd = *reinterpret_cast<const short *>(data.data() + cmdAt);
				cmdAt += sizeof(short);
				if (cmd == 0)
					break;
				bool fan = cmd < 0;
				int n = fan ? -cmd : cmd;
				if (!InFile(data, cmdAt, sizeof(short) * 4 * static_cast<size_t>(n)))
					break;

				const short *entry = reinterpret_cast<const short *>(data.data() + cmdAt);
				cmdAt += sizeof(short) * 4 * static_cast<size_t>(n);

				for (int k = 2; k < n; ++k)
				{
					int ia, ib, ic = k;
					if (fan)
					{
						ia = 0;
						ib = k - 1;
					}
					else if (k % 2 == 0)
					{
						ia = k - 2;
						ib = k - 1;
					}
					else
					{
						ia = k - 1;
						ib = k - 2;
					}

					int order[3] = { ia, ib, ic };
					Tri tri;
					bool valid = true;
					for (int i = 0; i < 3; ++i)
					{
						const short *e = entry + order[i] * 4;
						if (e[0] < 0 || e[0] >= sub.numverts)
						{
							valid = false;
							break;
						}
						for (int j = 0; j < 3; ++j)
							tri.p[i][j] = xv[static_cast<size_t>(e[0]) * 3 + j];
						tri.st[i][0] = e[2];
						tri.st[i][1] = e[3];
					}
					if (!valid || !WindingNormal(tri.p[0], tri.p[1], tri.p[2], tri.n))
						continue;

					tri.pixels = pixels;
					tri.width = width;
					tri.height = height;
					tri.opacity = opacity;
					tri.masked = masked;
					++stats.triangles;
					tris.push_back(tri);
				}
			}
		}
	}

	BuildChunks(tris, chunks);
	if (chunks.empty())
	{
		why = "no visible triangles";
		return false;
	}
	return true;
}

namespace
{

template <typename T>
void Put(std::vector<uint8_t> &out, const T &value)
{
	const uint8_t *p = reinterpret_cast<const uint8_t *>(&value);
	out.insert(out.end(), p, p + sizeof(T));
}

template <typename T>
bool Take(const uint8_t *&p, const uint8_t *end, T &value)
{
	if (static_cast<size_t>(end - p) < sizeof(T))
		return false;
	std::memcpy(&value, p, sizeof(T));
	p += sizeof(T);
	return true;
}

}

void AppendChunks(std::vector<uint8_t> &out, const std::vector<Model> &chunks)
{
	Put(out, static_cast<uint32_t>(chunks.size()));
	for (const Model &model : chunks)
	{
		for (int k = 0; k < 3; ++k)
			Put(out, model.mins[k]);
		for (int k = 0; k < 3; ++k)
			Put(out, model.maxs[k]);
		for (const View &view : model.views)
		{
			Put(out, view.u0);
			Put(out, view.v0);
			Put(out, view.w);
			Put(out, view.h);
			out.insert(out.end(), view.cells.begin(), view.cells.end());
		}
	}
}

bool ParseChunks(const uint8_t *p, const uint8_t *end, std::vector<Model> &chunks)
{
	uint32_t count = 0;
	if (!Take(p, end, count) || count > 1000000)
		return false;
	chunks.resize(count);
	for (Model &model : chunks)
	{
		for (int k = 0; k < 3; ++k)
			if (!Take(p, end, model.mins[k]))
				return false;
		for (int k = 0; k < 3; ++k)
			if (!Take(p, end, model.maxs[k]))
				return false;
		for (View &view : model.views)
		{
			if (!Take(p, end, view.u0) || !Take(p, end, view.v0) || !Take(p, end, view.w) || !Take(p, end, view.h))
				return false;
			size_t cells = static_cast<size_t>(view.w) * view.h;
			if (static_cast<size_t>(end - p) < cells)
				return false;
			view.cells.assign(p, p + cells);
			p += cells;
		}
	}
	return p == end;
}

}