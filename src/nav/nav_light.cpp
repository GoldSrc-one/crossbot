#include "nav_light.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "const.h"
#include "bspfile.h"

namespace Nav
{

namespace
{

constexpr int kTexSpecial = 1;
constexpr int kNoStyle = 255;
constexpr int kDefaultStyleValue = 256;
constexpr int kStyleStep = 22;
constexpr float kStyleRate = 10.f;

template <typename T>
bool ReadLump(const std::vector<unsigned char> &data, const dheader_t &header, int lump, std::vector<T> &out)
{
	const lump_t &l = header.lumps[lump];
	if (l.fileofs < 0 || l.filelen < 0 || static_cast<size_t>(l.fileofs) + static_cast<size_t>(l.filelen) > data.size())
		return false;
	if (static_cast<size_t>(l.filelen) % sizeof(T) != 0)
		return false;
	out.resize(static_cast<size_t>(l.filelen) / sizeof(T));
	if (!out.empty())
		std::memcpy(out.data(), data.data() + l.fileofs, static_cast<size_t>(l.filelen));
	return true;
}

}

float LightVisibility(float lt, float lb)
{
	float contrast = std::max(lt, std::fabs(lt - lb));
	return std::min(1.f, std::max(0.f, contrast / kLitLevel));
}

bool LightMap::Load(const char *bspPath)
{
	Clear();

	FILE *f = std::fopen(bspPath, "rb");
	if (!f)
		return false;

	std::fseek(f, 0, SEEK_END);
	long size = std::ftell(f);
	std::fseek(f, 0, SEEK_SET);
	if (size < static_cast<long>(sizeof(dheader_t)))
	{
		std::fclose(f);
		return false;
	}

	std::vector<unsigned char> data(static_cast<size_t>(size));
	size_t readBytes = std::fread(data.data(), 1, data.size(), f);
	std::fclose(f);
	if (readBytes != data.size())
		return false;

	dheader_t header;
	std::memcpy(&header, data.data(), sizeof(header));
	if (header.version != HLBSP_VERSION)
		return false;

	std::vector<dplane_t> planes;
	std::vector<dnode_t> nodes;
	std::vector<dface_t> faces;
	std::vector<texinfo_t> texinfo;
	std::vector<dvertex_t> verts;
	std::vector<dedge_t> edges;
	std::vector<int> surfedges;
	std::vector<dmodel_t> models;
	std::vector<unsigned char> lighting;
	if (!ReadLump(data, header, LUMP_PLANES, planes) || !ReadLump(data, header, LUMP_NODES, nodes) ||
		!ReadLump(data, header, LUMP_FACES, faces) || !ReadLump(data, header, LUMP_TEXINFO, texinfo) ||
		!ReadLump(data, header, LUMP_VERTEXES, verts) || !ReadLump(data, header, LUMP_EDGES, edges) ||
		!ReadLump(data, header, LUMP_SURFEDGES, surfedges) || !ReadLump(data, header, LUMP_MODELS, models) ||
		!ReadLump(data, header, LUMP_LIGHTING, lighting))
		return false;
	if (lighting.empty() || models.empty() || nodes.empty())
		return false;

	m_planes.resize(planes.size());
	for (size_t i = 0; i < planes.size(); ++i)
	{
		m_planes[i].normal = Vector(planes[i].normal[0], planes[i].normal[1], planes[i].normal[2]);
		m_planes[i].dist = planes[i].dist;
	}

	m_faces.resize(faces.size());
	for (size_t i = 0; i < faces.size(); ++i)
	{
		const dface_t &in = faces[i];
		Face &out = m_faces[i];
		if (in.texinfo < 0 || static_cast<size_t>(in.texinfo) >= texinfo.size())
			return false;
		const texinfo_t &tex = texinfo[in.texinfo];
		std::memcpy(out.vecs, tex.vecs, sizeof(out.vecs));
		std::memcpy(out.styles, in.styles, sizeof(out.styles));
		out.special = (tex.flags & kTexSpecial) != 0;
		out.lightofs = in.lightofs;

		float mins[2] = {999999.f, 999999.f};
		float maxs[2] = {-999999.f, -999999.f};
		for (int e = 0; e < in.numedges; ++e)
		{
			size_t se = static_cast<size_t>(in.firstedge) + static_cast<size_t>(e);
			if (se >= surfedges.size())
				return false;
			int edge = surfedges[se];
			size_t ei = static_cast<size_t>(edge >= 0 ? edge : -edge);
			if (ei >= edges.size())
				return false;
			unsigned short vi = edge >= 0 ? edges[ei].v[0] : edges[ei].v[1];
			if (vi >= verts.size())
				return false;
			const float *p = verts[vi].point;
			for (int j = 0; j < 2; ++j)
			{
				float val = p[0] * tex.vecs[j][0] + p[1] * tex.vecs[j][1] + p[2] * tex.vecs[j][2] + tex.vecs[j][3];
				mins[j] = std::min(mins[j], val);
				maxs[j] = std::max(maxs[j], val);
			}
		}
		for (int j = 0; j < 2; ++j)
		{
			int bmin = static_cast<int>(std::floor(mins[j] / 16.f));
			int bmax = static_cast<int>(std::ceil(maxs[j] / 16.f));
			out.mins[j] = bmin * 16;
			out.extents[j] = (bmax - bmin) * 16;
		}
	}

	m_nodes.resize(nodes.size());
	for (size_t i = 0; i < nodes.size(); ++i)
	{
		const dnode_t &in = nodes[i];
		Node &out = m_nodes[i];
		if (in.planenum < 0 || static_cast<size_t>(in.planenum) >= m_planes.size() ||
			static_cast<size_t>(in.firstface) + in.numfaces > m_faces.size())
			return false;
		out.plane = in.planenum;
		out.children[0] = in.children[0];
		out.children[1] = in.children[1];
		out.firstFace = in.firstface;
		out.numFaces = in.numfaces;
	}

	m_headNode = models[0].headnode[0];
	m_lighting.swap(lighting);
	return true;
}

void LightMap::Clear()
{
	m_planes.clear();
	m_nodes.clear();
	m_faces.clear();
	m_lighting.clear();
	m_headNode = 0;
}

void LightMap::SetStyle(int style, const char *pattern)
{
	if (style >= 0 && style < kLightStyles)
		m_styles[style] = pattern ? pattern : "";
}

void LightMap::ClearStyles()
{
	for (std::string &s : m_styles)
		s.clear();
}

int LightMap::StyleValue(int style, float time) const
{
	if (style < 0 || style >= kLightStyles || m_styles[style].empty())
		return kDefaultStyleValue;
	const std::string &s = m_styles[style];
	size_t k = static_cast<size_t>(std::max(0.f, time * kStyleRate)) % s.size();
	return (s[k] - 'a') * kStyleStep;
}

bool LightMap::Sample(const Vector &start, const Vector &end, float time, float &level) const
{
	if (!Loaded())
		return false;
	return Recurse(m_headNode, start, end, time, level) > 0;
}

int LightMap::Recurse(int nodeIndex, const Vector &start, const Vector &end, float time, float &level) const
{
	if (nodeIndex < 0 || static_cast<size_t>(nodeIndex) >= m_nodes.size())
		return 0;

	const Node &node = m_nodes[nodeIndex];
	const Plane &plane = m_planes[node.plane];
	float front = DotProduct(start, plane.normal) - plane.dist;
	float back = DotProduct(end, plane.normal) - plane.dist;
	int side = front < 0.f;

	if ((back < 0.f) == static_cast<bool>(side))
		return Recurse(node.children[side], start, end, time, level);

	float frac = front / (front - back);
	Vector mid = start + (end - start) * frac;

	int r = Recurse(node.children[side], start, mid, time, level);
	if (r > 0)
		return r;

	for (int i = 0; i < node.numFaces; ++i)
	{
		const Face &face = m_faces[node.firstFace + i];
		float s = DotProduct(mid, Vector(face.vecs[0][0], face.vecs[0][1], face.vecs[0][2])) + face.vecs[0][3];
		float t = DotProduct(mid, Vector(face.vecs[1][0], face.vecs[1][1], face.vecs[1][2])) + face.vecs[1][3];
		if (s < face.mins[0] || t < face.mins[1])
			continue;
		int ds = static_cast<int>(s) - face.mins[0];
		int dt = static_cast<int>(t) - face.mins[1];
		if (ds > face.extents[0] || dt > face.extents[1])
			continue;

		if (face.special)
		{
			level = kSpecialLight;
			return 1;
		}

		level = 0.f;
		if (face.lightofs < 0)
			return 1;

		int w = (face.extents[0] >> 4) + 1;
		int h = (face.extents[1] >> 4) + 1;
		size_t ofs = static_cast<size_t>(face.lightofs) + static_cast<size_t>((dt >> 4) * w + (ds >> 4)) * 3;
		size_t stride = static_cast<size_t>(w * h) * 3;
		int rgb[3] = {0, 0, 0};
		for (int m = 0; m < 4 && face.styles[m] != kNoStyle; ++m, ofs += stride)
		{
			if (ofs + 3 > m_lighting.size())
				break;
			int scale = StyleValue(face.styles[m], time);
			for (int c = 0; c < 3; ++c)
				rgb[c] += m_lighting[ofs + c] * scale;
		}
		level = static_cast<float>((rgb[0] >> 8) + (rgb[1] >> 8) + (rgb[2] >> 8)) / 3.f;
		return 1;
	}

	return Recurse(node.children[!side], mid, end, time, level);
}

}
