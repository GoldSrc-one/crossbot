#pragma once

#include <cmath>

#include "mathlib.h"
#include "vector.h"

#include <string>
#include <vector>

namespace Nav
{

constexpr int kLightStyles = 64;
constexpr float kLitLevel = 64.f;
constexpr float kSpecialLight = 255.f;

float LightVisibility(float lt, float lb);

class LightMap
{
public:
	bool Load(const char *bspPath);
	void Clear();
	bool Loaded() const { return !m_lighting.empty(); }

	void SetStyle(int style, const char *pattern);
	void ClearStyles();

	bool Sample(const Vector &start, const Vector &end, float time, float &level) const;

private:
	struct Plane
	{
		Vector normal;
		float dist = 0.f;
	};

	struct Node
	{
		int plane = 0;
		int children[2] = {0, 0};
		int firstFace = 0;
		int numFaces = 0;
	};

	struct Face
	{
		float vecs[2][4] = {};
		int mins[2] = {0, 0};
		int extents[2] = {0, 0};
		int lightofs = -1;
		unsigned char styles[4] = {255, 255, 255, 255};
		bool special = false;
	};

	int StyleValue(int style, float time) const;
	int Recurse(int node, const Vector &start, const Vector &end, float time, float &level) const;

	std::vector<Plane> m_planes;
	std::vector<Node> m_nodes;
	std::vector<Face> m_faces;
	std::vector<unsigned char> m_lighting;
	int m_headNode = 0;
	std::string m_styles[kLightStyles];
};

}
