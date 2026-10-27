#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace SightCoverage
{

constexpr int kViewCount = 34;
constexpr float kCellSize = 32.f;
constexpr float kChunkSize = 256.f;
constexpr uint32_t kFormatVersion = 3;

constexpr int kRenderNormal = 0;
constexpr int kRenderTransColor = 1;
constexpr int kRenderTransTexture = 2;
constexpr int kRenderGlow = 3;
constexpr int kRenderTransAlpha = 4;
constexpr int kRenderTransAdd = 5;

struct View
{
	float u0 = 0.f;
	float v0 = 0.f;
	uint16_t w = 0;
	uint16_t h = 0;
	std::vector<uint8_t> cells;
};

struct Model
{
	float mins[3] = {};
	float maxs[3] = {};
	View views[kViewCount];
};

struct Instance
{
	const Model *model = nullptr;
	float origin[3] = {};
	float axes[3][3] = {};
	float absmin[3] = {};
	float absmax[3] = {};
};

struct Placed
{
	std::vector<Instance> chunks;
	float origin[3] = {};
	float angles[3] = {};
	float absmin[3] = {};
	float absmax[3] = {};
};

class Grid
{
public:
	void Build(const std::vector<const Placed *> &placed);
	bool Empty() const { return m_chunks.empty(); }
	const float *Mins() const { return m_mins; }
	const float *Maxs() const { return m_maxs; }
	float Clear(const float from[3], const float to[3], float stopBelow) const;

private:
	std::vector<const Instance *> m_chunks;
	std::vector<uint32_t> m_cellStart;
	std::vector<uint32_t> m_items;
	float m_mins[3] = {};
	float m_maxs[3] = {};
	int m_dims[3] = {};
};

struct Stats
{
	int triangles = 0;
	int windingMismatch = 0;
	int windingChecked = 0;
};

float RenderOpacity(int rendermode, float renderamt);

void ViewBasis(int view, float d[3], float u[3], float v[3]);
int NearestView(const float dir[3]);
float MeanAlpha(const View &view);

void Place(Placed &placed, const std::vector<Model> &chunks, const float origin[3], const float angles[3],
	bool studio);
float Alpha(const Instance &instance, const float from[3], const float to[3]);
float Alpha(const Placed &placed, const float from[3], const float to[3]);

bool ReadFile(const std::string &path, std::vector<uint8_t> &out);
std::string AssetRoot(const std::string &bspPath);
uint32_t StudioChecksum(const std::string &root, const std::string &modelPath);

class Bsp
{
public:
	bool Open(const std::string &bspPath);
	uint32_t Checksum() const { return m_checksum; }
	int ModelCount() const { return m_modelCount; }
	const std::string &EntityText() const { return m_entityText; }
	const std::vector<std::string> &Missing() const { return m_missing; }
	void BuildBrush(int submodel, int rendermode, float renderamt, std::vector<Model> &chunks, Stats &stats);

private:
	struct Texture
	{
		std::string name;
		int width = 0;
		int height = 0;
		const uint8_t *pixels = nullptr;
		bool resolved = false;
	};

	const uint8_t *Lump(int index, size_t elemSize, int &count) const;
	const Texture &TextureAt(int miptex);
	void LoadWads();

	std::string m_root;
	std::vector<uint8_t> m_data;
	std::string m_entityText;
	int m_modelCount = 0;
	uint32_t m_checksum = 0;
	std::vector<Texture> m_textures;
	std::vector<std::string> m_wadNames;
	std::deque<std::vector<uint8_t>> m_wads;
	bool m_wadsLoaded = false;
	std::vector<std::string> m_missing;
};

bool BuildStudio(const std::string &root, const std::string &modelPath, int sequence, const char *sequenceLabel,
	int body, int skin, int rendermode, float renderamt, std::vector<Model> &chunks, Stats &stats,
	std::string &why);

void AppendChunks(std::vector<uint8_t> &out, const std::vector<Model> &chunks);
bool ParseChunks(const uint8_t *p, const uint8_t *end, std::vector<Model> &chunks);

}
