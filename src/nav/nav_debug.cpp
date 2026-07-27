#include "nav_debug.h"

#include <extdll.h>
#include <meta_api.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "nav.h"
#include "nav_query.h"
#include "../bot/bot.h"
#include "../bot/bot_manager.h"

namespace
{

	struct Winding
	{
		std::vector<Vector> points;
	};

	constexpr float kClipEpsilon = 0.01f;

	Winding BaseWindingForPlane(const Vector &normal, float dist, const Vector &centerPoint, float radius)
	{

		Vector seed(0.f, 0.f, 1.f);
		if (fabsf(normal.z) > 0.9f)
			seed = Vector(1.f, 0.f, 0.f);

		Vector right = (seed - normal * DotProduct(seed, normal)).Normalize();
		Vector up = CrossProduct(normal, right);

		Vector origin = centerPoint - normal * (DotProduct(normal, centerPoint) - dist);
		Winding w;
		w.points = {
			origin + right * radius + up * radius,
			origin - right * radius + up * radius,
			origin - right * radius - up * radius,
			origin + right * radius - up * radius,
		};
		return w;
	}

	bool ClipWinding(Winding &w, const Vector &planeNormal, float planeDist)
	{
		size_t n = w.points.size();
		std::vector<float> dist(n);
		for (size_t i = 0; i < n; ++i)
			dist[i] = DotProduct(w.points[i], planeNormal) - planeDist;

		std::vector<Vector> out;
		for (size_t i = 0; i < n; ++i)
		{
			size_t j = (i + 1) % n;
			bool inI = dist[i] >= -kClipEpsilon;
			bool inJ = dist[j] >= -kClipEpsilon;

			if (inI)
				out.push_back(w.points[i]);
			if (inI != inJ)
			{
				float t = dist[i] / (dist[i] - dist[j]);
				out.push_back(w.points[i] + (w.points[j] - w.points[i]) * t);
			}
		}

		w.points = std::move(out);
		return w.points.size() >= 3;
	}
	constexpr float kPi = 3.14159265f;

	bool g_pathBeamsEnabled = true;
	bool g_areaBeamsEnabled = true;
	short g_beamSprite = 0;

	float g_nextDrawTime = 0.f;
	constexpr float kDrawInterval = 0.2f;

	constexpr float kBeamLife = 0.3f;

	Vector ForwardFromAngles(const Vector &angles)
	{
		float pitch = angles.x * (kPi / 180.f);
		float yaw = angles.y * (kPi / 180.f);
		float cp = cosf(pitch), sp = sinf(pitch);
		float cy = cosf(yaw), sy = sinf(yaw);
		return Vector(cp * cy, cp * sy, -sp);
	}

	void SendBeam(const Vector &start, const Vector &end, int r, int g, int b, int width)
	{
		if (!g_beamSprite)
			return;

		MESSAGE_BEGIN(MSG_BROADCAST, SVC_TEMPENTITY);
			WRITE_BYTE(TE_BEAMPOINTS);
			WRITE_COORD(start.x);
			WRITE_COORD(start.y);
			WRITE_COORD(start.z);
			WRITE_COORD(end.x);
			WRITE_COORD(end.y);
			WRITE_COORD(end.z);
			WRITE_SHORT(g_beamSprite);
			WRITE_BYTE(0);
			WRITE_BYTE(0);
			WRITE_BYTE(static_cast<int>(kBeamLife * 10.f));
			WRITE_BYTE(width);
			WRITE_BYTE(0);
			WRITE_BYTE(r);
			WRITE_BYTE(g);
			WRITE_BYTE(b);
			WRITE_BYTE(220);
			WRITE_BYTE(0);
		MESSAGE_END();
	}

	constexpr float kFloorNormalThreshold = 0.7f;

	void DrawAreaFloor(const Nav::Area &area, int r, int g, int b)
	{

		const Vector &mn = area.mins;
		const Vector &mx = area.maxs;
		float radius = (mx - mn).Length() + 64.f;
		Vector areaCenter((mn.x + mx.x) * 0.5f, (mn.y + mx.y) * 0.5f, (mn.z + mx.z) * 0.5f);

		bool drewAny = false;
		for (const Nav::AreaPlane &floorPlane : area.planes)
		{
			if (floorPlane.normal.z < kFloorNormalThreshold)
				continue;

			Winding w = BaseWindingForPlane(floorPlane.normal, floorPlane.dist, areaCenter, radius);
			bool valid = true;
			for (const Nav::AreaPlane &bound : area.planes)
			{
				if (&bound == &floorPlane)
					continue;
				if (!ClipWinding(w, bound.normal, bound.dist))
				{
					valid = false;
					break;
				}
			}
			if (!valid)
				continue;

			for (size_t i = 0; i < w.points.size(); ++i)
				SendBeam(w.points[i], w.points[(i + 1) % w.points.size()], r, g, b, 2);
			drewAny = true;
		}

		if (drewAny)
			return;

		float z = area.mins.z;
		Vector c[4] = {
			Vector(area.mins.x, area.mins.y, z), Vector(area.maxs.x, area.mins.y, z),
			Vector(area.maxs.x, area.maxs.y, z), Vector(area.mins.x, area.maxs.y, z),
		};
		SendBeam(c[0], c[1], r, g, b, 2);
		SendBeam(c[1], c[2], r, g, b, 2);
		SendBeam(c[2], c[3], r, g, b, 2);
		SendBeam(c[3], c[0], r, g, b, 2);
	}

	void DrawBotPath(const CBot &bot)
	{
		if (bot.waypointIndex >= bot.waypoints.size())
			return;

		Vector prev = bot.pEdict->v.origin;
		for (size_t i = bot.waypointIndex; i < bot.waypoints.size(); ++i)
		{
			const Nav::Waypoint &wp = bot.waypoints[i];

			int r = 255, g = 255, b = 0;
			if (wp.requiresJump)
			{
				r = 255; g = 140; b = 0;
			}
			else if (wp.requiresElevatorRide)
			{
				r = 0; g = 220; b = 220;
			}

			SendBeam(prev, wp.point, r, g, b, 3);
			prev = wp.point;
		}
	}

	void DrawLookedAtArea(const Nav::NavMesh &mesh, edict_t *pPlayer)
	{
		Vector eyeStart = pPlayer->v.origin + pPlayer->v.view_ofs;
		Vector forward = ForwardFromAngles(pPlayer->v.v_angle);
		constexpr float kMaxAimTraceDistance = 4096.f;
		Vector eyeEnd = eyeStart + forward * kMaxAimTraceDistance;

		TraceResult tr;
		g_engfuncs.pfnTraceLine(eyeStart, eyeEnd, ignore_monsters, pPlayer, &tr);
		if (tr.flFraction >= 1.0f)
			return;

		const Nav::Area *area = Nav::FindAreaForPoint(mesh, tr.vecEndPos);
		if (!area)
			return;

		DrawAreaFloor(*area, 255, 40, 40);

		constexpr size_t kMaxNeighborsDrawn = 8;
		size_t drawn = 0;
		for (uint32_t connIdx : area->connIndices)
		{
			if (drawn >= kMaxNeighborsDrawn)
				break;
			if (connIdx >= mesh.conns.size())
				continue;
			const Nav::Connection &conn = mesh.conns[connIdx];
			uint32_t neighborId = (conn.areaA == area->id) ? conn.areaB : conn.areaA;
			const Nav::Area *neighbor = Nav::FindAreaById(mesh, neighborId);
			if (neighbor)
			{
				DrawAreaFloor(*neighbor, 40, 160, 255);
				++drawn;
			}
		}
	}
}

namespace NavDebug
{

void Precache()
{
	g_beamSprite = static_cast<short>(PRECACHE_MODEL("sprites/laserbeam.spr"));
}

void Draw()
{
	if (gpGlobals->time < g_nextDrawTime)
		return;
	g_nextDrawTime = gpGlobals->time + kDrawInterval;

	if (!g_beamSprite)
		return;

	const Nav::NavMesh *mesh = Nav::Get();
	if (!mesh)
		return;

	if (g_pathBeamsEnabled)
	{
		constexpr int kMaxBotsForDebug = 16;
		const CBot *bots[kMaxBotsForDebug];
		int n = BotManager::GetBotsForDebug(bots, kMaxBotsForDebug);
		for (int i = 0; i < n; ++i)
			DrawBotPath(*bots[i]);
	}

	if (g_areaBeamsEnabled)
	{
		for (int i = 1; i <= gpGlobals->maxClients; ++i)
		{
			edict_t *pPlayer = g_engfuncs.pfnPEntityOfEntIndex(i);
			if (!pPlayer || pPlayer->free)
				continue;
			if (!(pPlayer->v.flags & FL_CLIENT) || (pPlayer->v.flags & FL_FAKECLIENT))
				continue;
			if (!pPlayer->pvPrivateData)
				continue;

			DrawLookedAtArea(*mesh, pPlayer);
		}
	}
}

void SetPathBeamsEnabled(bool enabled) { g_pathBeamsEnabled = enabled; }
bool PathBeamsEnabled() { return g_pathBeamsEnabled; }
void SetAreaBeamsEnabled(bool enabled) { g_areaBeamsEnabled = enabled; }
bool AreaBeamsEnabled() { return g_areaBeamsEnabled; }

}

