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
	// Real bounded-polygon reconstruction for one plane of a convex
	// polytope, the same "start with a huge quad on the plane, clip it
	// against every other bounding plane" technique navgen's own offline
	// leaf-extraction uses (tools/navgen/src/leaf_extractor.cpp's
	// BuildPolytopeFromChain) -- ported here because a live player pointed
	// out, correctly, that a real BSP clip face can be an arbitrary polygon
	// (triangle, L-shape, whatever the brush geometry produced), not just a
	// rectangle, and an earlier version of this draw call was projecting
	// the area's XY *AABB* corners onto the floor plane's equation instead
	// of finding the plane's actual bounded face.
	//
	// Plain Vector (float), same as everywhere else in this file -- an
	// earlier version of this used double, on the theory that a 16384-unit
	// starting quad left too little mantissa for a room only tens/hundreds
	// of units across. That diagnosis was wrong: the *actual* bug was that
	// the starting quad was centered at the plane's closest approach to the
	// world origin (`normal * dist`), nowhere near the real room, so no
	// amount of precision would have helped -- clipping something centered
	// 1400 units away just fails cleanly regardless of float vs double.
	// Once BaseWindingForPlane took the area's own centroid as its center
	// point, plain float coordinates (in the same few-thousand-unit range
	// as the map itself) resolve well below kClipEpsilon -- double was
	// solving a problem that centering, not precision, actually had.
	struct Winding
	{
		std::vector<Vector> points;
	};

	constexpr float kClipEpsilon = 0.01f;

	// `centerPoint` should be some point actually inside/near the area (its
	// centroid) -- the starting quad is centered on `centerPoint` PROJECTED
	// onto the plane, not on the plane's closest approach to the world
	// origin. That distinction is the whole fix for a real, confirmed bug:
	// an earlier version centered the quad at `normal * dist`, which for a
	// floor plane like normal=(0,0,1) dist=-1500 is world point (0,0,-1500)
	// -- nowhere near a real room sitting at, say, y=1376.
	Winding BaseWindingForPlane(const Vector &normal, float dist, const Vector &centerPoint, float radius)
	{
		// Pick whichever basis axis is least aligned with normal to seed an
		// orthogonal (right, up) pair via Gram-Schmidt -- standard
		// BaseWindingForPlane technique, avoids the degenerate near-zero
		// cross product a poorly chosen seed axis would risk.
		Vector seed(0.f, 0.f, 1.f);
		if (fabsf(normal.z) > 0.9f)
			seed = Vector(1.f, 0.f, 0.f);

		Vector right = (seed - normal * DotProduct(seed, normal)).Normalize();
		Vector up = CrossProduct(normal, right);

		// Project centerPoint onto the plane: subtract however far
		// off-plane it is, along the normal.
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

	// Sutherland-Hodgman: keeps the winding's portion on the >= side of
	// (planeNormal, planeDist), i.e. the same half-space convention every
	// AreaPlane in nav_mesh.h already uses (Nav::ContainsPoint). Returns
	// false if nothing survives (degenerate/fully-clipped-away face).
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
	// Longer than kDrawInterval so consecutive redraws overlap instead of
	// leaving a visible gap/flicker between them.
	constexpr float kBeamLife = 0.3f;
	// Same forward.z = -sin(pitch) sign convention confirmed live earlier
	// this milestone (Bot_UpdateFacing / hl-bigbrain's render_view): positive
	// pitch looks/aims down.
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

		// MSG_BROADCAST (unreliable), not MSG_ALL -- confirmed live the hard
		// way: MSG_ALL queues onto each client's *reliable* channel, which is
		// a small fixed-size buffer, and a busy area (dozens of portal-
		// adjacent neighbors, each a multi-edge box, redrawn every
		// kDrawInterval) can fill it faster than it drains, overflowing the
		// channel and crashing the server outright. A dropped debug beam on
		// a lossy connection just flickers for one redraw; a reliable-buffer
		// overflow takes the whole server down -- worth the tradeoff.
		MESSAGE_BEGIN(MSG_BROADCAST, SVC_TEMPENTITY);
			WRITE_BYTE(TE_BEAMPOINTS);
			WRITE_COORD(start.x);
			WRITE_COORD(start.y);
			WRITE_COORD(start.z);
			WRITE_COORD(end.x);
			WRITE_COORD(end.y);
			WRITE_COORD(end.z);
			WRITE_SHORT(g_beamSprite);
			WRITE_BYTE(0);                                 // starting frame
			WRITE_BYTE(0);                                 // frame rate
			WRITE_BYTE(static_cast<int>(kBeamLife * 10.f)); // life, 0.1s units
			WRITE_BYTE(width);                              // line width, 0.1s units
			WRITE_BYTE(0);                                  // noise amplitude
			WRITE_BYTE(r);
			WRITE_BYTE(g);
			WRITE_BYTE(b);
			WRITE_BYTE(220);                                // brightness
			WRITE_BYTE(0);                                  // scroll speed
		MESSAGE_END();
	}

	// Matches navgen's own edge_tagger.cpp kFloorNormalThreshold exactly --
	// GoldSrc's real max walkable slope (dot(normal, up) >= ~0.7, ~45 degrees
	// from vertical). A plane steeper than this is a wall or ceiling, not
	// something a bot ever stands on, and confirmed live to be worth
	// excluding: an earlier version of this draw call used the area's full
	// AABB wireframe (all 12 edges, walls and ceiling included), which
	// doesn't correspond to anything the nav mesh actually treats as
	// standable and reads as "why does the box include the wall."
	constexpr float kFloorNormalThreshold = 0.7f;

	// The baked format carries each area's full convex-polytope half-space
	// plane list (nav_format.h v4), which is everything Winding/ClipWinding
	// above need to reconstruct each floor plane's true bounded face --
	// exactly what navgen's own offline leaf extraction does, just run here
	// at draw time instead of bake time. For each floor-normal plane, clip
	// a huge starting quad against every other plane in the same area, and
	// draw whatever real polygon survives.
	void DrawAreaFloor(const Nav::Area &area, int r, int g, int b)
	{
		// Sized to this specific area (its own AABB diagonal plus margin),
		// not a whole-map-sized constant -- keeps the starting quad's
		// coordinates in the same ballpark as the real geometry it's about
		// to be clipped against, which is what double precision alone isn't
		// meant to paper over.
		const Vector &mn = area.mins;
		const Vector &mx = area.maxs;
		float radius = (mx - mn).Length() + 64.f;
		Vector areaCenter((mn.x + mx.x) * 0.5f, (mn.y + mx.y) * 0.5f, (mn.z + mx.z) * 0.5f);

		bool drewAny = false;
		for (const Nav::AreaPlane &floorPlane : area.planes)
		{
			if (floorPlane.normal.z < kFloorNormalThreshold)
				continue; // wall or ceiling face -- not standable, don't draw it

			Winding w = BaseWindingForPlane(floorPlane.normal, floorPlane.dist, areaCenter, radius);
			bool valid = true;
			for (const Nav::AreaPlane &bound : area.planes)
			{
				if (&bound == &floorPlane)
					continue; // the floor plane itself doesn't bound its own face
				if (!ClipWinding(w, bound.normal, bound.dist))
				{
					valid = false; // clipped away entirely -- this plane contributes no real face
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

		// Shouldn't happen in practice (every real area has at least one
		// floor-normal plane that survives clipping), but fail visible
		// rather than silent: draw the AABB footprint at its own floor
		// height so there's still something on screen to flag the area as
		// unusual.
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
			// Orange for a jump hop, cyan for an elevator ride, yellow for a
			// normal walk -- lets a spectator tell at a glance which segment
			// type a bot is currently lined up on.
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
			return; // aiming into open space -- nothing to highlight

		const Nav::Area *area = Nav::FindAreaForPoint(mesh, tr.vecEndPos);
		if (!area)
			return;

		DrawAreaFloor(*area, 255, 40, 40); // looked-at area: red

		// Capped -- a busy hub area can legitimately have dozens of portal-
		// adjacent neighbors, and drawing all of them every kDrawInterval is
		// exactly the message volume that overflowed the reliable channel
		// and crashed the server before SendBeam switched to MSG_BROADCAST
		// (see its own comment); keeping this cap too, on top of that fix,
		// rather than relying on the unreliable channel alone to absorb an
		// unbounded burst. Lowered from 12 after disabling MergeAreas made
		// neighbor counts jump (far more, smaller areas per real room) and
		// a live player reported the result reading as visual clutter --
		// 8 is still enough to see the local connectivity without it turning
		// into a tangle of overlapping outlines.
		constexpr size_t kMaxNeighborsDrawn = 8;
		size_t drawn = 0;
		for (uint32_t portalIdx : area->portalIndices)
		{
			if (drawn >= kMaxNeighborsDrawn)
				break;
			if (portalIdx >= mesh.portals.size())
				continue;
			const Nav::Portal &portal = mesh.portals[portalIdx];
			uint32_t neighborId = (portal.areaA == area->id) ? portal.areaB : portal.areaA;
			const Nav::Area *neighbor = Nav::FindAreaById(mesh, neighborId);
			if (neighbor)
			{
				DrawAreaFloor(*neighbor, 40, 160, 255); // neighbor: blue
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
		return; // precache failed or hasn't run yet -- nothing to draw with

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
				continue; // skip empty slots and bots -- this is for a human spectator's own aim
			if (!pPlayer->pvPrivateData)
				continue; // not fully in-game yet

			DrawLookedAtArea(*mesh, pPlayer);
		}
	}
}

void SetPathBeamsEnabled(bool enabled) { g_pathBeamsEnabled = enabled; }
bool PathBeamsEnabled() { return g_pathBeamsEnabled; }
void SetAreaBeamsEnabled(bool enabled) { g_areaBeamsEnabled = enabled; }
bool AreaBeamsEnabled() { return g_areaBeamsEnabled; }

} // namespace NavDebug
