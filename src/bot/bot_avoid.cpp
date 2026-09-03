#include <extdll.h>
#include <meta_api.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "bot_avoid.h"
#include "bot.h"
#include "bot_break.h"
#include "bot_deploy.h"
#include "bot_log.h"
#include "../nav/nav_mesh.h"
#include "../nav/nav_query.h"
#include "../nav/waypoint_plan.h"

namespace
{
	constexpr int kHullHuman = 1;
	constexpr int kHullDuck = 3;

	constexpr float kDuckHullHalf = NavPlan::kPlanStandHeight;
	constexpr float kStandHullHalf = kDuckHullHalf * 2.f;
	constexpr float kDuckClipHeight = kDuckHullHalf * 2.f;
	constexpr float kAvoidStepHeight = 18.f;

	constexpr float kAvoidLookaheadTime = 0.5f;
	constexpr float kAvoidLookaheadMin = 64.f;
	constexpr float kAvoidLookaheadMax = 160.f;

	constexpr float kAvoidDuckClearance = kDuckClipHeight + 4.f;
	constexpr float kAvoidCrawlPenalty = 1.25f;
	constexpr float kAvoidDuckMinHold = 0.5f;
	constexpr float kAvoidDuckMaxHold = 4.f;

	constexpr float kAvoidJumpMargin = 4.f;
	constexpr float kAvoidJumpLead = 8.f;
	constexpr float kAvoidJumpCooldown = 0.2f;

	constexpr size_t kAvoidRouteLookahead = 3;

	constexpr float kAvoidHullHalf = 16.f;
	constexpr float kAvoidCornerMargin = 2.f;
	constexpr float kAvoidGroundDrop = 64.f;
	constexpr float kAvoidFloorNormal = 0.7f;

	constexpr int kAvoidMaxDepth = 6;
	constexpr int kAvoidMaxTraces = 256;

	constexpr size_t kAvoidDestLookahead = 4;
	constexpr float kAvoidDestStep = 16.f;
	constexpr int kAvoidDestProbes = 16;
	constexpr float kAvoidDetourMaxLength = 768.f;
	constexpr float kAvoidSearchCooldown = 1.f;
	constexpr float kAvoidDetourMaxTime = 10.f;
	constexpr float kAvoidMoverSpeed = 16.f;
	constexpr float kAvoidSpinRadiusMin = 4.f;
	constexpr float kAvoidDegToRad = 0.01745329252f;

	constexpr float kAvoidBeamStep = 8.f;
	constexpr int kAvoidBeamMaxSteps = 48;
	constexpr float kAvoidBeamDepth = 32.f;
	constexpr float kAvoidBeamPad = 4.f;

	const char *ClassOf(edict_t *ent)
	{
		if (!ent || !gpGlobals->pStringBase)
			return "?";
		const char *s = gpGlobals->pStringBase + static_cast<unsigned int>(ent->v.classname);
		return (s && s[0]) ? s : "?";
	}

	float Length2D(const Vector &v)
	{
		return sqrtf(v.x * v.x + v.y * v.y);
	}

	float Dist2D(const Vector &a, const Vector &b)
	{
		float dx = a.x - b.x;
		float dy = a.y - b.y;
		return sqrtf(dx * dx + dy * dy);
	}

	float Dot2D(const Vector &a, const Vector &b)
	{
		return a.x * b.x + a.y * b.y;
	}

	Vector MoverVelocityAt(edict_t *ent, const Vector &point)
	{
		const Vector &av = ent->v.avelocity;
		if (av.x == 0.f && av.y == 0.f && av.z == 0.f)
			return ent->v.velocity;

		Vector r = point - ent->v.origin;
		float len = Length2D(r);
		if (len < kAvoidSpinRadiusMin)
			return ent->v.velocity;

		float ex = std::max(fabsf(ent->v.mins.x), fabsf(ent->v.maxs.x));
		float ey = std::max(fabsf(ent->v.mins.y), fabsf(ent->v.maxs.y));
		float reach = sqrtf(ex * ex + ey * ey);
		if (reach > len)
		{
			float scale = reach / len;
			r.x *= scale;
			r.y *= scale;
		}

		Vector w(av.z * kAvoidDegToRad, av.x * kAvoidDegToRad, av.y * kAvoidDegToRad);
		Vector spin(w.y * r.z - w.z * r.y, w.z * r.x - w.x * r.z, w.x * r.y - w.y * r.x);
		return ent->v.velocity + spin;
	}

	void HullExtents(int hull, Vector &mins, Vector &maxs)
	{
		float half = hull == kHullDuck ? kDuckHullHalf : kStandHullHalf;
		mins = Vector(-kAvoidHullHalf, -kAvoidHullHalf, -half);
		maxs = Vector(kAvoidHullHalf, kAvoidHullHalf, half);
	}

	bool ClipSegmentToBox(const Vector &a, const Vector &b, const Vector &mins, const Vector &maxs,
		float &t0, float &t1)
	{
		t0 = 0.f;
		t1 = 1.f;

		const float av[3] = { a.x, a.y, a.z };
		const float bv[3] = { b.x, b.y, b.z };
		const float lo[3] = { mins.x, mins.y, mins.z };
		const float hi[3] = { maxs.x, maxs.y, maxs.z };

		for (int i = 0; i < 3; ++i)
		{
			float d = bv[i] - av[i];
			if (fabsf(d) < 1e-6f)
			{
				if (av[i] < lo[i] || av[i] > hi[i])
					return false;
				continue;
			}

			float p0 = (lo[i] - av[i]) / d;
			float p1 = (hi[i] - av[i]) / d;
			if (p0 > p1)
				std::swap(p0, p1);

			if (p0 > t0) t0 = p0;
			if (p1 < t1) t1 = p1;
			if (t0 > t1)
				return false;
		}
		return true;
	}

	bool BeamMeetsHull(const BotDeploy::Beam &beam, const Vector &at, int hull, float &zLo,
		float &zHi)
	{
		Vector mins, maxs;
		HullExtents(hull, mins, maxs);
		mins = at + mins;
		maxs = at + maxs;

		float t0 = 0.f, t1 = 1.f;
		if (!ClipSegmentToBox(beam.start, beam.end, mins, maxs, t0, t1))
			return false;

		float za = beam.start.z + (beam.end.z - beam.start.z) * t0;
		float zb = beam.start.z + (beam.end.z - beam.start.z) * t1;
		zLo = std::min(za, zb);
		zHi = std::max(za, zb);
		return true;
	}

	const BotDeploy::Beam *BeamAlong(const Vector &from, const Vector &to, int hull, float &distOut,
		float &zLo, float &zHi)
	{
		const std::vector<BotDeploy::Beam> &beams = BotDeploy::Beams();
		if (beams.empty())
			return nullptr;

		Vector delta = to - from;
		float len = delta.Length();
		if (len < 1.f)
			return nullptr;

		int steps = static_cast<int>(len / kAvoidBeamStep) + 1;
		if (steps > kAvoidBeamMaxSteps)
			steps = kAvoidBeamMaxSteps;

		for (int s = 0; s <= steps; ++s)
		{
			float t = static_cast<float>(s) / static_cast<float>(steps);
			Vector at = from + delta * t;

			for (size_t i = 0; i < beams.size(); ++i)
			{
				if (!beams[i].ent || beams[i].ent->free)
					continue;
				if (!BeamMeetsHull(beams[i], at, hull, zLo, zHi))
					continue;

				distOut = len * t;
				return &beams[i];
			}
		}
		return nullptr;
	}

	void BeamBounds(const BotDeploy::Beam &beam, Vector &mins, Vector &maxs)
	{
		mins.x = std::min(beam.start.x, beam.end.x);
		mins.y = std::min(beam.start.y, beam.end.y);
		mins.z = std::min(beam.start.z, beam.end.z);
		maxs.x = std::max(beam.start.x, beam.end.x);
		maxs.y = std::max(beam.start.y, beam.end.y);
		maxs.z = std::max(beam.start.z, beam.end.z);
	}

	bool AabbOverlap(const Vector &aMin, const Vector &aMax, const Vector &bMin, const Vector &bMax)
	{
		return aMin.x <= bMax.x && aMax.x >= bMin.x
			&& aMin.y <= bMax.y && aMax.y >= bMin.y
			&& aMin.z <= bMax.z && aMax.z >= bMin.z;
	}

	float RiseTime(float rise, float gravity)
	{
		if (rise <= 0.f)
			return 0.f;
		if (gravity <= 1.f)
			return -1.f;
		float disc = PmSim::kJumpImpulse * PmSim::kJumpImpulse - 2.f * gravity * rise;
		if (disc < 0.f)
			return -1.f;
		return (PmSim::kJumpImpulse - sqrtf(disc)) / gravity;
	}

	bool RouteExemptsHit(const CBot &bot, const Nav::NavMesh &mesh, edict_t *hit, int solid,
		bool *wantsBreak)
	{
		if (bot.goalItem.Is(hit))
			return true;

		if (solid != SOLID_BSP)
			return false;

		size_t begin = std::max(bot.waypointIndex, bot.avoid.detourEnd);
		size_t end = std::min(bot.waypoints.size(), begin + kAvoidRouteLookahead);
		for (size_t i = begin; i < end; ++i)
		{
			const Nav::Waypoint &wp = bot.waypoints[i];
			if (wp.requiresElevatorRide)
				return true;
			if (wp.requiresButtonUse &&
				AabbOverlap(wp.activatorCenter, wp.activatorCenter, hit->v.absmin, hit->v.absmax))
				return true;
			if (!wp.requiresDoor && !wp.requiresBreak)
				continue;
			if (wp.targetAreaId < 0)
				continue;

			const Nav::Area *area = Nav::FindAreaById(mesh, static_cast<uint32_t>(wp.targetAreaId));
			if (!area)
				continue;
			if (!AabbOverlap(area->mins, area->maxs, hit->v.absmin, hit->v.absmax))
				continue;

			if (wantsBreak && wp.requiresBreak)
				*wantsBreak = true;
			return true;
		}
		return false;
	}

	bool HitQualifies(const CBot &bot, const Nav::NavMesh &mesh, edict_t *hit, bool *wantsBreak = nullptr)
	{
		if (!hit)
			return false;
		if (g_engfuncs.pfnIndexOfEdict(hit) == 0)
			return false;

		int solid = hit->v.solid;
		if (solid != SOLID_BSP && solid != SOLID_BBOX && solid != SOLID_SLIDEBOX)
			return false;
		if (solid == SOLID_BSP && (hit->v.flags & FL_WORLDBRUSH) != 0)
			return false;

		return !RouteExemptsHit(bot, mesh, hit, solid, wantsBreak);
	}

	bool WaypointIsFlagged(const Nav::Waypoint &wp)
	{
		return wp.requiresCrouch || wp.requiresDoor || wp.requiresBreak || wp.requiresButtonUse
			|| wp.requiresTriggerTouch || wp.requiresTeleport || wp.requiresElevatorRide
			|| wp.airmoveLaunch || wp.requiresJump || wp.requiresLadder;
	}

	struct SearchCtx
	{
		const CBot *bot = nullptr;
		const Nav::NavMesh *mesh = nullptr;
		edict_t *ignore = nullptr;
		int hull = kHullHuman;
		int traces = 0;
		const BotDeploy::Beam *lastBeam = nullptr;
	};

	enum ProbeResult
	{
		PROBE_CLEAR = 0,
		PROBE_WORLD,
		PROBE_ENTITY
	};

	ProbeResult Probe(SearchCtx &ctx, const Vector &a, const Vector &b, TraceResult &tr)
	{
		ctx.lastBeam = nullptr;

		if (ctx.traces >= kAvoidMaxTraces)
			return PROBE_WORLD;
		++ctx.traces;

		g_engfuncs.pfnTraceHull(a, b, dont_ignore_monsters, ctx.hull, ctx.ignore, &tr);
		if (tr.fStartSolid || tr.fAllSolid)
			return PROBE_WORLD;

		float solidDist = tr.flFraction >= 1.f ? (b - a).Length() : (tr.vecEndPos - a).Length();

		float beamDist = 0.f, zLo = 0.f, zHi = 0.f;
		const BotDeploy::Beam *beam = BeamAlong(a, b, ctx.hull, beamDist, zLo, zHi);
		if (beam && beamDist <= solidDist)
		{
			ctx.lastBeam = beam;
			tr.pHit = beam->ent;
			tr.vecEndPos = a + (b - a) * (beamDist / std::max(1.f, (b - a).Length()));
			return PROBE_ENTITY;
		}

		if (tr.flFraction >= 1.f)
			return PROBE_CLEAR;
		if (const BotDeploy::Beam *body = BotDeploy::BeamFor(tr.pHit))
		{
			ctx.lastBeam = body;
			return PROBE_ENTITY;
		}
		if (HitQualifies(*ctx.bot, *ctx.mesh, tr.pHit))
			return PROBE_ENTITY;

		if (tr.vecPlaneNormal.z < kAvoidFloorNormal)
			return PROBE_WORLD;
		if (ctx.traces >= kAvoidMaxTraces)
			return PROBE_WORLD;
		++ctx.traces;

		Vector lift(0.f, 0.f, kAvoidStepHeight);
		TraceResult up;
		g_engfuncs.pfnTraceHull(a + lift, b + lift, dont_ignore_monsters, ctx.hull, ctx.ignore, &up);
		if (up.fStartSolid || up.fAllSolid)
			return PROBE_WORLD;
		if (up.flFraction >= 1.f)
			return PROBE_CLEAR;
		if (HitQualifies(*ctx.bot, *ctx.mesh, up.pHit))
		{
			tr = up;
			return PROBE_ENTITY;
		}
		return PROBE_WORLD;
	}

	bool Ground(SearchCtx &ctx, float x, float y, float refZ, Vector &out)
	{
		if (ctx.traces >= kAvoidMaxTraces)
			return false;
		++ctx.traces;

		Vector from(x, y, refZ + kAvoidStepHeight);
		Vector to(x, y, refZ - kAvoidGroundDrop);
		TraceResult tr;
		g_engfuncs.pfnTraceHull(from, to, dont_ignore_monsters, ctx.hull, ctx.ignore, &tr);
		if (tr.fStartSolid || tr.fAllSolid || tr.flFraction >= 1.f)
			return false;

		out = tr.vecEndPos;
		return true;
	}

	bool SolveAround(SearchCtx &ctx, const Vector &from, const Vector &to, edict_t *hit,
		const BotDeploy::Beam *beam, const Vector &contact, std::vector<int> &visited, int depth,
		std::vector<Vector> &out, float &length);

	bool Solve(SearchCtx &ctx, const Vector &from, const Vector &to, std::vector<int> &visited,
		int depth, std::vector<Vector> &out, float &length)
	{
		TraceResult tr;
		ProbeResult probe = Probe(ctx, from, to, tr);
		if (probe == PROBE_CLEAR)
		{
			length += Dist2D(from, to);
			return true;
		}
		if (probe == PROBE_WORLD)
			return false;
		return SolveAround(ctx, from, to, tr.pHit, ctx.lastBeam, tr.vecEndPos, visited, depth, out,
			length);
	}

	bool SolveAround(SearchCtx &ctx, const Vector &from, const Vector &to, edict_t *hit,
		const BotDeploy::Beam *beam, const Vector &contact, std::vector<int> &visited, int depth,
		std::vector<Vector> &out, float &length)
	{
		if (depth >= kAvoidMaxDepth)
			return false;

		int hitIndex = g_engfuncs.pfnIndexOfEdict(hit);
		for (int seen : visited)
		{
			if (seen == hitIndex)
				return false;
		}

		Vector bmin = hit->v.absmin;
		Vector bmax = hit->v.absmax;
		if (beam)
			BeamBounds(*beam, bmin, bmax);

		float pad = kAvoidHullHalf + kAvoidCornerMargin;
		float minX = bmin.x - pad;
		float minY = bmin.y - pad;
		float maxX = bmax.x + pad;
		float maxY = bmax.y + pad;

		Vector fwd((minX + maxX) * 0.5f - from.x, (minY + maxY) * 0.5f - from.y, 0.f);
		float fwdLen = Length2D(fwd);
		if (fwdLen < 0.0001f)
		{
			fwd = Vector(to.x - from.x, to.y - from.y, 0.f);
			fwdLen = Length2D(fwd);
			if (fwdLen < 0.0001f)
				return false;
		}
		fwd.x /= fwdLen;
		fwd.y /= fwdLen;
		Vector right(fwd.y, -fwd.x, 0.f);

		struct Corner
		{
			float x, y, sx, sy;
		};
		Corner corners[4] = {
			{ minX, minY, 0.f, 0.f },
			{ maxX, minY, 0.f, 0.f },
			{ minX, maxY, 0.f, 0.f },
			{ maxX, maxY, 0.f, 0.f }
		};
		for (Corner &c : corners)
		{
			float dx = c.x - from.x;
			float dy = c.y - from.y;
			c.sx = dx * right.x + dy * right.y;
			c.sy = dx * fwd.x + dy * fwd.y;
		}
		std::sort(corners, corners + 4, [](const Corner &a, const Corner &b) { return a.sx < b.sx; });
		if (corners[1].sy > corners[0].sy)
			std::swap(corners[0], corners[1]);
		if (corners[3].sy < corners[2].sy)
			std::swap(corners[2], corners[3]);

		const Corner *sides[2][4] = {
			{ &corners[1], &corners[0], &corners[3], &corners[2] },
			{ &corners[2], &corners[3], &corners[0], &corners[1] }
		};

		int order[2] = { 1, 0 };
		bool exclusive = false;
		float vlat = Dot2D(MoverVelocityAt(hit, contact), right);
		if (fabsf(vlat) > kAvoidMoverSpeed)
		{
			order[0] = (vlat < 0.f) ? 1 : 0;
			order[1] = 1 - order[0];
			exclusive = true;
		}

		std::vector<Vector> best;
		float bestLength = 0.f;
		bool haveBest = false;
		size_t mark = visited.size();

		for (int k = 0; k < 2; ++k)
		{
			int side = order[k];

			visited.resize(mark);
			visited.push_back(hitIndex);

			std::vector<Vector> points;
			float sideLength = 0.f;
			Vector cur = from;
			bool done = false;

			for (int i = 0; i < 4 && !done; ++i)
			{
				Vector corner;
				if (!Ground(ctx, sides[side][i]->x, sides[side][i]->y, cur.z, corner))
					corner = Vector(sides[side][i]->x, sides[side][i]->y, cur.z);

				TraceResult leg;
				ProbeResult legProbe = Probe(ctx, cur, corner, leg);
				if (legProbe == PROBE_WORLD)
					continue;

				std::vector<Vector> tail;
				float tailLength = 0.f;

				if (legProbe == PROBE_ENTITY)
				{
					if (!SolveAround(ctx, cur, to, leg.pHit, ctx.lastBeam, leg.vecEndPos, visited,
							depth + 1, tail, tailLength))
						continue;
				}
				else
				{
					sideLength += Dist2D(cur, corner);
					points.push_back(corner);
					cur = corner;

					if (!Solve(ctx, cur, to, visited, depth + 1, tail, tailLength))
						continue;
				}

				points.insert(points.end(), tail.begin(), tail.end());
				sideLength += tailLength;
				done = true;
			}

			if (!done)
				continue;

			if (!haveBest || sideLength < bestLength)
			{
				best = std::move(points);
				bestLength = sideLength;
				haveBest = true;
			}
			if (exclusive)
				break;
		}

		visited.resize(mark);
		if (!haveBest)
			return false;
		visited.push_back(hitIndex);

		out.insert(out.end(), best.begin(), best.end());
		length += bestLength;
		return true;
	}

	void Record(BotAvoid::State &state, BotAvoid::Kind kind, edict_t *hit, float gap, float top)
	{
		state.lastKind = kind;
		state.lastGap = gap;
		state.lastTop = top;
		strncpy(state.lastHit, ClassOf(hit), sizeof(state.lastHit) - 1);
		state.lastHit[sizeof(state.lastHit) - 1] = '\0';
	}

	void Announce(const CBot &bot, const char *what, edict_t *hit, float gap, float top, float dist,
		float speed, float hold)
	{
		BOT_VLOG("CrossBot: %s avoid %s %s at (%.1f,%.1f,%.1f) gap %.0f top %.0f dist %.0f speed %.0f "
			"hold %.2f\n",
			bot.name, what, ClassOf(hit), bot.pEdict->v.origin.x, bot.pEdict->v.origin.y,
			bot.pEdict->v.origin.z, gap, top, dist, speed, hold);
	}

	bool DetourActive(const CBot &bot, const BotAvoid::State &state, float now)
	{
		return bot.waypointIndex < state.detourEnd && now < state.detourExpire;
	}

	bool PickDestination(const CBot &bot, SearchCtx &ctx, edict_t *hit,
		const BotDeploy::Beam *beam, const Vector &origin, float hullHalf, Vector &destOut,
		size_t &indexOut)
	{
		Vector bmin = hit->v.absmin;
		Vector bmax = hit->v.absmax;
		if (beam)
			BeamBounds(*beam, bmin, bmax);

		float pad = kAvoidHullHalf + kAvoidCornerMargin;
		float minX = bmin.x - pad;
		float minY = bmin.y - pad;
		float maxX = bmax.x + pad;
		float maxY = bmax.y + pad;

		int probes = 0;
		Vector prev = origin;
		size_t limit = std::min(bot.waypoints.size(), bot.waypointIndex + kAvoidDestLookahead);
		for (size_t i = bot.waypointIndex; i < limit; ++i)
		{
			const Nav::Waypoint &wp = bot.waypoints[i];
			Vector target(wp.point.x, wp.point.y,
				wp.point.z + hullHalf - NavPlan::kPlanStandHeight);

			float len = Dist2D(prev, target);
			int steps = std::max(1, static_cast<int>(len / kAvoidDestStep));
			for (int s = 1; s <= steps; ++s)
			{
				float t = static_cast<float>(s) / static_cast<float>(steps);
				float x = prev.x + (target.x - prev.x) * t;
				float y = prev.y + (target.y - prev.y) * t;
				if (x >= minX && x <= maxX && y >= minY && y <= maxY)
					continue;
				if (probes >= kAvoidDestProbes)
					return false;
				++probes;
				if (!Ground(ctx, x, y, prev.z, destOut))
					continue;
				indexOut = i;
				return true;
			}

			if (WaypointIsFlagged(wp))
				break;
			prev = target;
		}
		return false;
	}

	bool TakeMineTarget(CBot &bot, BotAvoid::Decision &out, edict_t *hit,
		const BotDeploy::Beam *beam, const char *why)
	{
		if (!beam || !hit)
			return false;

		out.mineTarget = hit;
		bot.avoid.lastKind = BotAvoid::AVOID_SHOOT;

		BOT_VLOG("CrossBot: %s avoid shooting %s at (%.1f,%.1f,%.1f), %s\n",
			bot.name, ClassOf(hit), bot.pEdict->v.origin.x, bot.pEdict->v.origin.y,
			bot.pEdict->v.origin.z, why);

		return true;
	}

	bool TakeBreakTarget(CBot &bot, BotAvoid::Decision &out, edict_t *hit, const char *why)
	{
		if (!BotBreak::IsTarget(hit))
			return false;

		out.breakTarget = hit;
		bot.avoid.lastKind = BotAvoid::AVOID_BREAK;

		BOT_VLOG("CrossBot: %s avoid breaking %s at (%.1f,%.1f,%.1f), %s, health %.0f\n",
			bot.name, ClassOf(hit), bot.pEdict->v.origin.x, bot.pEdict->v.origin.y,
			bot.pEdict->v.origin.z, why, hit->v.health);

		return true;
	}

	void RunSideSearch(CBot &bot, const Nav::NavMesh &mesh, BotAvoid::Decision &out, edict_t *hit,
		const BotDeploy::Beam *beam, const Vector &contact, const Vector &pathDir, float gap,
		float top, int hull, float hullHalf, float now)
	{
		BotAvoid::State &state = bot.avoid;
		const Vector &origin = bot.pEdict->v.origin;

		Record(state, BotAvoid::AVOID_SIDE, hit, gap, top);
		state.nextSearchTime = now + kAvoidSearchCooldown;

		Vector travel = pathDir;
		if (bot.waypointIndex < bot.waypoints.size())
		{
			Vector step = bot.waypoints[bot.waypointIndex].point - origin;
			step.z = 0.f;
			float stepLen = Length2D(step);
			if (stepLen > 1.f)
			{
				travel.x = step.x / stepLen;
				travel.y = step.y / stepLen;
				travel.z = 0.f;
			}
		}

		float closing = Dot2D(MoverVelocityAt(hit, contact), travel);
		bool recedes = closing > kAvoidMoverSpeed;

		SearchCtx ctx;
		ctx.bot = &bot;
		ctx.mesh = &mesh;
		ctx.ignore = bot.pEdict;
		ctx.hull = hull;

		Vector dest;
		size_t d = bot.waypointIndex;
		if (!PickDestination(bot, ctx, hit, beam, origin, hullHalf, dest, d))
		{
			if (!recedes && TakeMineTarget(bot, out, hit, beam, "no clear rejoin point"))
				return;
			if (!recedes && TakeBreakTarget(bot, out, hit, "no clear rejoin point"))
				return;

			out.searchFailed = true;
			out.hitRecedes = recedes;
			state.lastKind = recedes ? BotAvoid::AVOID_FOLLOW : BotAvoid::AVOID_BLOCKED;
			if (!recedes)
				state.blockedUntil = now + 2.f * kAvoidSearchCooldown;
			BOT_VLOG("CrossBot: %s avoid %s %s at (%.1f,%.1f,%.1f) no clear rejoin point "
				"closing %.0f traces %d\n",
				bot.name, recedes ? "following" : "blocked by", ClassOf(hit), origin.x, origin.y,
				origin.z, closing, ctx.traces);
			return;
		}

		std::vector<int> visited;
		std::vector<Vector> points;
		float length = 0.f;
		bool ok = Solve(ctx, origin, dest, visited, 0, points, length);

		if (ok && points.empty())
			return;
		if (ok)
			points.push_back(dest);

		float direct = Dist2D(origin, dest);
		float budget = kAvoidDetourMaxLength;

		if (ok && length <= budget)
		{
			out.detourPoints = std::move(points);
			out.detourReplaceUntil = d;
			state.detourHit = g_engfuncs.pfnIndexOfEdict(hit);
			state.detourExpire = now + kAvoidDetourMaxTime;

			BOT_VLOG("CrossBot: %s avoid side %s at (%.1f,%.1f,%.1f) points %zu length %.0f direct %.0f "
				"traces %d to wp %zu/%zu\n",
				bot.name, ClassOf(hit), origin.x, origin.y, origin.z, out.detourPoints.size(), length,
				direct, ctx.traces, d, bot.waypoints.size());
			return;
		}

		if (!recedes && TakeMineTarget(bot, out, hit, beam, ok ? "too long" : "no way around"))
			return;
		if (!recedes && TakeBreakTarget(bot, out, hit, ok ? "too long" : "no way around"))
			return;

		out.searchFailed = true;
		out.hitRecedes = recedes;
		state.lastKind = recedes ? BotAvoid::AVOID_FOLLOW : BotAvoid::AVOID_BLOCKED;
		if (!recedes)
			state.blockedUntil = now + 2.f * kAvoidSearchCooldown;

		BOT_VLOG("CrossBot: %s avoid %s %s at (%.1f,%.1f,%.1f) %s length %.0f budget %.0f "
			"closing %.0f traces %d\n",
			bot.name, recedes ? "following" : "blocked by", ClassOf(hit), origin.x, origin.y,
			origin.z, ok ? "too long" : "no way around", length, budget, closing, ctx.traces);
	}
}

namespace BotAvoid
{

const char *KindName(Kind kind)
{
	switch (kind)
	{
	case AVOID_DUCK: return "duck";
	case AVOID_JUMP: return "jump";
	case AVOID_SIDE: return "side";
	case AVOID_FOLLOW: return "follow";
	case AVOID_BLOCKED: return "blocked";
	case AVOID_BREAK: return "break";
	case AVOID_SHOOT: return "shoot";
	default: return "none";
	}
}

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_nav_avoid");
	return !cvar || cvar->value != 0.f;
}

Decision Update(CBot &bot, const Nav::NavMesh &mesh, const Vector &pathDir,
	const PmSim::PmPhysics &physics)
{
	Decision out;

	State &state = bot.avoid;
	edict_t *pEdict = bot.pEdict;
	float now = gpGlobals->time;

	bool onGround = (pEdict->v.flags & FL_ONGROUND) != 0;
	bool ducking = (pEdict->v.flags & FL_DUCKING) != 0;

	if (state.jumpArmed)
	{
		if (onGround && now >= state.jumpCooldownUntil)
			state.jumpArmed = false;
		else
			out.duck = true;
	}

	const Vector &origin = pEdict->v.origin;
	const Vector &vel = pEdict->v.velocity;
	float speed = Length2D(vel);

	if (onGround && !state.jumpArmed && (pathDir.x != 0.f || pathDir.y != 0.f))
	{
		int hull = ducking ? kHullDuck : kHullHuman;
		float hullHalf = ducking ? kDuckHullHalf : kStandHullHalf;
		float feetZ = origin.z - hullHalf;

		float lookahead = speed * kAvoidLookaheadTime;
		lookahead = std::max(kAvoidLookaheadMin, std::min(kAvoidLookaheadMax, lookahead));

		TraceResult tr;
		Vector end = origin + pathDir * lookahead;
		g_engfuncs.pfnTraceHull(origin, end, dont_ignore_monsters, hull, pEdict, &tr);

		bool wantsBreak = false;
		bool qualifies = tr.flFraction < 1.f && HitQualifies(bot, mesh, tr.pHit, &wantsBreak);

		float solidDist = tr.flFraction < 1.f ? Length2D(tr.vecEndPos - origin) : lookahead;

		float beamDist = 0.f, beamLo = 0.f, beamHi = 0.f;
		const BotDeploy::Beam *beam = BeamAlong(origin, end, hull, beamDist, beamLo, beamHi);
		if (beam && qualifies && beamDist > solidDist)
			beam = nullptr;

		if (!beam && tr.flFraction < 1.f)
		{
			if (const BotDeploy::Beam *body = BotDeploy::BeamFor(tr.pHit))
			{
				beam = body;
				beamDist = solidDist;
				beamLo = std::min(body->start.z, body->end.z);
				beamHi = std::max(body->start.z, body->end.z);
			}
		}

		if (beam)
		{
			qualifies = false;
			wantsBreak = false;
		}

		if (tr.flFraction < 1.f && !qualifies && !beam && wantsBreak
			&& BotBreak::IsTarget(tr.pHit))
		{
			edict_t *hit = tr.pHit;
			out.breakTarget = hit;
			Record(state, AVOID_BREAK, hit, hit->v.absmin.z - feetZ, hit->v.absmax.z - feetZ);
		}

		if (qualifies || beam)
		{
			edict_t *hit = beam ? beam->ent : tr.pHit;
			float dist = beam ? beamDist : Length2D(tr.vecEndPos - origin);
			float gap = beam ? (beamLo - kAvoidBeamPad - feetZ) : (hit->v.absmin.z - feetZ);
			float top = beam ? (beamHi + kAvoidBeamPad - feetZ) : (hit->v.absmax.z - feetZ);
			float depth = beam ? kAvoidBeamDepth
				: std::min(hit->v.absmax.x - hit->v.absmin.x,
					hit->v.absmax.y - hit->v.absmin.y);

			float duckSpeed = physics.maxSpeed * PmSim::kDuckingMultiplier;
			if (duckSpeed < 1.f)
				duckSpeed = 1.f;

			float plainJump = PmSim::PlainJumpHeight(physics.gravity);
			float crouchJump = plainJump + kDuckHullHalf;

			bool canJump = (beam || top > kAvoidStepHeight)
				&& top <= crouchJump - kAvoidJumpMargin;
			bool canDuck = gap >= kAvoidDuckClearance;

			if (canJump)
			{
				if (ducking && top > plainJump)
				{
					state.duckUntil = 0.f;
					out.duck = false;
					out.suppressPlanCrouch = true;
					Record(state, AVOID_JUMP, hit, gap, top);
				}
				else
				{
					float riseTime = RiseTime(top - kDuckHullHalf, physics.gravity);
					if (riseTime >= 0.f && now >= state.jumpCooldownUntil
						&& dist <= speed * riseTime + kAvoidJumpLead)
					{
						float hold = riseTime
							+ (depth + 2.f * NavPlan::kPlanClearance) / duckSpeed;

						out.jump = true;
						state.jumpArmed = true;
						state.jumpCooldownUntil = now + kAvoidJumpCooldown;
						state.duckUntil = std::max(state.duckUntil, now + hold);

						Announce(bot, "jump", hit, gap, top, dist, speed, hold);
					}
					Record(state, AVOID_JUMP, hit, gap, top);
				}
			}
			else if (canDuck)
			{
				if (ducking)
				{
					Record(state, AVOID_DUCK, hit, gap, top);
				}
				else
				{
					float hold = (dist + depth + 2.f * NavPlan::kPlanClearance) / duckSpeed
						* kAvoidCrawlPenalty;
					hold = std::max(kAvoidDuckMinHold, std::min(kAvoidDuckMaxHold, hold));

					if (state.duckUntil <= now)
						Announce(bot, "duck", hit, gap, top, dist, speed, hold);

					state.duckUntil = std::max(state.duckUntil, now + hold);
					Record(state, AVOID_DUCK, hit, gap, top);
				}
			}
			else if (now >= state.nextSearchTime && !DetourActive(bot, state, now)
				&& bot.waypointIndex < bot.waypoints.size())
			{
				RunSideSearch(bot, mesh, out, hit, beam, tr.vecEndPos, pathDir, gap, top, hull,
					hullHalf, now);
			}
		}
	}

	if (now < state.duckUntil)
		out.duck = true;
	if (out.duck)
		out.suppressPlanCrouch = false;

	out.slowApproach = out.duck && !ducking;

	bool sideSticky = (state.lastKind == AVOID_SIDE || state.lastKind == AVOID_FOLLOW
			|| state.lastKind == AVOID_BLOCKED)
		&& (now < state.detourExpire || now < state.nextSearchTime);

	if (state.lastKind == AVOID_BREAK && out.breakTarget)
		sideSticky = true;

	if (!out.duck && !out.jump && !out.suppressPlanCrouch && !state.jumpArmed && !sideSticky)
	{
		state.lastKind = AVOID_NONE;
		state.lastHit[0] = '\0';
	}

	return out;
}

}
