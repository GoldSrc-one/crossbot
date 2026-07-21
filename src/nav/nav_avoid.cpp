#include "nav_avoid.h"

#include <extdll.h>
#include <meta_api.h>

#include "../bot/bot.h"
#include "../bot/bot_manager.h"

namespace
{
	bool g_enabled = true;

	// Horizontal distance at which bots start deflecting around each other.
	// Bigger than a single hull width (32 units) so a pair starts steering
	// apart before they're actually touching, not after.
	constexpr float kAvoidRadius = 80.f;
	constexpr float kDeflectionStrength = 1.2f;
	constexpr int kMaxOtherBots = 16; // matches BotManager's kMaxBots
}

namespace NavAvoid
{

Vector Deflect(CBot &bot, const Vector &desiredDir)
{
	if (!g_enabled)
		return desiredDir;

	edict_t *others[kMaxOtherBots];
	int count = BotManager::GetOtherBotEdicts(bot.pEdict, others, kMaxOtherBots);
	if (count == 0)
		return desiredDir;

	Vector selfOrigin = bot.pEdict->v.origin;
	int selfIndex = g_engfuncs.pfnIndexOfEdict(bot.pEdict);

	Vector deflection(0.f, 0.f, 0.f);
	for (int i = 0; i < count; ++i)
	{
		Vector toSelf = selfOrigin - others[i]->v.origin;
		toSelf.z = 0.f;
		float dist = toSelf.Length();
		if (dist < 1.f || dist >= kAvoidRadius)
			continue; // too far to matter, or degenerate (same spot)

		// Falls off linearly to zero at kAvoidRadius so there's no
		// discontinuity right at the boundary.
		float weight = (kAvoidRadius - dist) / kAvoidRadius;

		// Right-of-way tiebreak: without this, two bots approaching head-on
		// compute equal-and-opposite deflections every tick and can end up
		// oscillating in lockstep instead of settling into a stable pass.
		// Giving the lower-indexed bot of each pair a lighter deflection
		// weight than the higher-indexed one breaks that symmetry.
		int otherIndex = g_engfuncs.pfnIndexOfEdict(others[i]);
		weight *= (selfIndex < otherIndex) ? 0.5f : 1.5f;

		deflection = deflection + (toSelf / dist) * weight;
	}

	if (deflection.Length() < 0.001f)
		return desiredDir;

	// Normalize desiredDir's horizontal direction first so the deflection
	// (itself unit-ish per neighbor) has a comparable, distance-to-waypoint
	// independent effect on the resulting angle -- otherwise a far-off
	// waypoint's much larger raw vector would swamp the deflection entirely.
	float desiredLen = sqrtf(desiredDir.x * desiredDir.x + desiredDir.y * desiredDir.y);
	Vector desiredNorm = (desiredLen > 0.001f)
		? Vector(desiredDir.x / desiredLen, desiredDir.y / desiredLen, 0.f)
		: Vector(1.f, 0.f, 0.f);

	return desiredNorm + deflection * kDeflectionStrength;
}

void SetEnabled(bool enabled)
{
	g_enabled = enabled;
}

bool IsEnabled()
{
	return g_enabled;
}

} // namespace NavAvoid
