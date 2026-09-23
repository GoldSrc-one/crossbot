#include <extdll.h>
#include <meta_api.h>

#include <algorithm>

#include "bot_fire.h"
#include "bot_move.h"
#include "bot.h"
#include "bot_aim.h"
#include "bot_camp.h"
#include "bot_difficulty.h"
#include "bot_break.h"
#include "bot_deploy.h"
#include "bot_fun.h"
#include "bot_gauss.h"
#include "bot_weapons.h"
#include "../game/game_profile.h"
#include "../game/give_item.h"
#include "../nav/nav_math.h"
#include "../nav/pm_sim.h"
#include "../nav/path_profile.h"
#include "bot_move_nav.h"

namespace
{
	using NavMath::kPiF;

	constexpr float kHitscanRange = 8192.f;

	bool Zoomed(const edict_t *pEdict)
	{
		return pEdict->v.fov != 0.f && pEdict->v.fov < 90.f;
	}

	constexpr float kSelectInterval = 0.5f;
	constexpr float kIdleSelectInterval = 3.f;
	constexpr float kCombatMemory = 3.f;
	constexpr float kStickyWindow = 0.5f;
	constexpr float kCarryEpsilon = 0.05f;
	constexpr float kEquipInterval = 0.5f;

	constexpr float kMaxFlightTime = 1.5f;

	constexpr int kLeadPasses = 4;

	constexpr float kHomingCone = 20.f;

	constexpr float kUphillSlack = 16.f;

	constexpr float kChargeSafety = 0.5f;

	constexpr float kChargeBurn = 0.1f;

	constexpr float kPolicyChargeShare = 0.5f;

	constexpr float kPolicyChargeBonus = 1.f;

	constexpr float kZoomIn = 500.f;

	constexpr float kZoomOut = 300.f;

	constexpr float kZoomConeEnter = 0.5f;

	constexpr float kArcStep = 0.1f;

	constexpr int kArcMaxSteps = 24;

	constexpr float kArcMuzzleFwd = 16.f;

	constexpr float kSplashAccept = 0.5f;

	constexpr float kThrowPitchMin = -89.f;

	constexpr float kThrowPitchMax = 60.f;

	constexpr int kThrowScan = 48;

	constexpr int kThrowBisect = 10;

	constexpr float kTapTimeout = 1.5f;

	constexpr float kGuideMax = 6.f;

	constexpr float kSpinupSlack = 0.2f;

	constexpr float kSpreadMaxWait = 1.5f;

	constexpr float kFeetLift = 2.f;

	constexpr float kBraceTraceInterval = 0.1f;

	constexpr float kDuckEyeDrop = 34.f;

	constexpr int kMaxChoices = 64;

	char g_pin[32] = "";

	int g_pinMode = 0;

	int g_pinZoom = -1;

	struct Choice
	{
		int index;
		BotFire::Mode mode;
	};

	float RandomUnit(BotFire::State &fire)
	{
		fire.rngState = fire.rngState * 1664525u + 1013904223u;
		return static_cast<float>((fire.rngState >> 8) & 0xFFFFFFu) / 16777216.f;
	}

	void SeedFire(CBot &bot)
	{
		BotFire::State &fire = bot.fire;
		if (fire.rngSeeded)
			return;

		fire.rngSeeded = true;
		fire.rngState = static_cast<unsigned int>(
			g_engfuncs.pfnIndexOfEdict(bot.pEdict)) * 2654435761u + 2131u;
	}

	bool Slab(float origin, float dir, float lo, float hi, float &tmin, float &tmax)
	{
		if (fabsf(dir) < 1e-6f)
			return origin >= lo && origin <= hi;

		float inv = 1.f / dir;
		float t1 = (lo - origin) * inv;
		float t2 = (hi - origin) * inv;
		if (t1 > t2)
			std::swap(t1, t2);

		if (t1 > tmin)
			tmin = t1;
		if (t2 < tmax)
			tmax = t2;

		return tmin <= tmax;
	}

	bool RayHitsBox(const Vector &origin, const Vector &dir, const Vector &centre, const Vector &half,
		float range, float &entry)
	{
		float tmin = 0.f;
		float tmax = range;

		if (!Slab(origin.x, dir.x, centre.x - half.x, centre.x + half.x, tmin, tmax))
			return false;
		if (!Slab(origin.y, dir.y, centre.y - half.y, centre.y + half.y, tmin, tmax))
			return false;
		if (!Slab(origin.z, dir.z, centre.z - half.z, centre.z + half.z, tmin, tmax))
			return false;

		entry = tmin;
		return true;
	}

	int AmmoCount(const BotInventory &inv, int type)
	{
		if (type < 0 || type >= MAX_AMMO_SLOTS)
			return 0;
		return inv.ammo[type];
	}

	bool IsHoming(const WeaponMode &m)
	{
		return (m.quirks & WPN_Q_HOMING) != 0;
	}

	bool IsArc(const WeaponMode &m)
	{
		return m.gravity > 0.f && m.splash > 0.f && !IsHoming(m);
	}

	bool IsThrow(const WeaponMode &m)
	{
		return (m.quirks & WPN_Q_THROW_ARC) != 0;
	}

	bool ArcDetonates(const WeaponMode &m)
	{
		return IsArc(m) && !IsThrow(m);
	}

	bool IsSticky(const WeaponMode &m)
	{
		return (m.quirks & WPN_Q_STICKY) != 0;
	}

	bool StickyAllowed(const CBot &bot, const WeaponMode &m)
	{
		return IsSticky(m) && gpGlobals->time - bot.sense.targetTime <= kStickyWindow;
	}

	bool IsGuided(const WeaponMode &m)
	{
		return (m.quirks & WPN_Q_KEEP_AIMING) != 0;
	}

	float ArcGravity(const WeaponMode &m)
	{
		return PmSim::kGravity * m.gravity;
	}

	bool IsChargeWeapon(const WeaponSpec &spec)
	{
		return spec.secondary.fire == FIRE_CHARGE;
	}

	bool IsScopeWeapon(const WeaponSpec &spec)
	{
		return spec.primaryZoomed.fire != FIRE_NONE && spec.secondary.toggle == TOGGLE_ZOOM;
	}

	bool ScopeCommitted(const WeaponSpec &spec)
	{
		return BotDiff::GaussPolicy() && IsScopeWeapon(spec);
	}

	bool EffectiveZoom(const CBot &bot, const WeaponSpec &spec)
	{
		return bot.fire.zoomed || ScopeCommitted(spec);
	}

	bool AimWithinCone(const CBot &bot, const WeaponSpec &spec, const Vector &eye,
		float fraction)
	{
		if (!bot.fire.aimValid || spec.zoomFov <= 0.f)
			return false;

		Vector to = bot.fire.aimPoint - eye;
		float len = to.Length();
		if (len < 0.0001f)
			return true;

		MAKE_VECTORS(bot.pEdict->v.v_angle);
		float dot = (to.x * gpGlobals->v_forward.x + to.y * gpGlobals->v_forward.y +
			to.z * gpGlobals->v_forward.z) / len;

		return dot >= cosf(spec.zoomFov * 0.5f * fraction * kPiF / 180.f);
	}

	float ChargeTime(const WeaponMode &m)
	{
		if (m.chargeFull <= 0.f)
			return 0.f;
		return BotDiff::GaussPolicy() ? m.chargeFull * kPolicyChargeShare : m.chargeFull;
	}

	int ShotCost(const WeaponMode &m)
	{
		int per = (m.ammoPerShot > 0) ? m.ammoPerShot : 1;
		if (m.fire != FIRE_CHARGE || m.chargeFull <= 0.f)
			return per;
		return per * (1 + static_cast<int>(ChargeTime(m) / kChargeBurn));
	}

	bool AmmoReady(const BotInventory &inv, const BotWeapon &w, const WeaponMode &m, bool strict)
	{
		int need = ShotCost(m);

		switch (m.ammo)
		{
		case AMMO_NONE:
			return true;
		case AMMO_CLIP:
			if (w.clip < 0 || w.clip >= need)
				return true;
			if (strict)
				return false;
			return AmmoCount(inv, w.primaryAmmoType) > 0;
		case AMMO_PRIMARY:
			return AmmoCount(inv, w.primaryAmmoType) >= need;
		case AMMO_SECONDARY:
			return AmmoCount(inv, w.secondaryAmmoType) >= need;
		}

		return false;
	}

	float ModeSpeed(const CBot &bot, const WeaponMode &m)
	{
		float speed = m.speed;
		if ((m.quirks & WPN_Q_SLOW_IN_WATER) != 0 && bot.pEdict->v.waterlevel == 3)
			speed *= 0.5f;
		return speed;
	}

	Vector InheritedVelocity(const CBot &bot, const WeaponMode &m)
	{
		if ((m.quirks & WPN_Q_INHERIT_VEL) == 0)
			return Vector(0.f, 0.f, 0.f);
		return bot.pEdict->v.velocity;
	}

	Vector AimBase(const CBot &bot, const WeaponMode &m, const Vector &basePoint,
		const Vector &baseVel)
	{
		Vector base = basePoint;

		if ((m.quirks & WPN_Q_SPINUP) != 0)
			base = base + baseVel * m.chargeFull;

		edict_t *target = bot.sense.target.Get();
		if ((m.quirks & WPN_Q_AIM_FEET) != 0 && target)
			base.z += target->v.origin.z + target->v.mins.z + kFeetLift - bot.sense.targetPoint.z;

		return base;
	}

	bool SolveLead(const Vector &eye, const Vector &base, const Vector &vel, float speed,
		Vector &point, float &flight)
	{
		if (speed <= 0.f)
			return false;

		Vector p = base;
		float t = 0.f;

		for (int i = 0; i < kLeadPasses; ++i)
		{
			t = (p - eye).Length() / speed;
			p = base + vel * t;
		}

		if (t > kMaxFlightTime)
			return false;

		point = p;
		flight = t;
		return true;
	}

	bool SolveArc(const Vector &eye, const Vector &target, float speed, float gravity,
		Vector &dir, float &flight)
	{
		if (speed <= 0.f || gravity <= 0.f)
			return false;

		Vector flat(target.x - eye.x, target.y - eye.y, 0.f);
		float d = flat.Length();
		float h = target.z - eye.z;

		if (d < 1.f)
			return false;

		float vv = speed * speed;
		float disc = vv * vv - gravity * (gravity * d * d + 2.f * h * vv);
		if (disc < 0.f)
			return false;

		float tangent = (vv - sqrtf(disc)) / (gravity * d);
		float pitch = atanf(tangent);

		float cosine = cosf(pitch);
		if (cosine < 0.001f)
			return false;

		flight = d / (speed * cosine);
		if (flight > kMaxFlightTime)
			return false;

		flat = flat * (1.f / d);
		dir.x = flat.x * cosine;
		dir.y = flat.y * cosine;
		dir.z = sinf(pitch);
		return true;
	}

	bool SolveArcLead(const Vector &eye, const Vector &base, const Vector &vel, float speed,
		float gravity, Vector &dir, Vector &point, float &flight)
	{
		Vector p = base;
		float t = 0.f;

		for (int i = 0; i < kLeadPasses; ++i)
		{
			if (!SolveArc(eye, p, speed, gravity, dir, t))
				return false;
			if (i + 1 < kLeadPasses)
				p = base + vel * t;
		}

		point = p;
		flight = t;
		return true;
	}

	struct ThrowShot
	{
		Vector dir;
		float speed;
		float pitch;
		float flight;
		float error;
	};

	bool SolveThrowAt(const WeaponMode &m, float pitch, const Vector &src, const Vector &target,
		const Vector &pvel, float gravity, ThrowShot &out)
	{
		float elevation = 0.f;
		float speed = 0.f;
		if (!g_pGameProfile->ThrowLaunch(m, pitch, elevation, speed))
			return false;

		float rise = elevation * kPiF / 180.f;
		float flatSpeed = speed * cosf(rise);
		if (flatSpeed < 1.f)
			return false;

		Vector flat(target.x - src.x, target.y - src.y, 0.f);
		Vector drift(pvel.x, pvel.y, 0.f);

		float a = DotProduct(flat, flat);
		if (a < 1.f)
			return false;

		float b = DotProduct(flat, drift);
		float c = DotProduct(drift, drift) - flatSpeed * flatSpeed;

		float disc = b * b - a * c;
		if (disc < 0.f)
			return false;

		float inverse = (b + sqrtf(disc)) / a;
		if (inverse <= 0.f)
			return false;

		float flight = 1.f / inverse;
		if (flight > kMaxFlightTime)
			return false;

		Vector course = (flat * inverse - drift) * (1.f / flatSpeed);

		float cosine = cosf(rise);
		out.dir.x = course.x * cosine;
		out.dir.y = course.y * cosine;
		out.dir.z = sinf(rise);
		out.speed = speed;
		out.pitch = pitch;
		out.flight = flight;
		out.error = (speed * sinf(rise) + pvel.z) * flight -
			0.5f * gravity * flight * flight - (target.z - src.z);
		return true;
	}

	bool SolveThrow(const WeaponMode &m, const Vector &src, const Vector &target,
		const Vector &pvel, float gravity, ThrowShot &out)
	{
		float step = (kThrowPitchMax - kThrowPitchMin) / static_cast<float>(kThrowScan);
		float under = 0.f;
		bool bracket = false;

		ThrowShot shot;
		ThrowShot last;
		bool haveLast = false;

		for (int i = 0; i <= kThrowScan; ++i)
		{
			float pitch = kThrowPitchMax - step * static_cast<float>(i);
			if (!SolveThrowAt(m, pitch, src, target, pvel, gravity, shot))
				continue;

			if (shot.error >= 0.f)
			{
				if (!haveLast)
					return false;
				out = shot;
				under = last.pitch;
				bracket = true;
				break;
			}

			last = shot;
			haveLast = true;
		}

		if (!bracket)
			return false;

		float over = out.pitch;
		for (int i = 0; i < kThrowBisect; ++i)
		{
			float mid = (under + over) * 0.5f;
			if (!SolveThrowAt(m, mid, src, target, pvel, gravity, shot))
				break;

			if (shot.error >= 0.f)
			{
				out = shot;
				over = mid;
			}
			else
			{
				under = mid;
			}
		}

		return true;
	}

	bool SolveThrowLead(const WeaponMode &m, const Vector &eye, const Vector &base,
		const Vector &vel, const Vector &pvel, float gravity, ThrowShot &out, Vector &point)
	{
		Vector src = eye;
		Vector p = base;

		for (int i = 0; i < kLeadPasses; ++i)
		{
			if (!SolveThrow(m, src, p, pvel, gravity, out))
				return false;
			if (i + 1 < kLeadPasses)
			{
				src = eye + out.dir * kArcMuzzleFwd;
				p = base + vel * out.flight;
			}
		}

		point = p;
		return true;
	}

	Vector SimulateArc(edict_t *pEdict, const Vector &start, const Vector &launch, float gravity,
		bool &hit)
	{
		Vector pos = start;
		Vector vel = launch;

		for (int i = 0; i < kArcMaxSteps; ++i)
		{
			vel.z -= gravity * kArcStep;
			Vector next = pos + vel * kArcStep;

			TraceResult tr;
			TRACE_LINE(pos, next, dont_ignore_monsters, pEdict, &tr);
			if (tr.flFraction < 1.f)
			{
				hit = true;
				return tr.vecEndPos;
			}

			pos = next;
		}

		hit = false;
		return pos;
	}

	float AxisGap(float point, float centre, float half)
	{
		float off = fabsf(point - centre) - half;
		return (off > 0.f) ? off : 0.f;
	}

	float DistanceToBox(const Vector &point, const Vector &centre, const Vector &half)
	{
		Vector d(AxisGap(point.x, centre.x, half.x),
			AxisGap(point.y, centre.y, half.y),
			AxisGap(point.z, centre.z, half.z));

		return d.Length();
	}

	bool SplashSafe(BotFire::State &fire, const Vector &eye, const Vector &impact)
	{
		if ((impact - fire.targetPoint).Length() < (impact - eye).Length())
			return true;

		++fire.selfVetoes;
		return false;
	}

	void PublishAim(BotFire::State &fire, const WeaponMode &m, const Vector &point,
		const Vector &target, float flight)
	{
		fire.aimPoint = point;
		fire.targetPoint = target;
		fire.flightTime = flight;
		fire.aimValid = true;
		fire.aimArc = IsArc(m);
		fire.aimHoming = IsHoming(m);
		fire.aimLead = m.aim == AIM_LEAD && !fire.aimHoming && !fire.aimArc;
		if (m.aim == AIM_TARGET && !fire.aimHoming && !fire.aimArc)
			++fire.targetAims;
	}

	void ClearAim(BotFire::State &fire)
	{
		fire.aimValid = false;
		fire.aimLead = false;
		fire.aimHoming = false;
		fire.aimArc = false;
		fire.flightTime = 0.f;
	}

	void CountShots(CBot &bot)
	{
		BotFire::State &fire = bot.fire;
		const BotInventory &inv = bot.inventory;

		bool pressed = fire.pressed;
		bool pressedLead = fire.pressedLead;
		bool pressedArc = fire.pressedArc;
		bool pressedSticky = fire.pressedSticky;
		bool pressedWall = fire.pressedWall;
		fire.pressed = false;
		fire.pressedLead = false;
		fire.pressedArc = false;
		fire.pressedSticky = false;
		fire.pressedWall = false;

		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
		{
			fire.lastTimerId = 0;
			return;
		}

		const BotWeapon &w = inv.weapons[inv.activeIndex];

		if (fire.lastTimerId == w.id)
		{
			bool fired = (fire.lastPrimaryTimer <= 0.f && w.nextPrimaryAttack > 0.f) ||
				(fire.lastSecondaryTimer <= 0.f && w.nextSecondaryAttack > 0.f);

			if (pressedWall && (fired || (fire.lastChargeState != 0 && w.chargeState == 0)))
				++fire.wallbangs;

			if (pressed)
			{
				if (fired)
				{
					++fire.shots;
					fire.lastShotTime = gpGlobals->time;
					if (pressedLead)
						++fire.leadShots;
					if (pressedArc)
						++fire.arcShots;
					if (pressedSticky)
						++fire.stickyShots;
				}
			}

			if (fire.lastChargeState != 0 && w.chargeState == 0)
			{
				++fire.shots;
				fire.lastShotTime = gpGlobals->time;
				++fire.chargeShots;
			}

			if (fire.lastThrowStart != 0.f && w.throwStart == 0.f)
			{
				++fire.shots;
				fire.lastShotTime = gpGlobals->time;
				++fire.throwShots;
			}
		}

		fire.lastTimerId = w.id;
		fire.lastPrimaryTimer = w.nextPrimaryAttack;
		fire.lastSecondaryTimer = w.nextSecondaryAttack;
		fire.lastChargeState = w.chargeState;
		fire.lastThrowStart = w.throwStart;
	}

	void Idle(BotFire::State &fire, const char *reason)
	{
		fire.mode = BotFire::MODE_NONE;
		fire.weaponId = 0;
		fire.wantReload = false;
		fire.commit = BotFire::COMMIT_NONE;
		ClearAim(fire);
		fire.lastReason = reason;
	}

	bool StickyHold(CBot &bot)
	{
		BotFire::State &fire = bot.fire;

		if (fire.mode == BotFire::MODE_NONE || !fire.aimValid || !g_pGameProfile)
			return false;

		const BotInventory &inv = bot.inventory;
		if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
			return false;
		if (inv.weapons[inv.activeIndex].id != fire.weaponId)
			return false;

		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(fire.weaponId);
		if (!spec)
			return false;

		const WeaponMode *m = BotFire::ModeOf(*spec, fire.mode, fire.zoomed);
		return m && StickyAllowed(bot, *m);
	}

	enum RayVeto
	{
		RAY_HIT = 0,
		RAY_WIDE,
		RAY_BLOCKED,
		RAY_SELF,
	};

	RayVeto RayOnTarget(CBot &bot, const Vector &angles, const WeaponMode &m, const Vector &eye)
	{
		BotFire::State &fire = bot.fire;
		edict_t *pEdict = bot.pEdict;
		fire.rayWall = false;

		MAKE_VECTORS(angles + pEdict->v.punchangle);
		Vector dir = gpGlobals->v_forward;

		Vector centre = fire.aimPoint;
		Vector los = centre - eye;
		float dist = los.Length();

		Vector half = fire.targetSize * 0.5f;

		if (fire.aimHoming)
		{
			float wide = dist * tanf(kHomingCone * kPiF / 180.f);
			if (wide > half.x)
				half.x = wide;
			if (wide > half.y)
				half.y = wide;
			if (wide > half.z)
				half.z = wide;
		}

		if (dist > 0.001f)
		{
			float dot = DotProduct(dir, los * (1.f / dist));
			if (dot > 1.f)
				dot = 1.f;
			else if (dot < -1.f)
				dot = -1.f;

			fire.lastMiss = acosf(dot) * (180.f / kPiF);

			float big = std::max(half.x, std::max(half.y, half.z));
			fire.lastTol = atanf(big / dist) * (180.f / kPiF);
		}

		float range = (m.range > 0.f) ? m.range : kHitscanRange;

		float entry = 0.f;
		if (!RayHitsBox(eye, dir, centre, half, range, entry))
		{
			fire.lastReason = "off-target";
			++fire.aimVetoes;
			return RAY_WIDE;
		}

		TraceResult tr;
		TRACE_LINE(eye, eye + dir * range, ignore_monsters, pEdict, &tr);
		float reach = tr.flFraction * range;
		if (reach < entry)
		{
			if (!BotDiff::Wallbang() || !g_pGameProfile->Penetrates(bot, fire.weaponId, eye, dir, entry))
			{
				fire.lastReason = "blocked";
				return RAY_BLOCKED;
			}
			fire.lastReason = "wallbang";
			fire.rayWall = true;
		}

		if (m.splash > 0.f && !SplashSafe(fire, eye, eye + dir * entry))
		{
			fire.lastReason = "splash-self";
			return RAY_SELF;
		}

		return RAY_HIT;
	}

	bool ArcImpactHits(CBot &bot, const WeaponMode &m, const Vector &eye, const Vector &impact,
		bool hit)
	{
		BotFire::State &fire = bot.fire;

		Vector half = fire.targetSize * 0.5f;
		float gap = DistanceToBox(impact, fire.targetPoint, half);
		float allow = m.splash * kSplashAccept;

		fire.lastMiss = gap;
		fire.lastTol = allow;

		if (gap > allow)
		{
			fire.lastReason = hit ? "arc-blocked" : "arc-wide";
			return false;
		}

		if (!SplashSafe(fire, eye, impact))
		{
			fire.lastReason = "arc-self";
			return false;
		}

		return true;
	}

	bool ArcOnTarget(CBot &bot, const Vector &angles, const WeaponMode &m, const Vector &eye)
	{
		Vector view = angles + bot.pEdict->v.punchangle;
		float speed = ModeSpeed(bot, m);
		Vector drift = InheritedVelocity(bot, m);

		if (IsThrow(m))
		{
			float elevation = 0.f;
			if (!g_pGameProfile->ThrowLaunch(m, view.x, elevation, speed))
			{
				bot.fire.lastReason = "arc-wide";
				return false;
			}

			view.x = -elevation;
		}

		MAKE_VECTORS(view);
		Vector dir = gpGlobals->v_forward;

		bool hit = false;
		Vector impact = SimulateArc(bot.pEdict, eye + dir * kArcMuzzleFwd, dir * speed + drift,
			ArcGravity(m), hit);

		return ArcImpactHits(bot, m, eye, impact, hit);
	}

	unsigned short ZoomButtons(CBot &bot, const WeaponSpec &spec, const BotWeapon &w, const Vector &eye)
	{
		BotFire::State &fire = bot.fire;

		if (spec.primaryZoomed.fire == FIRE_NONE || spec.secondary.toggle != TOGGLE_ZOOM)
			return 0;
		if (w.nextSecondaryAttack > 0.f)
			return 0;

		bool want = false;
		if (ScopeCommitted(spec))
		{
			want = AimWithinCone(bot, spec, eye, fire.zoomed ? 1.f : kZoomConeEnter);
		}
		else if (fire.aimValid)
		{
			float dist = (fire.aimPoint - eye).Length();
			want = fire.zoomed ? (dist >= kZoomOut) : (dist > kZoomIn);
		}
		else if (!BotCamp::WantZoom(bot, want))
			return 0;

		if (g_pinZoom >= 0)
			want = (g_pinZoom != 0);
		if (want == fire.zoomed)
			return 0;

		return IN_ATTACK2;
	}

	unsigned short AttackButton(BotFire::Mode mode)
	{
		return (mode == BotFire::MODE_SECONDARY) ? IN_ATTACK2 : IN_ATTACK;
	}

	void StartCommit(BotFire::State &fire, BotFire::Commit kind)
	{
		fire.commit = kind;
		fire.commitStart = gpGlobals->time;
		fire.commitSeen = false;
	}

	bool ChargeExpired(const BotFire::State &fire, const BotWeapon &w)
	{
		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
		const WeaponMode *m = spec ? BotFire::ModeOf(*spec, fire.mode, fire.zoomed) : nullptr;
		return m && m->chargeMax > 0.f && gpGlobals->time - fire.commitStart >= m->chargeMax - kChargeSafety;
	}

	unsigned short ChargeButtons(BotFire::State &fire, const WeaponMode &m, bool onTarget)
	{
		unsigned short button = AttackButton(fire.mode);
		float now = gpGlobals->time;

		if (fire.commit != BotFire::COMMIT_CHARGE)
		{
			StartCommit(fire, BotFire::COMMIT_CHARGE);
			fire.lastReason = "charging";
			return button;
		}

		float held = now - fire.commitStart;

		if (m.chargeMax > 0.f && held >= m.chargeMax - kChargeSafety)
		{
			fire.commit = BotFire::COMMIT_NONE;
			fire.lastReason = "charge-timeout";
			return 0;
		}

		if (held < ChargeTime(m))
		{
			fire.lastReason = "charging";
			return button;
		}

		if (!onTarget && !fire.releaseCharge)
		{
			fire.lastReason = "charged";
			return button;
		}

		if (!onTarget)
			++fire.gaussJumps;

		fire.commit = BotFire::COMMIT_NONE;
		fire.releaseCharge = false;
		fire.lastReason = !onTarget ? "gauss-jump" : fire.pressedWall ? "wallbang" : "firing";
		return 0;
	}

	unsigned short TapButtons(BotFire::State &fire, const BotWeapon &w, const WeaponMode &m,
		bool onTarget)
	{
		bool thrown = IsThrow(m);

		if (thrown && (fire.commit == BotFire::COMMIT_THROW || w.throwStart != 0.f))
		{
			fire.lastReason = "primed";
			return 0;
		}

		float timer = (fire.mode == BotFire::MODE_SECONDARY)
			? w.nextSecondaryAttack : w.nextPrimaryAttack;
		if (timer > 0.f)
			return 0;

		if (!onTarget)
			return 0;

		if (thrown)
			StartCommit(fire, BotFire::COMMIT_THROW);
		if ((m.quirks & WPN_Q_SPINUP) != 0)
		{
			StartCommit(fire, BotFire::COMMIT_SPINUP);
			fire.commitFor = m.chargeFull + kSpinupSlack;
			++fire.spinups;
		}
		fire.lastReason = fire.pressedWall ? "wallbang" : "firing";
		return AttackButton(fire.mode);
	}

	bool SpreadHold(CBot &bot, const BotWeapon &w, const Vector &eye, bool onTarget)
	{
		BotFire::State &fire = bot.fire;
		float now = gpGlobals->time;
		if (fire.spreadUntil > now && fire.spreadUntil - now < kSpreadMaxWait)
			return true;

		float fit = BotDiff::SpreadFit();
		if (!onTarget || fit <= 0.f || !fire.haveSize || w.clip == fire.spreadClip)
			return false;

		float dist = (fire.aimPoint - eye).Length();
		if (dist < 1.f)
			return false;

		float half = std::min(fire.targetSize.x, fire.targetSize.y) * 0.5f;
		float ready = 0.f;
		if (!g_pGameProfile->SpreadWide(bot.pEdict, w.id, w.priv, fit * half / dist, ready))
			return false;

		float lo = 0.f, hi = 0.f;
		BotDiff::SpreadWait(lo, hi);
		fire.spreadUntil = ready + lo + (hi - lo) * RandomUnit(fire);
		fire.spreadClip = w.clip;
		++fire.spreadWaits;
		return fire.spreadUntil > now;
	}
}

namespace BotFire
{

bool Enabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_fire");
	return cvar && cvar->value != 0.f;
}

void Pin(const char *classname, int mode, int zoom)
{
	g_pinMode = mode;
	g_pinZoom = zoom;

	if (!classname || !classname[0])
	{
		g_pin[0] = 0;
		g_pinMode = 0;
		g_pinZoom = -1;
		return;
	}

	snprintf(g_pin, sizeof(g_pin), "%s", classname);
}

const char *Pinned()
{
	return g_pin[0] ? g_pin : nullptr;
}

const char *PinnedMode()
{
	if (g_pinMode == 1)
		return "primary";
	if (g_pinMode == 2)
		return "secondary";
	return "any";
}

const char *PinnedZoom()
{
	if (g_pinZoom == 1)
		return "zoom";
	if (g_pinZoom == 0)
		return "nozoom";
	return "any";
}

int PinnedZoomState()
{
	return g_pinZoom;
}

const char *ModeName(Mode mode)
{
	switch (mode)
	{
	case MODE_NONE: return "none";
	case MODE_PRIMARY: return "primary";
	case MODE_SECONDARY: return "secondary";
	}
	return "?";
}

const char *CommitName(Commit commit)
{
	switch (commit)
	{
	case COMMIT_NONE: return "none";
	case COMMIT_CHARGE: return "charge";
	case COMMIT_THROW: return "throw";
	case COMMIT_GUIDE: return "guide";
	case COMMIT_SPINUP: return "spinup";
	}
	return "?";
}

bool Supported(const WeaponMode &mode)
{
	if (mode.fire != FIRE_HOLD && mode.fire != FIRE_CHARGE && mode.fire != FIRE_TAP)
		return false;
	if (mode.toggle != TOGGLE_NONE)
		return false;
	if (mode.aim != AIM_TARGET && mode.aim != AIM_LEAD)
		return false;
	if (mode.aim == AIM_LEAD && mode.speed <= 0.f)
		return false;

	bool homing = IsHoming(mode);
	bool arc = IsArc(mode);
	bool thrown = IsThrow(mode);

	if (mode.gravity != 0.f && !homing && !arc)
		return false;
	if (thrown && !arc)
		return false;

	unsigned int reject = WPN_Q_HIDDEN_STATE;
	if (!homing && !thrown && !arc && !IsGuided(mode))
		reject |= WPN_Q_INHERIT_VEL;

	return (mode.quirks & reject) == 0;
}

bool SolveOffhand(const WeaponMode &mode, const Vector &src, const Vector &base, const Vector &vel,
	Vector &look, float &flight)
{
	ThrowShot shot;
	Vector point;
	if (!g_pGameProfile || !SolveThrowLead(mode, src, base, vel, Vector(0.f, 0.f, 0.f), ArcGravity(mode),
		shot, point))
		return false;

	MAKE_VECTORS(Vector(shot.pitch, atan2f(shot.dir.y, shot.dir.x) * (180.f / kPiF), 0.f));
	look = gpGlobals->v_forward;
	flight = shot.flight;
	return true;
}

const WeaponMode *ModeOf(const WeaponSpec &spec, Mode mode, bool zoomed)
{
	if (mode == MODE_PRIMARY)
		return (zoomed && spec.primaryZoomed.fire != FIRE_NONE) ? &spec.primaryZoomed : &spec.primary;
	if (mode == MODE_SECONDARY)
		return &spec.secondary;
	return nullptr;
}

bool WeaponArmed(const BotInventory &inv, const BotWeapon &w)
{
	const WeaponSpec *spec = g_pGameProfile ? g_pGameProfile->GetWeaponSpec(w.id) : nullptr;
	if (!spec)
		return false;

	const WeaponMode *modes[2] = { &spec->primary, &spec->secondary };
	for (const WeaponMode *m : modes)
	{
		if (m->fire == FIRE_NONE || m->aim == AIM_NONE)
			continue;
		if (AmmoReady(inv, w, *m, false))
			return true;
	}

	return false;
}

bool ModeViable(CBot &bot, const BotWeapon &w, const WeaponMode &m, bool strict)
{
	if (!Supported(m))
		return false;

	if (g_pin[0] && g_pGameProfile)
	{
		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
		if (!spec || !spec->classname || strcmp(spec->classname, g_pin) != 0)
			return false;
		if (g_pinMode == 1 && &m != &spec->primary && &m != &spec->primaryZoomed)
			return false;
		if (g_pinMode == 2 && &m != &spec->secondary)
			return false;
	}

	if ((m.quirks & WPN_Q_NO_UNDERWATER) != 0 && bot.pEdict->v.waterlevel == 3)
		return false;

	if (BotDiff::GaussPolicy() && g_pGameProfile)
	{
		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
		if (spec && IsChargeWeapon(*spec) && &m != &spec->secondary)
			return false;
	}

	if (g_pGameProfile)
	{
		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
		if (spec && !g_pGameProfile->ModeAvailable(bot.pEdict, *spec, m))
			return false;
	}

	return AmmoReady(bot.inventory, w, m, strict);
}

bool ModeUsable(CBot &bot, const BotWeapon &w, const WeaponMode &m, float dist, float rise,
	bool strict)
{
	if (!ModeViable(bot, w, m, strict))
		return false;

	if (m.gravity > 0.f && !ArcDetonates(m) && rise > kUphillSlack)
	{
		++bot.fire.uphillVetoes;
		return false;
	}

	float range = (m.range > 0.f) ? m.range : kHitscanRange;
	if (dist > range)
		return false;

	if (m.aim == AIM_LEAD)
	{
		float speed = ModeSpeed(bot, m);
		if (speed <= 0.f || dist / speed > kMaxFlightTime)
			return false;
	}

	return true;
}

float CarryOf(const CBot &bot, int weaponId)
{
	if (weaponId <= 0 || weaponId >= kMaxWeaponIds)
		return 0.f;

	return bot.weaponCarry.seconds[weaponId];
}

void AccrueCarry(CBot &bot)
{
	const BotInventory &inv = bot.inventory;
	float dt = Bot_FrameTime();

	for (int i = 0; i < inv.count; ++i)
	{
		int id = inv.weapons[i].id;
		if (id <= 0 || id >= kMaxWeaponIds)
			continue;

		bot.weaponCarry.seconds[id] += dt;
	}
}

void Reset(CBot &bot)
{
	bot.fire = State();
}

static void PinEquip(CBot &bot)
{
	if (!g_pin[0] || !g_pGameProfile)
		return;

	State &fire = bot.fire;
	if (gpGlobals->time < fire.nextEquipTime)
		return;
	fire.nextEquipTime = gpGlobals->time + kEquipInterval;

	if (g_pGameProfile->IsDead(bot.pEdict))
		return;

	const BotInventory &inv = bot.inventory;
	const BotWeapon *held = nullptr;
	const WeaponSpec *spec = nullptr;

	for (int i = 0; i < inv.count; ++i)
	{
		const WeaponSpec *s = g_pGameProfile->GetWeaponSpec(inv.weapons[i].id);
		if (!s || !s->classname || strcmp(s->classname, g_pin) != 0)
			continue;
		held = &inv.weapons[i];
		spec = s;
		break;
	}

	if (!held)
	{
		GiveItem::Give(bot.pEdict, g_pin);
		return;
	}

	Mode order[2] = { MODE_PRIMARY, MODE_SECONDARY };
	int tries = 2;

	if (g_pinMode == 1)
	{
		tries = 1;
	}
	else if (g_pinMode == 2)
	{
		order[0] = MODE_SECONDARY;
		tries = 1;
	}

	bool zoomed = Zoomed(bot.pEdict);

	for (int i = 0; i < tries; ++i)
	{
		const WeaponMode *m = ModeOf(*spec, order[i], zoomed);
		if (!m || !Supported(*m))
			continue;
		if (AmmoReady(inv, *held, *m, false))
			return;

		const char *give = g_pGameProfile->AmmoEntity(held->id, m->ammo);
		GiveItem::Give(bot.pEdict, give ? give : g_pin);
		return;
	}
}

static float WeaponScore(int weaponId)
{
	const WeaponSpec *spec = g_pGameProfile ? g_pGameProfile->GetWeaponSpec(weaponId) : nullptr;
	if (!spec)
		return 0.f;

	if (BotDiff::GaussPolicy() && IsChargeWeapon(*spec))
		return spec->value + kPolicyChargeBonus;

	return spec->value;
}

static bool WeaponViable(CBot &bot, const BotWeapon &w)
{
	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
	if (!spec)
		return false;

	return ModeViable(bot, w, spec->primary, false) || ModeViable(bot, w, spec->secondary, false);
}

static int ReloadDeficit(CBot &bot, const BotWeapon &w)
{
	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
	if (!spec || spec->clipSize <= 0 || w.clip < 0 || w.clip >= spec->clipSize)
		return 0;

	if (!WeaponViable(bot, w))
		return 0;

	int deficit = spec->clipSize - w.clip;
	int reserve = AmmoCount(bot.inventory, w.primaryAmmoType);

	return (deficit < reserve) ? deficit : reserve;
}

static int BuildChoices(CBot &bot, bool strict, bool geometry, float dist, float rise, int skipId,
	Choice *choices, int max)
{
	const BotInventory &inv = bot.inventory;
	int count = 0;

	for (int i = 0; i < inv.count && count < max; ++i)
	{
		const BotWeapon &w = inv.weapons[i];
		if (skipId != 0 && w.id == skipId)
			continue;

		const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
		if (!spec)
			continue;

		const Mode modes[2] = { MODE_PRIMARY, MODE_SECONDARY };

		for (int j = 0; j < 2 && count < max; ++j)
		{
			const WeaponMode &m = (modes[j] == MODE_PRIMARY) ? spec->primary : spec->secondary;
			bool ok = geometry ? ModeUsable(bot, w, m, dist, rise, strict)
					: ModeViable(bot, w, m, strict);
			if (!ok)
				continue;

			choices[count].index = i;
			choices[count].mode = modes[j];
			++count;
		}
	}

	return count;
}

static bool IsAmbushWeapon(int id)
{
	if (id <= 0 || !g_pGameProfile)
		return false;

	for (int k = 0; k < AMBUSH_COUNT; ++k)
	{
		if (g_pGameProfile->AmbushWeapon(static_cast<AmbushKind>(k)) == id)
			return true;
	}

	return false;
}

enum Pass
{
	PASS_NO_MELEE = 0,
	PASS_ANY,
	PASS_MELEE,
	PASS_REACH,
	PASS_THROWN,
	PASS_LAST,
};

static bool IsMeleeId(int id)
{
	return id != 0 && g_pGameProfile && g_pGameProfile->IsMeleeWeapon(id);
}

static bool ChoiceEligible(const CBot &bot, const Choice &choice, Pass pass, float dist)
{
	int id = bot.inventory.weapons[choice.index].id;
	int last = g_pGameProfile ? g_pGameProfile->AmbushWeapon(AMBUSH_GRAPPLE) : 0;

	const WeaponSpec *spec = g_pGameProfile ? g_pGameProfile->GetWeaponSpec(id) : nullptr;
	const WeaponMode *m = spec ? ModeOf(*spec, choice.mode, bot.fire.zoomed) : nullptr;

	switch (pass)
	{
	case PASS_ANY:
		return true;
	case PASS_NO_MELEE:
		return !IsMeleeId(id);
	case PASS_MELEE:
		return IsMeleeId(id);
	case PASS_REACH:
		return IsMeleeId(id) && m && m->range > 0.f;
	case PASS_LAST:
		return id != 0 && id == last;
	case PASS_THROWN:
		break;
	}

	if (!IsAmbushWeapon(id) || id == last || !m)
		return false;

	return m->splash <= 0.f || dist >= m->splash;
}

static int PickBest(CBot &bot, const Choice *choices, int count, Pass pass, float dist)
{
	const BotInventory &inv = bot.inventory;

	float bestScore = 0.f;
	bool haveScore = false;

	for (int i = 0; i < count; ++i)
	{
		if (!ChoiceEligible(bot, choices[i], pass, dist))
			continue;

		float score = WeaponScore(inv.weapons[choices[i].index].id);
		if (!haveScore || score > bestScore)
		{
			bestScore = score;
			haveScore = true;
		}
	}

	if (!haveScore)
		return -1;

	int bestId = 0;
	float bestCarry = 0.f;
	int carryTies = 0;

	for (int i = 0; i < count; ++i)
	{
		if (!ChoiceEligible(bot, choices[i], pass, dist))
			continue;

		int id = inv.weapons[choices[i].index].id;
		if (WeaponScore(id) < bestScore)
			continue;

		bool seen = false;
		for (int j = 0; j < i && !seen; ++j)
			seen = ChoiceEligible(bot, choices[j], pass, dist) &&
				inv.weapons[choices[j].index].id == id;
		if (seen)
			continue;

		float carry = CarryOf(bot, id);

		if (bestId == 0 || carry < bestCarry - kCarryEpsilon)
		{
			bestId = id;
			bestCarry = carry;
			carryTies = 1;
			continue;
		}

		if (carry > bestCarry + kCarryEpsilon)
			continue;

		++carryTies;
		if (RandomUnit(bot.fire) * static_cast<float>(carryTies) < 1.f)
			bestId = id;
	}

	if (bestId == 0)
		return -1;

	int pick = -1;
	int modeTies = 0;

	for (int i = 0; i < count; ++i)
	{
		if (!ChoiceEligible(bot, choices[i], pass, dist))
			continue;
		if (inv.weapons[choices[i].index].id != bestId)
			continue;

		++modeTies;
		if (RandomUnit(bot.fire) * static_cast<float>(modeTies) < 1.f)
			pick = i;
	}

	return pick;
}

static int PickChoice(CBot &bot, const Choice *choices, int count, float dist)
{
	if (count <= 0)
		return -1;

	int pick = PickBest(bot, choices, count, PASS_NO_MELEE, dist);
	if (pick < 0)
		pick = PickBest(bot, choices, count, PASS_ANY, dist);

	return pick;
}

static int PickAmbush(CBot &bot, const Choice *choices, int count, float dist)
{
	if (count <= 0)
		return -1;

	int pick = PickBest(bot, choices, count, PASS_REACH, dist);
	if (pick < 0)
		pick = PickBest(bot, choices, count, PASS_THROWN, dist);
	if (pick < 0)
		pick = PickBest(bot, choices, count, PASS_LAST, dist);

	return pick;
}

static bool SelectWeapon(CBot &bot, const Choice &choice, const char *reason)
{
	State &fire = bot.fire;
	const BotWeapon &want = bot.inventory.weapons[choice.index];
	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(want.id);

	fire.nextSelectTime = gpGlobals->time + kSelectInterval;
	fire.pendingId = want.id;
	fire.pendingMode = choice.mode;

	Idle(fire, reason);

	if (!spec || !spec->classname || !g_pGameProfile->SelectWeapon(bot.pEdict, *spec))
		return false;

	++fire.switches;
	return true;
}

static bool TryKeep(CBot &bot, const BotWeapon &active, const Vector &basePoint,
	const Vector &baseVel, const Vector &eye, float dist, float rise, bool holding, bool strict,
	bool &triedBoth)
{
	State &fire = bot.fire;
	triedBoth = false;

	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(active.id);
	if (!spec)
		return false;

	Mode order[2] = { MODE_PRIMARY, MODE_SECONDARY };
	int tries = 2;

	if (holding)
	{
		order[0] = fire.mode;
		tries = 1;
	}
	else if (fire.pendingId == active.id && fire.pendingMode != MODE_NONE)
	{
		order[0] = fire.pendingMode;
		order[1] = (fire.pendingMode == MODE_PRIMARY) ? MODE_SECONDARY : MODE_PRIMARY;
	}
	else if (fire.weaponId == active.id && fire.mode == MODE_SECONDARY)
	{
		order[0] = MODE_SECONDARY;
		order[1] = MODE_PRIMARY;
	}

	triedBoth = (tries == 2);

	for (int i = 0; i < tries; ++i)
	{
		const WeaponMode *m = ModeOf(*spec, order[i], EffectiveZoom(bot, *spec));
		if (!m)
			continue;
		if (!holding && !ModeUsable(bot, active, *m, dist, rise, strict))
			continue;

		Vector aimBase = AimBase(bot, *m, basePoint, baseVel);
		Vector point = aimBase;
		Vector look = aimBase;
		float flight = 0.f;

		if (IsArc(*m))
		{
			Vector dir;
			Vector launch;

			if (IsThrow(*m))
			{
				ThrowShot shot;
				if (!SolveThrowLead(*m, eye, aimBase, baseVel,
					InheritedVelocity(bot, *m), ArcGravity(*m), shot, point))
					continue;

				dir = shot.dir;
				flight = shot.flight;
				launch = dir * shot.speed + bot.pEdict->v.velocity;

				MAKE_VECTORS(Vector(shot.pitch,
					atan2f(dir.y, dir.x) * (180.f / kPiF), 0.f));
				look = eye + gpGlobals->v_forward * (point - eye).Length();
			}
			else
			{
				Vector drift = InheritedVelocity(bot, *m);
				if (!SolveArcLead(eye, aimBase, baseVel - drift, ModeSpeed(bot, *m),
					ArcGravity(*m), dir, point, flight))
					continue;

				point = point + drift * flight;
				launch = dir * ModeSpeed(bot, *m) + drift;
				look = eye + dir * (point - eye).Length();
			}

			fire.targetPoint = point;

			bool hit = false;
			Vector impact = SimulateArc(bot.pEdict, eye + dir * kArcMuzzleFwd,
				launch, ArcGravity(*m), hit);
			if (!ArcImpactHits(bot, *m, eye, impact, hit))
				continue;
		}
		else
		{
			if (m->aim == AIM_LEAD && !IsHoming(*m) &&
				!SolveLead(eye, aimBase, baseVel, ModeSpeed(bot, *m), point, flight))
				continue;

			look = point;
		}

		bool wasReload = fire.wantReload;

		PublishAim(fire, *m, look, point, flight);

		fire.mode = order[i];
		fire.weaponId = active.id;
		fire.wantReload = !holding && m->ammo == AMMO_CLIP && active.clip >= 0 &&
			active.clip < ShotCost(*m) && !active.reloading &&
			AmmoCount(bot.inventory, active.primaryAmmoType) > 0;
		fire.lastReason = holding ? CommitName(fire.commit)
				: (fire.wantReload ? "reloading" : "ready");

		if (fire.wantReload && !wasReload)
			++fire.reloads;

		fire.pendingId = 0;
		fire.pendingMode = MODE_NONE;

		return true;
	}

	return false;
}

static bool HeldDry(CBot &bot, const BotWeapon &active, float dist, float rise)
{
	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(active.id);
	if (!spec)
		return false;

	const Mode modes[2] = { MODE_PRIMARY, MODE_SECONDARY };

	for (int i = 0; i < 2; ++i)
	{
		const WeaponMode *m = ModeOf(*spec, modes[i], bot.fire.zoomed);
		if (!m || !ModeUsable(bot, active, *m, dist, rise, false))
			continue;
		if (!ModeUsable(bot, active, *m, dist, rise, true))
			return true;
	}

	return false;
}

static void IdleSelect(CBot &bot)
{
	State &fire = bot.fire;
	const BotInventory &inv = bot.inventory;

	if (gpGlobals->time < fire.nextIdleTime)
		return;
	fire.nextIdleTime = gpGlobals->time + kIdleSelectInterval;

	if (g_pGameProfile->IsDead(bot.pEdict))
		return;

	const BotWeapon *active = (inv.activeIndex >= 0 && inv.activeIndex < inv.count)
		? &inv.weapons[inv.activeIndex] : nullptr;

	int reloadPick = -1;
	int bestDeficit = 0;

	for (int i = 0; i < inv.count; ++i)
	{
		const BotWeapon &w = inv.weapons[i];
		int deficit = ReloadDeficit(bot, w);
		if (deficit <= 0)
			continue;

		if (active && w.id == active->id)
		{
			reloadPick = i;
			break;
		}

		if (reloadPick >= 0 && deficit < bestDeficit)
			continue;
		if (reloadPick >= 0 && deficit == bestDeficit &&
			CarryOf(bot, w.id) >= CarryOf(bot, inv.weapons[reloadPick].id))
			continue;

		reloadPick = i;
		bestDeficit = deficit;
	}

	if (reloadPick >= 0)
	{
		const BotWeapon &want = inv.weapons[reloadPick];
		fire.idleReloadId = want.id;

		if (active && active->id == want.id)
		{
			Idle(fire, "idle-reload");
			return;
		}

		Choice choice;
		choice.index = reloadPick;
		choice.mode = MODE_PRIMARY;

		if (SelectWeapon(bot, choice, "idle-reload"))
			++fire.idleSwitches;
		return;
	}

	fire.idleReloadId = 0;

	Choice choices[kMaxChoices];
	int count = BuildChoices(bot, true, false, 0.f, 0.f, 0, choices, kMaxChoices);

	int pick = PickBest(bot, choices, count, PASS_NO_MELEE, 0.f);
	if (pick < 0)
		return;

	int wantId = inv.weapons[choices[pick].index].id;

	if (active && !IsMeleeId(active->id))
	{
		bool heldViable = false;

		for (int i = 0; i < count && !heldViable; ++i)
			heldViable = inv.weapons[choices[i].index].id == active->id;

		if (heldViable)
		{
			float heldScore = WeaponScore(active->id);
			float wantScore = WeaponScore(wantId);

			if (wantScore < heldScore)
				return;
			if (wantScore == heldScore &&
				CarryOf(bot, wantId) >= CarryOf(bot, active->id) - kCarryEpsilon)
				return;
		}
	}

	if (SelectWeapon(bot, choices[pick], "idle-switch"))
		++fire.idleSwitches;
}

static bool IdleCharge(CBot &bot, const BotWeapon *active)
{
	if (!active || !g_pGameProfile || !BotGauss::Scared(bot))
		return false;

	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(active->id);
	if (!spec || !IsChargeWeapon(*spec))
		return false;

	if (!ModeViable(bot, *active, spec->secondary, true))
		return false;

	State &fire = bot.fire;
	fire.mode = MODE_SECONDARY;
	fire.weaponId = active->id;
	fire.wantReload = false;
	ClearAim(fire);
	fire.lastReason = "idle-charge";
	return true;
}

void Think(CBot &bot)
{
	State &fire = bot.fire;

	fire.lastMiss = -1.f;
	fire.lastTol = -1.f;

	CountShots(bot);

	if (!Enabled() || !g_pGameProfile)
	{
		Idle(fire, "off");
		return;
	}

	if (BotBreak::Active(bot))
	{
		fire.idleReloadId = 0;
		Idle(fire, "breaking");
		return;
	}

	if (BotAbility::Active(bot.ability) && !BotAbility::FiresDuring(bot.ability))
	{
		fire.idleReloadId = 0;
		Idle(fire, "ability");
		return;
	}

	if (g_pGameProfile->HoldFire(bot))
	{
		fire.idleReloadId = 0;
		Idle(fire, "disguised");
		return;
	}

	if (BotDeploy::Active(bot))
	{
		fire.idleReloadId = 0;
		Idle(fire, "deploying");
		return;
	}

	if (BotFun::Mounted(bot))
	{
		fire.idleReloadId = 0;
		Idle(fire, "on a tank");
		return;
	}

	if (BotFun::Pressing(bot))
	{
		fire.idleReloadId = 0;
		Idle(fire, "pressing");
		return;
	}

	SeedFire(bot);

	PinEquip(bot);

	const BotInventory &inv = bot.inventory;
	const BotWeapon *active = (inv.activeIndex >= 0 && inv.activeIndex < inv.count)
		? &inv.weapons[inv.activeIndex] : nullptr;

	fire.zoomed = Zoomed(bot.pEdict);

	bool holding = fire.commit != COMMIT_NONE && active && active->id == fire.weaponId;
	if (!holding)
	{
		fire.commit = COMMIT_NONE;
	}
	else if (fire.commit == COMMIT_THROW || fire.commit == COMMIT_GUIDE)
	{
		bool live = (fire.commit == COMMIT_THROW)
			? (active->throwStart != 0.f) : (active->guidedCount > 0);
		float cap = (fire.commit == COMMIT_THROW) ? kTapTimeout : kGuideMax;

		if (live)
			fire.commitSeen = true;

		if (gpGlobals->time - fire.commitStart > cap)
		{
			++fire.commitCaps;
			fire.commit = COMMIT_NONE;
			holding = false;
		}
		else if (fire.commitSeen && !live)
		{
			fire.commit = COMMIT_NONE;
			holding = false;
		}
	}
	else if (fire.commit == COMMIT_SPINUP && gpGlobals->time - fire.commitStart > fire.commitFor)
	{
		fire.commit = COMMIT_NONE;
		holding = false;
	}

	edict_t *target = bot.sense.target.Get();
	if (target)
	{
		fire.targetSize = target->v.maxs - target->v.mins;
		fire.haveSize = true;
	}

	Vector basePoint;
	Vector baseVel(0.f, 0.f, 0.f);
	bool haveTarget = fire.haveSize &&
		BotAim::ReadPredicted(bot, BotAim::Delay(bot), basePoint, &baseVel);

	if (!haveTarget)
	{
		if (holding && fire.commit == COMMIT_CHARGE && ChargeExpired(fire, *active))
			holding = false;

		if (holding)
		{
			if (fire.commit != COMMIT_GUIDE && fire.commit != COMMIT_SPINUP)
				ClearAim(fire);

			fire.lastReason = (fire.commit == COMMIT_GUIDE) ? "guiding"
				: (fire.commit == COMMIT_SPINUP) ? "spinning" : "carrying";
			return;
		}

		if (StickyHold(bot))
		{
			fire.lastReason = "sticky";
			return;
		}

		if (BotDiff::GaussPolicy() && IdleCharge(bot, active))
			return;

		ClearAim(fire);
		Idle(fire, "no-target");

		if (gpGlobals->time - bot.sense.targetTime > kCombatMemory)
			IdleSelect(bot);
		return;
	}

	fire.idleReloadId = 0;

	Vector eye = bot.pEdict->v.origin + bot.pEdict->v.view_ofs;
	float dist = (basePoint - eye).Length();
	fire.lastRange = dist;
	float rise = basePoint.z - eye.z;

	bool triedBoth = false;
	bool dry = false;

	bool throttled = gpGlobals->time < fire.nextSelectTime;
	Choice choices[kMaxChoices];

	if (!throttled && !holding && BotCombat::Ambushing(bot) &&
		(!active || !IsAmbushWeapon(active->id)))
	{
		int count = BuildChoices(bot, true, true, dist, rise, 0, choices, kMaxChoices);
		int pick = PickAmbush(bot, choices, count, dist);

		if (pick >= 0 && SelectWeapon(bot, choices[pick], "ambush"))
		{
			++fire.ambushes;
			return;
		}
	}

	if (!throttled && !holding && BotCombat::Dueling(bot) &&
		(!active || !IsMeleeId(active->id)))
	{
		int count = BuildChoices(bot, true, false, dist, rise, 0, choices, kMaxChoices);
		int pick = PickBest(bot, choices, count, PASS_MELEE, dist);

		if (pick >= 0 && SelectWeapon(bot, choices[pick], "duel"))
		{
			++fire.duels;
			return;
		}
	}

	if (active && TryKeep(bot, *active, basePoint, baseVel, eye, dist, rise, holding, true,
		triedBoth))
		return;

	if (active && triedBoth)
		dry = HeldDry(bot, *active, dist, rise);

	if (!throttled)
	{
		int skipId = (active && triedBoth) ? active->id : 0;
		int count = BuildChoices(bot, true, true, dist, rise, skipId, choices, kMaxChoices);
		int pick = PickChoice(bot, choices, count, dist);

		if (pick >= 0)
		{
			if (SelectWeapon(bot, choices[pick], dry ? "empty-switch" : "select") && dry)
				++fire.emptySwitches;
			return;
		}
	}

	if (active && TryKeep(bot, *active, basePoint, baseVel, eye, dist, rise, holding, false,
		triedBoth))
		return;

	if (!throttled)
	{
		int skipId = (active && triedBoth) ? active->id : 0;
		int count = BuildChoices(bot, false, true, dist, rise, skipId, choices, kMaxChoices);
		int pick = PickChoice(bot, choices, count, dist);

		if (pick >= 0)
		{
			SelectWeapon(bot, choices[pick], "select");
			return;
		}
	}

	Idle(fire, throttled ? "switching" : "nothing-usable");
}

bool AimPoint(const CBot &bot, Vector &point)
{
	const State &fire = bot.fire;
	if (!fire.aimValid)
		return false;

	point = fire.aimPoint;
	return true;
}

static unsigned short IdleReloadButton(CBot &bot, const BotWeapon &w)
{
	State &fire = bot.fire;

	if (fire.idleReloadId == 0 || w.id != fire.idleReloadId)
		return 0;

	if (w.reloading || bot.inventory.nextAttack > 0.f || w.nextPrimaryAttack > 0.f)
		return 0;

	if (ReloadDeficit(bot, w) <= 0)
	{
		fire.idleReloadId = 0;
		return 0;
	}

	++fire.idleReloads;
	fire.lastReason = "idle-reload";
	return IN_RELOAD;
}

unsigned short Buttons(CBot &bot, const Vector &angles)
{
	State &fire = bot.fire;

	if (!Enabled() || !g_pGameProfile)
		return 0;

	const BotInventory &inv = bot.inventory;
	if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
		return 0;

	const BotWeapon &w = inv.weapons[inv.activeIndex];

	fire.zoomed = Zoomed(bot.pEdict);

	if (fire.zoomed)
	{
		const WeaponSpec *held = g_pGameProfile->GetWeaponSpec(w.id);
		if (held)
		{
			unsigned short unzoom = ZoomButtons(bot, *held, w,
				bot.pEdict->v.origin + bot.pEdict->v.view_ofs);
			if (unzoom != 0)
			{
				fire.lastReason = "unzooming";
				return unzoom;
			}
		}
	}

	if (fire.commit == COMMIT_NONE && !w.reloading && inv.nextAttack <= 0.f)
	{
		const WeaponSpec *held = g_pGameProfile->GetWeaponSpec(w.id);
		if (held && (held->secondary.quirks & WPN_Q_LASER_ON) != 0 && w.laserActive == 0 &&
			w.nextSecondaryAttack <= 0.f)
		{
			++fire.lasers;
			fire.lastReason = "laser-on";
			return IN_ATTACK2;
		}
	}

	if (!w.reloading && inv.nextAttack <= 0.f && w.nextSecondaryAttack <= 0.f)
	{
		unsigned short toggle = g_pGameProfile->FireToggle(bot, fire.mode != MODE_NONE && w.id == fire.weaponId);
		if (toggle != 0)
		{
			fire.lastReason = "toggle";
			return toggle;
		}
	}

	if (fire.mode == MODE_NONE)
		return IdleReloadButton(bot, w);

	if (w.id != fire.weaponId)
		return 0;

	if (fire.commit == COMMIT_GUIDE || fire.commit == COMMIT_SPINUP)
	{
		fire.lastReason = (fire.commit == COMMIT_GUIDE) ? "guiding" : "spinning";
		return 0;
	}

	if (fire.wantReload)
		return IN_RELOAD;

	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
	if (!spec)
		return 0;

	const WeaponMode *m = ModeOf(*spec, fire.mode, EffectiveZoom(bot, *spec));
	if (!m)
		return 0;

	bool charge = m->fire == FIRE_CHARGE;
	bool tap = m->fire == FIRE_TAP;

	if (!charge && (w.reloading || inv.nextAttack > 0.f))
		return 0;

	if (!charge && m->ammo == AMMO_CLIP && w.clip >= 0 && w.clip < ShotCost(*m))
	{
		fire.lastReason = "empty";
		return 0;
	}

	edict_t *pEdict = bot.pEdict;
	Vector eye = pEdict->v.origin + pEdict->v.view_ofs;

	if (!charge)
	{
		unsigned short toggle = ZoomButtons(bot, *spec, w, eye);
		if (toggle != 0)
		{
			fire.lastReason = "zooming";
			return toggle;
		}
	}

	if (!charge && BotDiff::GaussPolicy() && IsScopeWeapon(*spec) && !fire.zoomed)
	{
		fire.lastReason = "unzoomed";
		return 0;
	}

	bool onTarget = false;
	bool stickyable = false;

	if (fire.aimValid)
	{
		if (IsArc(*m))
		{
			onTarget = ArcOnTarget(bot, angles, *m, eye);
		}
		else
		{
			RayVeto veto = RayOnTarget(bot, angles, *m, eye);
			onTarget = veto == RAY_HIT;
			stickyable = veto == RAY_WIDE;
			if (!charge && SpreadHold(bot, w, eye, onTarget))
			{
				onTarget = false;
				stickyable = false;
				fire.lastReason = "spread";
			}
		}
	}

	fire.pressedWall = onTarget && fire.rayWall;

	if (charge)
	{
		return ChargeButtons(fire, *m, onTarget);
	}

	if (tap)
	{
		return TapButtons(fire, w, *m, onTarget);
	}

	if (!onTarget)
	{
		if (!stickyable || !StickyAllowed(bot, *m))
			return 0;

		fire.lastReason = "sticky";
		fire.pressed = true;
		fire.pressedSticky = true;
		return AttackButton(fire.mode);
	}

	if (IsGuided(*m) && w.nextPrimaryAttack <= 0.f)
	{
		StartCommit(fire, COMMIT_GUIDE);
		++fire.guides;
	}

	fire.lastReason = fire.pressedWall ? "wallbang" : "firing";
	fire.pressed = true;
	fire.pressedLead = fire.aimLead;
	fire.pressedArc = fire.aimArc;
	return AttackButton(fire.mode);
}

bool BraceEnabled()
{
	cvar_t *cvar = g_engfuncs.pfnCVarGetPointer("crossbot_brace");
	return !cvar || cvar->value != 0.f;
}

bool Brace(CBot &bot, unsigned short buttons, bool &duck, bool &stanceOnly)
{
	State &fire = bot.fire;
	bool was = fire.bracing != 0;
	fire.bracing = 0;
	duck = false;
	stanceOnly = false;

	if (!BraceEnabled() || !Enabled() || !g_pGameProfile)
		return false;
	if (fire.mode == MODE_NONE || !fire.aimValid || (buttons & IN_JUMP) != 0)
		return false;

	const BotInventory &inv = bot.inventory;
	if (inv.activeIndex < 0 || inv.activeIndex >= inv.count)
		return false;

	const BotWeapon &w = inv.weapons[inv.activeIndex];
	if (w.id != fire.weaponId || w.reloading)
		return false;

	const WeaponSpec *spec = g_pGameProfile->GetWeaponSpec(w.id);
	const WeaponMode *m = spec ? ModeOf(*spec, fire.mode, fire.zoomed) : nullptr;
	if (!m || (m->quirks & WPN_Q_BRACE) == 0)
		return false;

	edict_t *pEdict = bot.pEdict;
	if ((pEdict->v.flags & FL_ONGROUND) == 0 || pEdict->v.movetype == MOVETYPE_FLY ||
		pEdict->v.waterlevel >= 2)
		return false;

	float timer = (fire.mode == MODE_SECONDARY) ? w.nextSecondaryAttack : w.nextPrimaryAttack;
	float wait = std::max(timer, inv.nextAttack);

	if (BotDiff::BraceMoving() && strcmp(g_pGameProfile->GetName(), "Gearbox") == 0)
	{
		if (wait > 0.f || (buttons & AttackButton(fire.mode)) == 0)
			return false;
		stanceOnly = true;
		duck = true;
		fire.bracing = 3;
		if (!was)
			++fire.braces;
		return true;
	}

	PmSim::PmPhysics physics = Bot_DrivePhysics(pEdict);
	Vector flat(pEdict->v.velocity.x, pEdict->v.velocity.y, 0.f);
	float stop = NavPlan::StopTime(flat.Length(), physics.groundFriction * physics.friction,
		physics.stopSpeed);
	if (wait > stop)
		return false;

	if (gpGlobals->time >= fire.braceTraceTime)
	{
		Vector eye = pEdict->v.origin + pEdict->v.view_ofs;
		if ((pEdict->v.flags & FL_DUCKING) == 0)
			eye.z -= kDuckEyeDrop;

		TraceResult tr;
		TRACE_LINE(eye, fire.targetPoint, ignore_monsters, pEdict, &tr);
		fire.braceDuckClear = tr.flFraction >= 1.f;
		fire.braceTraceTime = gpGlobals->time + kBraceTraceInterval;
	}

	duck = fire.braceDuckClear;
	fire.bracing = duck ? 1 : 2;
	if (!was)
		++fire.braces;
	return true;
}

}
