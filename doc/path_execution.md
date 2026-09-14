# Path Execution

How the bot follows a planned route: the profile tracker, the velocity servo, drift
compensation, and the physics it integrates against. The mesh and the plan it executes are `doc/path_planning.md`.

## path_follower.cpp

### NavFollow::Track
Follows a `PlanProfile`. All the old heuristics (chase point, corner cut, corridor pull, `brakeRadius`
blend, arrival radii) are gone — the geometry is precomputed, so the tracker only has to stay on it.

**Projection** - monotone: each frame searches only `[s - projectBack, s + speed*dt*4 + projectAhead]`,
so the bot cannot snap onto a distant part of the route that happens to pass nearby.

**Aim direction** - the tangent one frame ahead, plus a cross-track correction toward the chain scaled
by `min(1, offset/rejoinDist)`. Deliberately *not* pure pursuit: aiming at a point some lookahead
ahead makes the bot start turning as soon as that point rounds a corner, cutting the corner by up to
the lookahead distance. Since the arc already encodes the turn, no anticipation is wanted.

**Speed** - the profile speed at the current arclength, derated by `1 - (lateral - C)/C` (floored at
`derateFloor`) when outside the clearance budget, so the bot slows to rejoin instead of arcing wider.

**Velocity servo** - `SolveDrive`, below.

**Abort** - `needRepath` once lateral exceeds `rejoinAbort` (96). Previously nothing detected being
off-path; the corridor pull fought forever until the stall timer fired.

**`done` means arrived, and only that.** The cross-track correction can cancel the tangent exactly — a
bot barred at a node by `sLimit` and standing beyond it along the tangent sees the chain point directly
behind, and the two unit vectors sum to zero. That used to fall through to `d.done`, which
`Bot_MoveNav` announces as reaching the goal and acts on by dropping it, wherever the bot happened to
be. `test18_elevator` hit it while barred at a lift, printing `reached goal` 312 units away and mid
shaft. A degenerate blend now keeps the raw tangent, and the surviving zero-direction case — which
needs an empty tangent, not a cancelled one — asks for a repath instead of claiming success.

The rejoin pull can cancel the tangent exactly -- a bot barred at a node and standing beyond it
along the tangent sees the chain point directly behind. Falling through to `done` there reports
an arrival wherever the bot happens to be, so keep the raw tangent and let the lateral test
decide whether this is worth a repath.

### NavFollow::SolveDrive
Inverts one engine frame to hit a target velocity `V*`. This is the only correct way to ask the engine
for a speed, and everything that drives the bot goes through it — `Track` for ordinary following,
`RunAirmoveDrive` for the air-move approach.

```
v_af      = velocity after this frame's friction   (guard |v| < 0.1, as PM_Friction does)
need      = V* - v_af
wishdir   = normalize(need)
wishspeed = max(|need| / (sv_accelerate * dt * friction), along + |need|)
```

The second term matters: `PM_Accelerate` caps its delta at `wishspeed - v.wishdir`, so the rate-based
form alone commands zero acceleration when already cruising at the target. One formula covers
acceleration, braking, counter-strafing and arc tracking, saturating when a frame cannot deliver.

The *first* term is what makes low speeds reachable at all, and it is the reason commanding `V*`
directly as a wish speed is a bug rather than an approximation. Below `sv_stopspeed` the engine's
friction drop is the constant `stopspeed * f * dt` no matter how slowly the bot is moving, while
`PM_Accelerate` delivers `sv_accelerate * wishspeed * dt`. A raw wish speed therefore does nothing
whatsoever until

```
wishspeed > stopSpeed * groundFriction * edgeFriction / accelerate
```

which at defaults is 40 on open ground and **80 within a hull's reach of a ledge**, where the engine
doubles friction by `sv_edgefriction`. `byRate` sidesteps the *stall*: the further the velocity is from
`V*`, the larger the commanded wish speed, so the bot saturates the frame and closes on `V*` from any
starting speed. See `doc/bot.md`, `RunAirmoveApproach`, for the dropoff this stalled.

What it does *not* do by itself is arrive at `V*`, and that is the next rule.

**`byRate` does not cover for an unmodelled friction — it converts it into a standing speed error.**
The formula is a deadbeat solve: it asks for the one wish speed whose `sv_accelerate * dt * wishspeed`
exactly replaces the drop, so its gain is set by whatever `f` the caller passed. Feed it a friction
lower than the engine's and it settles short, permanently. At the fixed point the engine takes
`stopSpeed * f_real * dt` per frame and returns at most `accelerate * dt * W`, so holding still needs
`W = stopSpeed * f_real / accelerate`; `byRate` produces that W at

```
v = V* - stopSpeed * (f_real - f_model) * dt
```

which for an unmodelled ledge doubling is a flat `V* - 400 * dt` at defaults — 4 units short at 100 fps,
6.7 at 60, whatever `V*` is. Measured on test53_ladder4's crouch jump: 66.8 / 66.4 / 66.2 against a
design 70.8, on frames of 10.0 / 11.0 / 11.6 ms.

The error exists only *below* the `stopSpeed * groundFriction * edgeFriction / accelerate` threshold
above — 80 at defaults. Above it `byCap` wins instead, `W = V*`, and a target that high is
self-sustaining against edge friction anyway, so the model error is invisible. That is why this hid for
so long: it needs a design speed under 80, and the one place that reliably produces one is a **crouched**
approach, capped at `maxspeed * kDuckingMultiplier` ≈ 90 and usually decelerating into a launch below
that.

`physics.edgeFriction` is therefore consumed by the follower, but not by `SolveDrive` itself, which stays
a pure function of the `PmPhysics` it is handed. `Bot_MoveNav` runs the probe and hands it physics whose
`groundFriction` is already doubled — see `BotDrivePhysics` in `doc/bot.md`. `BuildPathProfile` is
deliberately left alone: it is a whole-route model built once at plan time, and folding a momentary
ledge state into it would make design speeds depend on where the bot stood when it replanned.

## Following a drifting path

`NavFollow::Config::driftVelocity` is the world-frame shove the bot is currently subject to.
`Bot_DriftVelocity` reads it live from the engine — `groundentity->v.movedir * speed` when the ground entity
has `FL_CONVEYOR`, plus the cached `Bot_PushFields` closure at the bot's origin — so a `Use`-reversed
conveyor or a switched-on field is handled without rebaking. The closure is cached on `CBot::pushFields` and
refreshed with each repath, because scanning every edict per frame per bot is not free.

`Track` does two things with it. The world-speed cap becomes `DriftSpeedAlong(drift, aimDir, maxSpeed)`
instead of `physics.maxSpeed`, so a bot riding a belt is allowed to exceed its own run speed. And the wanted
velocity handed to `SolveDrive` is `aimDir * target - drift`: `pev->velocity` is own-only, `SolveDrive`
solves in the own frame, and its clamp to `physics.maxSpeed` stays correct untouched. Without that
subtraction a bot crossing undertow's 420 u/s current is simply carried away from its own plan.

`BuildPathProfile`'s `NodeMaxSpeed` applies the same `DriftSpeedAlong` using the waypoint's outgoing segment
direction (the last waypoint uses its incoming one), which is where `PlanWaypoint::drift` — copied from
`PlanArea::drift` in `PlanPath` — is used. Corner radii, the forward/backward speed pass and `totalTime` all
follow from it, so `estimatedArrivalTime` stops being wrong by up to 2x on a belt.

## Physics simulation with gravity

In `pm_sim.cpp`, `PM_AddCorrectGravity` folds the vertical push component into velocity as an acceleration and clears it from the base velocity, so only the horizontal remainder wraps `FlyMove` below.

The horizontal base velocity wraps the move the way `PM_AirMove` and `PM_WalkMove` do: it is added before the clip-and-slide loop, then pulled back out once the ground check has seen it. This ensures consistent handling across all movement types.
