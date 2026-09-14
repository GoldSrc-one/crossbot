# Perception: sensing and aiming

How the bot sees and where it looks: hearing, vision, suspicion, the aim hand and its delay queue.
What it does about what it sees is `doc/combat.md`; what it wants otherwise is `doc/goals.md`.

## bot_aim.cpp

Everything the bot looks at goes through one function. `bot.desiredAngles` is the request, written by six
sites across `bot_move_nav.cpp` plus `BotSense::ApplyLook`; `Bot_UpdateFacing` is the only writer of `pev->v_angle` anywhere in the
plugin, and every drive branch calls it exactly once per frame and threads its return value into both
`Bot_DecomposeMove` and `pfnRunPlayerMove`. Aim is therefore fully decoupled from movement: the view can
point anywhere without disturbing where the bot walks, because the wish direction is world-space and gets
reprojected onto whatever yaw was actually commanded that frame.

### Why the flat turn rate had to go

The original controller stepped toward `desiredAngles` at a flat 300 deg/s, pitch and yaw clamped
independently. It turned at full speed from the first frame and stopped dead on the last one. Nothing
holding a mouse does that, and it was the single most obvious tell that a bot was not a player.

### Hand space, not angle space

The integrator state (`BotAim::State::yawVel`, `pitchVel`) is **not** angular velocity. It is hand velocity,
in degrees per second at a reference 90 degree FOV. Angles are converted in and out of it with
`s = fov / 90` (`pev->fov <= 0` means default, which is the normal case -- nothing else in the plugin reads
or writes `fov` today).

This is what makes zoom correct for free. A scope shrinking FOV from 90 to 20 means the same mouse movement
now covers 4.5x less angle, so the bot needs 4.5x more hand travel to make the same turn and gets there 4.5x
slower. Doing it in hand space means that when the scope snaps, hand velocity is untouched -- the hand keeps
its physical speed -- and the angular velocity drops on its own. Had the state been angular, the FOV change
would have been a velocity discontinuity needing its own special case.

### The control law

Per axis, time-optimal with a saturating acceleration:

    step     = aMax * dt
    envelope = sqrt(2 * aMax * |e| + step^2 / 4) - step / 2
    v_target = sign(e) * min( |e| / dt, envelope, kMaxHandSpeed )
    a        = clamp( (v_target - v) / dt, +/-aMax )
    v += a * dt ;  angle += v * s * dt

The envelope term is the whole model. It is the velocity of a body decelerating at `aMax` that arrives at
the target with zero velocity, so following it gives the accelerate-then-decelerate profile without any
explicit phase switching: far from target the required acceleration exceeds `aMax` and the clamp saturates,
near it the envelope takes over and brakes.

### Why the envelope carries a step correction

The obvious form is `sqrt(2 * aMax * |e|)`, the continuous-time optimum. Sampled at a finite frame rate it
overshoots badly, because it assumes braking begins the instant the envelope is touched when in fact it
begins up to a full frame later. Measured at 2880 deg/s^2, the plain form overshoots a 180 by 3.6 degrees at
100fps, 5.9 at 60fps and **12.4 at 30fps** -- and the framerate dependence is the real problem, because it
makes aim accuracy a function of server load. At the shipped 4500 it is worse still, since the error scales
with `aMax * dt`.

The `+ step^2 / 4` under the root and the `- step / 2` outside it invert the *discrete* stopping distance
rather than the continuous one. That drops overshoot below 0.7 degrees at every frame rate tested, and it
converges *faster* despite braking earlier (0.617s to 0.467s for a 180 at 60fps, at 2880), because none of the budget
is spent recovering from an overshoot. This matters beyond tidiness: deliberate over/undershoot is a planned
feature, and it has to be the only source of overshoot or it cannot be tuned.

The `|e| / dt` term is a dead-beat clamp. Capping the target velocity at "the speed that lands exactly this
frame" makes any error small enough to close in one step close in one step.

It is worth being precise about what it buys, because the obvious way to make the bot less surgical is to
delete it and that turns out not to work. The "bang-bang limit-cycles at roughly `aMax * dt^2`" claim
applies to the **plain** `sqrt(2 a |e|)` envelope. With the discrete correction above, the envelope
linearises near zero to `2e/dt` -- exactly twice the dead-beat speed -- so a step lands on `-e` and no
further. Squaring out `sqrt(2ae + s^2/4) - s/2 <= 2e/dt` with `s = a*dt` reduces to `0 <= 4e^2/dt^2`, which
holds for every `e > 0`, with equality only in the limit. So there is no limit cycle at all: dropping the
clamp gives a ring-down that decays quickly at first and stalls around `aMax * dt^2 / 8` -- 0.056 degrees at
100fps, 0.156 at 60, 1.41 at the `kMaxFrameTime` clamp. That is an order of magnitude finer than the hand
quantum at normal tick rates and three times coarser at 10 Hz: wrong in both directions, and frame-rate
coupled in a controller whose entire envelope design exists to not be. Coarseness comes from the quantum
below instead, and the clamp stays.

`kMaxHandSpeed` is a hand speed ceiling. It is near-inert at the current constants, since the largest
reachable error is 180 degrees and that only peaks at 720 deg/s, but it is the knob for a bot whose hand
should be slower than its acceleration allows.

### The hand has a resolution, and that is what zoom buys

A real hand on a real mouse cannot move an arbitrarily small amount, and cannot change speed by an
arbitrarily small amount. Two quantisations model that, both in hand space, both applied at the bottom of
`StepAxis`:

    velOut   = round(vel / impulse) * impulse       // commanded speed lives on a grid
    pending += velOut * dt
    counts   = trunc(pending / quantum)             // whole mouse counts only
    pending -= counts * quantum
    angle   += counts * quantum * s

`crossbot_aim_quantum` is degrees of hand travel per mouse count and `crossbot_aim_impulse` is the smallest
non-zero hand speed, in hand deg/s. Both cvars hold the value itself rather than a multiplier, and either at
0 switches its own quantisation off; both at 0 is the exact pre-quantum controller, which is what the
regression run in `test_aim_error.ps1` uses.

The default quantum is `atan(1/128)` = 0.4476 degrees: one unit of miss per 128 units of range. That is
deliberately chosen for its consequence at distance. Because both quantisations are in hand space, `s`
scales them the same way it scales `kMaxAccelYaw`:

| fov | angular quantum | 1 unit per | miss at 4096 units |
|---|---|---|---|
| 90 | 0.448 deg | 128u | +/- 32u, and a player is 32 wide |
| 20 (crossbow) | 0.0995 deg | 576u | +/- 7u |

**This is the reason zoom is worth anything to a bot.** Before it existed, `FovScale` priced zoom as pure
cost -- see "Zoom is a range decision" below -- and a bot that zoomed simply turned 4.5x slower for a mode
swap. Now the same 4.5x buys 4.5x finer placement. Measured on a 40 degree flick, ten samples each: mean
resting error 0.299 degrees at fov 90 against 0.052 at fov 20, for settle times of 0.180s and 0.393s. Peak
hand speed came out 414 and 891 deg/s -- the fov 20 figure matching the 890 of a 180 at fov 90, because a 40
degree turn through a 20 degree scope is the same 180 degrees of hand travel.

**Why the speed grid is applied to the output and not to the change in speed.** Quantising `dv` and keeping
`vel` itself on the grid is the tidier-looking form and it deadlocks. `dv` is clamped to `maxAccel * dt`
first, so as soon as `maxAccel * dt` drops below half an impulse every `dv` rounds to zero and the velocity
can never change again -- the view freezes solid. That threshold is `dt < impulse / (2 * maxAccel)`, i.e.
600fps on yaw and 480 on pitch at the defaults, and `sys_ticrate` goes to 1000. Leaving `vel` continuous and
snapping only the value that gets integrated has no such threshold.

**Why the commanded speed is floored and not merely snapped.** `round(vel / impulse)` returns zero whenever
`|vel| < impulse / 2`, and the dead-beat term turns a small error into a small velocity -- so without a
floor any error under `impulse * dt / 2` is invisible and the hand parks early. Measured, that put the
resting error at 0.065 degrees at 100fps: three times finer than the quantum intends and frame-rate coupled,
which is both of the things this design is trying to avoid. So the control law floors `limit` at `impulse`,
capped at `quantum / dt` so the floor can never demand more than one count per frame at a low tick rate.
The hand therefore never comes fully to rest: it dithers across the count it is nearest, roughly 20 Hz at
100fps, which is about where physiological hand tremor sits. `crossbot_aim_status` shows this as a hand
speed that idles at 21 deg/s rather than 0 -- `sqrt(15^2 + 15^2)`, both axes at the floor.

The tremor is not cosmetic. It is what makes long range *probabilistic*: `RayOnTarget` gates the trigger on
the ray actually entering the bounding box, so a wandering ray produces intermittent potshots at a distant
target rather than a clean yes or no that holds for the whole engagement.

**`kMaxQuantum` and `kMaxImpulse` are cvar clamp bounds, not physics.** They belong with `kMaxDrift`,
`kMaxDelay` and `kMaxErrorScale`, not with `kMaxAccelYaw` in the same block: acceleration keeps its ceiling
and the impulse adds a floor, opposite ends of one axis and both needed. `kMaxQuantum` is 5 degrees because
that is under `kLadderAimTolerance`, so no cvar value can leave a nav gate unlatchable; `kMaxImpulse` is 100
deg/s because past that the slowest possible tracking speed outruns most engagements.

Resting error is bounded by whichever quantisation binds, `max(quantum, impulse * dt) / 2` in hand space:
0.22 degrees at 100fps, 0.375 at the 20 Hz clamp. Both sit under `kTestSettleTolerance`, and the tremor
speed sits far under `kTestSettleSpeed`, so `crossbot_aim_test` still reports a settle. The nav gates have
more than 10x margin, and `crossbot_aim_drift` defaults to a degree of sine, comfortably above the quantum,
so idle drift still moves the view -- in steps now.

### Where the constants come from

`kMaxAccelYaw` is derived, not tuned. For a symmetric accelerate-then-decelerate turn covering angle `t` in
time `T`, `t = a * T^2 / 4`. Requiring a 180 in 0.4s gives `a = 4 * 180 / 0.16 = 4500 deg/s^2`, peaking at
900 deg/s halfway through. `kMaxAccelPitch` is 80% of that: vertical mouse travel is the awkward direction
for a human wrist. For reference, the CS zbot uses a hand-tuned 3000 deg/s^2 and peaks around 750 deg/s on
the same turn, so this bot's hand is somewhat quicker than a zbot's.

The first cut used 0.5s (2880 deg/s^2). It was raised to 0.4s because 0.5s measurably slowed
`test10_doorbutton` -- see below. `kMaxHandSpeed` is deliberately above the 900 deg/s peak so it stays a
ceiling rather than a clip; at exactly 900 it would bind at the top of every 180.

### The cost at the aim gates

Below about 80 degrees of error (the crossover, `4 * 300^2 / aMax`) the controller is *slower* than the old
flat 300 deg/s rate, because it spends the start of the turn accelerating. A 6 degree correction goes from
20ms to 73ms. The places that care are the two that **zero drive until the view has converged**: the ladder
gate (`kLadderAimTolerance`, 6 degrees) and the activation gate (`kActivationAimTolerance`, 8 degrees).

`test10_doorbutton` is the map that feels it. Isolated repeat runs, goal-arrival time:

| controller | 8.1s | 11.1s |
|---|---|---|
| old flat 300 deg/s | 8 of 8 | 0 |
| aMax 2880 (180 in 0.5s) | 0 | 2 of 2 |
| aMax 4500 (180 in 0.4s) | ~7 of 16 | ~9 of 16 |

**Read that table with the quantization in mind.** `test_nav_goal.ps1` polls for arrival on a grid -- the
observed times are 2.0, 5.0, 8.1, 11.1 -- so consecutive buckets are ~3s apart and a run that misses the
8.1s poll by a tenth of a second reports 11.1s. The real slowdown is well under a second; the table measures
which side of a threshold the bot lands on, not how much time it lost. Raising the acceleration moved it
back across that threshold about half the time.

Two things that look like they explain this and do **not**: every run on this map, on *every* controller
including the original, logs exactly one `mid-route repath (stalled)` at the route reversal in front of the
button and exactly one `avoid side func_door_rotating`. Neither is caused by the aim model, and neither
discriminates the fast runs from the slow ones.

For calibration when reading sweep numbers: `test45_slopes2` looked like a 5.2s-to-8.1s regression in the
parallel sweep and was not one -- the old controller measures 8.1s in isolation too, and the 5.2s came from
a run at `-MaxParallel 6`. The general rule for this suite: a single parallel-sweep number proves nothing,
and neither does a two-run isolated sample when the metric is bucketed. Repeat until the distribution is
visible.

### dt is clamped

The old first-order ramp could survive a raw `gpGlobals->frametime`: the worst a hitch did was let it snap
further than intended, bounded by the remaining error. A second-order integrator has memory, so a hitch
feeds a huge `dt` into both the acceleration and the position step and flings the view. `dt` is clamped to
[1/1000, 1/20], the same defensive range `Bot_Physics` uses.

### Pitch clamp and velocity reset

Pitch is clamped to +/-89 to avoid gimbal lock, and `pitchVel` is zeroed when it hits the stop -- otherwise
velocity winds up against the limit and the view stays pinned for several frames after the target comes
back into range. `TeleportAllBots` zeroes both velocities alongside the `v_angle` reset for the same reason.
Everything else is covered by `bot = CBot()`, since `BotAim::State` is default-initialized.

### crossbot_aim_test

`crossbot_aim_test <bot> <yawDelta> [pitchDelta]` commands a flick relative to the bot's current view and
prints the time to settle within 0.5 degrees plus the peak hand speed. It exists because there is otherwise
no way to check the 180-in-0.5s claim against the running server.

The test angles are an **override**, not a write to `desiredAngles`: navigation rewrites `desiredAngles`
every frame, so a plain write would be stomped before the next frame. The override releases itself after 3
seconds and the bot goes back to whatever it was looking at.

Expected, from an offline simulation of the same step function at the shipped constants: `180 0` settles to
within 0.5 degrees in 0.37-0.38s at a peak of 900 deg/s hand speed, and `0 90` in about 0.30s at a peak near
540-560. The spread is the frame rate -- lower fps settles slower. Peak hand speed is the more stable number
and the better thing to check: it is set by the constants, not the sampling, and 900 for a 180 is the
derivation above closing on itself.

### The reaction delay queue

Everything above is the motor half: how fast the hand can move once it decides to. The queue is the
perception half: the bot acts on what it saw `crossbot_aim_delay` seconds ago, never on the present frame.
Default 0.25s, clamped to [0, 0.5]. Human simple visual reaction is roughly 200-250ms; 0.5s is
choice-reaction territory and reads as sluggish, so the clamp is a usable range rather than a safety rail.

`BotAim::State` carries a 256-slot ring of `{ time, point }`. 256 rather than 128 because 128 slots at
250fps only spans 0.512s, right on the boundary of the maximum delay.

**Slots hold a world point, not angles**, and the write extrapolates only the *target*:

    push { now, targetPoint + targetVelocity * delay }

The push is conditional: that predicted point is traced first, and a lead that has already crossed into
geometry is dropped rather than queued. See `### The lead point is traced before it is queued` under
`## bot_sense.cpp`.

The read takes the newest slot older than `now - delay`, linearly interpolates it with the next newer one
so the served delay is exact regardless of frame rate rather than quantized to a frame, and only then
converts to angles -- from the bot's **actual current eye**, in `ApplyLook`.

The extrapolation is the entire point. At time `T` the bot records where the target will be at `T + delay`.
At `T + delay` it reads that back. If the target moved at constant velocity over the window the prediction
was exact, and the bot is aimed exactly at it despite acting on quarter-second-old information. Only
*acceleration* -- a target changing direction -- produces error. That is the human signature being modelled:
a good player tracks steady movement perfectly and gets beaten by the change of direction, not by the speed.

The bot's own motion is deliberately **not** extrapolated, and neither is its aiming velocity. Where the bot
is standing is known exactly at read time, so predicting it would replace a fact with a guess; and turning
the view does not move the eye, so it never enters the geometry at all.

### The two queue edges are the behaviour, not error handling

**Underflow.** When a target has just been acquired the ring holds nothing older than `delay`, so the read
fails and the queue writes nothing -- the bot keeps whatever `desiredAngles` the navigation branch set. It
therefore sits perfectly still for a full reaction time and *then* flicks. The obvious alternative, falling
back to the oldest slot, would snap the head to truth the instant a target appears, which is exactly zero
reaction time. The silent quarter second before anything moves is the most human artifact the whole system
produces, and it costs nothing.

**Drain.** On losing line of sight the bot stops *writing* but keeps *reading*. It therefore continues
tracking a target that has already left view, for exactly `delay`, and then stops -- because its input is
`delay` stale and it does not know yet. Also free: it falls out of the ring rather than being coded.

The ring is cleared on acquiring a target, on teleport, and on respawn (the last via `bot = CBot()`).
Clearing on acquire is what makes the reaction time apply to every reacquisition, not just the first.

**The drain did not actually work until firing needed it.** `UpdateVision` compared the new target
against the old and cleared on any difference, and *losing* a target is a difference — so the ring was
wiped on the same frame line of sight broke and there was no drain at all, only the suspicion fallback
in `ApplyLook`. The test is now `best && best != sense.target.Get()`: clear when a different live
target is acquired, never when one is lost. Reacquiring the same enemy after a gap still clears,
because the handle was set to null in between, so the reaction delay is still paid every time.
Nothing above this paragraph changed — the behaviour it described is simply true now.

### What feeds the queue

`BotSense::Update` does, once per frame, from the slot in `Bot_MoveNav` that `UpdateGaze` used to hold.
See `## bot_sense.cpp` below for the whole producer. The contract the queue cares about is one
`PushDelaySlot` per frame while a target is held *and its lead point comes back clear*, the target
extrapolated forward by `crossbot_aim_delay`, and `ClearDelayQueue` on acquiring a *different* target.

The one thing that did change is what point gets pushed. It is no longer the target's eye but a randomly
chosen hitbox bone, re-picked every frame. The queue therefore stores points that jitter over the enemy's
body, and the acceleration-limited controller in `Bot_UpdateFacing` is what averages them back down -- it
is already a low-pass filter, so no explicit smoothing was added. The visible result is a head that settles
near body centre while the underlying samples wander, which is both what a human does and what makes a
partially-hidden enemy still trackable.

`ClearDelayQueue` keys on the target *entity*, not the bone. Keying it on the sample point would clear the
ring every frame and destroy the reaction delay entirely.

### Slots also hold velocity

Beside the point, and interpolated the same way. The reader is `BotFire`, which needs the target's
velocity to lead a projectile, and needs it as **perceived** rather than as it is right now — a lead
computed from live velocity would give a bot with a quarter-second reaction time perfect prediction.
`ReadDelayed` takes it as an optional out pointer, so `ApplyLook`, which only wants the point, is
unchanged. See `## bot_fire.cpp` in `doc/combat.md` for what the composition means.

### Why slots hold points and not angles

Slots held `{ time, yaw, pitch }` first, which forced the write to guess where the *bot itself* would be one
reaction time later and bake that guess into the stored angles. The guess only holds while the bot moves at
constant velocity. A bot that jumps, gets stopped by `BotAvoid`, rides an elevator or repaths inside the
delay window breaks it, and the recalled angles are then right about *where the target is* and wrong about
*where the bot is looking from*. That is parallax error, worst at close range where a small positional error
subtends a large angle -- which is exactly the range where aim decides fights. It is also unattributable: it
lands in `err` mixed together with genuine target misprediction, so neither number can be read cleanly.

Storing the point deletes that term rather than modelling it. The bot's eye enters once, at read time, as
its true present value, so there is nothing left to mispredict. Where the prediction would have been right
the two forms are identical; where it would have been wrong, only the angle form was wrong. The target-side
extrapolation -- the half that actually models human lead -- is untouched.

The cost is a `LosAngles` call moved from the write to the read, which is the same call either way, and a
three-float lerp in place of a two-float one that also had to take yaw the short way round. The one real
consequence is that `LosAngles` can now fail at *read* time, when the recalled point sits on the bot's own
eye. `ApplyLook` falls through to the suspicion blend there and leaves `servedDelay` at zero rather than
claiming a serve it did not make.

### crossbot_aim_status

Prints, per bot: the sense target, which bone it is currently sampling, ring occupancy, how much history
the ring holds, the delay actually served, hand speed, and `err` -- the angle between the bot's real
`v_angle` and the true present-time angles to its target.

`err` is the number that decides whether any of this works, and it is why the command exists at all.
Spectating cannot distinguish "correctly delayed" from "delayed and also wrong": both look like a head
lagging behind a target. `err` separates them. At `crossbot_aim_delay 0` it is the controller's own tracking
error with no queue involved, which is the baseline. At 0.25 it should still be flat whenever the
target is moving at constant velocity, and spike only on direction reversals. A large `err` under steady
target motion means the extrapolation or the read index is wrong, not that the delay is working.

**`err` no longer bottoms out at zero.** `AnglesToTarget` measures against the target's eye, while the bot
is aiming at a randomly chosen hitbox bone. The difference between those two points is a standing floor,
and it is a floor that *grows as the range closes* -- the same bone spread subtends a wider angle up close,
and every bone sits below the eye, so the residual is almost pure negative pitch. At the point-blank range
`test_sense.ps1` converges the bots to, it is tens of degrees. That floor is the bone sampling working, not
a defect, and it is why `err` is no longer a number to read absolutely. Read the shape across the delay
sweep instead: the thing that would still be a real extrapolation bug is `err` *trending* with
`crossbot_aim_delay` at a fixed range.

`scripts/test_sense.ps1` drives exactly this: two bots on `crossfire`, sense on, sweeping the delay while
polling the command. The nav sweep cannot substitute for it -- `test_nav_goal.ps1` spawns a single bot, and
with line-of-sight gating a lone bot never has a target, so the sweep can only ever prove the feature is
inert when switched off.

Measured on `crossfire`, both bots converged on one spawn and idling, drift off:

| `crossbot_aim_delay` | served | ring |
|---|---|---|
| 0.000 | 0.000 | fills to 256/256 |
| 0.250 | 0.250 | fills to 256/256 |
| 0.500 | 0.500 | fills to 256/256 |

`served` tracking the request exactly is the interpolated read working: without the lerp it would quantize
to whole frames and drift with the frame rate. A `served` of 0.000 at a nonzero delay is not a defect
either -- it is the underflow silence, caught in the second or so after a re-acquire while the ring refills.

`err` is deliberately absent from that table. The script converges the bots to hold line of sight, and that
is also where the hitbox-bone spread subtends its widest angle: every bone sits below the eye that
`AnglesToTarget` measures against, so err runs to tens of degrees and is almost pure negative pitch. That is
geometry, not tracking error.

**What the static phase does not prove.** Both bots are *stationary*, so `velocity * delay` is zero and the
extrapolation is never exercised -- it proves the ring, the read index and the interpolation, not the
prediction. Constant-velocity tracking has to be checked by hand, by joining and strafing at a steady speed;
see `doc/testing.md`. Roaming bots are not a substitute: `crossfire` has no stretch long enough to hold a
constant velocity, and the roam phase mixes genuine target acceleration with the gate suppression pointing
the bot at a ladder. err there has been seen anywhere between 2 and 48 degrees across runs, with no way to
attribute any of it -- it is not a number to compare against a previous run.

### The brake multiplier, and why b<1 is not really an undershoot

The controller in `StepAxis` is time-optimal, so it arrives with zero velocity every time. That is the one
thing a hand never does. Phase 3 makes it miss on purpose with a single number: `brake`, multiplying the
deceleration envelope and nothing else.

```
envelope = (sqrt(2*a*m + 0.25*s^2) - 0.5*s) * brake
```

The dead-beat `m/dt` term is deliberately left alone. It is what closes an error small enough to close this
frame, and multiplying it too would make tiny errors miss forever instead of terminating.

`brake > 1` lets the axis carry more speed than it can shed, so it arrives moving and sails past. The
overshoot is exact: deceleration starts at remaining `v^2/(2*a*b^2)` but stopping needs `v^2/(2*a)`, so it
overshoots by `v^2/(2*a) * (1 - 1/b^2)`. For a large flick the peak is `v^2/(2*a) ~ E/2`, which makes the
overshoot a fixed *fraction* of the movement rather than a fixed angle -- constant relative error, which is
the Fitts' law signature, and it falls out of the algebra rather than being imposed.

`brake < 1` is the asymmetric case and the name is a slight lie. Decelerating early does not leave the axis
short of the target: solving `dm/dt = -b*sqrt(2*a*m)` gives `sqrt(m)` falling linearly to zero in
`sqrt(2*E/a)/b`, so it still converges, just `1/b` times slower. What you see is a soft landing, not a
discrete undershoot and correction. A real undershoot needs the *aim point* to be wrong, not the braking,
and that is an offset model rather than this one. The soft landing is worth having anyway, so it stays.

### Sampled once per flick, not driven by a sine

The original sketch modulated the error with a sine. That was dropped: anything periodic reads as robotic
once you watch it for a minute, which is the exact failure the model exists to avoid.

`UpdateBrake` resamples both axes when the combined error magnitude is above `kFlickMinError` (3 degrees)
*and* grew by more than `kFlickGrowth` (0.5) since last frame. During a converging movement the error falls
monotonically, so nothing retriggers. Two things make it grow: a genuinely new target, and the bot's own
overshoot flipping the error sign and pushing it back up through the threshold. So the correction movement
gets its own independent brake for free, and the visible result is one ballistic movement followed by one or
two smaller ones -- again, not imposed anywhere.

The chain terminates on its own. At 60fps the dead-beat term dominates below `2*a*b^2*dt^2` ~ 2.5 degrees,
so a correction smaller than that closes in a single frame no matter what brake was drawn, and there is no
limit cycle at the end of a flick.

Both axes are sampled independently on the same trigger. That is deliberate: a shared brake would make every
miss land exactly along the movement's own line, and humans miss to the side too.

`crossbot_aim_status` prints the live `brake` pair, and **a frozen value there is the correct reading, not a
stuck one.** A bot idling on a stationary target never lets the error grow past 3 degrees, so it never
resamples and holds whatever it drew on the last real flick — which is exactly what the static phase of
`test_sense.ps1` shows. Watch it change during the roam phase instead.

### The RNG is per bot and seeded from the entity index

`RandomUnit` is a plain LCG on `BotAim::State`, not `RANDOM_FLOAT`. Two reasons. It does not consume the
shared engine RNG, which the game uses for weapon spread. More importantly it is seeded from
`pfnIndexOfEdict` alone, so a given bot draws the same sequence on every run of a map -- the nav sweep stays
reproducible, which matters more here than usual, because the measurement discipline this work needed
(`doc/testing.md`) already has enough sources of run-to-run noise without aim error adding another.

`kBrakeSpread` is 0.06, scaled by `crossbot_aim_error`. Sampling is uniform and symmetric about 1. Motor
control literature says the primary submovement usually undershoots rather than overshooting, so biasing the
sample low is the obvious next tuning knob; it is not done because there is no measurement here that could
justify a particular bias.

### Idle micro-drift

A bot holding still with a settled view is visibly a statue. `Bot_UpdateFacing` adds a slow sine to the
*local* copy of the desired angles -- `crossbot_aim_drift` degrees of yaw at 1.7 rad/s, 60% of that in pitch
at 1.19 rad/s, per-bot phases drawn at seed time. The two rates are incommensurate, so the pattern does not
visibly repeat.

It is added to a local copy, not to `bot.desiredAngles`, so `RunActivation` and `RunChargerUse` keep
comparing the view against the same angles they wrote and their gates are untouched. The ladder gate at
`kLadderAimTolerance` compares against its own locally computed `aimYaw`, so drift does show up there as
steady-state error -- 1 degree of it, against a 6 degree tolerance.

**Drift is suppressed while `testActive`.** This is not optional. `crossbot_aim_test` settles on a 0.5 degree
tolerance and the drift amplitude is 1 degree, so with drift running the test could never settle at all.

### The settle gate had to learn about overshoot

`UpdateTest` previously reported settle on the first frame inside `kTestSettleTolerance`. With overshoot in
play, a view flying through the target at 200+ deg/s on its way past can land inside that band for one frame
and report a settle that never happened. It now also requires hand speed below `kTestSettleSpeed` (150
deg/s), which is above the ~50 deg/s the dead-beat term produces on genuine arrival at typical frame rates
and well below flythrough speed.

This is a threshold, not a proof: an overshoot small enough to cross the target slowly can still slip under
it. Read the spread across many flicks rather than any single number, which is what
`scripts/test_aim_error.ps1` exists to produce.

### What the error actually measures out at

45/45 on the nav sweep at the shipped defaults (`crossbot_aim_error 1`, `crossbot_aim_drift 1`), so the
gate-latch cost the phase 1 controller introduced is not compounded by the error model. Eight flicks per
cell, `crossfire`, one bot:

| `crossbot_aim_error` | flick | settle min | avg | max | spread | peak |
|---|---|---|---|---|---|---|
| 0 | 180 0 | 0.308 | 0.349 | 0.372 | 0.064s | 895 deg/s |
| 0 | 0 90 | 0.255 | 0.266 | 0.297 | 0.042s | 563 deg/s |
| 1 | 180 0 | 0.296 | 0.398 | 0.543 | 0.247s | 908 deg/s |
| 1 | 0 90 | 0.245 | 0.285 | 0.354 | 0.109s | 557 deg/s |

The `crossbot_aim_error 0` rows are the regression check: brake forced to exactly 1, and the numbers land on
the phase 1 measurements (0.37s at 899 deg/s for a 180, 0.28s at 557 for 90 of pitch). The residual 0.06s
spread there is frame-phase jitter, not the model.

At scale 1 the distribution opens up and, importantly, opens up *asymmetrically*: the mean moves up 0.05s
but the max moves up 0.17s. That long tail is the overshoot-and-correct case, and it is the shape that was
wanted. The minimum going slightly *below* the scale-0 minimum is not an anomaly -- a small `brake > 1`
carries more speed in and lands with an overshoot under the dead-beat threshold, so it arrives faster than
time-optimal-with-a-full-stop does.

## bot_sense.cpp

What the bot knows about the world and how it found out. Three inputs -- vision, heard sounds and playback
events, and damage taken -- feed one per-bot ring of suspicions, and two outputs come back out: an aim
target pushed into the delay queue, and a look direction for when there is no target.

`crossbot_sense` gates the whole thing. `crossbot_sense_decay` (default 5s) is the only other knob; the
view cone (90 degrees) and vision range (4096) are constants, because they are geometry rather than policy.

### The engine hooks were dead code

`GetEngineFunctions` and `GetEngineFunctions_Post` had existed in `engine_hooks.cpp` since the beginning,
correctly written, and Metamod had never once called them: `meta_api.cpp` left slots 7 and 8 of
`META_FUNCTIONS` at `NULL`. Filling a table entry alone does nothing. If a sound hook ever appears to be
silent, check those two slots first.

Both tables are now `memset` plus assignment by member name. They used to be ~160 bare `NULL,` entries
each, positional against a 158-member struct; `pfnEmitSound` is member 29 and `pfnPlaybackEvent` is member
123, and counting to those by hand is a defect waiting to happen.

Both hooks are in the **post** table and return `MRES_IGNORED`. Post for the same reason item detection is
post: a pre hook fires even when another plugin returns `MRES_SUPERCEDE` and the sound never actually
plays, which would record something that did not happen.

The hooks fan out through `BotManager::OnSound` / `OnEvent`, which loop `g_bots` inline. The work per bot
is a distance compare and a ring push, so there is no deferred sound queue -- unlike foxbot, whose
`BotSoundSense` does waypoint route-distance queries inside the engine callback.

### Sounds carry no position, and the origin is not v.origin

`pfnEmitSound` passes an entity and nothing else spatial. The engine derives the position as the emitter's
bbox centre, so `EntitySoundOrigin` does the same: `origin + (mins + maxs) * 0.5`. For brush entities --
doors, lifts, breakables -- `v.origin` is `(0,0,0)` and using it would put every door sound at the map
origin.

Audible radius follows the engine's own spatialization, which is inaudible past a 1000-unit nominal clip
distance scaled by attenuation:

    attenuation <= 0  ->  audible everywhere (ATTN_NONE)
    otherwise         ->  radius = 1000 * volume / attenuation

`ATTN_NORM` (0.8) at full volume is about 1250 units, `ATTN_STATIC` (1.25) is 800, `ATTN_IDLE` (2.0) is
500. GoldSrc does not occlude sound, so a plain distance test is faithful and no trace is needed.

`sample` is never inspected. Two filters replace it, both chosen off a dump of what actually arrives on
`crossfire`: drop `volume <= 0`, and drop `common/null.wav`. Between them those catch every stop-sound and
silence command, which were a quarter of all traffic. **`CHAN_STATIC` is deliberately kept** even though it
is nominally the looping-ambient channel -- in practice HL brush entities emit door and lift sounds on it,
and those are among the few genuinely interesting world sounds on a deathmatch map.

### Footsteps arrive through pmove, not through pfnEmitSound

Hooking `pfnEmitSound` sees zero footsteps, and the reason is not that footsteps are client-side. They are
emitted server-side like anything else -- they just take a different road into the engine.

There are two entry points into `SV_StartSound`. `pfnEmitSound` is one of them, and it is the only one the
`enginefuncs_t` table can reach. The other is `PM_SV_PlaySound`, which the engine hangs off
`pmove->PM_PlaySound` and pmove calls directly. Everything pmove emits -- footsteps, ladder steps, water
wade and slosh, jump landings, fall grunts -- goes down that second road and is invisible to the
enginefuncs hook.

`PM_PlayStepSound` does have a `!pmove->runfuncs` early-out, but it never trips on a server: `SV_SetupMove`
sets `runfuncs = TRUE` unconditionally, immediately before calling `pfnPM_Move`.

So the hook goes on the callback itself. `pfnPM_Move` is hooked both pre and post: pre saves
`ppmove->PM_PlaySound` and swaps in a trampoline, post puts the original back. The trampoline calls the
original first and then feeds `BotManager::OnSound` with the moving player's edict, so footsteps land in
the suspicion list as ordinary `SUSP_SOUND` entries and every existing filter and the attenuation-radius
test apply unchanged.

Three things make this the only workable shape:

- **The engine reassigns `PM_PlaySound` on every `SV_RunCmd`**, so there is no such thing as a persistent
  install. Inside `pfnPM_Move` is the only place the swap can live. (jk_botti installs "permanently" and
  restores at unload; because of the reassignment its pre-hook is in fact re-installing every single call,
  which is the same thing written less obviously.)
- **The identity guard `ppmove->PM_PlaySound != PM_PlaySoundHook` is load-bearing.** Without it a double
  install saves the trampoline as its own original and the first footstep recurses until the stack dies.
- **`ppmove->player_index` is 0-based** -- the edict index is `+ 1`. Getting this wrong misattributes every
  footstep to the neighbouring player, which looks entirely plausible and is entirely wrong.

The install is skipped when `crossbot_sense` is 0, so that path stays free; the post restore is guarded on
the identity check instead, so flipping the cvar mid-frame cannot strand the pointer.

`pm_shared` never calls `PM_PlaybackEventFull`, so there is no matching blind spot on the event side --
`pfnPlaybackEvent` really is the only road for events.

### What footsteps are worth, without a single tuning constant

The volume pmove passes is already graded, and the existing `1000 * volume / attenuation` radius turns it
straight into a hearing distance. Everything below is read out of `pm_shared.c` and confirmed against a
live dump of 85 step sounds from two bots on `crossfire`:

| Sound | Volume | Audible at (ATTN_NORM) |
|---|---|---|
| Running step, concrete/metal/tile/grate/slosh | 0.50 | ~625 |
| Running step, dirt | 0.55 | ~690 |
| Running step, vent | 0.70 | ~875 |
| Ladder step | 0.35 | ~440 |
| Wading into water | 0.65-1.0 | ~810-1250 |
| Jump launch, and landing by fall speed | 0.5 / 0.85 / 1.0 | ~625-1250 |
| Fall-pain grunt (`CHAN_VOICE`, hard landing only) | 1.0 | ~1250 |

Two engine rules come along for free and are worth knowing because they are behaviour, not detail:

- **Walking is silent in multiplayer.** Any step with horizontal speed `<= 220` is dropped outright. This
  is the real rule that foxbot approximates by hand with a velocity threshold, having had no way to reach
  it. It also means a bot that wants to move quietly already has a mechanism -- move slower -- and nothing
  in `bot_move_nav.cpp` uses it yet.
- **`sv_footsteps 0` disables the whole sense** from outside the plugin, since the gate is inside
  `PM_PlayStepSound`. If footsteps ever appear to have stopped working, check that first.

A bot does not hear its own steps: `OnSound` already skips the emitting entity. Note that this means the
nav sweep, which spawns a single bot, is completely unaffected by any of this.

### Events come from pInvoker, never from the origin argument

`pfnPlaybackEvent`'s `origin` is `(0,0,0)` for essentially all weapon fire -- the engine fills it in from
`pInvoker->v.origin` when `FEVENT_ORIGIN` is unset. Reading the argument would put every gunshot at the map
origin.

Filters: `pInvoker` must be a live player, and `FEV_HOSTONLY` is dropped as per-client viewmodel cosmetics.
`FEV_CLIENT` needs no filter, as those never reach the server. There is no radius test -- a gunshot in HL
is `LOUD_GUN_VOLUME` class and is meant to carry.

There is deliberately no `pfnPrecacheEvent` index-to-name table. jk_botti keeps one for per-weapon
loudness; a player-invoked non-`FEV_HOSTONLY` event is essentially always weapon fire in HLDM, and the
table is per-map bookkeeping for a refinement nothing uses yet.

### Damage must be read before Bot_RunPlayerMove

The trigger is `pev->dmg_take != 0`, and the position is `EntitySoundOrigin(pev->dmg_inflictor)`.

The ordering is the whole trick. `dmg_take` is zeroed by `CBasePlayer::UpdateClientData`, which runs inside
`PlayerPostThink` -- and for a fakeclient the only thing that runs `PlayerPostThink` is the bot's own
`pfnRunPlayerMove`, the last action of its think. So the field accumulates over exactly the window from the
previous frame's `RunPlayerMove` to this frame's `BotSense::Update`, and other players' bullets always land
inside that window because they are fired during *their* think. Move the read after `Bot_RunPlayerMove` and
it reads zero essentially always.

`dmg_inflictor` is never cleared by anything, so on its own it answers "who hit me" and not "was I hit". It
is only meaningful read in the same breath as a non-zero `dmg_take`.

The blind spot is damage taken during the bot's own pmove -- fall damage, walking into a `trigger_hurt` --
which `UpdateClientData` clears before the next update sees it. Those are self- and world-inflicted and the
inflictor filter discards them anyway.

### Vision spends at most two tracelines per bot

It used to spend one per candidate. `UpdateVision` traced line of sight to every live player that passed
range and the view cone, every frame, with no throttle, stagger or cache, and of the expensive kind:
`dont_ignore_monsters` makes the engine walk the entity list. `pfnGetBonePosition` rode along at the same
rate. The gates that should have bounded this do not: 4096 units covers all of crossfire, and a 90-degree
cone rejects half the server.

Measured on crossfire with 32 bots, the mean in-cone candidate count (`cands` in
`crossbot_sense_status`) is **6.18**, with a tail to 15 -- so the old scan cost about 198 perception
traces per frame across the server, not the 992 the worst case suggests. After the change `ZONE_SN_LOS`
reports **51.6 traces/frame over ~30.5 active bots, or 1.69 per bot**, and that figure already includes
the lead trace below. Roughly a fourfold cut, and the tail is what it really removes: the bot in the
middle of a firefight was the one paying 15.

The scan pass now spends no traces at all. It keeps the same gates in the same order and its only job is to
**choose**, by weighted reservoir (`acc += w; if (RandomUnit(...) * acc < w) pick = ent;` -- the idiom from
`bot_fire.cpp`, single-pass and needing no storage). The weight is

```
(dot(theirViewDir, dirFromThemToUs) + 1) / dist
```

so a bot spends its one discretionary trace on someone close *and* looking at it -- the person about to
shoot. Then at most two traces run:

1. **The incumbent**, if it passed the gates. That is `BotCombat::Enemy` if combat is active, else last
   frame's `sense.target`. The bot re-checks the enemy it is already fighting before looking for another.
2. **The weighted pick**, only if there was no incumbent or the incumbent came back occluded.

`kViewRange` and the cone are unchanged, and the lead trace below is unchanged, so the per-bot budget is
1-2 `dont_ignore_monsters` traces plus 0-1 world-only.

**The incumbent still has to pass `Candidate`, range and the cone.** It is not traced from outside the view
cone -- skipping the gates for it would let a bot see and shoot an enemy standing directly behind it. A bot
that turns away from its enemy drops `sense.target` exactly as it always did; combat's own five-second
persistence is what carries the enemy across the gap, not vision.

**`Incumbent` reads last frame's combat enemy.** `BotSense::Update` runs before `BotCombat::Update`, which
in turn reads this frame's `sense.target`. The one-frame lag is deliberate and stable, but a reader who
assumes same-frame coherence will misread the loop; the `sense.target` fallback is what covers the frame
before combat has entered.

`visionCands` is how many players passed range and cone — how many tracelines the old
every-candidate scan would have spent — and `visionKeep` says which of the at most two traces won.
`visionKeep` alone is one frame, so the cumulative `keeps[]` tally carries the same claim across polls:
during a duel it should read incumbent nearly every frame, and a run of fresh picks means the lock is broken.

**Use `pfnAngleVectors` for the target's facing, never `MAKE_VECTORS`.** The latter writes
`gpGlobals->v_forward`, which the function is still holding as *this* bot's forward. Clobbering it mid-loop
would rotate the bot's own view cone to each candidate's facing in turn -- a bug that would look like
erratic acquisition, not like an aliasing error. And read `v.v_angle` without flipping pitch: it is already
the true view angle, unlike `pev->angles`, which is why `bot_deploy.cpp` negates x when it reads a
tripmine's orientation. Copying that flip here would make every bot prefer targets facing *away* from it.

### Teammates never reach the trace budget

`Candidate` rejects anything `Teams::Attackable` says this bot may not shoot -- see `doc/games.md`,
`teams.cpp`, for what that means and why it is a no-op on a deathmatch map. The gate goes here rather
than downstream because "do the POV checks only for enemies" is both the behaviour and the cheaper
arrangement: a rejected player never enters the weighted reservoir, never costs one of the two
tracelines, and never becomes `sense.target`. Everything that shoots reads `sense.target` or
`sense.enemy` and nothing else, so one gate covers `bot_fire.cpp`, the ambush, the duel, the cover
shuffle and the func_tank in `bot_fun.cpp` at once, and none of them needed an edit.

**Unknown is rejected too, not just a match.** A player whose team reads zero is not shot at while
teams are active. The bot only ever engages a confirmed enemy; the case where nobody has a team is
handled one level up, by `Teams::Active` falling back to today's behaviour for the whole bot.

**The suspicion table is deliberately left unfiltered.** `Hear`, `Notice` and `Push` still record
teammates, so a bot still turns its head toward a friend's footsteps -- that is the gaze, and it looks
right. The table drives an attack in exactly one place, the satchel, which is gated separately in
`NearestLive`. Camping reads teams a second time for the friend-in-view calm, but that read never
aims or shoots, so the one-attack-gate claim still holds.

### Monsters are candidates only while they are hunting us

The scan used to run `1 .. maxClients` and nothing else, and `Candidate` demanded `FL_CLIENT` on top of
that. Between them, a monster could never become `sense.target`, never become `combat.enemy` and never be
shot at -- so bots walked past snarks that were actively eating them. A monster still reached the
suspicion table through sound, damage and events, which steered the gaze and did nothing else.

A second loop now runs after the client loop, over a **shared per-frame monster list**:
`BotSense::RefreshMonsters` scans the edict array once per frame in `BotManager::StartFrame`, keeping the
first 64 entities that are live, have private data, carry `FL_MONSTER`, and are damageable. One scan per
frame for the whole server, not one per bot -- a per-bot scan of `maxEntities` is the thing this list
exists to avoid.

The gate on that list is a single question: **is its `CBaseMonster::m_hEnemy` us?** `MonsterCandidate`
asks `g_pGameProfile->MonsterEnemy(ent) == bot.pEdict` and nothing else beyond a liveness test. A snark
hunting someone else is ignored; so is every tripmine, satchel and grenade, all of which are `CGrenade`
descendants carrying `FL_MONSTER` and a null `m_hEnemy`. This is deliberately narrower than "shoot
monsters": the bot answers threats to itself and does not go hunting.

**The budget does not change.** Monsters are drawn into the *same* weighted reservoir as players and
compete for the same single discretionary trace, so the per-bot cost is still 1-2 `dont_ignore_monsters`
traces plus 0-1 world-only. Their gaze term is a fixed `kMonsterGaze = 2.f`, the maximum the player branch
can produce, because a monster whose `m_hEnemy` is us is facing us by construction. `v.v_angle` and
`v.fov` are meaningless on a monster and are never read for one -- the same reason `EnemySees` short
circuits in `bot_combat.cpp`.

**The trace goes to the box centre, not a bone.** `SamplePoint` returns `EntitySoundOrigin` for anything
without `FL_CLIENT` and never builds a bone set for it. A snark is an 8-unit cube with a zero `view_ofs`,
so the eye fallback would aim at its feet and the bone cache would fill with models nothing else samples.

The list is emptied by `ResetBoneCache`, which is the wrong name for it and the right call site:
that is where the two map transitions already flush per-map entity caches, and edict pointers must
not survive one. Nothing reads the list without bots on the server anyway, since `StartFrame` only
refreshes it when a bot exists and `crossbot_monsters` refreshes before it prints.

`monSeen`, `monMine`, `monTraces` and `monHits` are cumulative per bot and readable in
`crossbot_sense_status`. They separate the three ways this can fail: no monsters in the list, monsters in
the list that never name us, and monsters that name us but whose traces never land.

**With teams on, a hostile monster does not have to be hunting *us*.** `MonsterCandidate` asks
`Teams::Of` first: a monster on a different non-zero team is a candidate outright, one on our team
never is, and only `REL_UNKNOWN` -- which is every monster on a deathmatch map, and every monster in a
mod that leaves `pev->team` alone -- falls back to the `m_hEnemy == us` rule above. A monster carrying
an owner inherits that owner's team, so an enemy's snark is engaged and a teammate's is not.

**A null `m_hEnemy` is still a hard reject, and it has to be.** Tripmines, satchels and hand grenades
are all `CGrenade` descendants carrying `FL_MONSTER`, and they all inherit a team from `pev->owner` --
so the team test alone would have bots shooting at enemy ordnance lying on the floor. Requiring
`MonsterEnemy(ent)` to name *someone* keeps them out, because only a real `CBaseMonster` ever sets it.
The widening is therefore "a hostile monster hunting anyone" rather than "anything hostile", which is
also the honest reading of engaging it before it turns on us.

#### Four consequences, all intended

- **`sense.target` no longer means "nearest visible".** It means "the one I traced and saw". The old strict
  nearest-wins rule is gone, so a bot can keep fighting a mid-range enemy while someone closer runs past.
  That is the point of the incumbent rule and reads as commitment rather than the old snap-to-nearest
  twitch. `bot_fire`, `bot_combat`, `bot_fun` and `bot_deploy` all inherit it.
- **`SUSP_SIGHT` is pushed for at most one entity per frame**, where every visible player used to get one.
  The case that matters is unaffected: the active enemy *is* the incumbent and is traced every frame it is
  in the cone, so `BotSense::Senses` keeps holding combat open. Everyone else still enters the table
  through sound, damage and events. The PLAYER tier in `LookDirection` thins out, which is the same
  direction the one-entry-per-entity change already took.
- **Acquisition is probabilistic.** A bot facing N enemies draws one per frame instead of testing all N, so
  first contact takes a frame or two rather than being guaranteed. The incumbent lock holds afterwards, so
  this is invisible at 100 Hz and about 0.1-0.2s at `sys_ticrate 10`. `test_fire.ps1` is the gate on it.
  The measurable cost is a small drop in the fraction of frames on which a bot is holding a target: with
  four bots and combat off it reads 27%, and `keep=new` is well under 1% of samples because a pick only
  wins on the *first* frame of an engagement and the incumbent holds every frame after. Downstream, a
  target lost mid-commit is what makes a grenade throw run to its 1.5s safety cap, so
  `test_fire.ps1`'s zero-tolerance `capped` check on `weapon_handgrenade` is the assertion most likely to
  feel this; it was seen firing once in three runs at one cap per 35 throws.
- **`(dot + 1)` gives an exactly-back-turned target weight zero.** This gates *acquisition only* -- once
  acquired, the incumbent path traces the enemy regardless of facing, so an enemy that turns and runs stays
  tracked. The residual is spotting someone already running directly away who has made no noise, which
  footsteps generally cover. If that ever reads wrong in game, the fix is an additive floor in `gaze`;
  this is the tuning point, and it deliberately has no cvar.

### Vision samples a hitbox bone, not the eye

Per traced player, in this order: range, then a 3D cone test, then a bone position, then the trace. The
order is cost -- `pfnGetBonePosition` runs a full `SV_StudioSetupBones` on every single call, one skeleton
setup per bone, so it goes last and only for the one or two candidates that survived selection.

The cone is a genuine 3D dot against `v_forward`, not foxbot's yaw-only `FInViewCone`, which lets a bot see
targets directly above it. Half-angle is `fov / 2`, so a symmetric cone against the *horizontal* FOV --
slightly generous vertically, since HL's vertical FOV at 4:3 is nearer 74 degrees. Accepted rather than
building a real frustum.

The trace is `dont_ignore_monsters`, and that is mandatory, not stylistic: with `ignore_monsters` the trace
passes straight through players and `tr.pHit == ent` can never be true.

The sample point is a bone drawn at random from the set of bones that carry a hitbox. Sampling all of
`0..numbones` instead would include helper bones -- the `Bip01` root sits at the feet, and weapon and blend
bones sit off the body -- which drag the averaged aim point down and make visibility flicker on floor
geometry. Every bone that owns a hitbox is a point that can actually be shot.

Two engine caveats. `pfnGetBonePosition`'s `rgflAngles` out-parameter is **never written**; it is passed
`nullptr` here and only the origin is ever read. That is also why hitbox *centres* are not used, since
transforming `bbmin`/`bbmax` into world space would need the bone's rotation. And stock GoldSrc only
bounds-checks `iBone` under `#ifdef REHLDS_FIXES`, so an out-of-range index is an out-of-bounds read on
vanilla HLDS -- the bone set is built from `numhitboxes` and every index checked against `numbones`.

### The lead point is traced before it is queued

The queue write pushes `bestPoint + targetVelocity * delay` -- where the target is predicted to be one
reaction time from now. For a long time nothing checked that this predicted point was anywhere a bullet
could reach. At the default 0.25s and full run speed the lead runs to about 80 units, so it crosses into a
wall roughly a whole reaction time before the target does, and a bot leading someone into cover spent the
entire drain window aimed at brickwork.

So the lead is traced, from the bot's real current eye, and the slot is pushed only if it comes back clear.
A blocked lead pushes nothing at all.

**It gates the queue write and nothing else.** The perception trace above still runs against the raw bone
point, so `SUSP_SIGHT` and target selection are untouched. Pointing that one trace at the lead instead --
the obvious version -- would let an occluded lead null out `best`, and when it returned the next frame
`best != sense.target.Get()` would fire `ClearDelayQueue` and wipe the whole ring. With 80-unit leads and a
hitbox bone redrawn at random every frame, that flickers at every doorway and railing, and each flicker
costs a full reaction delay of silence. Gating only the write also costs one trace per bot per frame,
because the write happens once, for the winner.

**The flags differ from the perception trace, deliberately.** This one is `ignore_monsters`; the one above
is `dont_ignore_monsters` and has to be, or `tr.pHit == ent` could never be true. The two ask different
questions. Perception asks *can I see this player*. The lead trace asks *is this predicted point inside or
behind world geometry* -- another player, or the target's own body, standing between the eye and a point 80
units past it must not suppress the lead, and under `dont_ignore_monsters` the target itself would be
`tr.pHit` most of the time. Brush entities still block, which is right: a closed door is a wall. It is the
same flag and the same question as the fire-side trace in `BotFire::Buttons`.

**The trace is skipped when the lead is the sample point**, which covers both a stationary target and
`crossbot_aim_delay 0`. That is not only an optimisation -- it is what makes `test_sense.ps1`'s static phase
provably unchanged, since two motionless bots produce no lead, which keeps that phase usable as a regression
check for everything else in the queue.

Nothing is pushed in place of a vetoed lead. The ring drains, and if the block outlasts `delay` the read
underflows and `ApplyLook` falls through to the suspicion blend, which points at the last `SUSP_SIGHT`
entry -- the target's real last-seen position, at the corner. `ReadDelayed` needs no help with the gap: it
interpolates on time, so a hole in the write history is just a longer span, and the served point slides
across it smoothly when writes resume rather than jumping.

Both `crossbot_aim_status` and `crossbot_sense_status` report `leadblk`, a per-bot count of vetoes since
spawn. A count rather than a current-frame flag because a two-second poll would almost never catch the flag
set, and the thing worth verifying is that the veto fires at all.

### The bone cache and when it flushes

`g_boneSets` is a file-static array of 64 entries, map-scoped rather than per-bot. 64 rather than one per
player model so the same cache serves monster models if perception ever extends past players; the whole
table is under 3 KB.

The key is `v.modelindex`, not the header pointer. Modelindex is stable for the life of a map, and a player
changing model with `setinfo model` lands on a different modelindex and therefore a different slot.

It flushes:

- on `ServerActivate` -- the only flush actually required, since modelindex-to-model mapping is rebuilt per
  map and every entry is meaningless afterwards;
- on `ServerDeactivate`, so nothing survives a level change;
- per entry, opportunistically: each lookup compares the stored `studiohdr_t*` against the current
  `pfnGetModelPtr`, and rebuilds in place if they differ. GoldSrc's model cache can free and reload studio
  extradata under memory pressure, and this makes that self-healing rather than something to detect.

Never on a timer and never per bot -- the set of bones a model has does not change while it is loaded.

### One entry per entity, and why it stopped being a ring

The list used to append. Sound, event and damage entries each took a fresh slot, and only `SUSP_SIGHT` was
deduplicated per entity. The reasoning was that an enemy firing while moving leaves a trail of positions,
which is real information about their direction of travel -- and it is, but the price was that **the
trail outvoted the head**. A visible enemy is one entry; the same enemy running away lays down a footstep
entry every ~0.3s, so within the 5-second window they hold fifteen votes at fifteen different points and
the weighted mean is dragged backwards along the way they came. An elevator cycling in a shaft did the
same thing for free. Footsteps did not create this defect, but they made it constant.

Now every entity gets exactly one slot. `Push` looks for a live handle matching the emitter, overwrites it
in place if it finds one, appends if there is room, and otherwise reuses the entry with the smallest
`time`. Nothing can vote twice no matter how much noise it makes, and the direction the bot faces stops
being a function of emission rate.

The dedup key is a `CBotEntHandle`, not an entity index. That matters at exactly one moment: a breakable
is destroyed and something else is later allocated into the same edict slot. Under an index it would
inherit the old entry and its stale position; the handle's serial number makes `Is` return false across
that boundary, so the new entity gets a new slot and the orphan is unmatchable and ages out.

`head` is gone, and that deletion is forced rather than tidy. Every read used to walk backwards from
`head` and `break` at the first zero-weight entry, which was only ever valid because appending kept the
ring in time order. Overwriting a slot's `time` in place destroys that order, so a `break` would silently
truncate the list at the first stale entry -- the one failure mode here that looks like it works. Every
scan now runs `[0, count)` and uses `continue`. With one entry per entity the live set is a handful, so
scanning the array costs less than the ring ever did.

`count` is a high-water mark of distinct entities seen, not a queue depth. Eviction only begins after 256
distinct entities, and since decayed and orphaned entries are by definition the oldest, it reclaims those
long before it touches anything live.

### Where the bot looks when it has no target

Reading an entry goes through `LiveWeight`, and every scan in the file uses it -- tier selection,
accumulation, both refine passes, `LiveCount`, `Freshest`. One choke point is the design: a second
liveness test that disagreed with the first is how a tier gets selected and then contributes nothing.

`LiveWeight` drops an entry whose entity is gone, drops a player or monster that is dead, and otherwise
returns `max(decay - age, 0) / decay`. The handle check is what makes gibs and destroyed breakables stop
steering the bot; the dead check is what stops it staring at a corpse for the five seconds after a kill.

**The dead check is scoped to players and monsters on purpose.** `IsDead` is
`deadflag != DEAD_NO || health <= 0`, and a `func_door` or `func_train` has `health == 0`. Applying it to
`KIND_OTHER` would silently discard every piece of map machinery from the list and leave the fallback tier
permanently empty -- and nothing in the test suite would say so, because the player tier would still work.

Then the vote picks **one tier and ignores the rest**: players if any are live, else monsters, else
everything else. The enum order is the priority order so this is a `min` and not a switch. The point is
that a player entry decayed to weight 0.1 still beats a door at 1.0 -- the bot glances at the fading
contact instead of snapping to the lift. That is why the tier filter runs before `bestWeight` rather than
after it.

**A monster that is hunting us votes at the player tier.** `Suspicion` carries a `hunting` flag, set in
`Push` when the entry is a monster whose `m_hEnemy` is this bot, and `Tier(entry)` returns `KIND_PLAYER`
for it. Without this the rest of the monster work is nearly inert: a bot with any live player contact --
including one decayed to 0.1 -- would never turn its head toward the snark eating it, because the tier
vote would have discarded the monster entry before scoring began. Tracing a monster you never look at
buys nothing.

The flag is stored beside `kind` rather than overwriting it, so `kinds=p:,m:,o:` in
`crossbot_sense_status` still counts what it says it counts and `hunt=` reports the promotion separately.
Writing `KIND_PLAYER` into `kind` would have been one line fewer and would have made the status line lie.

### Nothing is averaged: the winner is one entry, chosen by cone

Within the winning tier every live entry is scored

    score = LiveWeight(entry) * ConeWeight(vantage, dirToEntry)

and the **single highest scorer wins outright**. There is no vector sum, no mean-shift refinement and no
degenerate-mean fallback any more -- all three existed to make averaging survive threats on opposite
sides, and an argmax does not have that problem to solve. `authority` is still the winner's recency
weight alone, so the cone data decides *which* entry the bot looks at and never how hard it commits to
looking: a fresh contact from an unwatched direction still commands a full turn.

`ConeWeight` reads the bot's blended vantage signature (`nav_vantage.cpp`), which says which cones
everything with line of sight to this spot arrives through:

    rel  = watch[k].amount / watch[0].amount        // watch is amount-descending, so rel in (0,1]
    best = max over k of  rel * max(0, dot(dir, watchDir[k]))
    return kVantageConeFloor + (1 - kVantageConeFloor) * best

**Normalising against the strongest cone, not the sum, is what keeps the scale honest.** A dead-on hit on
the main cone scores 1.0 whether the spot has one cone or four. Dividing by the sum would cap a four-cone
spot at 0.25 and the floor would swamp every real hit on it.

`kVantageConeFloor = 0.15` is the whole reason this is a bias and not a blindfold. An entry the map says
nothing ever arrives from still keeps 15% of its recency weight, so a close, fresh threat from an
unwatched direction outscores a stale one down the main sightline and the bot turns. Without the floor a
bot could be structurally incapable of reacting to someone shooting it from a direction navgen never saw
a sightline through.

The point is the wall-staring case: a noise behind the bot in a direction with no sightline used to drag
the averaged mean into the wall between it and everything else. Now it simply loses.

With no vantage data -- an old map, `--no-vantage`, or `crossbot_vantage 0` -- `ConeWeight` returns 1 and
this degenerates to "the freshest live entry in the winning tier wins", which is the old behaviour minus
the averaging.

`ApplyLook` then blends, rather than overriding:

    desiredAngles.yaw = base.yaw + shortWay(want.yaw - base.yaw) * authority
    desiredAngles.pitch = base.pitch + (want.pitch - base.pitch) * authority

`base` is whatever `desiredAngles` already holds -- path facing at the walk site, the last commanded angles
at the idle sites. At `authority == 0` the write is a no-op and the behaviour is bit-identical to having no
sense system at all, which is what keeps the nav sweep honest. As the freshest suspicion ages, the head
drifts back onto the path by itself instead of snapping.

An aim target always wins: if `ReadDelayed` serves a point, `ApplyLook` aims at it and returns before any
of the above.

### When there is no suspicion at all, the vantage field drives the head

`ApplyLook`'s `authority == 0` branch used to level the pitch and leave the yaw alone. It now calls
`ApplyVantageLook`, which splits on whether a path is running:

- **No path** -- write `watch[0]`'s yaw and pitch. That is the direction most of the sightlines into this
  spot arrive from, so a bot standing still is watching the way it is most likely to be shot from.
- **Path running** -- `desiredAngles.y` at this point is the path yaw, freshly written from the next
  waypoint in `bot_move_nav.cpp`. Pick the watch direction closest to it in yaw; snap to it if it is
  within `kVantagePathSnap = 60` degrees, otherwise keep the path yaw and level the pitch. So a bot
  rounding a corner is already looking down the corridor rather than at its own next waypoint.

**The 60-degree clamp is not cosmetic: yaw is the movement basis.** `Bot_DecomposeMove` flattens to yaw
alone, so an unbounded snap turns every corridor into a sideways strafe. The clamp buys the head-turn
without giving up forward drive.

**Both branches write absolutely, not through the `authority` blend, and that is deliberate** -- see the
ratchet note below. `base` is the previous frame's own output at the idle call sites, so a blend there
compounds and the head freezes wherever it drifted to. An absolute write cannot ratchet.

The blend itself is refreshed once every `kVantageInterval = 0.1` s per bot from the bot's eye position,
not per frame; the field is a smooth spatial function and a bot does not move far in 100 ms.

`crossbot_vantage 0` skips the refresh entirely, so the blend is empty, `ConeWeight` returns 1 and this
branch falls through to the old `desiredAngles.x = 0`. That switches off everything the vantage field
does, which is what the nav-sweep A/B needs -- but it is **not** a full revert to the pre-field runtime,
because the argmax replaced the averaging unconditionally and does not depend on the cvar.

**Senses outrank the vantage look, and on a busy map that matters more than it sounds.** The tier vote
falls through to `KIND_OTHER`, and map machinery -- doors, chargers, item respawns -- pushes `SUSP_SOUND`
entries constantly. A live `KIND_OTHER` entry at weight 0.1 still takes the head away from the vantage
direction, because it is a real sense and this is the fallback. `crossbot_vantage_status` reports which
branch actually won per bot (`look=sense` / `vantage-idle` / `vantage-path` / `vantage-offpath` /
`gated`), which is the only reliable way to tell them apart from outside.

### The gate veto survived from the gaze scaffold

`ApplyLook` refuses entirely when any waypoint in `waypointIndex .. +2` needs a ladder, an airmove launch,
an elevator ride, a button use or a trigger touch. Without that lookahead the bot arrives at a gate with
its head turned and pays the full `kLadderAimTolerance` / `kActivationAimTolerance` latch cost, which is the
controller's one real cost.

It is also still **opt-in**, applied at five sites in `bot_move_nav.cpp` and never on the branches where
the view drives the movement. Opt-in fails safe -- a missed site means no looking around there. Opt-out
would fail dangerous, since a missed site breaks a ladder.

### Two gaps worth knowing about

**A bot with nothing in its cone and no suspicion used to never turn.** This is now solved, and the
solution is the Parabot one rather than the foxbot one. Parabot scored the four compass directions per
map cell by how much of the map is visible that way, weighted by where players historically died, then
swept the head around the winner. navgen's vantage field is that measurement done properly -- an actual
traced visibility signature per sampled eye position rather than four compass buckets -- and
`ApplyVantageLook` points the head at its `watch[0]`. Foxbot's cheap version (random yaw every 1-3s,
rejected unless a 200-unit trace is clear) was the alternative and is not needed.

The sweep exists now, at hard and expert only. `BotDiff::BackCheck` gives `ApplyLook` a period, and a
weighted-random pick over `watch[0..3]` by `amount` turns the head to one of them for 0.4s before the
ordinary logic takes it back. A pick within 60 degrees of where the bot already means to look is
rejected and the period spent, so the glance is always at something the bot was *not* watching. At
normal and easy the head still goes to `watch[0]` and stays there. See `bot_difficulty.cpp` in `doc/difficulty.md` for
why it sits above the suspicion blend rather than inside this function, and for the measured rates.

Footsteps narrow the remaining gap. A running enemy is heard at ~625 units regardless of facing, so a bot
turns round on its own as soon as anyone moves nearby. The uncovered case is the enemy who is stationary
or walking, since below 220 units/s they emit nothing at all -- and a bot facing its main sightline is
much better positioned for that case than one facing a wall.

**Pitch is levelled, and the suspicion pitch is clamped, for a reason.** When `ApplyLook` finds no target,
no live suspicion and no usable vantage direction it writes `desiredAngles.x = 0` rather than leaving the
angles alone -- and the vantage branches sidestep the same trap by writing absolutely. Without that the
blend ratchets: `base` is the previous frame's output, so each frame moves further toward the suspicion and
nothing ever moves it back — when the authority decays to zero the head simply freezes wherever it got to.
A door sound in a shaft below the bot was enough to leave two bots permanently staring at the floor at
about 70 degrees of pitch, which also put each other outside their view cones so neither could ever
reacquire. Levelling is safe at the walk site because navigation always commands pitch 0 there; pitch is
purely a look concern. The `kLookPitchLimit` clamp of 45 degrees bounds how far a very close suspicion can
crank the head in the first place, and does not apply to a real aim target, which needs the full range to
aim at someone below.

### crossbot_sense_status

Per bot: the aim target and the bone currently sampled, the remembered enemy and how stale that
sighting is, live suspicion count broken out by kind, which tier won the vote, the freshest entry with
its source and age and position, the look direction with its authority and how many entries survived
the cone refinement, and cumulative push counts per source.

`target=` against `enemy=` is the pair that shows the memory doing its job. They agree while the bot
can see someone; `target=- enemy=Sense2 enemyAge=1.8s` is the whole point of the feature, and
`enemy=-` returning after `enemyAge` passes `crossbot_sense_decay` is the record expiring on schedule.

`kept` is now simply the live entry count in the winning tier, not a mean-shift survivor count -- the
direction is one entry's, so there is nothing to survive.

`kinds` against `tier` is the second pair, and it is the one assertion worth automating: `tier` must read
`PLAYER` whenever `kinds=p:` is nonzero. `test_sense.ps1` checks exactly that on every poll.

`fresh=` is the freshest entry *overall*, not the freshest in the winning tier, so `tier=PLAYER` next to
`fresh=SOUND ent=148` is correct and not a contradiction -- a door was heard more recently than the
player, and lost anyway.

`pushed=` exists because dedup destroyed the old way of proving a source fires. With one entry per entity
an enemy's `DAMAGE` entry is overwritten by their own `SIGHT` entry inside the same `BotSense::Update`,
so `fresh=` reads `SIGHT` almost always and `DAMAGE` essentially never. `test_fire.ps1` used to sample
this command four times per poll to work around that; the counters are cumulative, so one poll cannot
miss a source that fired between polls, and the sampling loop is gone.

### crossbot_vantage_status

`crossbot_vantage_status` with no arguments reports the loaded field (point count, spacing, resolved
grid cell and dimensions, `maxExposure`, and the cvar) and then, per bot, the blend at its own eye:
neighbours used, distance to the nearest, `exposure` raw and as a fraction of the map maximum,
every merged watch direction, and a drive line giving `view=` (actual `v_angle`),
`desired=` (what the look code commanded this frame), `look=` (which branch produced it) and the
waypoint index.

`crossbot_vantage_status <x> <y> <z>` blends at a world point instead. **This is the round-trip test:**
at a stored vantage's exact position the distance-0 neighbour dominates the IDW, so the output must be
that vantage's own record back -- watch yaws, pitches, spreads and normalised amounts all matching. It is
the cheapest way to prove the loader, the grid and the angular merge are all correct at once.

`desired=` next to the watch list is what makes the look behaviour checkable from outside. On a
`look=vantage-path` sample the commanded yaw *and* pitch equal one of the listed watch directions
exactly, because the branch writes the direction rather than blending toward it. Two caveats when reading
it: the blend printed here is recomputed at print time, so on a moving bot it can be one sample ahead of
the one `ApplyLook` used, and `look=vantage-offpath` prints pitch 0 with the path yaw, which is the
60-degree clamp refusing the snap rather than a missing match.

### The enemy handle outlives the sight of it

`sense.target` is not memory. `UpdateVision` writes `sense.target.Set(best)` unconditionally, `best`
included when it is null, so the handle drops the frame line of sight breaks. That is correct for what
it feeds -- aim and the trigger -- and useless for deciding where to walk.

`sense.enemy` is the memory. It is written from the same site as `target` whenever a target is visible
and survives the loss: handle, position, and the time it was recorded. It is dropped only when the
entity is gone, when the entity is dead, or when the record is older than `crossbot_sense_decay` --
the same three tests `LiveWeight` applies to a suspicion, because it is the same question.

**The position stored is `v.origin`, not `bestPoint`.** `bestPoint` is a randomly chosen hitbox bone
and can be a hand held out over a ledge; `v.origin` is at the feet, which is what `FindAreaExact` can
resolve to a nav area. A bone-derived goal resolves to no area, or to the wrong one, in exactly the
cases where the enemy is somewhere interesting.

**It is deliberately refreshed only by sight.** Sound, gunfire and damage from that same enemy keep
updating its `Suspicion` slot and keep steering the head, but they do not move the remembered
position. So the bot walks to where it last *saw* someone rather than where it last heard a footstep,
and the goal holds still while it walks there.

### Sense reports, it does not decide

`sense.enemy` used to reach forward into navigation: `ForgetEnemy` cleared `bot.goalIsEnemy` and reset
the path, and a sight-loss edge armed a `rescore` flag that `Bot_MoveNav` consumed by dropping the
goal. Both are gone. Sense answers questions and owns nothing downstream; `BotCombat` reads the
answers and makes every transition (`doc/combat.md`, `bot_combat.cpp`).

`Senses(bot, ent)` is the one question the table could not answer before: is there still *any* live
entry pointing at this entity? Sight, sound, damage and events all count, so a bot that can hear its
enemy has not lost it. Entries expire on `crossbot_sense_decay` in the same sweep every other reader
uses, which is what makes this "have I lost every trace of it" rather than "did I ever see it".

`bot.sense` is not covered by anything that resets on death, so `BotSense::OnRespawn` clears the enemy
alongside `BotItems::OnRespawn`, `BotFire::Reset` and `BotCombat::OnRespawn`.
