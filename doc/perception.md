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
100fps, 0.156 at 60, 1.41 at the `kMaxFrameTime` clamp. That is frame-rate coupled in a controller whose
entire envelope design exists to not be, so the clamp stays.

`kMaxHandSpeed` is a hand speed ceiling. It is near-inert at the current constants, since the largest
reachable error is 180 degrees and that only peaks at 720 deg/s, but it is the knob for a bot whose hand
should be slower than its acceleration allows.

### Where the constants come from

These are the normal tier; `turnScale` multiplies all three. `kMaxAccelYaw` is 3000 deg/s^2, the CS zbot's
hand-tuned figure, and `kMaxAccelPitch` is 2000: vertical mouse travel is the awkward direction for a human
wrist. For a symmetric accelerate-then-decelerate turn the peak speed is `sqrt(a * t)`, and `kMaxHandSpeed`
is set from it: `sqrt(3000 * 90) = 520 deg/s`, so a 90 degree yaw flick just touches the ceiling.

**The ceiling is now a clip on purpose.** Anything bigger than 90 cruises at 520 in the middle, so a 180
takes about 0.52s instead of the 0.35s pure acceleration would give, and the time grows linearly rather than
with the square root past that point. A 90 of yaw is about 0.35s, a 90 of pitch about 0.42s (peak 424, under
the ceiling). Big turnarounds being slow is the human shape -- a wrist re-centres, it does not spin faster.

History: the first cut was 2880 (180 in 0.5s), raised to 4500 (180 in 0.4s, ceiling 1200 deliberately
unreached) because 0.5s measurably slowed `test10_doorbutton` -- see below. The present constants put the
180 back past 0.5s, so that map is the one to watch on the nav sweep.

### The cost at the aim gates

Below about 80 degrees of error at the old 4500 (120 at today's 3000; the crossover is `4 * 300^2 / aMax`) the controller is *slower* than the old
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

Expected at the normal tier with `crossbot_aim_gain 0`: `180 0` settles in about 0.5s at a peak of 520 deg/s
(the ceiling), `90 0` in about 0.35s peaking just under 520, and `0 90` in about 0.42s at a peak near 424. The
spread is the frame rate -- lower fps settles slower. Peak hand speed is the more stable number and the
better thing to check: it is set by the constants, not the sampling. With the gain mismatch on, the test
is one of the two places it applies, so settle times grow a reaction-time tail; pin it to 0 to check the
controller alone.

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

### The bot sees its own view late

The target queue makes the bot see the *enemy* a reaction time late. It still read its own `v_angle` live,
so the hand was surgical about where it was pointing. The view queue fixes the other half: the controller
and the trigger both act on what the bot saw of its own view `BotAim::Delay(bot)` ago, the same reaction
time the target queue uses.

The hand noise this replaced -- a mouse-count quantum with a speed floor (the tremor), an idle sine drift,
and a random multiplier on the deceleration envelope (the brake) -- has been deleted. The only deliberate
motor error left is the gain mismatch below.

**A pure delay cannot drive this controller.** Feeding `view(t - d)` into `StepAxis` breaks the dead-beat
term outright. A 6 degree error closes in one frame, and then the stale view keeps reporting 6 degrees
for the next quarter second, so the hand flicks another 6 every frame and oscillates far past the
target. A slow proportional controller would survive it (stable while `gain * d < pi/2`), but that makes
a 180 take most of a second.

**So the bot keeps an efference copy**, the way a brain does. It knows what it told its hand to do, and
only learns from its eyes what it did *not* command. Per frame, in `Bot_UpdateFacing`:

    own      += commanded step (post pitch clamp, pre AdjustView)
    push       { now, off = v_angle - own, punchangle }
    read       off, punch at now - delay  (interpolated, yaw the short way round)
    seen      = own + off
    error     = desired - seen ;  step applied to the real v_angle

`off` is the disturbance: everything that moved the view without the hand. With no disturbance it is
constant and `seen == v_angle` exactly, so turning, the nav gates and `crossbot_aim_test` are untouched at
any delay. What the bot sees late is the disturbances: `AdjustView` (DoD recoil kick, scope sway, bipod
clamp), anything else writing `v_angle`, and `punchangle`.

**The slot stores the offset, not the view and the copy separately.** Differencing two absolute
accumulators across the window needs an unwrapped yaw, and an unwrapped float yaw loses precision over a
long life. The offset only changes on disturbances, so `own` and `off` can both stay wrapped.

**The pitch clamp counts as commanded.** The stop is on screen, so the bot knows about it. Counting it
as a disturbance would make the bot believe it was still pitching past 89 for a quarter second.

**The trigger reads it too.** `Bot_RunPlayerMove` hands `aim.perceived` (seen plus this frame's step) to
`BotFire::Buttons`, and `RayOnTarget` / `ArcOnTarget` add `aim.perceivedPunch` instead of the live
`punchangle`. A spraying bot therefore keeps pulling the trigger on a crosshair that has already
kicked off target, and stops a reaction time after the kick rather than on the same frame. Movement does
not read it: `Bot_DecomposeMove`, the nav aim gates and `AimWithinCone` stay on the real view, because
they are physics and stance rather than hand and trigger.

The ring is cleared in `TeleportAllBots` and on respawn (`bot = CBot()`), the same as the target ring.
Recoil *compensation* is not modelled. The controller still steers `v_angle` and ignores the punch, and
the perceived punch is what a compensation step would read.

### The efference copy is wrong on purpose

With the hand noise all at 0, the efference copy is a Smith predictor with a perfect plant model: `own`
advances by exactly what the hand did, so the bot never misses except through the target queue. A real
predictor's error is model mismatch, and that is the error this adds. On each flick both axes draw a gain,
and the hand moves `step * gain` while `own` still advances by the commanded `step`:

    gain  = 1 + s * (u * 1.5 - 1),   u uniform [0, 1)   ->   [1 - s, 1 + s/2]
    v_angle += step * gain ;  own += step

The difference lands in `off = v_angle - own`, which is exactly the disturbance channel the view queue
already carries. So the bot believes it arrived, sees the miss one reaction time later, and makes a
correction. That is a real undershoot, which shaping the deceleration cannot produce (braking early only
lands softly and slower): the aim point is right, the hand's belief about where it went is wrong. It is proportional to the movement (the Fitts
shape again), the correction gets its own gain on the re-trigger, and the loop converges because each
reaction time the remaining offset rate is multiplied by `-(gain - 1)`. `kMaxGainSpread` caps `s` at 0.5,
well inside the `|gain - 1| < 1` stability bound.

The range is biased low because the primary submovement of a human aimed movement usually undershoots.
`s` is `crossbot_aim_gain`: easy 0.25, normal 0.15, hard 0.10, expert 0.05, so a normal bot can land a
90 degree flick up to 13.5 degrees short or 6.75 degrees long.

**A flick is what `UpdateGain` detects:** the combined error is above `kFlickMinError` (3 degrees) *and*
grew by more than `kFlickGrowth` (0.5) since last frame. A converging movement's error falls monotonically,
so nothing retriggers; a genuinely new target does, and so does the bot's own miss pushing the error back
up through the threshold, so the correction gets its own independent gain. Both axes draw independently,
because a shared gain would make every miss land along the movement's own line, and humans miss to the
side too. `crossbot_aim_status` prints the pair, and a frozen value on a bot idling on a stationary
target is the correct reading: the error never grows past 3 degrees, so it never redraws.

**Only with a target, or during `crossbot_aim_test`.** Gains reset to 1 otherwise. The nav aim gates hold
drive until the view converges, so a mismatched nav turn would cost a reaction time at every ladder and
button. The test is included so the error can be measured.

**The pitch clamp is still commanded.** On a frame that hits the stop, `own` advances by the real step,
so a stop overshot through gain does not leave the bot believing it is still pitching.

Measured with `test_aim_error.ps1 -Gains 0,0.15`, normal tier, eight flicks per cell, `crossfire`:

| gain | flick | settle min | avg | max | peak |
|---|---|---|---|---|---|
| 0 | 180 0 | 0.479 | 0.485 | 0.494 | 520 |
| 0 | 90 0 | 0.308 | 0.339 | 0.487 | 512 |
| 0 | 0 90 | 0.041 | 0.340 | 0.389 | 382 |
| 0.15 | 180 0 | 0.626 | 0.785 | 1.012 | 522 |
| 0.15 | 90 0 | 0.291 | 0.593 | 0.820 | 515 |
| 0.15 | 0 90 | 0.489 | 0.780 | 0.908 | 417 |

The mean moves up by about one reaction time (normal's 0.30), which is the correction submovement, and
the resting error stays 0. The pitch row's low minimum and peak are the 89 degree stop cutting the flick
short when the test starts from a non-level view, not the controller.

### crossbot_aim_status

Prints, per bot: the sense target, which bone it is currently sampling, ring occupancy, how much history
the ring holds, the delay actually served, hand speed, and `err` -- the angle between the bot's real
`v_angle` and the true present-time angles to its target.

`seen` is the bot's belief about its own view and `perr` is the angle between `seen + perceivedPunch` and
`v_angle + punchangle`. `perr` must read 0 whenever nothing but the hand moves the view, at any delay,
once the gain mismatch is accounted for: with a target held and `crossbot_aim_gain` above 0 it is the
mismatch the bot has not seen yet, and it drains to 0 within a reaction time of the view coming to rest.
`gain` prints the pair drawn on the last flick.
Under fire it is nonzero even at `crossbot_aim_delay 0`, because the punch it compares against is one
`pfnRunPlayerMove` newer (the shot and the pmove decay both land after the read). A nonzero `perr` at idle
with no recoil means the efference copy is missing a step.

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

Measured on `crossfire`, both bots converged on one spawn and idling:

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

### The RNG is per bot and seeded from the entity index

`RandomUnit` is a plain LCG on `BotAim::State`, not `RANDOM_FLOAT`. Two reasons. It does not consume the
shared engine RNG, which the game uses for weapon spread. More importantly it is seeded from
`pfnIndexOfEdict` alone, so a given bot draws the same sequence on every run of a map -- the nav sweep stays
reproducible, which matters more here than usual, because the measurement discipline this work needed
(`doc/testing.md`) already has enough sources of run-to-run noise without aim error adding another.

### The settle gate had to learn about overshoot

`UpdateTest` previously reported settle on the first frame inside `kTestSettleTolerance`. With overshoot in
play (a gain above 1), a view flying through the target at 200+ deg/s on its way past can land inside that band for one frame
and report a settle that never happened. It now also requires hand speed below `kTestSettleSpeed` (150
deg/s), which is above the ~50 deg/s the dead-beat term produces on genuine arrival at typical frame rates
and well below flythrough speed.

This is a threshold, not a proof: an overshoot small enough to cross the target slowly can still slip under
it. Read the spread across many flicks rather than any single number, which is what
`scripts/test_aim_error.ps1` exists to produce.

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

**Teammates never reach the suspicion table either.** `Push` drops anything `Teams::Of` calls
`REL_FRIEND`, for every source at once -- sound, event, damage and sight. It used to record them on
the theory that a head turning toward a friend's footsteps looks right. With one friend it does; with
a CS team it does not. Every friendly step lands in the PLAYER tier, which outranks everything else in
`LookDirection`, so on `fy_snow` the gaze spent its whole life on teammates. The same entries also
made `BotGauss::Scared` count a friend nearby as a threat.

**The filter is `== REL_FRIEND`, never `!Attackable`.** A door, lift or breakable has team zero and
reads `REL_UNKNOWN`, which `Attackable` rejects while teams are active -- so the `Attackable` form
would silently delete every world sound on a team map. `REL_FRIEND` only ever matches a confirmed
teammate, or something a teammate owns: their grenades, satchels and snarks inherit the owner's team
through `TeamOf`, and so do the bot's own. On a deathmatch map, with `crossbot_teams 0`, or before the
bot has a team, `Of` answers `REL_UNKNOWN` and nothing is dropped.

The table drives an attack in exactly one place, the satchel, which is still gated separately in
`NearestLive`. Camping reads teams a second time for the friend-in-view calm straight off the player
list, not the table, and that read never aims or shoots, so the one-attack-gate claim still holds.

### Teammate hurt (`SUSP_ALLY`)

A teammate losing health is the one way a friend reaches the table, and it enters as the *attacker*, not the
friend. `BotSense::RefreshAllies` runs once per frame beside `RefreshMonsters`, compares each client's
`v.health` with last frame's, and keeps the victims whose health fell (a respawn raises health, so it never
counts). Each bot then runs `UpdateAllyHurt`: for every hurt `REL_FRIEND` that is not itself, resolve the
attacker with `BotCombat::Attacker` (the half of `DamageSource` that does not need `dmg_take`, safe here only
because it is read next to a real health drop) and `Push` it at the **victim's** position. Fall damage,
`trigger_hurt` and friendly fire resolve to no attacker and push nothing. There is no range or line-of-sight
gate: every teammate feels it. The entry is a `Coned` source like sound, so `ConeWeight` scales its score.
It is gaze only; combat still needs `sense.target`. `crossbot_sense_status` reports it as `ally:`.

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
returns `Falloff(age)`. The handle check is what makes gibs and destroyed breakables stop
steering the bot; the dead check is what stops it staring at a corpse for the five seconds after a kill.

### Recency falls off exponentially

    h = decay / 4                                     // 1.25 s at the default 5 s
    Falloff(age) = (2^(-age/h) - 1/16) / (15/16)      // age < decay, else 0

The `1/16` is `2^-4`, the value at `age == decay`, subtracted and renormalised so the curve lands on
exactly zero there. That keeps "live" meaning `age < decay`, which every liveness reader -- camp scare,
grenade, satchel, TFC pipes, the combat hold in `Senses`, `LiveCounts` -- depends on and did not change.
1 s old is 0.55, 2.5 s is 0.23, 4 s is 0.05.

**Linear falloff never separated anyone in a fight.** A running player pushes a footstep every 0.3 s and
a shooting one pushes an event every shot, so every entity making noise sat within a few percent of 1.0
and the cone term alone decided the vote. The exponential makes a contact that went quiet a second ago
worth half of one that is still talking, which is the separation the vote needs.

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

Within the winning tier every live entry is scored per sense, and the entry keeps its best sense:

    score = max over s of  Falloff(now - sourceTime[s]) * (s is DAMAGE or SIGHT ? 1 : ConeWeight(vantage, dir))

and the **single highest scorer wins outright**. There is no vector sum, no mean-shift refinement and no
degenerate-mean fallback any more -- all three existed to make averaging survive threats on opposite
sides, and an argmax does not have that problem to solve. `authority` is the winning sense's recency
alone, so the cone data decides *which* entry the bot looks at and never how hard it commits to
looking: a fresh contact from an unwatched direction still commands a full turn.

**Damage and sight skip the cone.** The cone is a prior on where threats usually come from, which is
the right discount for a sound. Being hit, or having traced someone, is not a guess, and discounting it
is how bots came to ignore a player shooting them in the back: the attacker scored 0.15 while any bot
running down the main cone scored near 1.0. There are no per-sense multipliers on top -- sound and
gunfire are already held back by the cone, and that turned out to be enough of a handicap.

**Each entry carries a time per sense, `sourceTime[SUSP_COUNT]`.** There is still one entry per entity
and `time`, `source` and `pos` still mean "the latest push", so every other reader is untouched. The
array exists because the attacker's own gunshot lands on the same entry a frame after each hit: with only
`source`, the hit would be overwritten by an event and fall back under the cone every other frame. `Push`
zeroes the array whenever a slot changes owner.

`ConeWeight` reads the bot's blended vantage signature (`nav_vantage.cpp`), which says which cones
everything with line of sight to this spot arrives through:

    rel  = watch[k].amount / watch[0].amount        // watch is amount-descending, so rel in (0,1]
    best = max over k of  rel * max(0, dot(dir, watchDir[k]))
    return kVantageConeFloor + (1 - kVantageConeFloor) * best

**Normalising against the strongest cone, not the sum, is what keeps the scale honest.** A dead-on hit on
the main cone scores 1.0 whether the spot has one cone or four. Dividing by the sum would cap a four-cone
spot at 0.25 and the floor would swamp every real hit on it.

`kVantageConeFloor = 0.15` is the whole reason this is a bias and not a blindfold. An entry the map says
nothing ever arrives from still keeps 15% of its recency weight, so a noise from an unwatched direction
can still win once everything down the main sightline has gone quiet. It no longer carries the
shot-in-the-back case -- damage skips the cone for that -- and on its own it never could: 15% of a fresh
hit loses to any contact down the main cone younger than about 2.9 s.

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

## bot_daze.cpp

Flash grenades and concussion grenades, both learned the way a real client learns them: from the user
message the game DLL addresses to the player. Nothing else in the plugin reads network messages.

### The message hook

`engine_hooks.cpp` hooks `pfnRegUserMsg`, `pfnMessageBegin`, `pfnWriteByte`, `pfnWriteShort` and
`pfnMessageEnd`, all post. `pfnRegUserMsg` records the ids of `ScreenFade` and `Concuss` as they are
registered. The engine does not dedupe by name alone: under valve+cstrike+tfc, `ResetHUD`, `Battery`
and `StatusValue` each hold two ids, apparently where the games disagree on size. `ScreenFade` (96)
and `Concuss` (156) come out as one id each. A game registering `ScreenFade` with a different size
would overwrite the stored id and blind only that game's victims; `crossbot_usermsgs` shows it.
`MessageBegin` arms capture only for `MSG_ONE` / `MSG_ONE_UNRELIABLE` to one of our bots, and only for
those two ids. Every write hook returns on a single branch while capture is disarmed, which is the
whole cost on the common path.

**The recipient test is `BotManager::FindBot`, never `FL_FAKECLIENT`.** CS wipes that flag on every
spawn (`doc/games.md`, *Rules*), so the first version of this hook processed a cstrike bot's
flashbangs for its first life and silently dropped every one after its first respawn -- while
`crossbot_daze_test`, which bypasses the hook, kept passing. The id compare runs first -- two integer
compares against the two cached ids -- so every message that is not one of ours is rejected before
`FindBot` walks `g_bots`.

Only bytes and shorts are hooked because those are the only field types the two messages use. A third
message with any other field type must hook those writers too, or its field indices will be wrong.

The ids come only from the hook, so the plugin must be loaded before the game DLLs register -- true for
`plugins.ini` at startup, false for a later `meta load`. `crossbot_usermsgs` says which case applies.

The handlers touch `CBot` fields and `gpGlobals->time` and nothing else, so they need no game context.

### ScreenFade

10 bytes: `duration` and `holdTime` as 4.12 fixed-point shorts (`seconds = value / 4096`),
`fadeFlags`, then r, g, b, a. The hook sees the `int` the game DLL passed, before the engine truncates
it, and masks to 16 bits.

A fade blinds only when `a >= 200` and `FFADE_MODULATE` is clear. 200 is CS's minimum flash alpha
(`RadiusFlash` sends 255 facing the grenade, 200 facing away), and it is what keeps HL's charged-gauss
self-fade -- orange, alpha 128 -- from blinding every bot that fires one. A map `env_fade` at full
alpha does blind bots; the screen really is covered.

Blindness lasts `kBlindScale * duration`, 0.33: ReGameDLL's `PlayerBlind` calls
`Blind(fadeTime * 0.33, ...)` for its own bots, so a bot is blind for a third of the time a human's
screen stays white. Point blank and facing the grenade that is 12s of fade and 4s of blindness. A
second flash extends to whichever ends later. That formula is `IGameProfile::BlindSeconds`'s default,
and it is looked up on the *bot's* game (`XPlay::ProfileFor(bot.gameDll)`), because the handler runs
outside any game context. DoD overrides it to `hold + scale * duration`. Its only full-alpha fade is a
headshot's (alpha 200, dark red, 3 s fade after a 2 s hold, dod.c:86590), and the hold is time the
screen really is covered, so a headshot victim stays blind for about 3 s.

While blind, `BotSense::Update` skips `UpdateVision` and clears `target`, `targetBone`, `visionCands`
and `visionKeep`, emptying the delay queue every frame -- the same cleanup as
`crossbot_sense 0` except that **`sense.enemy` survives**. A flashed bot still knows someone was
there. Sound, events and damage reach the suspicion list outside `UpdateVision`, so the bot keeps
turning toward noise.

`BotCombat::Run` also drives while blind, with or without an open fight. The exposure shuffle never
reads the enemy, so a blind bot drops its goal and walks toward cover exactly as a fighting one does,
and `DuelDrive` declines for want of an enemy. `combat.active` is not touched, so
`crossbot_combat_status` still answers whether there is a fight. When blindness ends with no fight,
`Run` releases the drive and zeroes `nextSelectTime`, as `Exit` does. Under `crossbot_combat 0` the
blind drive is off as well, because `Update` calls `Exit` every frame there and the goal would be
dropped again on each one.

**The queue is emptied every frame of blindness, not just on the edge.** `BotFire`'s `haveTarget` is
`BotAim::ReadPredicted`, and that falls back to extrapolating from the newest queued slot for up to
`kPredictHorizon` (1 s) whenever `predictLatched && combat.active`. Clearing only when
`sense.target` was live -- the first version -- left every bot flashed in the gap between sightings
with its slots intact, so it kept firing accurately for over a second after going blind. With the
queue emptied each frame nothing can serve a target, `BotFire` reads `no-target`, and the shooting
stops for the whole blindness. `BotFire` itself is still untouched; it simply has nothing to aim at.

### Concuss

One byte. A TFC client stores it in `v_idlescale`, and `V_AddIdle` adds
`scale * sin(t * cycle) * level` to the rendered view: pitch cycle 1 level 0.3, yaw cycle 2 level
0.3, and roll, which a bot does not have, dropped. `Bot_UpdateFacing` adds the same to its desired
angles before the controller runs, so the hand controller smooths it into a wander and
`Bot_DecomposeMove` makes the bot walk crooked for free. `t` is absolute time, as on the client.

TFC's server, decompiled: `TeamFortress_Concuss` sets the value to 100 for 15s, or 7s for a medic;
each think recomputes `100 * remaining / duration`, and a message goes out whenever that integer
changes. About 100 bytes arrive, the last one zero, and the bot tracks the latest -- there is nothing
to model. At 100 the sway is ±30 degrees on both axes.

On a client the sway is cosmetic and the real aim is untouched; a human misaims because what they see
is moving. Applying it to the bot's actual angles is the same miss, taken directly.

Respawn resets the whole state.
