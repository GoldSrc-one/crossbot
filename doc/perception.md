# Perception: sensing and aiming

How the bot sees and where it looks: the aim hand and its delay queues, hearing, vision, suspicion, and
being dazed. What it does about what it sees is `doc/combat.md`; what it wants otherwise is
`doc/goals.md`; the per-tier numbers are `doc/difficulty.md`.

## bot_aim.cpp

Everything the bot looks at goes through one function. `bot.desiredAngles` is the request, written by the
drive branches in `bot_move_nav.cpp`, the camp, fun, deploy, break and directive runners, and
`BotSense::ApplyLook`.
`Bot_UpdateFacing` is the only per-frame writer of `pev->v_angle` (`TeleportAllBots` resets it), and every
drive branch calls it exactly once per frame and threads its result into both `Bot_DecomposeMove` and
`pfnRunPlayerMove`.

Aim is therefore decoupled from movement: the wish direction is world-space and is reprojected onto
whatever yaw was actually commanded that frame, so the view can point anywhere without changing where
the bot walks.

### Hand space, not angle space

The controller replaced a flat 300 deg/s turn that started at full speed and stopped dead, the most
obvious tell that a bot was not a player. It models a hand on a mouse instead.

The integrator state (`BotAim::State::yawVel`, `pitchVel`) is **hand** velocity, in degrees per second at a
reference 90° FOV, not angular velocity. Angles convert with `s = fov / 90` (`pev->fov <= 0` means
default, the normal case).

This makes zoom correct for free. A scope going from 90 to 20 means the same hand movement covers 4.5x
less angle, so the same turn needs 4.5x more hand travel and takes longer. When the scope snaps, hand
velocity is untouched and the angular velocity drops by itself. An angular state would have needed a
special case for the velocity discontinuity.

### The control law

Per axis, time-optimal with a saturating acceleration:

    step     = aMax * dt
    envelope = sqrt(2 * aMax * |e| + step^2 / 4) - step / 2
    v_target = sign(e) * min( |e| / dt, envelope, kMaxHandSpeed )
    a        = clamp( (v_target - v) / dt, +/-aMax )
    v += a * dt ;  angle += v * s * dt

The envelope is the whole model: the speed of a body that decelerates at `aMax` and arrives with zero
velocity. Following it gives accelerate-then-brake with no phase switching: far from the target the
clamp saturates, near it the envelope brakes.

**The step correction makes it frame-rate independent.** The continuous optimum `sqrt(2 * aMax * |e|)`
assumes braking starts the instant the envelope is touched, when in fact it starts up to a frame later.
At 2880 deg/s² it overshoots a 180 by 3.6° at 100 fps, 5.9° at 60 and **12.4° at 30**, which makes aim
accuracy a function of server load (the error scales with `aMax * dt`). The `+ step^2 / 4` and
`- step / 2` invert the *discrete* stopping distance instead: overshoot stays below 0.7° at every frame
rate tested, and convergence is faster (a 180 at 60 fps takes 0.467 s against the continuous form's
0.617 s at 2880) because nothing is spent recovering. Deliberate aim error has to be the only source of overshoot, or it cannot
be tuned.

**`|e| / dt` is a dead-beat clamp**: any error small enough to close in one step does. Deleting it is the
obvious way to make the bot less surgical, and it does not work. With the step correction the envelope
linearises near zero to `2e/dt`, twice the dead-beat speed (`sqrt(2ae + s^2/4) - s/2 <= 2e/dt` with
`s = a*dt` reduces to `0 <= 4e^2/dt^2`), so a step lands on `-e` and no further. There is no limit
cycle; without the clamp the ring-down stalls around `aMax * dt^2 / 8`: 0.056° at 100 fps, 0.156° at 60,
1.41° at the `kMaxFrameTime` clamp. That is frame-rate coupling again, so the clamp stays.

**`kMaxHandSpeed` is the hand-speed ceiling**, the knob for a hand slower than its acceleration allows.

### Where the constants come from

These are the normal tier; `turnScale` (`crossbot_turn_scale`) multiplies all three.

| constant | value | why |
| --- | --- | --- |
| `kMaxAccelYaw` | 3000 deg/s² | the CS zbot's hand-tuned figure |
| `kMaxAccelPitch` | 2000 deg/s² | vertical travel is the awkward direction for a wrist |
| `kMaxHandSpeed` | 520 deg/s | `sqrt(3000 * 90)`: a symmetric turn peaks at `sqrt(a * t)`, so a 90° yaw flick just touches it |

**The ceiling clips on purpose.** Anything over 90° cruises at 520 in the middle, so a 180 takes about
0.52 s rather than the 0.35 s pure acceleration would give, and time grows linearly past 90° instead of
with the square root. A 90° yaw takes about 0.35 s, a 90° pitch about 0.42 s (peak 424). Slow big
turnarounds are the human shape: a wrist re-centres, it does not spin faster.

**The cost is at the aim gates.** Below the crossover `4 * 300^2 / aMax` (120° at 3000) the controller is
*slower* than a flat 300 deg/s limit, because it spends the start of the turn accelerating: a 6°
correction goes from 20 ms to 73 ms. Two places zero drive until the view has converged and so feel it:
the ladder gate (`kLadderAimTolerance`, 6°) and the activation gate (`kActivationAimTolerance`, 8°).

`test10_doorbutton` is the map to watch on the nav sweep. Goal-arrival time over isolated repeat runs:

| controller | 8.1 s | 11.1 s |
| --- | --- | --- |
| old flat 300 deg/s | 8 of 8 | 0 |
| aMax 2880 (180 in 0.5 s) | 0 | 2 of 2 |
| aMax 4500 (180 in 0.4 s) | ~7 of 16 | ~9 of 16 |

`test_nav_goal.ps1` polls arrival on a ~3 s grid (2.0, 5.0, 8.1, 11.1), so the table shows which side of a
poll the bot lands on, not how much time it lost; the real slowdown is well under a second. Today's 3000
puts the 180 back past 0.5 s. Every run of this map, on every controller, logs exactly one
`mid-route repath (stalled)` before the button and one `avoid side func_door_rotating`; neither comes from
the aim model or separates fast runs from slow.

### Edge cases

- **`dt` is clamped to [1/1000, 1/20]**, the same range `Bot_Physics` uses. A second-order integrator has
  memory, so a raw `gpGlobals->frametime` hitch would feed a huge `dt` into both the acceleration and the
  position step and fling the view.
- **Pitch is clamped to ±89** (`kPitchLimit`) and `pitchVel` is zeroed at the stop. Otherwise velocity
  winds up against the limit and pins the view for several frames after the target comes back in range.
- **Resets.** `TeleportAllBots` zeroes both velocities with the `v_angle` reset for the same reason.
  Respawn is covered by `bot = CBot()`, since `BotAim::State` is default-initialised.

### crossbot_aim_test

`crossbot_aim_test <bot> <yawDelta> [pitchDelta]` commands a flick relative to the bot's current view and
prints the time to settle within 0.5° (`kTestSettleTolerance`) and the peak hand speed. It is the only way
to check the turn times against the running server.

- **It is an override**, not a write to `desiredAngles`, which navigation rewrites every frame. It releases
  after 3 s and the bot returns to whatever it was looking at.
- **Settle also requires hand speed below 150 deg/s** (`kTestSettleSpeed`). With gain overshoot a view
  flying through the target at 200+ deg/s can sit inside the band for one frame; genuine arrival runs
  about 50 deg/s at typical frame rates. It is a threshold, not a proof: a small, slow overshoot can slip
  under it, so read the spread over many flicks (`scripts/test_aim_error.ps1`).

Expected at the normal tier with `crossbot_aim_gain 0`: `180 0` settles in about 0.5 s peaking at 520
(the ceiling), `90 0` in about 0.35 s just under 520, `0 90` in about 0.42 s near 424. The spread is the
frame rate; peak hand speed is set by the constants and is the better number to check. The gain mismatch
applies during the test, so pin it to 0 to check the controller alone.

### The reaction delay queue

The controller is the motor half. The queue is the perception half: the bot acts on what it saw one
reaction time ago, never on the present frame. `BotAim::Delay(bot)` is rolled per engagement from the
difficulty tier's range (`doc/difficulty.md`), or is exactly `crossbot_aim_delay` when that is pinned
(≥ 0), and is clamped to [0, 0.5] (`kMaxDelay`). Human simple visual reaction is roughly 200-250 ms; 0.5 s
is choice-reaction territory and reads as sluggish.

`BotAim::State` carries a 256-slot ring (`kDelaySlots`) of `{ time, point, velocity }`: 128 slots at
250 fps span only 0.512 s, right at the maximum delay.

**The write extrapolates only the target:** `push { now, targetPoint + targetVelocity * delay }`, and only
if that lead point traces clear (see "The lead point is traced before it is queued").

**The read** takes the newest slot older than `now - delay`, interpolates it with the next newer one so the
served delay is exact rather than quantised to a frame, and only then converts to angles, from the bot's
**actual current eye**, in `ApplyLook`.

The extrapolation is the point. At `T` the bot records where the target will be at `T + delay`, and reads
it back at `T + delay`: at constant velocity it is aimed exactly at the target despite acting on old
information. Only *acceleration*, a change of direction, produces error, which is the human signature: a
good player tracks steady movement and is beaten by the direction change, not the speed.

The bot's own motion is **not** extrapolated, and neither is its view velocity. Where it stands is known
exactly at read time, and turning does not move the eye.

### The two queue edges are the behaviour, not error handling

- **Underflow.** A freshly acquired target has nothing older than `delay`, so the read fails and the bot
  keeps whatever `desiredAngles` navigation set: it sits still for a full reaction time and *then* flicks.
  Falling back to the oldest slot would snap to truth instantly, zero reaction time. The silent quarter
  second is the most human artifact in the system, for free.
- **Drain.** On losing sight the bot stops writing but keeps reading, so it tracks a vanished target for
  exactly `delay`, then stops, because its input is that stale.

The ring is cleared on acquiring a *different* target (`best && best != sense.target.Get()`), on teleport
(`TeleportAllBots`) and on respawn (`bot = CBot()`). Never clear on *losing* a target: losing is a
difference too, and clearing then wiped the ring on the frame sight broke, so there was no drain at all.
Reacquiring the same enemy after a gap still clears, because the handle went null in between, so the
reaction delay is paid every time.

### What feeds the queue

`BotSense::Update`, once per frame from `Bot_MoveNav` (see `## bot_sense.cpp`). The contract: one
`PushDelaySlot` per frame while a target is held *and its lead point is clear*, extrapolated by the
delay, and `ClearDelayQueue` on acquiring a different target.

- **The pushed point is a hitbox bone**, re-picked at random every frame, not the eye. The samples jitter
  over the body and the acceleration-limited controller averages them, being a low-pass filter already,
  so the head settles near body centre and a partly hidden enemy stays trackable.
- **`ClearDelayQueue` keys on the entity, not the bone.** Keyed on the sample point it would clear every
  frame and destroy the delay.
- **Slots hold velocity too**, interpolated the same way, for `BotFire`'s projectile lead. It needs the
  *perceived* velocity: a lead from live velocity would give a bot with a reaction time perfect
  prediction. `ReadDelayed` takes it as an optional out pointer. See `## bot_fire.cpp` in
  `doc/combat.md`.

### Why slots hold points and not angles

Angle slots had to bake in a guess of where the *bot* would be a reaction time later, which breaks when
the bot jumps, is stopped by `BotAvoid`, rides a lift or repaths within the window. The result is parallax
error, worst at close range where aim decides fights, and it lands in `err` mixed with genuine target
misprediction so neither can be read.

A point slot deletes that term: the eye enters once, at read time, at its true value. Where the angle
form's guess was right the two agree; where it was wrong, only the angle form was wrong. The cost is a
`LosAngles` call moved from write to read and a three-float lerp instead of a two-float one with yaw
wrap. `LosAngles` can now fail at read time, when the recalled point sits on the bot's own eye; `ApplyLook`
then falls through to the suspicion blend and leaves `servedDelay` at zero rather than claiming a serve.

### The bot sees its own view late

The target queue makes the bot see the *enemy* late. The view queue makes it see its own view late: the
controller and the trigger act on what the bot saw of its own view `BotAim::Delay(bot)` ago. The only
deliberate motor error is the gain mismatch below (the earlier hand noise, a mouse-count tremor, an idle
sine drift and a random brake multiplier, is gone).

**A pure delay cannot drive this controller.** Feeding `view(t - d)` to `StepAxis` breaks the dead-beat
term: a 6° error closes in one frame, the stale view keeps reporting 6° for a quarter second, and the hand
flicks another 6° every frame. A slow proportional controller would survive (stable while
`gain * d < pi/2`) but a 180 would take most of a second.

**So the bot keeps an efference copy**, as a brain does: it knows what it told its hand, and learns from
its eyes only what it did *not* command. Per frame, in `Bot_UpdateFacing`:

    own      += commanded step (post pitch clamp, pre AdjustView)
    push       { now, off = v_angle - own, punchangle }
    read       off, punch at now - delay  (interpolated, yaw the short way round)
    seen      = own + off
    error     = desired - seen ;  step applied to the real v_angle

`off` is the disturbance, everything that moved the view without the hand. With none it is constant and
`seen == v_angle`, so turning, the nav gates and `crossbot_aim_test` are untouched at any delay. What the
bot sees late is `AdjustView` (DoD recoil kick, scope sway, bipod clamp), anything else writing `v_angle`,
and `punchangle`.

- **The slot stores the offset**, not view and copy separately: differencing two absolute accumulators
  needs an unwrapped yaw, which loses float precision over a long life. The offset changes only on
  disturbances, so both stay wrapped.
- **The pitch clamp counts as commanded.** The stop is on screen, so the bot knows; as a disturbance it
  would believe it was still pitching past 89 for a quarter second.
- **The trigger reads it too.** `Bot_RunPlayerMove` passes `aim.perceived` (seen plus this frame's step)
  to `BotFire::Buttons`, and `RayOnTarget` / `ArcOnTarget` add `aim.perceivedPunch` instead of the live
  `punchangle`. A spraying bot keeps firing on a crosshair that has already kicked off target and stops a
  reaction time later. Movement does not: `Bot_DecomposeMove`, the nav aim gates and `AimWithinCone` use
  the real view, being physics and stance.
- The ring is cleared in `TeleportAllBots` and on respawn, as the target ring is.
- Recoil *compensation* is not modelled: the controller ignores the punch. The perceived punch is what a
  compensation step would read.

### The efference copy is wrong on purpose

With a perfect copy, `own` advances by exactly what the hand did and the bot never misses except through
the target queue: a Smith predictor with a perfect plant model. A real predictor's error is model
mismatch, and that is what this adds. On each flick both axes draw a gain, and the hand moves
`step * gain` while `own` advances by the commanded `step`:

    gain  = 1 + s * (u * 1.5 - 1),   u uniform [0, 1)   ->   [1 - s, 1 + s/2]
    v_angle += step * gain ;  own += step

The difference lands in `off`, the channel the view queue already carries. The bot believes it arrived,
sees the miss one reaction time later, and corrects. That is a real undershoot, which shaping the brake
cannot produce (braking early only lands softly): the aim point is right and the hand's belief is wrong.
It is proportional to the movement (Fitts' shape), the correction draws its own gain, and the loop
converges because each reaction time multiplies the remaining offset rate by `-(gain - 1)`.
`kMaxGainSpread` caps `s` at 0.5, well inside `|gain - 1| < 1`.

- **Biased low**, because the primary submovement of a human aimed movement usually undershoots. `s` is
  `crossbot_aim_gain` (easy 0.25, normal 0.15, hard 0.10, expert 0.05), so a normal bot can land a 90°
  flick up to 13.5° short or 6.75° long.
- **A flick is what `UpdateGain` detects**: combined error above `kFlickMinError` (3°) *and* grown by more
  than `kFlickGrowth` (0.5) since last frame. A converging movement's error falls monotonically, so it
  never re-triggers; a new target does, and so does the bot's own miss pushing the error back up, so the
  correction gets an independent gain. The axes draw independently, since humans miss to the side as
  well as along the movement.
- **Only with a target, or during `crossbot_aim_test`.** Gains reset to 1 otherwise: the nav aim gates
  hold drive until the view converges, so a mismatched nav turn would cost a reaction time at every ladder
  and button.
- **The pitch clamp is still commanded**: on a frame that hits the stop `own` advances by the real step,
  so a stop overshot through gain does not leave the bot believing it is still pitching.

`crossbot_aim_status` prints the pair. A frozen value on a bot idling on a still target is correct: the
error never grows past 3°, so it never redraws.

Measured with `test_aim_error.ps1 -Gains 0,0.15`, normal tier, eight flicks per cell, crossfire:

| gain | flick | settle min | avg | max | peak |
| --- | --- | --- | --- | --- | --- |
| 0 | 180 0 | 0.479 | 0.485 | 0.494 | 520 |
| 0 | 90 0 | 0.308 | 0.339 | 0.487 | 512 |
| 0 | 0 90 | 0.041 | 0.340 | 0.389 | 382 |
| 0.15 | 180 0 | 0.626 | 0.785 | 1.012 | 522 |
| 0.15 | 90 0 | 0.291 | 0.593 | 0.820 | 515 |
| 0.15 | 0 90 | 0.489 | 0.780 | 0.908 | 417 |

The mean rises by about one reaction time (normal's 0.30 s), the correction submovement, and the resting
error stays 0. The pitch row's low minimum and peak are the 89° stop cutting the flick short when the test
starts from a non-level view.

### crossbot_aim_status

Per bot: the sense target, the bone it is sampling, ring occupancy and history span, the delay actually
served, hand speed, `err`, `seen`, `perr`, `gain` and `leadblk`.

- **`err`** is the angle between the real `v_angle` and the true present-time angles to the target. It
  separates "correctly delayed" from "delayed and also wrong", which look identical when spectating. At
  delay 0 it is the controller's own tracking error. At a nonzero delay it should stay flat while the
  target moves at constant velocity and spike only on reversals; a large `err` under steady motion means
  the extrapolation or the read index is wrong.
- **`err` has a floor that grows as range closes.** `AnglesToTarget` measures to the eye while the bot aims
  at a random hitbox bone, every bone sits below the eye, and the same spread subtends more up close. At
  point blank, where `test_sense.ps1` converges its bots, it is tens of degrees and almost pure negative
  pitch. That is the bone sampling working. Read `err` for its *trend* across a delay sweep at fixed
  range, never its value.
- **`seen`** is the bot's belief about its own view. **`perr`** is the angle between
  `seen + perceivedPunch` and `v_angle + punchangle`: 0 whenever only the hand moves the view, at any
  delay. With a target and `crossbot_aim_gain` above 0 it is the mismatch not yet seen, draining to 0
  within a reaction time of the view coming to rest. Under fire it is nonzero even at delay 0, because the
  punch it compares against is one `pfnRunPlayerMove` newer. Nonzero at idle with no recoil means the
  efference copy is missing a step.
- **`gain`** is the pair drawn on the last flick.
- **`leadblk`** is the lead-veto count ("The lead point is traced before it is queued").

Measured on crossfire, two bots converged on one spawn and idling:

| `crossbot_aim_delay` | served | ring |
| --- | --- | --- |
| 0.000 | 0.000 | fills to 256/256 |
| 0.250 | 0.250 | fills to 256/256 |
| 0.500 | 0.500 | fills to 256/256 |

`served` matching the request exactly is the interpolated read working; without the lerp it would
quantise to frames and drift with frame rate. A `served` of 0.000 at a nonzero delay is the underflow
silence, caught in the second after a re-acquire. The static phase proves the ring, the read index and
the interpolation, not the prediction: still bots have no velocity to extrapolate. Constant-velocity
tracking is checked by hand (`doc/testing.md`, `scripts/test_sense.ps1`).

### The RNG is per bot and seeded from the entity index

`RandomUnit` is a plain LCG on `BotAim::State`, not `RANDOM_FLOAT`. It does not consume the engine RNG,
which the game uses for weapon spread, and it is seeded from `pfnIndexOfEdict` alone, so a bot draws the
same sequence on every run of a map and the nav sweep stays reproducible (`doc/testing.md` has enough
run-to-run noise already).

## bot_sense.cpp

What the bot knows about the world and how it found out. Three inputs (vision; heard sounds and playback
events; damage taken) feed one per-bot table of suspicions, and two outputs come out: an aim target
pushed into the delay queue, and a look direction for when there is no target.

`crossbot_sense` gates the whole thing; `crossbot_sense_decay` (default 5 s) is the only other knob. The
view cone (90°) and range (`kViewRange`, 4096) are constants: geometry, not policy.

### The engine hooks

`pfnEmitSound` and `pfnPlaybackEvent` are hooked in the engine **post** table (`engine_hooks.cpp`) and
return `MRES_IGNORED`. Post, as for item touches, because a pre hook also fires when another plugin
supersedes the call and the sound never plays.

- **Metamod must be handed the tables.** `meta_api.cpp` fills slots 7 and 8 of `META_FUNCTIONS`
  (`GetEngineFunctions`, `GetEngineFunctions_Post`); with them `NULL` the hooks are dead code. If a sound
  hook seems silent, check those first.
- **Both tables are `memset` plus assignment by member name**, never ~160 positional `NULL,` entries
  against a 158-member struct (`pfnEmitSound` is member 29, `pfnPlaybackEvent` 123).
- **No deferred queue.** The hooks fan out through `BotManager::OnSound` / `OnEvent`, which loop `g_bots`
  inline; per bot it is a distance compare and a table push. (foxbot's `BotSoundSense` instead runs
  waypoint route queries inside the engine callback.)

### Sounds carry no position, and the origin is not v.origin

`pfnEmitSound` passes an entity and nothing else spatial. The engine places the sound at the emitter's
bbox centre, so `EntitySoundOrigin` does the same, `origin + (mins + maxs) * 0.5`. For brush entities
(doors, lifts, breakables) `v.origin` is `(0,0,0)`, which would put every door at the map origin.

Audible radius follows the engine's spatialisation, inaudible past a 1000-unit clip distance
(`kSoundClipDist`) scaled by attenuation:

    attenuation <= 0  ->  audible everywhere (ATTN_NONE)
    otherwise         ->  radius = 1000 * volume / attenuation

`ATTN_NORM` (0.8) at full volume is about 1250 units, `ATTN_STATIC` (1.25) 800, `ATTN_IDLE` (2.0) 500.
GoldSrc does not occlude sound, so a distance test is faithful and needs no trace.

`sample` is never inspected. Two filters, chosen from a dump of real crossfire traffic: drop
`volume <= 0`, and drop `common/null.wav`. Together they catch every stop-sound and silence command, a
quarter of all traffic. **`CHAN_STATIC` is kept**: HL brush entities emit door and lift sounds on it, among
the few interesting world sounds on a deathmatch map.

### Footsteps arrive through pmove, not through pfnEmitSound

Footsteps are emitted server-side, but by a road `enginefuncs_t` cannot reach. `SV_StartSound` has two
entries: `pfnEmitSound`, and `PM_SV_PlaySound`, which the engine hangs off `pmove->PM_PlaySound`.
Everything pmove emits goes the second way: footsteps, ladder steps, water wade and slosh, jump landings,
fall grunts. (`PM_PlayStepSound`'s `!pmove->runfuncs` early-out never trips on a server: `SV_SetupMove` sets
`runfuncs = TRUE` right before `pfnPM_Move`.)

So `pfnPM_Move` is hooked pre and post: pre saves `ppmove->PM_PlaySound` and swaps in a trampoline, post
restores the original. The trampoline calls the original, then feeds `BotManager::OnSound` with the moving
player's edict, so footsteps are ordinary `SUSP_SOUND` entries under every existing filter.

- **There is no persistent install.** The engine reassigns `PM_PlaySound` on every `SV_RunCmd`, so the swap
  can only live inside `pfnPM_Move`. (jk_botti installs "permanently" and in effect re-installs every call.)
- **The identity guard `ppmove->PM_PlaySound != PM_PlaySoundHook` is load-bearing.** A double install saves
  the trampoline as its own original and the first footstep recurses until the stack dies.
- **`ppmove->player_index` is 0-based**; the edict index is `+ 1`. Off by one, every footstep is
  plausibly and wrongly attributed to the neighbouring player.
- The install is skipped when `crossbot_sense` is 0; the post restore is guarded on the identity check, so
  flipping the cvar mid-frame cannot strand the pointer.

`pm_shared` never calls `PM_PlaybackEventFull`, so `pfnPlaybackEvent` really is the only road for events.

### What footsteps are worth, without a single tuning constant

pmove already grades the volume, and `1000 * volume / attenuation` turns it into a hearing distance. Read
from `pm_shared.c` and confirmed against 85 live step sounds from two bots on crossfire:

| sound | volume | audible at (ATTN_NORM) |
| --- | --- | --- |
| running step, concrete/metal/tile/grate/slosh | 0.50 | ~625 |
| running step, dirt | 0.55 | ~690 |
| running step, vent | 0.70 | ~875 |
| ladder step | 0.35 | ~440 |
| wading into water | 0.65-1.0 | ~810-1250 |
| jump launch, and landing by fall speed | 0.5 / 0.85 / 1.0 | ~625-1250 |
| fall-pain grunt (`CHAN_VOICE`, hard landing only) | 1.0 | ~1250 |

Two engine rules come with it:

- **Walking is silent in multiplayer.** Steps at horizontal speed `<= 220` are dropped outright; foxbot
  approximates this by hand. A bot that wants to move quietly can just move slower (CS creeps at 130; see
  `doc/goals.md`, `bot_camp.cpp`).
- **`sv_footsteps 0` disables the whole sense** from outside the plugin, since the gate is inside
  `PM_PlayStepSound`. Check it first if footsteps stop.

A bot does not hear its own steps (`OnSound` skips the emitter), so the one-bot nav sweep is unaffected.

### Events come from pInvoker, never from the origin argument

`pfnPlaybackEvent`'s `origin` is `(0,0,0)` for essentially all weapon fire; the engine fills it from
`pInvoker->v.origin` when `FEVENT_ORIGIN` is unset.

- `pInvoker` must be a live player, and `FEV_HOSTONLY` (per-client viewmodel cosmetics) is dropped.
  `FEV_CLIENT` events never reach the server.
- No radius test: a gunshot is `LOUD_GUN_VOLUME` class and meant to carry.
- No `pfnPrecacheEvent` index-to-name table (jk_botti keeps one for per-weapon loudness): a player-invoked
  non-host-only event is essentially always weapon fire in HLDM, and nothing would use the refinement.

### Damage must be read before Bot_RunPlayerMove

The trigger is `pev->dmg_take != 0`; the position is `EntitySoundOrigin(pev->dmg_inflictor)`.

Ordering is the whole trick. `dmg_take` is zeroed by `CBasePlayer::UpdateClientData` inside
`PlayerPostThink`, and for a fakeclient only the bot's own `pfnRunPlayerMove`, the last act of its think,
runs that. So the field accumulates from the previous frame's `RunPlayerMove` to this frame's
`BotSense::Update`, and other players' bullets always land in that window because they fire during
*their* think. Read after `Bot_RunPlayerMove`, it is essentially always zero.

- `dmg_inflictor` is never cleared, so alone it answers "who hit me", not "was I hit"; read it only beside
  a non-zero `dmg_take`.
- Damage during the bot's own pmove (fall damage, a `trigger_hurt`) is cleared before the next read. It is
  self- or world-inflicted and the inflictor filter would discard it anyway.

### Vision spends at most two tracelines per bot

The scan pass spends no traces. It applies the gates (`Candidate`, range, cone) and **chooses** one
candidate by weighted reservoir (`acc += w; if (RandomUnit(...) * acc < w) pick = ent;`, the single-pass
idiom from `bot_fire.cpp`), weighted

```
(dot(theirViewDir, dirFromThemToUs) + 1) / dist
```

so the discretionary trace goes to someone close *and* looking at the bot: the one about to shoot. Then at
most two `dont_ignore_monsters` traces run:

1. **The incumbent**, if it passed the gates: `BotCombat::Enemy` when combat is active, else last frame's
   `sense.target`. The bot re-checks its current enemy before looking for another.
2. **The pick**, only with no incumbent or an occluded one.

With the lead trace below, the budget is 1-2 `dont_ignore_monsters` traces plus 0-1 world-only per bot.

**Why:** the alternative, one trace per in-cone candidate every frame, is expensive: `dont_ignore_monsters` makes
the engine walk the entity list (`pfnGetBonePosition` rode along at the same rate). The gates do not bound
it: 4096 units covers all of crossfire and the cone rejects only half the server. With 32 bots on
crossfire the mean in-cone count (`cands`) is **6.18** with a tail to 15, about 198 traces per frame;
`ZONE_SN_LOS` reads **51.6 traces/frame over ~30.5 active bots, 1.69 per bot**, lead trace included. The
tail is what it removes: a bot mid-firefight would pay 15.

- **The incumbent still passes `Candidate`, range and the cone.** Untested from outside the cone it would
  let a bot shoot an enemy directly behind it. A bot that turns away drops `sense.target`; combat's own
  five-second persistence carries the enemy across the gap, not vision.
- **`Incumbent` reads last frame's combat enemy**, since `BotSense::Update` runs before
  `BotCombat::Update`, which reads this frame's `sense.target`. The lag is deliberate and stable; the
  `sense.target` fallback covers the frame before combat has entered.
- **Use `pfnAngleVectors` for the target's facing, never `MAKE_VECTORS`**, which writes
  `gpGlobals->v_forward` while it still holds *this* bot's forward and would rotate the bot's own cone to
  each candidate in turn. And read `v.v_angle` without flipping pitch: unlike `pev->angles` (which
  `bot_deploy.cpp` negates for a tripmine) it is already the true view angle; flipped, every bot would
  prefer targets facing away.

`crossbot_sense_status` reports `cands` (`visionCands`: players that passed range and cone, what the old
scan would have traced) and `keep` (`visionKeep`: which of the two traces won this frame). `keep` is one
frame, so the cumulative `keeps=inc:,new:` tally carries the claim across polls: during a duel it should read incumbent nearly every frame,
and a run of fresh picks means the lock is broken.

Consequences, all intended:

- **`sense.target` means "the one I traced and saw"**, not "nearest visible". A bot keeps fighting a
  mid-range enemy while someone closer runs past, which reads as commitment instead of snap-to-nearest
  twitch. `bot_fire`, `bot_combat`, `bot_fun` and `bot_deploy` inherit it.
- **`SUSP_SIGHT` is pushed for at most one entity per frame.** The active enemy is the incumbent, traced
  every frame it is in the cone, so `BotSense::Senses` keeps combat open; everyone else enters the table
  through sound, damage and events.
- **Acquisition is probabilistic.** A bot facing N enemies draws one per frame, so first contact takes a
  frame or two: invisible at 100 Hz, about 0.1-0.2 s at `sys_ticrate 10`. With four bots and combat off,
  bots hold a target on 27% of frames, and `keep=new` is well under 1% of samples, because a pick wins only
  on the first frame of an engagement. A target lost mid-commit runs a grenade throw to its 1.5 s safety
  cap, so `test_fire.ps1`'s zero-tolerance `capped` check on `weapon_handgrenade` is the assertion most
  likely to feel this (once in three runs, one cap per 35 throws).
- **`(dot + 1)` gives an exactly back-turned target weight zero**, for *acquisition only*: once acquired,
  the incumbent path traces regardless of facing. The residual is spotting someone running directly away
  in silence, which footsteps generally cover. The tuning point, if needed, is an additive floor in
  `gaze`; it deliberately has no cvar.

### Teammates never reach the trace budget

`Candidate` rejects anything `Teams::Attackable` says this bot may not shoot (`doc/games.md`,
`teams.cpp`; a no-op on a deathmatch map). Gating here is both the behaviour and the cheap arrangement: a
rejected player never enters the reservoir, never costs a trace, never becomes `sense.target`. Everything
that shoots reads `sense.target` or `sense.enemy`, so one gate covers `bot_fire.cpp`, the ambush, the duel,
the cover shuffle and the func_tank in `bot_fun.cpp`.

- **Unknown is rejected too.** A player whose team reads zero is not shot at while teams are active. The
  case where nobody has a team is handled a level up, by `Teams::Active` falling back for the whole bot.
- **Teammates never reach the suspicion table.** `Push` drops anything `Teams::Of` calls `REL_FRIEND`, for
  every source. Recorded, every friendly footstep landed in the PLAYER tier and outranked everything in
  `LookDirection` (on fy_snow the gaze spent its life on teammates), and `BotGauss::Scared` counted a
  nearby friend as a threat.
- **The filter is `== REL_FRIEND`, never `!Attackable`.** A door, lift or breakable reads `REL_UNKNOWN`,
  which `Attackable` rejects while teams are active, so the `Attackable` form would silently delete every
  world sound on a team map. `REL_FRIEND` matches only a confirmed teammate or what one owns (grenades,
  satchels, snarks inherit the owner's team through `TeamOf`, as do the bot's own). On a deathmatch map,
  with `crossbot_teams 0`, or before the bot has a team, `Of` answers `REL_UNKNOWN` and nothing is dropped.

The table drives an attack in one place, the satchel, which `NearestLive` gates separately. Camping's
friend-in-view calm reads teams straight from the player list and never aims or shoots.

### Teammate hurt (`SUSP_ALLY`)

A teammate losing health is the one way a friend reaches the table, and it enters as the *attacker*.
`BotSense::RefreshAllies` runs once per frame beside `RefreshMonsters`, compares each client's `v.health`
with last frame's, and keeps the victims whose health fell (a respawn raises it, so never counts). Each bot
then runs `UpdateAllyHurt`: for every hurt `REL_FRIEND` other than itself, it resolves the attacker with
`BotCombat::Attacker` (the half of `DamageSource` that needs no `dmg_take`, safe only beside a real health
drop) and pushes it at the **victim's** position.

- Fall damage, `trigger_hurt` and friendly fire resolve to no attacker and push nothing.
- No range or line-of-sight gate: every teammate feels it.
- It is a coned source like sound, so `ConeWeight` scales it. It steers the gaze only; combat still needs
  `sense.target`. `crossbot_sense_status` reports it as `ally:`.

### Monsters are candidates only while they are hunting us

A second loop runs after the client loop over a **shared per-frame monster list**:
`BotSense::RefreshMonsters` scans the edicts once per frame in `BotManager::StartFrame` and keeps the first
64 (`kMaxMonsters`) that are live, have private data, carry `FL_MONSTER` and are damageable. One scan per
server, not per bot, which is the point of the list. (Before it, a monster could never become
`sense.target` or `combat.enemy`, and bots walked past snarks that were eating them.)

**The gate is one question: is its `CBaseMonster::m_hEnemy` us?** `MonsterCandidate` asks
`g_pGameProfile->MonsterEnemy(ent) == bot.pEdict` plus a liveness test. A snark hunting someone else is
ignored, and so is every tripmine, satchel and grenade: `CGrenade` descendants with `FL_MONSTER` and a null
`m_hEnemy`. The bot answers threats to itself and does not go hunting.

**With teams on, a hostile monster need not be hunting *us*.** `MonsterCandidate` asks `Teams::Of` first:
another non-zero team is a candidate outright, our team never is, and only `REL_UNKNOWN` (every monster on
a deathmatch map, and in any mod that leaves `pev->team` alone) falls back to the `m_hEnemy == us` rule.
An owned monster inherits its owner's team, so an enemy's snark is engaged and a teammate's is not.

**A null `m_hEnemy` is still a hard reject.** Grenades, satchels and tripmines inherit a team from
`pev->owner`, so the team test alone would shoot enemy ordnance on the floor. Requiring `MonsterEnemy` to
name *someone* keeps them out, since only a real `CBaseMonster` sets it. The rule is "a hostile monster
hunting anyone", not "anything hostile".

- **The budget does not change.** Monsters join the same reservoir as players and compete for the same
  discretionary trace. Their gaze term is a fixed `kMonsterGaze = 2`, the player branch's maximum, since a
  monster hunting us faces us by construction. `v.v_angle` and `v.fov` are meaningless on a monster and
  are never read (as `EnemySees` short-circuits in `bot_combat.cpp`).
- **The trace goes to the box centre, not a bone.** `SamplePoint` returns `EntitySoundOrigin` for anything
  without `FL_CLIENT` and builds no bone set: a snark is an 8-unit cube with zero `view_ofs`, so the eye
  fallback would aim at its feet.
- **The list is emptied by `ResetBoneCache`**, misnamed but the right call site: that is where both map
  transitions flush per-map entity caches, and edict pointers must not survive one. `StartFrame` refreshes
  it only when a bot exists, and `crossbot_monsters` refreshes before printing.

`mon=list:,seen:,mine:,trc:,hit:` in `crossbot_sense_status` (`monSeen`, `monMine`, `monTraces`,
`monHits`, cumulative per bot) tells the three failures apart: no monsters in the list, monsters that never name us, and monsters that name us
but whose traces never land.

### Vision samples a hitbox bone, not the eye

Per traced player, in cost order: range, a 3-D cone test, a bone position, then the trace.
`pfnGetBonePosition` runs a full `SV_StudioSetupBones` per call, so it goes last and only for the one or
two candidates that survived selection.

- **The cone is a true 3-D dot against `v_forward`**, not foxbot's yaw-only `FInViewCone`, which sees
  targets directly overhead. Half-angle `fov / 2` against the horizontal FOV is slightly generous
  vertically (HL's vertical FOV at 4:3 is nearer 74°); accepted instead of a real frustum.
- **The trace must be `dont_ignore_monsters`**; with `ignore_monsters` it passes through players and
  `tr.pHit == ent` is never true.
- **The sample is a random bone that owns a hitbox.** All of `0..numbones` would include helper bones (the
  `Bip01` root at the feet, weapon and blend bones off the body), which drag the aim point down and flicker
  visibility on floor geometry.
- **Engine caveats.** `pfnGetBonePosition`'s `rgflAngles` out-parameter is never written, so `nullptr` is
  passed and hitbox *centres* are not used (they would need the bone's rotation). Stock GoldSrc
  bounds-checks `iBone` only under `REHLDS_FIXES`, so the bone set is built from `numhitboxes` with every
  index checked against `numbones`.

### The lead point is traced before it is queued

The queue write pushes `bestPoint + targetVelocity * delay`. At the default delay and full run speed the
lead is about 80 units, so it crosses into a wall roughly a reaction time before the target does. So the
lead is traced from the bot's real eye, and the slot is pushed only if it comes back clear; a blocked lead
pushes nothing.

- **It gates the queue write and nothing else.** Perception still traces the raw bone point, so
  `SUSP_SIGHT` and target selection are untouched. Tracing the lead *instead* would let an occluded lead
  null `best`, and its return next frame would fire `ClearDelayQueue` and wipe the ring: with 80-unit leads
  and a random bone, that flickers at every doorway and costs a reaction delay of silence each time. It
  costs one trace per bot per frame, since the write is once, for the winner.
- **It is `ignore_monsters`**, unlike the perception trace. It asks whether the predicted point is inside
  or behind world geometry; another player or the target's own body 80 units short must not suppress it,
  and under `dont_ignore_monsters` the target would be `tr.pHit` most of the time. Brush entities still
  block: a closed door is a wall. It is the same flag and question as the fire-side trace in
  `BotFire::Buttons`.
- **It is skipped when the lead is the sample point**: a still target, or a delay of 0. That keeps
  `test_sense.ps1`'s static phase identical to a build without it, a regression check for the rest of the
  queue.

Nothing is pushed in place of a vetoed lead. The ring drains, and if the block outlasts the delay the read
underflows and `ApplyLook` falls through to the suspicion blend, which points at the last `SUSP_SIGHT`:
the target's real last-seen position, at the corner. `ReadDelayed` interpolates on time, so a hole in the
history is a longer span and the served point slides across it smoothly when writes resume.

`crossbot_aim_status` and `crossbot_sense_status` both report `leadblk`, the vetoes since spawn: a count,
because a two-second poll would almost never catch a one-frame flag.

### Notice delay: seeing is not noticing

A clean trace does not make a target. The traced entity (`seen`) first fills a per-bot notice meter, and
becomes `best` only when full. Without it an enemy at the cone's edge 3000 units away was spotted as
fast as one dead centre at point blank, and a re-peek round a wall was instant.

The time to notice is set in screen terms, normalised to fov 90:

    factor = tan(theta) / tan(fov/2)        on-screen offset: 0 at the crosshair, 1 at the edge
    small  = dist * tan(fov/2)              inverse on-screen size; at fov 90 this is just dist
    need   = factor * small * scale / 1000  =  dist * tan(theta) * scale / 1000   seconds

**The fov cancels**: zoom pushes an enemy toward the edge and enlarges it by the same factor, so zoom
changes only what fits in the cone, which the cone test already handles. At scale 1: 45° off-axis at
1000 units takes 1 s, 10° takes 0.18 s, dead centre is instant at any range. `theta` is measured to the
sampled bone, so a bone just outside the cone gets a large but finite `need`.

`scale` is `BotDiff::NoticeScale()`: `crossbot_notice`, or the tier (easy 1.5, normal 1.0, hard 0.75,
expert 0.5). `crossbot_notice 0` makes acquisition instant.

**One meter per enemy, draining, not resetting.** `sense.notices` holds `kNoticeSlots` (8)
`{ ent, progress, need }`:

    the entity seen this frame:  progress += dt / need     (need 0 -> 1), clamp 1; best iff progress == 1
    every other slot:            progress -= dt / kNoticeForget (2 s); at 0 the slot is freed

- A new enemy takes a free slot, else evicts the lowest progress; a slot whose handle died reads as free.
  Eight is far more than a 90° cone holds at 32 players.
- Everything not seen this frame drains, including enemies in the cone that were not traced; freezing them
  would leave an untraced enemy half-noticed forever.
- The incumbent is no exception. While held the meter sits at 1; out of sight it drains, and a re-peek
  after `t` seconds must refill `t / kNoticeForget * need`. A re-peek where the bot is already looking (the
  suspicion look points at the last `SUSP_SIGHT`, the corner) has a small `theta` and is fast; one far
  off-axis is slow. Two seconds is under combat's five, so the face is forgotten before combat lets go.
- Per-enemy slots matter: with one slot, tracing enemy B while A was occluded wiped A's memory and A's
  re-peek paid in full.

**The enemy closest to noticed takes the pick slot.** Of the non-incumbent candidates that pass the gates
and have a slot, the highest progress skips the reservoir and becomes `pick` outright. Otherwise the
reservoir would spread the one discretionary trace over every candidate, each meter filling on a fraction
of frames and draining on the rest. An occluded favourite drains to zero within `progress * 2 s` and frees
the pick. The budget is unchanged: incumbent first, then the pick.

**`SUSP_SIGHT` is pushed only for `best`**, not on every clean trace: an enemy seen but not yet noticed
would otherwise pull the gaze toward itself and give away exactly what the delay hides.

**Cloaked enemies are traced one frame in five.** `IGameProfile::Cloaked`, asked in the owner game's
context (only DMC answers yes, for the ring), gates both traces through `Glimpse`: unless the entity is
already `sense.target`, the trace runs only when `RandomUnit < kCloakSeeChance` (0.2), and a skipped roll is
an unseen frame. The meter fills at `0.2 / need` and drains at `0.8 / kNoticeForget`, so only an enemy with
`need` under about 0.5 s, near the crosshair, is ever noticed. Once noticed he is traced every frame until
the first obstructed trace clears `sense.target`. `crossbot_sense_status` prints `cloak=<skips>/<rolls>`
per watching bot; on crossfire, two DMC bots roaming and one holding the ring, it read 707/867 (81.5%), and
0/0 without the ring or for the holder.

Notice is detection and the delay is reaction, so they stack: the meter fills, `best` is a new target, the
queue clears, and the underflow silence follows. Being blinded (`BotDaze::Blind`), `crossbot_sense 0` and
respawning clear the whole table.

`test_sense.ps1` and `test_fire.ps1` take `-Notice <scale>` for A/B (`-Notice 0` is instant). Over four
150 s four-bot crossfire duels the delay did not starve combat: sticky shots 38 and 31 with it on, 12 and
19 off. With per-enemy slots the same probe read shots 300/263, sticky 27/52 and target held on 45/37 polls
with it on, against shots 141/132, sticky 40/12 and 20/25 polls off.

### Shadows: light and background scale the meter

A traced entity is also judged on its lighting and what it stands against, both sampled from the map's
lightmaps through `Nav::LightMap` (`src/nav/nav_light.cpp`):

    Lt  = light under the target:  Sample(origin, origin - 2048z)
    Lb  = what is behind it:       Sample(bonePoint, bonePoint + losDir * kViewRange)
    vis = clamp(max(Lt, |Lt - Lb|) / kLitLevel, 0, 1)
    need = dist * (tan(theta) + kShadowAxis * (1 - vis)) / vis * scale / 1000

`Lt` is the downward light point the client lights that player's model with; `Lb` is the first world face
on the bot's sightline past the target. A lit target scores 1, and so does a dark silhouette against a lit
wall, through the contrast term; only dark on dark is slow. At `vis = 1` this is the formula above, so a
lit map behaves as before. `kShadowAxis` (0.25) stops a shadowed enemy dead centre being instant: at
`vis = 0.25` and 1000 units, centre takes 0.75 s and 45° off takes 4.75 s. `kLitLevel` is 64.

- **Zero is invisible.** `vis == 0` turns `seen` back into nothing: the slot drains and a held target is
  lost as if occluded. `darkDrops` (`dark=` in `crossbot_sense_status`) counts those frames.
- **The engine cannot answer this.** `pfnGetEntityIllum` returns `v.light_level` for a client, which a
  fakeclient never fills (`pfnRunPlayerMove` carries no light level), and 128 for everything else on a
  dedicated server. So the plugin reads the live BSP from the path `XPlay::FindMapBsp` finds on
  `ServerActivate` and walks the node tree like the engine's `RecursiveLightPoint`. A map with no lighting
  lump is fullbright; a failed `Load` leaves `vis` at 1.
- **Compare in the lightmap's own space.** RAD gamma-encodes (qrad and ZHLT `-gamma` 0.5, VHLT about 0.55),
  so a byte is roughly `256 * linear^gamma`, close to perceived lightness, and an absolute difference is a
  fair contrast. A linear ratio blows up near zero (light 2 against 1 reads as full contrast while
  invisible). Styles are scaled in stored space as the engine does. Clients' own `brightness`/`gamma` are
  not modelled.
- **Light styles** come from a `pfnLightStyle` post hook, so a switched-off light is dark. Styles are set
  while entities spawn, before `ServerActivate`, so the table clears only on deactivate. The sample takes
  the pattern character at `time * 10`, as the engine animates it.
- **Effects override the samples.** `EF_DIMLIGHT` (flashlight), `EF_BRIGHTLIGHT` and `EF_MUZZLEFLASH` make
  the target lit; a sky or water face (`TEX_SPECIAL`) reads 255, so an enemy on a skyline is a silhouette.

`crossbot_sense_status` prints `light=<Lt>/<Lb>/<vis>` for the last traced entity. On crossfire open ground
reads 190-207, dim corners 24-60, and a dim target on a dim wall `light=24/46/0.37`.

The bots' own preference for dark ground is baked by navgen (`doc/navgen.md`, *Visibility*).

### What the bot sees through

`TRACE_LINE` answers the sight question wrongly both ways: it skips every `SOLID_NOT` entity, so bots saw
through `func_illusionary` foliage and walls and through `.mdl` props (`env_model`, which is how stock DoD
maps place bushes, ferns, grass and trees, and zero-size `cycler_sprite`, which the engine's area-link cull
skips); and it stops on every solid entity, so glass `func_wall` and `func_breakable` blinded bots.
`BotSight::Clarity` (`bot_sight.cpp`) replaces it wherever the bot *looks*: `TraceVisible`, the lead veto,
`FriendInView`, the item-taken check, the brain's `Sees`, and `crossbot_monsters`. Fire, blast and arc
traces stay physical: a bullet does not care how a wall is drawn.

**Coverage is rasterised, not guessed from `renderamt`.** `src/nav/sight_coverage.cpp`, shared verbatim
with navgen, draws each model's textured triangles at 1 unit per pixel, multiplies transmittance per pixel
by `1 - alpha` (`renderamt / 255`; a `{` texel of index 255 is a hole in TransAlpha mode, as is a
`STUDIO_NF_MASKED` texel; additive and glow draw nothing), and averages 32x32 blocks. A cell's alpha is the
chance that a 32-unit body behind it is hidden. Back faces are culled as the engine culls them (brush
winding agrees with the plane normal on 2326 of 2334 faces of dod_forest). Each model is drawn from 34
directions (`kViewCount`): rings of 16 at the horizon and 8 at ±45°, plus straight up and down.

**Models are cut into 256-unit chunks** (`kChunkSize`). A lookup takes the nearest of the 34 views and reads
the cell where the sightline crosses the plane through the chunk's centre, with error of chunk depth times
the view's angular error. A brush entity can be several disjoint parts, and chained foliage walls sharing
vertices make one connected part 3000 units across; cutting every model on a grid anchored at its own
minimum (triangles clipped, `st` interpolated) bounds both at about one and a half cells. A model's chunks
are grouped under the entity, and one box test rejects the group, which keeps 250 occluders cheap per
trace.

**Models live in model space; the entity moves them.** An instance applies `pev->origin` and `pev->angles`
(brush rotation is the engine's rotated-hull basis; studio pitch is negated as `StudioSetUpTransform`
does), re-placed whenever either changes, so doors and trains track. Brush vertices of an ORIGIN-brush
entity are stored relative to it, which the same transform restores. Studio models use the entity's
`sequence` at frame 0 with `body` and `skin`; animation is ignored.

**Occluders are chosen once, at `ServerActivate`**, from the live edicts: a model that is not a client,
monster or `MOVETYPE_FOLLOW` attachment, not `EF_NODRAW`, with render opacity above 0; and either a brush
model that is `SOLID_NOT` or not `kRenderNormal`, or a `.mdl` with `MOVETYPE_NONE` that is `SOLID_NOT` or
zero-size. Entities spawned later are not covered, and a held weapon never hides its owner.

**Clarity** is the product of `1 - alpha` over everything crossed. A trace that hits a solid occluder
multiplies by its coverage and restarts from the hit point skipping that entity, up to four panes
(`kMaxPanes`); a restart point is on a surface, so never inside the bot's own hull. The world, or a solid
non-occluder, gives 0. Then every non-solid occluder whose box the segment crosses multiplies in. There is
no cutoff: an opaque cell is 255, so alpha is exactly 1 and clarity exactly 0.

**Clarity scales the notice meter; it does not gate it.** It multiplies into `vis`, so an enemy behind a
half-covering bush takes twice as long to notice, and `kShadowAxis` keeps dead centre from being instant. A
held target stays held behind near-opaque cover, since a held meter sits at 1, as for shadow. The fire
gate stays physical, with one addition: a blocking breakable that is not `kRenderNormal` counts as a shot
when nothing else stops the ray, because bullets break glass. Unbreakable glass holds fire.

**The build runs on a worker thread and is cached.** `BotSight::Build` describes the occluders on the main
thread and hands the unique `(model, sequence, body, skin, render)` keys to a worker, which looks them up in
the cache, rasterises only what is missing, and appends it. `StartFrame` adopts the result when done; until
then `Clarity` is the plain trace. dod_forest builds in under 4 s cold and loads in 0.03 s warm.

**One append-only cache file for every map and model.** `addons/crossbot/coverage.cov` is a magic and
`SightCoverage::kFormatVersion`, then records of `{uint32 size, uint16 keyLen, key, chunks}`. The key
carries its own validity, so nothing is invalidated in place:

    b|<bsp checksum>|*N|rendermode|renderamt
    s|<game dir>|<.mdl+T.mdl crc>|<model path>|sequence|body|skin|rendermode|renderamt

A recompiled map or edited model stops matching its old records, which stay as dead weight. A studio model
is stored once for every map that places it (dod_charlie reused six of dod_forest's); the checksum costs
one read of each `.mdl` per map load.

- **Safe for several servers at once.** A reader holds a shared OS lock for the whole read (`LockFileEx`,
  or `flock` on Linux) and a writer an exclusive one. Readers walk records by their size prefix, skip
  without loading every key the map does not use, and stop at a short or malformed tail. A writer
  appends under its lock, and truncates and starts over if the header is missing or another version, so a
  format bump needs no manual cleanup. Two servers building one model at once both append it; readers keep
  the first. Verified with dod_forest and dod_charlie started together.
- **After a read, seek before writing.** C leaves a write directly after a read on the same stream
  undefined, and the MSVC CRT silently drops it: every append to an existing file vanished while the
  summary said "cache written". `AppendCache` seeks to the end after the header check and after a
  truncation.

Wads and models are read from the BSP's game directory (`<bsp dir>/..`), falling back to `valve`, where
DoD's `halflife.wad` lives; a missing texture draws opaque and is named once in the build summary.

`crossbot_coverage 0` turns it all off for A/B runs. `crossbot_monsters` lists the occluders,
`crossbot_sight` measures one sightline, and `crossbot_sense_status` prints `clar` and
`cov=<partial>/<blocked>/<traces>`.

Measured on dod_forest with `crossbot_sight`: 16 horizontal 440-unit lines through each model's centre at
20, 50 and 80% of its height, keeping only lines the plain traceline calls clear (all clarity 1 with
coverage off). Mean clarity with coverage on:

| occluder | low | mid | top |
| --- | --- | --- | --- |
| `arc_bush.mdl` (two instances) | 0.06 / 0.03 | 0.01 / 0.02 | 0.02 / 0.07 |
| `arc_fern.mdl` (two instances) | 0.48 / 0.26 | 0.41 / 0.62 | 0.84 / 0.97 |
| `func_illusionary *4`, TransAlpha foliage | 0.31 | 0.31 | 0.83 |

A 180-unit bush hides nearly everything, a fern is dense at the base and open in the fronds, and an
alpha-tested foliage brush lets about a third through.

### The bone cache and when it flushes

`g_boneSets` is a file-static array of 64 entries (`kMaxBoneSets`), map-scoped, not per bot; the table is
under 3 KB. The key is `v.modelindex`, stable for the life of a map; `setinfo model` lands on a different
modelindex and slot.

It flushes:

- on `ServerActivate`, the one flush actually required, since the modelindex-to-model mapping is rebuilt
  per map;
- on `ServerDeactivate`, so nothing survives a level change;
- per entry, opportunistically: each lookup compares the stored `studiohdr_t*` with the current
  `pfnGetModelPtr` and rebuilds if they differ, which makes GoldSrc freeing and reloading studio extradata
  under memory pressure self-healing.

Never on a timer and never per bot: a loaded model's bones do not change.

### One entry per entity

The suspicion table holds exactly one slot per entity. `Push` overwrites the slot whose live handle
matches the emitter, else appends if there is room, else reuses the entry with the smallest `time`. Nothing
votes twice however much noise it makes, so where the bot faces stops depending on emission rate.

**Why not a trail of entries:** a running enemy lays a footstep every ~0.3 s, fifteen votes at fifteen
points in the 5 s window, which dragged the weighted mean back along the way they came while a visible
enemy got one vote. A lift cycling in a shaft did the same.

- **The key is a `CBotEntHandle`, not an index.** When a breakable is destroyed and something else takes
  its edict slot, the handle's serial makes `Is` false, so the new entity gets a new slot and the orphan
  ages out instead of being inherited.
- **Every scan runs `[0, count)` with `continue`.** Overwriting `time` in place destroys time order, so a
  backward walk that `break`s at the first stale entry would silently truncate the list, the one failure
  here that looks like it works. With one entry per entity the live set is a handful.
- **`count` is a high-water mark of distinct entities** (`kSuspicionSlots`, 256), not a queue depth.
  Eviction starts only after 256 distinct entities and reclaims decayed and orphaned entries first, since
  they are oldest.

### Where the bot looks when it has no target

Every read of an entry goes through `LiveWeight`: tier selection, accumulation, `LiveCount`, `Freshest`.
One choke point is the design: a second liveness test that disagreed is how a tier gets chosen and then
contributes nothing. `LiveWeight` drops an entry whose entity is gone, drops a dead player or monster, and
otherwise returns `Falloff(age)`. The handle check stops gibs and broken breakables steering the bot; the
dead check stops it staring at a corpse for five seconds after a kill.

**The dead check is only for players and monsters.** `IsDead` is
`deadflag != DEAD_NO || health <= 0 || iuser1 > 0`, and a `func_door` or `func_train` has `health == 0`.
Applied to `KIND_OTHER` it would discard all map machinery and leave the fallback tier permanently empty,
with no test noticing because the player tier still works.

#### Recency falls off exponentially

    h = decay / 4                                     // 1.25 s at the default 5 s
    Falloff(age) = (2^(-age/h) - 1/16) / (15/16)      // age < decay, else 0

The `1/16` is `2^-4`, the value at `age == decay`, subtracted and renormalised so the curve hits zero
exactly there. "Live" still means `age < decay`, which every liveness reader (camp scare, grenade, satchel,
TFC pipes, the combat hold in `Senses`, `LiveCounts`) depends on. 1 s old is 0.55, 2.5 s is 0.23, 4 s is
0.05. Linear falloff never separated anyone in a fight: a running or shooting player pushes every 0.3 s or
every shot, so every noisy entity sat near 1.0 and the cone alone decided. The exponential makes a contact
quiet for a second worth half of one still talking.

#### The tier vote

The vote picks **one tier** and ignores the rest: players if any are live, else monsters, else everything
else. The enum order is the priority order, so this is a `min`. A player entry decayed to 0.1 still beats a
door at 1.0, so the bot glances at the fading contact instead of snapping to the lift; the tier filter runs
before `bestWeight` for that reason.

**A monster hunting us votes at the player tier.** `Suspicion::hunting` is set in `Push` for a monster
whose `m_hEnemy` is this bot, and `Tier(entry)` returns `KIND_PLAYER` for it; otherwise any live player
contact, even at 0.1, would keep the head off the snark eating the bot. The flag sits beside `kind` rather
than overwriting it, so `kinds=p:,m:,o:` in `crossbot_sense_status` still counts what it says and `hunt=`
reports the promotion.

#### Nothing is averaged: the winner is one entry, chosen by cone

Within the winning tier every live entry is scored per sense, keeping its best:

    score = max over s of  Falloff(now - sourceTime[s]) * (s is DAMAGE or SIGHT ? 1 : ConeWeight(vantage, dir))

and the **single highest scorer wins**. No vector sum, mean-shift or degenerate-mean fallback: those
existed to make averaging survive threats on opposite sides, which an argmax does not need.
`authority` is the winner's recency alone, so the cone data picks *which* entry and never how hard the bot
commits: a fresh contact from an unwatched direction still commands a full turn.

- **Damage and sight skip the cone.** The cone is a prior on where threats come from, right for a sound.
  Being hit or having traced someone is not a guess; coned, an attacker in the back scored 0.15 against
  near 1.0 for anyone down the main cone, and bots ignore being shot from behind. Damage does not enter
  combat (`doc/combat.md`), so this turn is the bot's whole reaction to an unseen attacker. There are no
  per-sense multipliers on top: the cone is handicap enough for sound and gunfire.
- **Each entry keeps a time per sense, `sourceTime[SUSP_COUNT]`**, while `time`, `source` and `pos` still
  mean "the latest push" for every other reader. Needed because the attacker's own gunshot lands on the
  same entry a frame after each hit; with only `source` the hit would fall back under the cone every other
  frame. `Push` zeroes the array when a slot changes owner.

`ConeWeight` reads the bot's blended vantage signature (`nav_vantage.cpp`), the cones through which
everything with line of sight to this spot arrives:

    rel  = watch[k].amount / watch[0].amount        // watch is amount-descending, so rel in (0,1]
    best = max over k of  rel * max(0, dot(dir, watchDir[k]))
    return kVantageConeFloor + (1 - kVantageConeFloor) * best

- **Normalise to the strongest cone, not the sum.** A dead-on hit on the main cone scores 1.0 whether the
  spot has one cone or four; dividing by the sum would cap a four-cone spot at 0.25 under the floor.
- **`kVantageConeFloor = 0.15` makes it a bias, not a blindfold.** A noise from a direction nothing arrives
  from keeps 15% of its weight and can win once the main sightline goes quiet. (On its own 15% of a fresh
  hit loses to any main-cone contact younger than about 2.9 s, which is why damage skips the cone.)
- The point is the wall-staring case: without the cone, a noise behind the bot with no sightline drags the
  mean into the wall. With it, the noise simply loses.
- With no vantage data (an old map, `--no-vantage`, or `crossbot_vantage 0`) `ConeWeight` returns 1 and the
  freshest live entry in the winning tier wins.

`ApplyLook` then blends rather than overrides:

    desiredAngles.yaw = base.yaw + shortWay(want.yaw - base.yaw) * authority
    desiredAngles.pitch = base.pitch + (want.pitch - base.pitch) * authority

`base` is whatever `desiredAngles` holds: the path facing at the walk site, the last commanded angles at
idle sites. At `authority == 0` the write is a no-op, bit-identical to no sense system, which keeps the nav
sweep honest. As the suspicion ages the head drifts back onto the path instead of snapping. An aim target
always wins: if `ReadDelayed` serves a point, `ApplyLook` aims at it and returns first.

### When there is no suspicion at all, the vantage field drives the head

At `authority == 0`, `ApplyLook` calls `ApplyVantageLook`, which splits on whether a path is running:

- **No path:** write `watch[0]`'s yaw and pitch, the direction most sightlines into this spot arrive from,
  so a standing bot watches the way it is most likely to be shot from.
- **Path running:** `desiredAngles.y` is the path yaw toward the next waypoint. Take the watch direction
  closest to it in yaw, and snap to it within `kVantagePathSnap` (60°); otherwise keep the path yaw and
  level the pitch. A bot rounding a corner is already looking down the corridor.

- **The 60° clamp is not cosmetic: yaw is the movement basis.** `Bot_DecomposeMove` flattens to yaw, so an
  unbounded snap turns every corridor into a sideways strafe.
- **Both branches write absolutely, not through the `authority` blend.** At idle sites `base` is the
  previous frame's output, so a blend compounds and the head freezes wherever it drifted (see the ratchet
  below).
- The blend is refreshed every `kVantageInterval` (0.1 s) per bot from its eye: the field is smooth and a
  bot does not move far in 100 ms.
- `crossbot_vantage 0` skips the refresh, so the blend is empty, `ConeWeight` returns 1 and this branch
  falls through to `desiredAngles.x = 0`. That switches off everything the field does, for the nav-sweep
  A/B, but is not a full revert: the argmax replaced averaging unconditionally.
- **Senses outrank the vantage look.** Map machinery (doors, chargers, item respawns) pushes `SUSP_SOUND`
  constantly, and a live `KIND_OTHER` entry at 0.1 still takes the head off the vantage direction, because
  it is a real sense and this is the fallback.

`crossbot_vantage_status` reports which branch won per bot as `look=`: `disabled`, `gated`, `aim`,
`predict`, `gauss`, `sense`, `camp`, `back-check`, `vantage-idle`, `vantage-path`, `vantage-offpath` or
`no-vantage`. It is the only reliable way to tell them apart from outside.

### The gate veto

`ApplyLook` does nothing while any waypoint in `waypointIndex .. +2` needs a ladder, an air-move launch, a
lift ride, a button use or a trigger touch. Without that lookahead the bot reaches a gate with its head
turned and pays the full `kLadderAimTolerance` / `kActivationAimTolerance` latch cost, the controller's one
real cost.

It is **opt-in**, called at each walk and idle site in `bot_move_nav.cpp` and never on branches where the
view drives the movement. Opt-in fails safe: a missed site means no looking around there. Opt-out would
fail dangerous: a missed site breaks a ladder.

### Two gaps worth knowing about

**A bot with nothing in its cone and no suspicion would never turn.** navgen's vantage field solves this
the Parabot way, done properly: Parabot scored four compass directions per map cell by how much was visible
and where players died, then swept the head round the winner; the vantage field is a traced visibility
signature per sampled eye position, and `ApplyVantageLook` points the head at `watch[0]`. foxbot's cheap
version (random yaw every 1-3 s, rejected unless a 200-unit trace is clear) is not needed.

At hard and expert there is also a sweep: `BotDiff::BackCheck` gives `ApplyLook` a period, and a
weighted-random pick over `watch[0..3]` by `amount` turns the head there for 0.4 s (`kBackCheckHold`)
before the ordinary logic takes it back. A pick within 60° of where the bot already means to look is
rejected and the period spent, so the glance is always somewhere it was not watching. At normal and easy
the head goes to `watch[0]` and stays. `doc/difficulty.md`, `bot_difficulty.cpp`, has why it sits above
the suspicion blend and the measured rates.

Footsteps narrow what is left: a running enemy is heard at ~625 units regardless of facing. The uncovered
case is a stationary or walking enemy (silent below 220 units/s), and a bot facing its main sightline is
far better placed for that than one facing a wall.

**Pitch is levelled, and the suspicion pitch is clamped.** With no target, no live suspicion and no usable
vantage direction, `ApplyLook` writes `desiredAngles.x = 0` instead of leaving the angles alone (the vantage
branches avoid the same trap by writing absolutely). Otherwise the blend ratchets: `base` is the previous
output, each frame moves further toward the suspicion and nothing moves it back, so when authority decays
the head freezes where it got to. A door sound in a shaft once left two bots staring at the floor at about
70° of pitch, out of each other's cones for good. Levelling is safe at the walk site because navigation
always commands pitch 0 there. `kLookPitchLimit` (45°) bounds how far a close suspicion can crank the head;
it does not apply to a real aim target, which needs the full range.

### crossbot_sense_status

Per bot: the aim target and its sampled bone, the remembered enemy and its age, live suspicions by kind,
the winning tier, the freshest entry with source, age and position, the look direction with its authority
and entry count, and cumulative push counts per source. Fields that need reading:

- **`target=` against `enemy=`**: they agree while the bot can see someone. `target=- enemy=Sense2
  enemyAge=1.8s` is the memory working, and `enemy=-` returning once `enemyAge` passes
  `crossbot_sense_decay` is the record expiring on schedule.
- **`notice=<count>:<name>:<progress>/<need>s`**: live slots, then the highest-progress one; `need` is the
  full-fill time at the angle and range last seen. Progress climbing with `target=-` is an enemy being
  noticed; falling is the meter draining.
- **`kinds` against `tier`**, the assertion worth automating: `tier` must read `PLAYER` whenever `kinds=p:`
  is nonzero. `test_sense.ps1` checks it every poll.
- **`fresh=`** is the freshest entry *overall*, so `tier=PLAYER` beside `fresh=SOUND ent=148` is correct: a
  door was heard more recently than the player and lost anyway.
- **`kept`** is the live entry count in the winning tier.
- **`pushed=`** counts pushes per source, cumulatively. With one entry per entity an enemy's `DAMAGE` entry
  is overwritten by their `SIGHT` within one `BotSense::Update`, so `fresh=` almost never shows `DAMAGE`;
  only the counters prove a source fires.
- `cands`, `keep=`/`keeps=inc:,new:`, `pushed=...ally:`, `mon=`, `hunt=`, `leadblk`, `cloak=`, `light=`,
  `dark=`, `clar`, `cov=`: see the sections above.

### crossbot_vantage_status

With no arguments it reports the loaded field (point count, spacing, resolved grid cell and dimensions,
`maxExposure`, the cvar) and then, per bot, the blend at its eye: neighbours used, distance to the nearest,
`exposure` raw and as a fraction of the map maximum, every merged watch direction, and a drive line with
`view=` (actual `v_angle`), `desired=` (what the look code commanded), `look=` (which branch) and the
waypoint index.

`crossbot_vantage_status <x> <y> <z>` blends at a world point instead. **This is the round-trip test:** at
a stored vantage's exact position the distance-0 neighbour dominates the IDW, so the output must be that
vantage's own record (watch yaws, pitches, spreads and normalised amounts). It proves the loader, the grid
and the angular merge at once.

`desired=` next to the watch list makes the look checkable: on a `look=vantage-path` sample the commanded
yaw *and* pitch equal one listed watch direction exactly. Two caveats: the blend is recomputed at print
time, so on a moving bot it can be one sample ahead of the one `ApplyLook` used; and `look=vantage-offpath`
prints pitch 0 with the path yaw, which is the 60° clamp refusing the snap.

### The enemy handle outlives the sight of it

`sense.target` is not memory: `UpdateVision` writes `sense.target.Set(best)` every frame, null included, so
it drops the frame sight breaks. That suits aim and the trigger, and is useless for deciding where to walk.

`sense.enemy` is the memory. It is written from the same site whenever a target is visible and survives
the loss: handle, position, and time. It drops only when the entity is gone or dead, or the record is
older than `crossbot_sense_decay`, the three tests `LiveWeight` applies to a suspicion.

- **The stored position is `v.origin`, not `bestPoint`.** A bone can be a hand out over a ledge; the feet
  resolve with `FindAreaExact`, exactly in the cases where the enemy is somewhere interesting.
- **It is refreshed only by sight.** Sound, gunfire and damage from the same enemy update its `Suspicion`
  slot and steer the head, but do not move the remembered position: the bot walks to where it last *saw*
  someone, and the goal holds still while it walks.

### Sense reports, it does not decide

Sense answers questions and owns nothing downstream; `BotCombat` reads the answers and makes every
transition (`doc/combat.md`, `bot_combat.cpp`). Nothing in sense touches the goal or the path.

`Senses(bot, ent)` asks whether any live entry still points at this entity. Sight, sound, damage and events
all count, so a bot that can hear its enemy has not lost it. Entries expire on `crossbot_sense_decay` in
the same sweep every reader uses, so this means "have I lost every trace of it".

`bot.sense` is not reset by anything on death, so `BotSense::OnRespawn` clears the enemy alongside
`BotItems::OnRespawn`, `BotFire::Reset` and `BotCombat::OnRespawn`.

## bot_daze.cpp

Flash and concussion grenades, learned the way a client learns them: from the user message the game DLL
addresses to the player. Nothing else in the plugin reads network messages.

### The message hook

`engine_hooks.cpp` hooks `pfnRegUserMsg`, `pfnMessageBegin`, `pfnWriteByte`, `pfnWriteShort` and
`pfnMessageEnd`, all post. `pfnRegUserMsg` records the ids of `ScreenFade` and `Concuss` as they register.
`MessageBegin` arms capture only for `MSG_ONE` / `MSG_ONE_UNRELIABLE` to one of our bots and only for those
two ids; every write hook returns on one branch while disarmed, which is the whole cost on the common path.

- **The recipient test is `BotManager::FindBot`, never `FL_FAKECLIENT`.** CS clears that flag on every spawn
  (`doc/games.md`, *Rules*), so a flag test handles a cstrike bot's flashbangs for its first life only,
  while `crossbot_daze_test`, which bypasses the hook, keeps passing. The two id compares run first, so a
  message that is not ours never reaches `FindBot`'s walk of `g_bots`.
- **Ids are per name, and may collide across games.** Under valve+cstrike+tfc, `ResetHUD`, `Battery` and
  `StatusValue` each hold two ids (the games disagree on size); `ScreenFade` (96) and `Concuss` (156) get one
  each. A game registering `ScreenFade` with a different size would overwrite the stored id and blind only
  that game's victims; `crossbot_usermsgs` shows it.
- **The ids come only from the hook**, so the plugin must load before the game DLLs register: true for
  `plugins.ini` at startup, false for a later `meta load`. `crossbot_usermsgs` says which.
- Only bytes and shorts are hooked, the only field types the two messages use. A third message with other
  field types must hook those writers too, or its field indices are wrong.
- The handlers touch `CBot` fields and `gpGlobals->time` only, so they need no game context.

### ScreenFade

10 bytes: `duration` and `holdTime` as 4.12 fixed-point shorts (`seconds = value / 4096`), `fadeFlags`, then
r, g, b, a. The hook sees the `int` the game passed, before engine truncation, and masks to 16 bits.

**A fade blinds only when `a >= 200` and `FFADE_MODULATE` is clear.** 200 is CS's minimum flash alpha
(`RadiusFlash` sends 255 facing the grenade, 200 facing away), and it keeps HL's charged-gauss self-fade
(orange, alpha 128) from blinding every bot that fires one. A full-alpha map `env_fade` does blind bots:
the screen really is covered.

**Duration is `IGameProfile::BlindSeconds`**, looked up on the *bot's* game
(`XPlay::ProfileFor(bot.gameDll)`), because the handler runs outside any game context. The default is
`kBlindScale * duration`, 0.33, after ReGameDLL's own bots (`PlayerBlind` calls
`Blind(fadeTime * 0.33, ...)`): blind for a third of the time a human's screen stays white, so 4 s of
blindness from 12 s of point-blank fade. A second flash extends to whichever ends later. DoD overrides it to
`hold + scale * duration`: its only full-alpha fade is a headshot's (alpha 200, dark red, 3 s fade after a
2 s hold, dod.c:86590), and the hold is time the screen really is covered, so a headshot blinds for about
3 s.

**While blind:**

- `BotSense::Update` skips `UpdateVision` and clears `target`, `targetBone`, `visionCands` and `visionKeep`,
  as `crossbot_sense 0` does, except that **`sense.enemy` survives**: a flashed bot still knows someone was
  there. Sound, events and damage still reach the suspicion table, so it keeps turning toward noise.
- **The delay queue is emptied every frame**, not just on the edge. `BotFire`'s `haveTarget` is
  `BotAim::ReadPredicted`, which extrapolates from the newest slot for up to `kPredictHorizon` (1 s)
  whenever `predictLatched && combat.active`; clearing only on the edge left a bot flashed between
  sightings firing accurately for over a second. With the queue empty, `BotFire` reads `no-target` and
  stops for the whole blindness, untouched itself.
- `BotCombat::Run` drives, with or without an open fight: the exposure shuffle never reads the enemy, so a
  blind bot drops its goal and walks toward cover, and `DuelDrive` declines for want of an enemy.
  `combat.active` is not touched, so `crossbot_combat_status` still says whether there is a fight. When
  blindness ends with no fight, `Run` releases the drive and zeroes `nextSelectTime`, as `Exit` does. Under
  `crossbot_combat 0` the blind drive is off too, since `Update` calls `Exit` every frame there.

### Concuss

One byte. A TFC client stores it in `v_idlescale`, and `V_AddIdle` adds `scale * sin(t * cycle) * level` to
the rendered view: pitch cycle 1 level 0.3, yaw cycle 2 level 0.3, and roll, which a bot lacks, dropped.
`Bot_UpdateFacing` adds the same to its desired angles before the controller, which smooths it into a
wander, and `Bot_DecomposeMove` makes the bot walk crooked for free. `t` is absolute time, as on the client.

TFC's server (`TeamFortress_Concuss`, decompiled) sets the value to 100 for 15 s, or 7 s for a medic; each
think recomputes `100 * remaining / duration` and sends a message whenever that integer changes. About 100
bytes arrive, the last zero, and the bot tracks the latest. At 100 the sway is ±30° on both axes.

On a client the sway is cosmetic and real aim is untouched; a human misaims because what they see moves.
Applying it to the bot's actual angles is the same miss, taken directly. Respawn resets the whole state.
