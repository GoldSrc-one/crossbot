# Difficulty

One global tier that scales error, delay and opportunism: the ladder, the pin rule, and what each tier flag reaches.
The systems it tunes live in `doc/perception.md`, `doc/combat.md` and `doc/movement.md`.

## bot_difficulty.cpp

### One global tier, read fresh every frame

`crossbot_difficulty` is a single server cvar, 0-3, easy/normal/hard/expert. Today's behaviour is
**normal**: easy is a handicap, hard and expert are additive competence. Nothing is per-bot -- the
only per-bot variation in the tree is RNG phase, which randomises texture rather than skill.

The model is the CS bot, vendored at `external/hlsdk-full/game_shared/bot/`: one `cv_bot_difficulty`,
an enum `BOT_EASY..BOT_EXPERT` (`bot_constants.h:27`), tiers that are cumulative.

Every accessor reads its cvar on every call, with no caching. That is deliberate and matches the rest
of the tree: an `rcon crossbot_difficulty 3` has to land on the next frame, and `CBot` has no natural
per-frame refresh point -- `Bot_MoveNav` has ten early returns, so "start of frame" is not one place.
The cost is two `pfnCVarGetPointer` string walks per accessor call, which is noise against the
traceline budget. If `crossbot_perf` ever disagrees, cache the `cvar_t*` pointers in file statics and
keep reading `->value` fresh; do not cache values.

### The ladder

| | easy `0` | normal `1` | hard `2` | expert `3` |
|---|---|---|---|---|
| `aimDelay` mid -> range | 0.40 -> .32-.48 | 0.30 -> .24-.36 | 0.25 -> .20-.30 | 0.20 -> .16-.24 |
| `aimError` scale | 2.00 | 1.00 | 0.60 | 0.35 |
| `turnScale` | 0.85 | 1.00 | 1.10 | 1.20 |
| `oppoInterval` | 0.50 | 0.25 | 0.25 | 0.25 |
| `backPeriod` mid -> range | off | off | 1.50 -> 1.0-2.0 | 1.125 -> .75-1.5 |
| `headPct` | 0 | 0 | 60 | 90 |
| `predictPct` | 0 | 0 | 70 | 90 |
| `combatInterrupt` | yes | yes | yes | **no** |
| `camp` | yes | yes | yes | **no** |
| `ambushOverride` | yes | yes | yes | **no** |
| `pickupPriority` | -- | -- | -- | **yes** |
| `gaussPolicy` | -- | -- | -- | **yes** |
| `bunnyHop` | -- | -- | -- | **yes** |

Easy and normal differ only in the two handicap numbers. Everything easy would otherwise "lose" is
behaviour normal already has, so there is nothing to take away.

Both ranges are one midpoint plus a shared spread constant, so the cvar that pins either stays a
single number: `kDelaySpread = 0.2` and `kBackSpread = 1/3`. The latter reproduces the two requested
back-check bands exactly. Easy's 0.48 stays under `BotAim::kMaxDelay = 0.5`.

**Normal's aim delay midpoint is 0.30, not 0.25.** Hard has to be *shorter* than normal, so normal is
the registered cvar default rather than the old compiled fallback `kDefaultDelay = 0.25`. Those two
disagreed for a long time; this resolves it in favour of the cvar and deletes the constant.

### The pin rule

> A difficulty-derived cvar defaults to `-1`. Negative means *derive from the tier*. Any value `>= 0`
> is an operator override and wins outright; for booleans `0` forces off and `1` forces on.

Same semantics as `crossbot_careful_pin` and `crossbot_range_pin`, applied to the *existing* cvar
rather than adding a second knob for one number. The other direction is not implementable: "did the
operator set this?" is not observable through `cvar_t`, because the engine reallocs `->string` on
every set.

Three boolean reads had to be rewritten rather than just re-defaulted. `BotCamp::Enabled`,
`BotOppo::Enabled` and `BotCombat::AmbushEnabled` were all of the form `cvar->value != 0.f`, which
reads `-1` as *on* and would have silently pinned all three the moment the default changed.

`crossbot_difficulty_status` prints the resolved tier and every row with its effective value and
whether it came from the tier or a pin. That command is the only way to audit the ladder from a live
server, so it exists from the first commit rather than being added later.

### What each tier flag reaches

| flag | site |
|---|---|
| `aimDelay` | `BotAim::Delay`, feeding the lead push in `UpdateVision`, `ApplyLook` and `BotFire::Think` |
| `aimError` | `BotAim::ErrorScale` |
| `turnScale` | `Bot_UpdateFacing`, scaling `kMaxAccelYaw` / `kMaxAccelPitch` / `kMaxHandSpeed` |
| `oppoInterval` | `BotOppo::TryJump`'s retry throttle |
| `backPeriod` | the back-check in `BotSense::ApplyLook` |
| `headPct` | `SamplePoint` via the per-engagement `headLatched` |
| `predictPct` | `BotAim::ReadPredicted` via the per-loss `predictLatched` |
| `combatInterrupt` | the movement takeover in `BotCombat::Run` |
| `camp` | `BotCamp::Enabled`, one choke point over `BuildRoute` / `Consider` / `Run` |
| `ambushOverride` | `BotCombat::AmbushEnabled`, reaching the ambush branch of `BotFire::Think` |
| `pickupPriority` | `BotItems::KeepPreferred`, promoting those items into the fun round |
| `gaussPolicy` | `ChargeTime`, `WeaponScore`, `ModeViable`, `ZoomButtons` and `BotGauss` |
| `bunnyHop` | `BotOppo::WantBunnyHop` |

### The delay is rolled per engagement, not per frame

`BotAim::Delay` returns a value rolled once when the engagement starts, at the same site that already
clears the ring on a target change. This is a correctness constraint, not a flavour one: the delay is
an *offset into a ring buffer*, and re-rolling it per frame would make the served point jitter
backwards and forwards through the history.

The idea is YaPB's (`float reaction[2]`, rolled with `rg()` at `src/botlib.cpp:336`). A single scalar
makes every bot on the server react on the same metronome, which reads as a machine rather than as a
lobby of players. Measured: two bots on crossfire at hard held 0.177 and 0.233 simultaneously, each
stable across frames, and re-rolled into easy's band (0.374) on their next target change.

`aimError` and `turnScale` are also YaPB's idea (`aimError`, and the controller stiffening at
`src/vision.cpp:160`). Without them the tiers differ only in *latency*, which reads as lag rather than
as skill. Unlike YaPB, expert aim error is 0.35 rather than zero: YaPB's aim error is a random offset,
so deleting it just makes the bot accurate, whereas this one is a hand model -- drift phase, quantum,
impulse, brake -- and zeroing it removes the human texture the whole of `bot_aim.cpp` exists to
produce. An expert should be fast and correct, not inhumanly still.

### Head aiming and shooting blind are probabilities, and both latch

`headPct` and `predictPct` are percentages rather than booleans for the reason YaPB's are: a bot that
*always* goes for the head and *always* fires through walls reads as a cheat rather than as a strong
player. Neither reaches 100 even at expert.

Both latch. `headLatched` is rolled once per engagement, because `SamplePoint` re-rolls its bone every
single frame and a per-frame head/body coin flip makes the aim visibly shaky -- YaPB latches for the
same stated reason (`m_enemyBodyPartSet`, `src/combat.cpp:746`). `predictLatched` is rolled once per
loss of sight, or the bot strobes between tracking and forgetting several times a second.

The head bone is found once per model in `BuildBoneSet` by looking for hitgroup 1, so the runtime pick
is an index rather than a search. `TraceVisible` tries it first and falls back to the ordinary uniform
pick when the head trace fails, which costs one extra traceline on the engagements that rolled head
and nothing on the others. Measured on crossfire: expert picked bone 39 on 18 of 18 on-target samples;
normal spread 8 samples over 8 different bones.

### Predicting an enemy you cannot see

`BotFire::Think` gates everything on `BotAim::ReadDelayed`, and the ring is only fed while the target
is visible -- so when an enemy breaks line of sight the queue drains and the bot stops both aiming and
firing. `BotAim::ReadPredicted` wraps it: when the ring is dry it dead-reckons the *newest* slot
forward using the velocity stored beside it.

Reusing the ring is what makes this small. The newest slot already holds the last point actually seen
together with the velocity it had, so there is no new last-known state to maintain and no second
memory to keep in sync with `sense.enemyPos`. The horizon is `kPredictHorizon = 1s`; beyond that the
extrapolation is fiction.

Leading and the decision to shoot need no new code at all. `SolveLead` already leads `AIM_LEAD` modes
off the velocity that comes back with the point, so a predicted position is led exactly like a visible
one, and `RayOnTarget` already boxes the served point and traces for occlusion -- "if the led position
can be hit, shoot" is the existing gate applied to a new point, and a wall between bot and prediction
refuses the shot on its own.

`LOOK_PREDICT` under-reports. `BotFire::AimPoint` is consulted first in `ApplyLook`, so a predicted
shot usually surfaces as `look=aim`. The honest signal is the cumulative `pred=` counter in
`crossbot_aim_status`, for the same reason `visionKeep` has a cumulative tally beside it: a one-frame
sample of a one-second window proves nothing. Measured over equal fight time on crossfire: 69 and 68
predicted reads at expert, 0 at normal.

### The back-check

A periodic weighted-random glance at a watch direction the bot would not otherwise look at, weighted
by `watch[i].amount`, re-rolled every period and held for `kBackCheckHold = 0.4s`. This closes the
sweep gap named below.

**It sits above the suspicion blend, not inside `ApplyVantageLook`.** The first attempt put it in the
vantage branch, which is only reached when there is no live suspicion at all -- and in a busy
deathmatch suspicion is nearly always live, so one bot glanced three times in twenty seconds and the
other never glanced once. Sitting under `LOOK_AIM` and above `LOOK_SENSE` is what makes it the rhythm
it is supposed to be. Two of the exclusions are then free: `Gated` (ladders, buttons, elevators,
triggers in the next three waypoints) and a live aim target both return before it. Combat needs the
explicit `bot.combat.active` test, since a bot in combat with no resolved aim point falls through.

**A pick within `kVantagePathSnap` (60 degrees) of the current desired yaw is rejected and the period
is spent.** That is how the glance coexists with the 60-degree clamp instead of weakening it. The
clamp exists because `Bot_DecomposeMove` builds the movement basis from yaw alone, so an unbounded
snap turns a corridor walk into a sideways strafe; the path branch keeps the clamp and owns everything
*inside* the cone, and the back-check owns everything outside it. The strafe cost is bounded in time
instead -- 0.4s of hold against a 0.75-2.0s period. If it ever proves disruptive the lever is a longer
period, not a shorter hold: the aim controller slews at a finite rate and `Bot_DecomposeMove` is fed
the *actual* post-`Bot_UpdateFacing` yaw, so a 150-degree glance is still swinging back into the next
period.

It has its own RNG stream (`SeedBack`). Drawing from the vision LCG would shift every subsequent bone
and candidate pick, changing sweep determinism at the tiers where the feature is off.

Measured on crossfire: 0 glances over 30s at normal; 19 and 11 over 60s at expert, with the difference
between the two bots being how much of that time each spent `gated`.

### Expert drops the goal for nothing

`combatInterrupt` off means `BotCombat::Run` never takes over movement and never calls
`BotGoal::Clear`. **It does not stop the bot shooting.** `BotFire::Think` runs before the combat
branch and `BotFire::Buttons` is folded into `Bot_RunPlayerMove`, both outside `BotCombat::Run`, so
"shoot and keep running" is what falls out for free. Do not later "fix" this into a full combat
disable. The state still enters, so `crossbot_combat_status` reports a bot that is `active` but never
`driving`, and the gate sets `lastReason = "difficulty"` so the status line says why.

`ambushOverride` off means an expert that catches someone unaware skips the grenade-or-crowbar
override and falls through to the ordinary value-ranked picker -- it opens with its best gun.

### Pickup priority is a promotion, not a filter

The tier loop walks STRATEGIC, FUN, GATHER and stops at the first non-empty tier, so GATHER only ever
wins when nothing else is on offer; ordinary items reach the bot through `PickStop`'s detour scan.
Expert priority uses that structure rather than fighting it: `BotItems::KeepPreferred` reduces a
gather collect to the preferred items only, and `BotGoal::Select` appends that list to the *fun*
round. Preferred items then compete with fun goals in the same `PickWeighted` lottery, and everything
else stays exactly where it was -- available as a detour on the way.

It was a hard filter first, erasing every non-preferred candidate at both call sites. That met the
letter of "only if none of these are available, go for the less interesting ones" and read badly in
play: an expert walked past a battery two steps off its route. The promotion keeps the priority and
gives back the detours.

**The promoted candidates keep `tier = GOAL_GATHER`.** That field is the dispatch key for
`TargetValid`, `CommitStop`'s `Accept` and `OnArrived`; retagging them `GOAL_FUN` to move them
between rounds would route an item through `BotFun::ValidFor` and `BotFun::Accept`. `Select` tracks
the *round* tier separately and uses that for what it actually governs -- weighted versus cheapest,
and whether to build `toTargetCosts`. `PickStop` keys its early return on `toTargetArea < 0` rather
than on the tier label, which is what lets a promoted item accumulate detours the way a fun goal
does. The goal log prints the candidate's own tier and marks a promotion, so the two cases stay
distinguishable on a live server.

The preferred set is game-specific and lives behind `IGameProfile::ItemPreferred`, beside
`MeleeWeapon` and `AmbushWeapon`. HLDM answers yes for the gauss, crossbow and egon, their ammo
entities (resolved through `AmmoEntity` rather than hardcoding the strings a second time),
`item_healthkit`, `item_battery`, `item_longjump`, and every charger and heal volume. The longjump
module is on that list because it feeds the movement code an expert leans on hardest.

**The tier loop is not the only path to an item.** `BotGoal::PickStop` runs a *second*
`BotItems::Collect` for detours whenever a fun or strategic goal is active. Under the old hard
filter that path had to be filtered too, and until it was, an expert on crossfire still collected
`weapon_rpg`, `weapon_9mmAR`, `weapon_357`, `weapon_satchel` and `weapon_snark` as goals. Under the
promotion the detour scan is deliberately left alone -- that scan *is* "still consider the other
pickups". Measured over 105s with three experts: 20 promoted goals (gauss and crossbow ammo,
chargers, batteries, the crossbow itself) alongside 75 detours to ordinary pickups.

### The gauss policy

The numbers come from `external/hlsdk-full/dlls/gauss.cpp`:

- `flDamage = 200 * (charge / 1.5)`, capped at 200 (`:318`). **100 damage is a 0.75s charge.**
- `velocity -= v_forward * flDamage * 5` (`:344`) -- the push is along the *negative* aim, so looking
  down pushes up, and at 100 damage it is exactly 500 units/s. In multiplayer `velocity.z` is not
  restored (`:349`), which is the only reason the jump works at all.
- Burn is one round to start plus one every 0.1s, stopping at full charge (`:228`). A 0.75s charge
  costs about 8 uranium, not the 16 `ShotCost` used to compute from `chargeFull`. `ShotCost` now
  models the charge time actually used, or the bot would refuse its own policy with half a clip.
- Reflect self-damage is `RadiusDamage(flDamage * n)` when `n = -dot(planeNormal, dir) < 0.5`
  (`:438`). For a flat floor `n = sin(pitch)`, so the pitch must exceed 30 degrees to avoid it. The
  jump uses **40-65 degrees** for margin: at 45 the push splits 354 horizontal / 354 up, at 60 it is
  250/433.

Nothing in the policy names the gauss or the crossbow. A charge weapon is one whose
`secondary.fire == FIRE_CHARGE` and a scope weapon is one with a `primaryZoomed` and a `TOGGLE_ZOOM`
secondary, both read straight off the spec table. In HLDM those resolve to exactly the gauss and the
crossbow, and any other game gets sensible behaviour for free. This also keeps `doc/games.md`'s
rejection of a `selfPush` weapon quirk intact: the push is modelled in `bot_gauss.cpp`, in the bot,
not in the spec.

**The gauss has to charge with no target, and that was the missing piece.** `BotFire::Think` idles out
at `no-target` long before the charge path, so a bot holding a gauss and seeing nobody simply carried
it. `IdleCharge` starts a secondary-mode charge under the policy; from then on `holding` is true and
the existing commit machinery keeps it. `ChargeButtons` gained a `releaseCharge` flag so something
other than an on-target verdict can let go of the button.

`BotGauss::Update` probes four random directions every 0.1s while a charge is held with no aim point,
builds `launchVel = velocity - dir * push`, simulates it with `Bot_SimulateAirmoveArc`, and requires a
landing with no fall damage whose `bot.oppoCosts` gain clears `kMinGainSeconds`. That reverse Dijkstra
is `BotOppo`'s, unchanged -- and because the goal is already a preferred item, "does this get me
closer to something worth having" is a question the existing field answers with no second search. The
chosen direction is published through `BotSense::ApplyLook` as `LOOK_GAUSS` rather than written
directly, so it arrives through the normal aim ladder and the hand model still has to get there. If
nothing pays within `kProbeWindow`, it blasts it anywhere.

#### The probe is a forecast; the shot is re-proved

Committing to a direction and firing when the view arrives is not the same event. The bot keeps
running while it slews, so the origin and velocity the probe simulated are stale by the time the
charge is released -- and the charge itself has grown in the meantime. `Update` therefore re-runs
`Evaluate` for the committed yaw and pitch at the moment the view is within `kAimTolerance`, from
where the bot actually stands, and drops the commitment if it no longer pays. The arc that survives
that second evaluation is stored as `gaussPredicted` and compared against where the bot really lands.

That comparison is the only reason the next two defects were found, and both were invisible by
reading. **The push was modelled at a flat 100 damage while the engine applied whatever charge the bot
actually held.** `ChargeButtons` holds the button until something releases it, and the view slew
outlasts the 0.75s policy charge, so the real shot was routinely a full 200-damage, 1000-unit push
against a modelled 500. **And the arc used the duck hull while the bot flies standing**, so the
simulation cleared gaps the real hull hits. First measured jump: predicted (-178, 537), landed
(-769, 830) -- **660 units of flat miss and 174 of vertical**. Modelling the real charge through
`ChargeDamage` and simulating with `kHullHuman` brings that to 14, 124, 141 and 262 units flat with
0, 0, 1 and -33 vertical, and flight times inside 0.05s of prediction.

**Known deviation: the shot lands at 200 damage, not the 100 the policy names.** Aiming now begins the
moment the charge does rather than halfway through it, and the release still waits for
`ChargeDamage >= kPolicyDamage`, but a slew of up to 180 degrees of yaw plus 40-65 of pitch still
outruns the 1.5s to full charge. The prediction is correct either way because it models the charge
actually held; what is *not* consistent is `ShotCost`, which still budgets the 0.75s charge, so an
expert burns roughly twice the uranium per jump that it accounts for. Fixing that properly means
aiming before charging rather than concurrently.

Measured: jumps at pitch 41, 48, 52 and 62 degrees, all inside the 40-65 safe band, no self-damage.

#### At expert the scope is the shot

Under the policy a scope weapon only fires zoomed -- the `unzoomed` gate in `Buttons` -- so the shot
the bot is lining up is always `primaryZoomed`, and for the crossbow that is hitscan `AIM_TARGET`,
not the 2000 u/s `AIM_LEAD` bolt. The aim used to be resolved against `ModeOf(spec, mode,
fire.zoomed)`, the *current* zoom, so for the whole slew before the zoom landed the bot led a
projectile it was never going to fire, then zoomed in with the crosshair off to one side of the
target. `EffectiveZoom` (`fire.zoomed || ScopeCommitted(spec)`) is what the aim ladder in `Select`
and the on-target test in `Buttons` pass now: the mode the shot will use, not the one the weapon is
in. The other `ModeOf` callers keep the live zoom -- the ammo, range and ambush checks read the same
numbers from either crossbow mode.

**The zoom follows the scope's own cone.** `ZoomButtons` zooms in once the aim point is within half
the scope's half-angle of the view (`kZoomConeEnter`, 5 degrees for the crossbow's fov 20) and holds
while it stays within the full half-angle (10 degrees) -- that is, while the target is still in the
picture the scope shows. Anything else unzooms: target dead, lost, or swung out of view. That is
"unzoom after the shot if nobody is in the scope" stated as state rather than as an event, so it
also covers a target lost without a shot. Two thresholds for the same reason as `kZoomIn` /
`kZoomOut`: a single cone flickers on its edge, and each toggle costs a frame and a second of
`m_flNextSecondaryAttack`. Entering at the tighter cone also means the zoom lands with the hand
already on the target, so the 4.5x `FovScale` turn penalty is paid on the fine correction rather
than on the slew.

**The zoom used to stick because nothing ran to take it off.** With no target, `Think` calls `Idle`,
which sets `fire.mode = MODE_NONE`, and `Buttons` returned on that before reaching `ZoomButtons`. The
bot stayed zoomed until its next target, turning at under a quarter of its normal rate in the
meantime. `Buttons` now runs the zoom decision ahead of the `MODE_NONE` and weapon-mismatch returns
whenever the weapon is zoomed. It can only unzoom there -- for a zoomed weapon a nonzero answer from
`ZoomButtons` means "want out" -- so no path that used to return early gains a zoom-in. It also
re-reads `pev->fov` itself, because `Think` has five early returns (off, breaking, deploying, on a
tank, pressing) ahead of the line that refreshes `fire.zoomed`.

Measured on crossfire with four experts and the crossbow pinned, over 80s: `leadshots=0` on all
four, every sampled aim `target`, and the longest zoomed-with-no-target stretch 0.8s -- the cooldown
before the unzoom press is allowed.

### Bunnyhop

Distinct from the opportunistic jump: that one picks a *different* landing area for a shortcut, this
one jumps *along* the path to go faster, with no search. `BotOppo::WantBunnyHop` fires on the ground
at `kBunnySpeedRatio` of maxspeed with a live path, simulates the jump along the current velocity yaw,
and accepts only an arc that lands with no fall damage, within `kBunnyCorridor` of the waypoint
polyline ahead, and at least `kBunnyMinProgress` further *along* that polyline. It adds `IN_JUMP` to
the ordinary walking frame rather than taking movement over, so the path follower keeps steering.

The clean-ground scan runs forward by *distance*, not by waypoint count. A hop covers about 200 units;
counting six waypoints could demand 600 clear, so a door two rooms away refused hops it had no
business refusing. A hop into a ladder, launch, elevator, button, trigger, teleport, break or crouch
waypoint within `kBunnyLookahead` is still refused outright.

**The airstrafe is explicit, not inherited from the path follower.** `AirmoveCorrection` is the only
other in-air steering routine and it runs solely under `RunAirmoveFlight`, which needs
`airmoveActive` -- a flag the hop deliberately does not set, because setting it would hand the bot to
the airmove landing-verdict and connection-penalty machinery. Left to the ordinary path wishdir the
hop bought nothing: air acceleration along the direction you are already moving is almost zero.
`BunnyAirSteer` replaces the wishdir with the horizontal perpendicular to velocity at full drive,
signed *toward* the path line, so the strafe accelerates and corrects at the same time -- which is
also why the curve it introduces does not invalidate the straight-line arc the probe verified. It
needs no view turning: `Bot_DecomposeMove` takes a world wishdir and the current yaw and emits
fmove/smove, so the strafe is pure sidemove and the head stays free for `ApplyLook`.

**Three things kept this near-inert, and only measurement found them.**

The probe simulated `jump = false`. On flat ground the simulation never went airborne, so `landed` was
never true and the hop was refused; the hops that did happen were the arc catching a *walk off a step*
-- flight times of 0.08 to 0.21s and a landing always below the start. With `jump = true` the arc is a
real 0.73s hop covering about 200 units.

The retry throttle was set on every probe, pass or fail. A hop is 0.73s of flight, so the bot is
grounded for one or two frames and a 0.1s window is usually still closed on exactly the frame that
could chain. Only refusals throttle now, and the speed gate -- which runs no simulation -- does not
throttle at all.

The corridor was measured to the nearest *waypoint*. A landing halfway between two waypoints 128
apart is 64 from either and was rejected while sitting dead on the path. It measures to the nearest
*segment* now, and carries the arclength to that point so the hop must also make forward progress.
Without the progress test the bot found pairs of hops that each landed near the path and bounced
between two spots indefinitely -- 162 hops in 60s with an average speed of 0.21 maxspeed.

`PmSim::PreventMegaBunnyJumping` clamps landing speed at 1.7x maxspeed. That is a faithful replication
of the engine's own anti-bhop rule and must not be touched -- it bounds the gain, it does not block it.

**`kBunnySpeedRatio` is 0.75, not the 0.98 the ask implies, and the reason is worth keeping.** Hopping
while slow is normally a trap: ground acceleration reaches maxspeed in about three frames, air
acceleration gains roughly 1.4 units/s per frame, so a bot that locks itself into hopping below
maxspeed is slower than one that runs. What changes it is that `BunnyAirSteer` drives at
`Bot_MaxSpeed` rather than at the follower's `wishSpeed`, so a hop is the one way the bot escapes the
path plan's conservative speed target. Observed speeds run to 349 against a maxspeed of 270.

Measured on crossfire: 0 hops at normal. At expert, 39 and 71 over 90s, every one advancing 139 to 219
units along the path, with visible chains of consecutive hops down a corridor. That is roughly a
third of wall-clock spent airborne, not the "almost always" the ask asked for, and the reason is not
in this function: `refuse=` in `crossbot_vantage_status` counts refusals by gate -- throttled, speed,
waypoint kind, arc, corridor, in that order -- and the speed gate
outnumbers every other gate combined by two orders of magnitude (4543 against 117). Instrumenting the
distribution showed only 8-9% of walk frames sit at 0.9 maxspeed or above and 12-20% at 0.7 or above:
the bot rarely runs flat out, because `SolveDrive` targets the plan profile's speed.

**Raising the path plan's speed was tried and rejected.** The obvious next move is to build the plan
as though maxspeed were higher, so that `NavFollow::Track`'s `target = min(here.speed, worldCap)`
saturates at the real maxspeed everywhere instead of tracking the profile's conservative per-segment
speed. One line in `MakePlanTraces` does it -- scale `traces.physics.maxSpeed` before it reaches
`BuildPathProfile` -- and at expert with a 1.5x factor it measurably worked as intended: over 120s
with three bots on crossfire the average speed rose from 178 to 190 against a maxspeed of 270, and the
share of samples at or above 0.7 maxspeed went from 54% to 60%.

It bought no hops. 472 against 489 is inside the run-to-run spread, because the speed gate is not the
only thing a faster bot changes: the corner slowdowns the profile exists to compute are exactly what
the inflated maxspeed erases, so the bot carries speed into corners it then has to correct out of, and
the corridor and waypoint-kind gates absorb what the speed gate stopped refusing. The cost was
unambiguous. `traces.physics` is the same physics the airmove connection sims run under, so every
stored arc was simulated at a launch speed the bot cannot reach: aborted airmoves went from 9 to 74 in
the same window, and goals reached fell from 87 to 74. The plan profile's speed is load-bearing for
cornering and for airmoves, and overdriving it trades navigation for nothing.

What is left is the gate itself, and it is nearly flat there too: 0.70 and 0.75 measure 472 and 450
hops over the same 120s window, which is noise, and 0.75 reached more goals (98 against 87). The gate
sits at 0.75 because that is the cheaper end of a flat curve, not because the number was found to
matter. Every accepted hop launches at 269 of 270 anyway -- the bot hops when it is already at
maxspeed, and the ratio only decides how much of the ramp-up it is willing to hop through.

### cvars

- `crossbot_difficulty` — the tier, 0-3. Default 1. Truncates, so 2.7 is hard
- `crossbot_back_check` — back-check period midpoint in seconds; 0 disables. Default -1 (tier)
- `crossbot_turn_scale` — scale on the aim controller's acceleration and hand-speed ceiling.
  Default -1 (tier)
- `crossbot_head_pct` — chance per engagement of aiming at the head first. Default -1 (tier)
- `crossbot_predict_pct` — chance per loss of sight of tracking the enemy through cover.
  Default -1 (tier)
- `crossbot_combat_interrupt` — may combat drop the goal and take movement over. Default -1 (tier)
- `crossbot_pickup_priority` — promote the profile's preferred items to compete with fun goals.
  Default -1 (tier)
- `crossbot_gauss` — the charge-weapon policy and the opportunistic gauss jump. Default -1 (tier)
- `crossbot_bunnyhop` — opportunistic bunnyhop. Default -1 (tier)
- `crossbot_oppo_interval` — opportunistic-jump retry throttle in seconds. Default -1 (tier)

`crossbot_aim_delay`, `crossbot_aim_error`, `crossbot_camp`, `crossbot_oppo_jump` and
`crossbot_ambush` now default to -1 and follow the tier; any value `>= 0` still pins them exactly as
before.
