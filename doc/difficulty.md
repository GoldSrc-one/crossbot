# Difficulty

One server-wide tier scales reaction, aim error and opportunism, and switches whole behaviours on or
off. This file owns the ladder, the pin rule and the expert-only behaviours (gauss policy, bunnyhop,
pickup priority). The systems being tuned live in `doc/perception.md`, `doc/combat.md` and
`doc/movement.md`.

Source: `src/bot/bot_difficulty.cpp` (`BotDiff`).

## One global tier, read fresh every frame

`crossbot_difficulty` is 0-3: easy, normal, hard, expert. Normal is the baseline behaviour; easy is a
handicap, hard and expert add competence. Nothing is per-bot: the only per-bot variation is RNG phase,
which varies texture, not skill.

The model is the CS bot (`external/hlsdk-full/game_shared/bot/`): one `cv_bot_difficulty`, an enum
`BOT_EASY..BOT_EXPERT`, cumulative tiers.

Every accessor re-reads its cvar on every call, with no caching:

- An `rcon crossbot_difficulty 3` has to land on the next frame.
- `CBot` has no single per-frame refresh point; `Bot_MoveNav` has many early returns.
- The cost is two `pfnCVarGetPointer` string walks per call, noise next to the traceline budget. If
  `crossbot_perf` ever disagrees, cache the `cvar_t *` pointers in file statics and keep reading
  `->value` fresh. Never cache the values.

## The ladder

| | easy `0` | normal `1` | hard `2` | expert `3` |
|---|---|---|---|---|
| `aimDelay` mid -> range | 0.40 -> .32-.48 | 0.30 -> .24-.36 | 0.25 -> .20-.30 | 0.20 -> .16-.24 |
| `turnScale` | 0.85 | 1.00 | 1.10 | 1.20 |
| `aimGain` | 0.25 | 0.15 | 0.10 | 0.05 |
| `oppoInterval` | 0.50 | 0.25 | 0.25 | 0.25 |
| `backPeriod` mid -> range | off | 2.00 -> 1.33-2.67 | 1.50 -> 1.0-2.0 | 1.125 -> .75-1.5 |
| `headPct` | 10 | 40 | 60 | 90 |
| `predictPct` | 0 | 0 | 70 | 90 |
| `combatInterrupt` | yes | yes | yes | **no** |
| `camp` | yes | yes | yes | **no** |
| `ambushOverride` | yes | yes | yes | **no** |
| `pickupPriority` | -- | -- | -- | **yes** |
| `gaussPolicy` | -- | -- | -- | **yes** |
| `bunnyHop` | -- | -- | -- | **yes** |
| `grenadeCook` | -- | -- | **yes** | **yes** |
| `backstab` | -- | -- | **yes** | **yes** |
| `braceMoving` | -- | -- | -- | **yes** |
| `wallbang` | -- | -- | -- | **yes** |
| `spreadFit` (CS) | 2.00 | 1.50 | 1.25 | 1.00 |
| `spreadEarly..Late` (CS) | -.15..+.25 | -.05..+.15 | 0..+.10 | 0..+.05 |
| `noticeScale` | 1.50 | 1.00 | 0.75 | 0.50 |

- Easy differs from normal only in the handicap numbers. Every behaviour easy might "lose" is one
  normal already has.
- Both ranges are a midpoint plus a shared spread, so the cvar that pins either is one number:
  `kDelaySpread = 0.2`, `kBackSpread = 1/3`. Easy's 0.48 stays under `BotAim::kMaxDelay` (0.5).
- Normal's aim delay is 0.30 so that hard's 0.25 is shorter than normal.

## The pin rule

> A difficulty-derived cvar defaults to `-1`, meaning *derive from the tier*. Any value `>= 0` is an
> operator override and wins outright; for booleans `0` forces off and `1` forces on.

The override lives in the existing cvar rather than a second knob. The reverse ("did the operator
set this?") cannot be implemented: the engine reallocs `cvar_t::string` on every set, so it is not
observable.

Boolean reads must test the tier accessor, never `cvar->value != 0.f`, which reads `-1` as on.
`BotCamp::Enabled` and `BotCombat::AmbushEnabled` go through `BotDiff::Camp` and
`BotDiff::AmbushOverride` for that reason, and `BotOppo` reads `BotDiff::OppoInterval`.

`crossbot_difficulty_status` prints the resolved tier and every row with its effective value and
whether it came from the tier or a pin. It is the only way to audit the ladder on a live server.

## What each tier flag reaches

| flag | site |
|---|---|
| `aimDelay` | `BotAim::Delay`, feeding the lead push in `UpdateVision`, `ApplyLook` and `BotFire::Think` |
| `turnScale` | `Bot_UpdateFacing`, scaling `kMaxAccelYaw` / `kMaxAccelPitch` / `kMaxHandSpeed` |
| `aimGain` | `BotAim::GainSpread`, the per-flick motor gain mismatch rolled in `UpdateGain`; see `doc/perception.md`, "The efference copy is wrong on purpose" |
| `oppoInterval` | `BotOppo::TryJump`'s retry throttle |
| `backPeriod` | the back-check in `BotSense::ApplyLook` |
| `headPct` | `SamplePoint`, via the per-engagement `headLatched` |
| `predictPct` | `BotAim::ReadPredicted`, via the per-loss `predictLatched` |
| `combatInterrupt` | the movement takeover in `BotCombat::Run` |
| `camp` | `BotCamp::Enabled`, one choke point over `BuildRoute` / `Consider` / `Run` |
| `ambushOverride` | `BotCombat::AmbushEnabled`, reaching the ambush branch of `BotFire::Think` |
| `pickupPriority` | `BotItems::KeepPreferred`, promoting preferred items into the fun round |
| `gaussPolicy` | `ChargeTime`, `WeaponScore`, `ModeViable`, `ZoomButtons` and `BotGauss` |
| `bunnyHop` | `BotOppo::WantBunnyHop` |
| `grenadeCook` | `BotGrenade`: hold an off-hand grenade for its fuse instead of throwing at once |
| `backstab` | `StartStab` in TFC (spy knife) and DoD (bayonet or rifle butt) |
| `braceMoving` | `BotFire::Brace`: in OP4, brace by clearing the move keys on the firing frame only |
| `wallbang` | `RayOnTarget`: a blocked ray may fire if `IGameProfile::Penetrates` says the round gets through |

## Aim delay is rolled per engagement

`BotAim::Delay` returns a value rolled once when an engagement starts, at the site that clears the
ring on a target change. This is a correctness constraint: the delay is an offset into a ring buffer,
and re-rolling per frame would make the served point jitter back and forth through history.

The idea is YaPB's (`float reaction[2]`, `src/botlib.cpp:336`). One scalar for everyone makes every
bot react on the same metronome. Measured: two bots on crossfire at hard held 0.177 and 0.233, stable
across frames, and re-rolled on their next target change.

`aimGain` and `turnScale` are also from YaPB (`aimError`, and the controller stiffening at
`src/vision.cpp:160`). Without them the tiers differ only in latency, which reads as lag rather than
skill. Expert keeps an aim error of 0.05 instead of YaPB's zero: YaPB's error is a random offset, so
zero just means accurate, while ours is a motor mismatch the bot must see and correct a reaction time
later. An expert should be fast and correct, not inhumanly exact.

## Head aim and blind fire are probabilities, and both latch

`headPct` and `predictPct` are percentages, not booleans: a bot that always goes for the head and
always fires through walls reads as a cheat. Neither reaches 100.

- `headLatched` is rolled once per engagement. `SamplePoint` re-rolls its bone every frame, and a
  per-frame head/body flip makes the aim visibly shaky (YaPB latches for the same reason,
  `m_enemyBodyPartSet`, `src/combat.cpp:746`).
- `predictLatched` is rolled once per loss of sight, or the bot strobes between tracking and
  forgetting.

The head bone is found once per model in `BuildBoneSet` (hitgroup 1), so the runtime pick is an
index. `headLatched` only orders the attempts: `TraceVisible` tries the head (or the uniform pick when
it rolled body) and, if that trace is blocked, tries the other, costing one extra traceline on an
obstructed first pick. Measured on crossfire: expert picked bone 39 on 18 of 18 on-target samples;
normal spread 8 samples over 8 bones.

## Predicting an enemy you cannot see

The aim ring is only fed while the target is visible, so when the enemy breaks line of sight
`BotAim::ReadDelayed` runs dry and the bot stops aiming and firing. `BotAim::ReadPredicted` wraps it:
when the ring is dry it dead-reckons the newest slot forward using the velocity stored beside it, up
to `kPredictHorizon` (1 s). Past that the extrapolation is fiction.

Reusing the ring keeps this small: the newest slot already holds the last seen point and its
velocity, so there is no second last-known state to keep in sync with `sense.enemyPos`. Shooting
needs no new code either. `SolveLead` leads the predicted point like a visible one, and `RayOnTarget`
already boxes the served point and traces for occlusion, so a wall between bot and prediction refuses
the shot by itself (unless `wallbang` lets it through, `doc/combat.md`, "Wall-banging").

`LOOK_PREDICT` under-reports, because `BotFire::AimPoint` is consulted first in `ApplyLook` and a
predicted shot usually shows as `look=aim`. Use the cumulative `pred=` counter in
`crossbot_aim_status`. Measured over equal fight time on crossfire: 69 and 68 predicted reads at
expert, 0 at normal.

## The back-check

A periodic glance at a watch direction the bot would not otherwise look at, picked at random weighted
by `watch[i].amount`, re-rolled every period and held for `kBackCheckHold` (0.4 s). It closes the
sweep gap in `doc/perception.md`, "Two gaps worth knowing about".

- **It sits above the suspicion blend, not in `ApplyVantageLook`.** The vantage branch only runs with
  no live suspicion, which in deathmatch is almost never (measured there: three glances in 20 s for
  one bot, none for the other). Sitting under `LOOK_AIM` and above `LOOK_SENSE` makes it a steady
  rhythm, and `Gated` (ladders, buttons, elevators, triggers in the next three waypoints) and a live
  aim target both return before it for free. Combat needs an explicit `bot.combat.active` test,
  because a bot in combat with no resolved aim point falls through.
- **A pick within `kVantagePathSnap` (60°) of the desired yaw is rejected and the period is spent.**
  The path branch keeps its 60° clamp (an unbounded snap turns a corridor walk into a strafe, since
  `Bot_DecomposeMove` builds the move basis from yaw alone) and owns everything inside the cone; the
  back-check owns everything outside. The strafe cost is bounded in time: 0.4 s of hold per 0.75-2.0 s
  period. If it proves disruptive, lengthen the period rather than shorten the hold: the aim slews at
  a finite rate, so a 150° glance is still swinging back into the next period.
- It has its own RNG stream (`SeedBack`). Drawing from the vision LCG would shift every later bone and
  candidate pick at the tiers where the feature is off.

Measured on crossfire at expert: 19 and 11 glances over 60 s, the difference being time spent
`gated`.

## Expert keeps its goal in a fight

With `combatInterrupt` off, `BotCombat::Run` never takes movement over and never calls
`BotGoal::Clear`. It does not stop the bot shooting: `BotFire::Think` and `BotFire::Buttons` run
outside `BotCombat::Run`, so "shoot and keep running" falls out. Do not turn this into a full combat
disable. Combat still enters, so `crossbot_combat_status` shows a bot that is `active` but never
`driving`, with `lastReason = "difficulty"`.

With `ambushOverride` off, an expert that catches someone unaware skips the grenade-or-melee opener
and uses the ordinary picker: it opens with its best gun.

## Pickup priority is a promotion, not a filter

`BotGoal::Select` walks STRATEGIC, FUN, GATHER and stops at the first non-empty tier, so ordinary
items mostly reach a bot through `PickStop`'s detour scan. Expert priority works with that:
`BotItems::KeepPreferred` reduces a gather collect to the preferred items, and `Select` appends them
to the *fun* round, where they compete with fun goals in the same `PickWeighted` lottery. Everything
else stays a detour.

- **Why not a filter.** A hard filter that erases non-preferred candidates makes an expert walk past a
  battery two steps off its route. The promotion keeps the priority and keeps the detours. The detour scan (`PickStop` runs a second `BotItems::Collect` whenever a fun or
  strategic goal is active) is deliberately left unfiltered; it *is* "still consider the other
  pickups".
- **Promoted candidates keep `tier = GOAL_GATHER`.** That field dispatches `TargetValid`,
  `CommitStop`'s `Accept` and `OnArrived`; retagging them `GOAL_FUN` would route an item through
  `BotFun::ValidFor` and `BotFun::Accept`. `Select` tracks the round tier separately for what it
  governs (weighted versus cheapest, and whether to build `toTargetCosts`). `PickStop` keys its early
  return on `toTargetArea < 0` rather than the tier label, so a promoted item gathers detours like a
  fun goal. The goal log prints the candidate's own tier and marks the promotion.

The preferred set is per game, behind `IGameProfile::ItemPreferred`. HLDM: gauss, crossbow, egon,
their ammo (resolved through `AmmoEntity`), `item_healthkit`, `item_battery`, `item_longjump` (it
feeds the movement an expert leans on hardest), and every charger and heal volume. Gearbox adds the
displacer and the sniper rifle (`doc/games.md`, "Opposing Force").

Measured over 105 s with three experts on crossfire: 20 promoted goals (gauss and crossbow ammo,
chargers, batteries, the crossbow) alongside 75 detours to ordinary pickups.

## The gauss policy

Expert charges the gauss deliberately and uses it to jump. Nothing in the policy names a weapon: a
charge weapon is one whose `secondary.fire == FIRE_CHARGE`, a scope weapon one with a `primaryZoomed`
and a `TOGGLE_ZOOM` secondary. In HLDM that is exactly the gauss and the crossbow. The push is
modelled in `bot_gauss.cpp`, not as a weapon quirk, which keeps the `selfPush` rejection in
`doc/games.md` intact.

The numbers, from `external/hlsdk-full/dlls/gauss.cpp`:

- `flDamage = 200 * (charge / 1.5)`, capped at 200 (`:318`). 100 damage is a 0.75 s charge.
- `velocity -= v_forward * flDamage * 5` (`:344`): the push is along the negative aim, so looking down
  pushes up; 100 damage is 500 u/s. Multiplayer does not restore `velocity.z` (`:349`), which is why
  the jump works at all.
- Burn is one round to start plus one per 0.1 s, stopping at full charge (`:228`). A 0.75 s charge
  costs about 8 uranium; `ShotCost` models the charge time actually used, or the bot would refuse its
  own policy with half a clip.
- Reflect self-damage is `RadiusDamage(flDamage * n)` when `n = -dot(planeNormal, dir) < 0.5`
  (`:438`). On a flat floor `n = sin(pitch)`, so pitch must exceed 30°. The jump uses 40-65° for
  margin: at 45 the push splits 354 horizontal / 354 up, at 60 it is 250/433.

### Charging with no target

`BotFire::Think` idles out at `no-target` before any charge path, so without help a bot with a gauss
and nobody in sight just carries it. `IdleCharge` starts a secondary charge under the policy; from
then on the commit machinery keeps it (`doc/combat.md`, "The charge is carried, not timed").
`ChargeButtons` takes a `releaseCharge` flag so something other than an on-target verdict can let go.

The charge is for enemies first; the jump gets what is left. `IdleCharge` only starts while
`BotGauss::Scared` holds (a live player or monster suspicion), and while it holds `BotGauss::Update`
does not probe: holding a full charge is free. Three ways out:

- The enemy shows up: the ordinary on-target release.
- The enemy was seen and broke line of sight: the prediction keeps an aim point, and the release goes
  through the wall if `Penetrates` proves the punch (`doc/combat.md`, "Wall-banging"). A wall that does
  not punch keeps the charge held.
- The suspicion decays, or the hold reaches `chargeMax - kProbeWindow - kOverchargeSlack` (7 s of the
  gauss's 10): the probes run and the charge becomes a jump, well before `charge-timeout` would spend
  it blind.

### The jump probe

While a charge is held with no aim point, `BotGauss::Update` probes four random directions every
0.1 s: `launchVel = velocity - dir * push`, simulated with `Bot_SimulateAirmoveArc`. A direction is
accepted if it lands with no fall damage and its `bot.oppoCosts` gain clears `kMinGainSeconds`. That
reverse Dijkstra is `BotOppo`'s, and since the goal is already a preferred item it answers "does this
get me closer to something worth having" with no second search. The direction is published through
`BotSense::ApplyLook` as `LOOK_GAUSS`, so the hand model still has to get there. If nothing pays
within `kProbeWindow`, the bot fires it anywhere.

The probe is a forecast; the shot is re-proved. The bot keeps running while it slews and the charge
keeps growing, so once the view is within `kAimTolerance`, `Update` re-runs `Evaluate` from where the
bot actually stands and drops the commitment if it no longer pays. The surviving arc is stored as
`gaussPredicted` and compared with where the bot lands.

Two modelling choices make the prediction hold, and that comparison is how to check them:

- **The push uses the charge actually held** (`ChargeDamage`), not the policy's 100. The slew usually
  outlasts the policy charge, so the real shot is often 200 damage, a 1000 u/s push.
- **The arc is simulated with `kHullHuman`**, because the bot flies standing; the duck hull clears gaps
  the real hull hits.

Modelling a flat 100 with the duck hull missed by 660 units flat and 174 vertical; with both choices,
measured misses are 14-262 flat and 0 to -33 vertical, with flight times within 0.05 s.

**Known deviation: the shot lands at 200 damage, not the 100 the policy names.** Aiming starts with
the charge, and the release waits for `ChargeDamage >= kPolicyDamage`, but a slew of up to 180° yaw
plus 40-65° pitch outruns the 1.5 s to full charge. The prediction stays correct because it models the
charge held; `ShotCost` still budgets 0.75 s, so an expert burns about twice the uranium per jump it
accounts for. The fix is to aim before charging rather than concurrently.

Measured: jumps at pitch 41, 48, 52 and 62°, all in the safe band, no self-damage.

### At expert the scope is the shot

Under the policy a scope weapon only fires zoomed (the `unzoomed` gate in `Buttons`), so the shot is
always `primaryZoomed`: for the crossbow, hitscan `AIM_TARGET` rather than the 2000 u/s bolt. The aim
ladder in `Select` and the on-target test in `Buttons` therefore use `EffectiveZoom`
(`fire.zoomed || ScopeCommitted(spec)`), the mode the shot will use. Resolving against the live zoom
would lead a bolt the bot never fires for the whole slew, then zoom in off target. Other `ModeOf` callers
keep the live zoom; the ammo, range and ambush checks read the same numbers from either mode.

The zoom follows the scope's own cone. `ZoomButtons` zooms in once the aim point is within half the
scope's half-angle (`kZoomConeEnter`, 5° for the crossbow's fov 20) and holds while it is within the
full half-angle (10°). Anything else unzooms: target dead, lost or out of the picture. Two thresholds
for the same reason as `kZoomIn`/`kZoomOut`: one cone flickers, and each toggle costs a frame and a
second of `m_flNextSecondaryAttack`. Entering at the tighter cone also means the 4.5x `FovScale` turn
penalty is paid on the fine correction, not the slew.

Unzooming must run with no target. With no target, `Think` calls `Idle` (`fire.mode = MODE_NONE`),
so `Buttons` runs the zoom decision ahead of the `MODE_NONE` and weapon-mismatch returns whenever the
weapon is zoomed; otherwise the bot would stay zoomed, turning at under a quarter rate, until its next
target. There it can only unzoom, so no early-return
path gains a zoom-in. It re-reads `pev->fov` itself, because `Think` has early returns (off, breaking,
deploying, on a tank, pressing) ahead of the line that refreshes `fire.zoomed`.

Measured on crossfire, four experts with the crossbow pinned, 80 s: `leadshots=0` on all four, every
sampled aim `target`, longest zoomed-with-no-target stretch 0.8 s (the cooldown before the unzoom
press).

## Bunnyhop

The opportunistic jump picks a different landing area for a shortcut; the bunnyhop jumps along the
path to go faster, with no search. `BotOppo::WantBunnyHop` fires on the ground at `kBunnySpeedRatio`
of maxspeed with a live path, simulates a jump along the current velocity yaw, and accepts an arc that
lands with no fall damage, within `kBunnyCorridor` of the waypoint polyline ahead, and at least
`kBunnyMinProgress` further along it. It adds `IN_JUMP` to the ordinary walking frame, so the path
follower keeps steering.

- **The clean-ground scan runs by distance, not waypoint count.** A hop covers about 200 units; six
  waypoints could demand 600 clear and refuse hops for a door two rooms away. A ladder, launch,
  elevator, button, trigger, teleport, break or crouch waypoint within `kBunnyLookahead` refuses the
  hop.
- **The airstrafe is explicit.** `AirmoveCorrection` only runs under `RunAirmoveFlight`, which needs
  `airmoveActive`; the hop deliberately does not set it, or it would inherit the airmove landing
  verdict and connection penalties. Air acceleration along the current velocity is almost zero, so
  `BunnyAirSteer` drives full speed along the horizontal perpendicular to velocity, signed toward the
  path line. It accelerates and corrects at once, which is why the curve does not invalidate the
  straight-line arc the probe checked. It is pure sidemove (`Bot_DecomposeMove` takes a world wishdir),
  so the head stays free for `ApplyLook`.
- **The probe simulates `jump = true`.** With `false` the arc never leaves flat ground and the only
  "hops" are walks off steps.
- **Only refusals throttle.** A hop grounds the bot for one or two frames, so a throttle set on every
  probe would still be closed on the frame that could chain. The speed gate runs no simulation and
  does not throttle at all.
- **The corridor is measured to the nearest segment, with arclength progress.** Measuring to the
  nearest waypoint rejects landings dead on the path between two waypoints 128 apart. Without the
  progress test the bot bounces between two spots indefinitely (measured: 162 hops in 60 s at 0.21
  maxspeed).

`PmSim::PreventMegaBunnyJumping` clamps landing speed at 1.7x maxspeed. It replicates the engine's own
rule and must not be touched; it bounds the gain without blocking it.

**`kBunnySpeedRatio` is 0.75.** Hopping while slow is normally a trap: ground acceleration reaches
maxspeed in about three frames, air acceleration gains about 1.4 u/s per frame. It pays here because
`BunnyAirSteer` drives at `Bot_MaxSpeed` instead of the follower's `wishSpeed`, the one way the bot
escapes the path plan's conservative speed target (observed up to 349 against a maxspeed of 270).
0.70 and 0.75 measure the same within noise (472 vs 450 hops in 120 s; 0.75 reached more goals, 98 vs
87). Every accepted hop launches at about 269 of 270 anyway; the ratio only decides how much of the
ramp-up the bot will hop through.

Measured on crossfire: 0 hops at normal; at expert 39 and 71 over 90 s, each advancing 139-219 units,
with visible chains down corridors. That is about a third of wall-clock airborne. The limit is the
speed gate: `refuse=` in `crossbot_vantage_status` counts refusals per gate (throttled, speed,
waypoint kind, arc, corridor), and speed outnumbers the rest combined by two orders of magnitude (4543
vs 117). Only 8-9% of walk frames sit at 0.9 maxspeed or above, because `SolveDrive` targets the plan
profile's speed.

**Rejected: raising the plan's speed.** Scaling `traces.physics.maxSpeed` by 1.5 in `MakePlanTraces`
before `BuildPathProfile` raises average speed from 178 to 190 (of 270; 120 s, three experts,
crossfire) but buys no hops (472 vs 489, inside run-to-run spread): the inflated profile erases the
corner slowdowns, and the corridor and waypoint gates absorb the difference. It also breaks navigation,
because `traces.physics` is what the airmove connection sims run under: aborted airmoves 9 → 74, goals
reached 87 → 74. The plan profile's speed is load-bearing for
cornering and airmoves.

## cvars

All default to `-1` (follow the tier) unless noted; see the pin rule.

- `crossbot_difficulty` — the tier, 0-3. Default 1. Truncates, so 2.7 is hard.
- `crossbot_aim_delay` — aim delay midpoint in seconds.
- `crossbot_turn_scale` — scale on the aim controller's acceleration and hand-speed ceiling.
- `crossbot_aim_gain` — spread of the per-flick motor gain mismatch, clamped [0, 0.5]; 0 makes the
  efference copy exact.
- `crossbot_oppo_interval` — opportunistic-jump retry throttle in seconds.
- `crossbot_back_check` — back-check period midpoint in seconds; 0 disables.
- `crossbot_head_pct` — chance per engagement of trying the head bone first.
- `crossbot_predict_pct` — chance per loss of sight of tracking the enemy through cover.
- `crossbot_combat_interrupt` — may combat drop the goal and take movement over.
- `crossbot_camp` — camping.
- `crossbot_ambush` — the ambush opener.
- `crossbot_pickup_priority` — promote the profile's preferred items to compete with fun goals.
- `crossbot_gauss` — the charge-weapon policy and the gauss jump.
- `crossbot_bunnyhop` — opportunistic bunnyhop.
- `crossbot_grenade_cook` — hold an off-hand grenade for its fuse.
- `crossbot_backstab` — TFC spy knife and DoD bayonet/rifle-butt attacks on backs.
- `crossbot_brace_moving` — in OP4, brace only on the firing frame so the bot keeps moving.
- `crossbot_wallbang` — fire through a wall when the round provably gets through (CS bullets, the
  charged gauss). See `doc/combat.md`, "Wall-banging".
- `crossbot_spread_fit` — in CS, the largest allowed spread cone as a multiple of the target's half
  width; 0 turns the spread gate and the CS brace off. See `doc/combat.md`, "The CS spread gate".
- `crossbot_notice` — seconds to notice an enemy 45° off-axis at 1000 units; 0 makes acquisition
  instant. See `doc/perception.md`, "Notice delay".
