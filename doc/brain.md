# Brain

The team brain lets humans talk to bots like teammates: team and all chat, CS radio, DoD voice commands,
hand signals, and voice chat. A cheap classifier (JEV) decides whether anything should happen, and an LLM decides
what, through a small set of game-agnostic tools that give bots orders. Nothing about who answers,
how many bots follow, or whether a bot is willing is hard-coded: the brain sees each bot's state and
decides. Both models are reached through OpenRouter with one key.

Code: `src/bot/bot_directive.*` (orders on a bot), `src/brain/*` (everything else).

## The pieces

```
comms + game log ──> BrainFeed (ring of events) ──> Brain::Frame (game thread)
                                                     trigger? ignored? rate ok? humans present?
                                                     └─ BrainContext::Build -> Job ──> BrainWorker (thread)
                                                                                          JEV  ─ no ─> verdict only
                                                                                          └ yes ─> LLM (tools)
Brain::Frame <── Result (verdict, tool calls) <───────────────────────────────────────────┘
  └─ ignore / recheck / BrainTools::Execute(bot, tool, args) -> BotDirective
```

## bot_directive.cpp: orders a bot can be given

A **directive** is an ordered list of up to 8 steps that replaces whatever the bot was doing. The bot
returns to its own play when the list ends, when it dies (released on respawn), when the list's
`seconds` run out (default 120), or on `release`. Steps:

| step | done when | how |
|---|---|---|
| `go(target, within)` | within `within` (64) horizontally and 72 vertically, of the target or of the nav point it snapped to | a `GOAL_DIRECTIVE` goal through `BotGoal::Commit` |
| `follow(player, within, seconds)` | `seconds` (60) elapsed | as `go`; idles while within `within` (128), resumes past 1.5x |
| `hold(target, watch, stance, seconds)` | held `seconds` (30) | walks to the nearest camp spot within 384u of the target, then stands facing `watch` unless a live suspicion pulls the eyes; see "Held spots" |
| `look(target, seconds)` | `seconds` (2) | faces the target |
| `select(item)` | the weapon is active (2s timeout) | `IGameProfile::SelectWeapon` |
| `drop(item)` | 0.3s after `drop <item>` | every game takes `drop <classname>`; the dropped box is ignored by item gathering for 20s so the bot does not pick it straight back up |
| `ability(name, arg, target)` | the profile's ability task ends, or at once for a command ability | `IGameProfile::Abilities` / `StartAbility`; a command ability with `args` sends `<command> <arg>` only after `AbilityArg` accepted and normalised the argument |
| `fire(item, target, count, seconds)` | `count` (1) shots or throws counted | a weapon goes through `BotFire::Order`, `gren1` / `gren2` through `BotGrenade::Order`; fails at `seconds` (10), on `hold_fire`, a non-melee item under `melee_only`, or a teammate as target |
| `place(item, target)` | `BotDeploy` counted a placement | walks within 64 of the target, then `BotDeploy::Reserve` picks a gate, the bot walks to its stand point on the mesh, and `StartReserved` mines it |
| `wait(seconds)` | `seconds` (2) | stands still |

Abilities per game: CS `plant`; DoD `dropammo`; TFC `discard`, `dropitems`, `saveme`, `heal`,
`build_sentry` / `_dispenser` / `_entrance` / `_exit`, `detsentry`, `detdispenser`,
`detentryteleporter`, `detexitteleporter`, `detpack`, `detpipe`, `feign`, `disguise_enemy`,
`disguise_friendly`. TFC command names were checked against the decompiled `tfc.dll`; the
disguise argument is a class name or number, sent as the number.

**An ordered step owns the ability slot.** While the current step is `ability`, `fire` or `place`,
`BotDirective::Busy` stops the TFC and DoD profiles and `BotDeploy::TryStart` from starting anything
of their own, or a medic would heal and an engineer repair forever before the order got a frame
(`AbilityRun` and `BotDeploy::Run` run before `BotDirective::Run`). A task already running finishes
first.

**Ordered builds** use the bot's facing, so `look` first aims the sentry; the sentry is turned to face
that way after it is up, as on a camp. An ordered `heal` needs the patient within reach and in the same
nav area, so `go` to them first. An ordered `detpack` takes the fuse as `arg` (5-50, default 20) and
skips the objective/scared gate but keeps the teammate-clear check.

**Ordered fire.** `BotFire::Order` gives `Think` a point when there is no sense target: it selects the
weapon, runs `TryKeep` on the point as if it were a 16u box, and lets a thrown weapon go through the
usual `Intend` / COMMIT_THROW path, so arcs, splash safety and team-unsafe vetoes are the combat ones.
The step's `IdleLooking` turns the bot through `ApplyLook`, which already prefers the fire aim.
`BotGrenade::Order` primes the named offhand slot without the cooldown or the 35% prime chance and
throws at the point; the friend-near-landing check still holds the throw.

**Ordered placement** reuses the autonomous mine code, so it only succeeds where `FindSpot` /
`FindDropSpot` find a gate in the bot's area or one connected to it (the nearest wins); it runs even
with `crossbot_deploy 0` and under `hold_fire`, but not under `melee_only`.

**Held spots.** On arrival `hold` calls `BotCamp::HoldDirected`, which fills `bot.camp.spot` from the
camp spot (duck, bipod) as an autonomous camp would and sets `camp.directed`. `HeldBipod` answers for
it, so a DoD machine gunner deploys at sandbags or prone; TFC `StartBuild`'s camp branch builds there
too. `stance` overrides: `crouch` ducks, `stand` drops both, `prone` (only where `CanProne`, i.e. DoD)
goes prone and deploys a machine gun. Any step ending clears it with `EndDirected`.

### What players ask for, and what answers it

| request | game | how |
|---|---|---|
| follow / cover me, go to X, hold X watching Y, rush, stack, rotate, fall back | all | go, follow, hold; one act per bot |
| give me your gun / the bomb / the flag | all | go me within 96, look me, drop; TFC `dropitems` |
| knives only, hold fire, leave the objective to me | all | rules |
| plant here | CS | go aim, `plant` |
| defuse, retake | CS | go to the site; a planted bomb makes the site urgent and the bot defuses on its own |
| ammo please | DoD, TFC | `dropammo`, `discard` |
| medic! / heal me | TFC | the medic heals hurt teammates nearby on its own; or go me, `heal` target me |
| sentry / dispenser / teleporter here | TFC | go aim, look the way it should guard, `build_*` |
| blow your sentry, detonate | TFC | `detsentry` etc., `detpipe` |
| detpack this | TFC | go aim, `detpack` with a fuse |
| disguise as X, play dead | TFC | `disguise_enemy` / `disguise_friendly` with a class, `feign` |
| deploy the MG here, go prone | DoD | hold aim watching the threat, stance prone, or a sandbag camp spot |
| crouch here | all | hold, stance crouch |
| flash / smoke / nade there, throw a satchel over there, shoot that | all | fire with the grenade or weapon classname, or gren1 / gren2 in TFC |
| tripmine / satchel there | HL, OP4 | place |
| buy X | CS | not supported |

Requests TFC players make of whole roles (scouts capping, a spy sapping) are left to the bots' own play.

**Combat still wins.** `BotDirective::Run` sits after `BotCombat::Run` in `Bot_MoveNav`, so a fight
interrupts a step and the step resumes after; `BotCombat` clearing the goal is why `go` re-commits.

**Moving targets.** A target that is a player is re-committed when it has moved more than
`max(128, 0.3 x distance)` from where it was committed, never during a maneuver and at most twice a
second. A fixed 128 made a follower re-plan from scratch every half second while chasing a running
leader and fall 2000u behind; scaling with distance keeps it within 80u once it catches up.

**Arrival against the snapped point.** A bomb-site target is the brush centre, often inside a wall or
in the air; the goal is snapped to the nearest walkable spot. Arrival also accepts that snapped point,
otherwise the bot stands on it forever re-committing. The snap only clamps x and y, so an `aim` point on a
wall 90u up kept its height and never arrived; standing in the snapped area within `within`
horizontally counts too.

**Route failure.** `OnRepathFailed` / `OnBlocked` on a directive goal count a failure; the third fails
the step and drops the whole directive.

**Rules** are standing limits, separate from steps, cleared at a CS round start and otherwise after
`seconds` (300): `melee_only` (only the melee weapon passes `BotFire::ModeViable`; no grenades, no
deploying), `hold_fire` (combat exits, fire idles, no grenades), `no_auto_objective` (the CS profile
does not start planting on its own; only an `ability(plant)` step does).

**Alert** makes `BotSense::ApplyLook` face a point for a while when no live suspicion has authority.
It is a gaze override, not a suspicion: suspicions are keyed by entity throughout `bot_sense.cpp`, and a
positional one would have touched every consumer.

**Queued commands.** `say`, `say_team` and radio are queued on the bot and run from `ThinkOneBot`,
inside the bot's game context, because `FakeCmd` is not re-entrant and a tool call can arrive in the
middle of another hook.

## brain_feed.cpp: what happened

One ring of 256 timestamped events per map: chat, radio, kills, objectives, rounds, scores, presence.
Each carries the actor's name, userid, team, game, alive/dead, and whether it is a human.

- **Game log.** Every game reports its events through `UTIL_LogPrintf`, which is `ALERT(at_logged)`
  whether or not `log on` is set. A `pfnAlertMessage` post hook reads the standard HL log grammar and
  rewrites `"Name<uid><auth><team>"` to `Name[team]`. CS gives kills, round start/end with the win
  reason and score, bomb pickup/drop/plant/defuse; DoD gives kills, control point captures by name and
  scores; TFC gives kills and its goal triggers by name. `Round_Start` also clears every bot's rules.
- **Chat** comes from a `pfnClientCommand` pre hook, not from the log, because CS only logs chat under
  `mp_logmessages`, which is operator config. Bots' own lines are recorded where they are issued:
  `FakeCmd` calls the game DLL directly and never passes plugin hooks.
- **CS radio** is the `SendAudio` user message (`%!MRAD_*`), so every input path (21 aliases and
  `radio1-3` + `menuselect`) is caught after the game's own alive check and flood limit. One copy goes
  to each teammate; copies within 0.5s are one event. `MRAD_FIREINHOLE` is a grenade throw, not a
  call, and is dropped. Radio within 2s of a round start is marked automatic: CS plays "Go go go" for a
  random player every round, and it must not wake the brain.
- **DoD voice** is a playback event (`events/misc/usvoice.sc`, `gervoice.sc`) with the command in
  `iparam1` (1..27, the order of `s_VoiceCommands`, confirmed live for five of them). CrossPlay rewrites
  event indices per game, so they are learned from a `pfnPrecacheEvent` post hook. **Hand signals**
  are the `HandSignal` user message (sender, signal).
- **Humans.** The game's spawn code clears `FL_FAKECLIENT`, so "is a bot" is: a CrossBot bot, or the
  engine's `BOT` auth id.

## brain_voice.cpp: voice chat

Players' voice is transcribed and enters the feed as a `voice` event, which wakes the brain like a chat
line. Bots answer out loud with the `speak` tool. Both directions go through OpenRouter's audio
endpoints with the same key: `openai/gpt-4o-mini-transcribe` for speech to text and the free
`fish-audio/s2.1-pro-free:free` for text to speech (about 1-2s, raw 16-bit PCM whose rate is in the
`Content-Type`, 44100 today). The speech endpoint only offers `mp3` and `pcm`; there is no WAV.

**Picking the speech to text model.** Measured 2026-10-09 on eight saved short lines, two rounds each:

| model | p50 | max | text |
|---|---|---|---|
| `openai/gpt-4o-mini-transcribe` | 0.76s | 1.71s | good; grunts come back empty, so they never wake the brain |
| `mistralai/voxtral-mini-transcribe` | 0.41s | 7.87s | good, but long tail and some Chinese |
| `openai/whisper-1` | 1.02s | 1.88s | good, but grunts become "Thank you." |
| `openai/whisper-large-v3-turbo` | 1.48s | 10.49s | fair; once took 42s |
| `qwen/qwen3-asr-0.6b` | 3.35s | 5.19s | grunts and Czech become Chinese; 0.3-0.8s on 2026-10-06, so provider latency drifts |

The HTTP calls run on three worker threads, so a transcription never waits behind another one or behind
a bot's text to speech.

**Needs ReHLDS.** The engine has no plugin hook for `clc_voicedata`. ReHLDS's `HandleNetCommand`
hookchain sees every client message by opcode; for opcode 8 the hook reads the length and payload
through `MSG_ReadShort` / `MSG_ReadBuf`, puts the read cursor and bad-read flag back, and calls the
next hook, so the engine still relays the voice to everyone as before. Sending writes
`svc_voicedata` (53), sender client index, length and payload into each listener's datagram, as
`SV_ParseVoiceData` does. On another engine voice is off and `speak` falls back to team chat.

**The wire format.** ReHLDS announces an empty codec, so Steam clients send Steam voice: an 8-byte
SteamID, opcodes, and a CRC32 of everything before it. Opcode 11 is the sample rate (uint16), 6 is
Opus with loss concealment: a uint16 length, then frames of [uint16 size][uint16 sequence][Opus
packet], with size 0xFFFF ending the stream. 4 (SILK, older clients) is recognised but not decoded.
Packets that fail the CRC are counted as "not steam voice" (non-Steam clients' Speex). The layout was
learned from ReVoice and reimplemented, not copied: ReVoice is GPL-3. libopus 1.4 (BSD) is vendored
in `external/opus` and built as a static library into `crossbot_mm` only.

**Hearing.** Each sender gets an Opus decoder at 16 kHz, whatever rate it encoded at, because that is
what the transcription wants. A sequence gap of up to 10 frames is filled with Opus concealment; a
larger one resets the decoder. An utterance ends at the 0xFFFF terminator, after `silence` seconds
without packets, or at `max_seconds`; shorter than `min_seconds` is dropped. Ignored players and bots
are never decoded. A line is team-only when teams are on and `sv_alltalk` is 0.

**Speaking.** The reply is resampled to `send_rate` (24 kHz), encoded in 20 ms mono Opus frames at
32 kbps, and sent from the bot's own client slot at real time plus 100 ms, five frames per packet, to
every human teammate, or every human when the line is not team-only. The line is added to the feed
when it is ordered, as the bot's voice.

**Why not the engine's listening bits.** Under CrossPlay every loaded game DLL runs its own
`CVoiceGameMgr`, and each rewrites every client's `Voice_SetClientListening` bits every 0.3 s. Only the
player's own game gets `VModEnable`, so the others clear the bits: with more than one game loaded,
`Voice_GetClientListening` reads 0 for every pair, and human-to-human voice is silenced the same way
(CrossPlay's to fix). With `-xgame1 cstrike` alone the bits are right. The cost of bypassing them: a
scoreboard mute does not silence a bot.

**Verified with a Steam client** (CS, HL25): it sends `steam opus, 24000 Hz`, the same format we
send, and it plays the bots' packets. `crossbot_voice_tone` and `crossbot_voice_echo` isolate the
two halves: a tone with no TTS, and a human's own packets replayed from a bot's slot. Both print a
per-client report of who would receive and why. The SteamID in sent packets is the same constant
ReVoice uses.

## brain.cpp: when to think

- **One brain per team**, one for everyone on a free-for-all map. A brain runs only while its team has
  a bot. Team chat and radio wake the sender's team brain; all chat wakes every running brain.
- **Triggers** are human chat and radio only. Bot lines, automatic radio, ignored players and players
  over `lines_per_minute` never start a job.
- **One job in flight per brain.** Newer lines while one is in flight coalesce into the next job.
  Results carry the job id and a map/dormancy epoch and are dropped if stale.
- **Radio waits for its sound.** A job woken by radio applies its result no earlier than `radio_hold`
  seconds after the call, so a bot does not answer over the caller's radio sound still playing. Time
  already spent in JEV and the LLM counts toward it.
- **Ignore.** JEV's `ignore` score becomes seconds; at or above `ignore_min` the sender is ignored and
  told so privately. Their lines are dropped before they can trigger, and are left out of every later
  context. `crossbot_brain_unignore` clears it.
- **Recheck.** JEV's `recheck` score arms one timer per brain. Any new trigger cancels it; when it
  fires it is a job whose trigger is "re-check of: <the line that armed it>". At most `recheck_chain`
  re-checks in a row without human input.
- **Dormant without humans.** No humans on the server means no jobs, no timers, and results dropped;
  directives already given keep running until they end.

## brain_context.cpp: what the models see

Built on the game thread per job; the worker never touches the engine.

- **JEV state** (short): map, round, the line to judge, the last 6 comms, the team's bots with
  distance to the speaker and what they are doing. Questions: `act` (noul), `involve_<n>` (one noul per
  bot, so the subset is a multi-select), `ignore` (score), `remember` (noul), `recheck` (score).
- **LLM context**: the team roster (bots: game, health, armor, weapons with the active one starred,
  objective item, position, distance and line of sight to the speaker, current orders or own plan;
  humans: health, held weapon, position), enemies the team's bots saw in the last 20s, objectives,
  what the speaker is looking at (point, texture, entity, floor/wall), this round's events, recent
  comms, memories, each game's radio commands and abilities, and the trigger. The worker appends JEV's
  verdict and the likely responders. The system prompt and tool list come first and never change, so
  a provider that caches prefixes can.
- **Positions** read as coordinates plus the named place, then "at site A", "650u from site B" or
  "near the CT spawn": "(-416 -250 36) in Suicide, near the T spawn". The header lists every place of
  the map, so the LLM knows the vocabulary it can target.
- **Site letters** come from the places named "Bombsite X" / "Site X", matched to the nearest bomb
  target within 1500u. Entity order is not the convention: on de_dust2 the first `func_bomb_target`
  is the site players call B. Unlabelled maps show `#n`.

## brain_sites.cpp: named places

`places.txt` (repo root, deployed by `build.bat`) holds lines `map x y z Name with spaces`. A place may
have many points; a position belongs to the nearest point within 900u (height counts double), taking
the first of the four nearest that a trace from the position can see, so a room does not name the
room behind its wall. As a target, a place resolves to its point nearest its own centroid; a name
matches exactly, then as a unique substring, and an exact place name wins over a player whose name
merely contains it.

Where the names came from, per game:

- **CS / CZ**: every czero nav place (`graveyard/czero_places`, thinned to one point per 256u cell),
  for the 54 maps that shipped a czero nav. de_dust2 also has the 33 callouts of the CS wiki's
  interactive map, projected from its CS2 radar image: a uniform-scale fit on the spawn and bomb
  target entities lands within 50u (scale 4.395, the radar's 4.4), and czero's coarse names are
  renamed to what players say (Side to Long A, Tunnel to B Tunnels). The same fit is 300u off on
  nuke and 1000u+ on inferno, cbble and vertigo: CS2 rebuilt those. de_inferno's czero "Alley" is
  the Banana and "Upstairs" the Apartments; its letters stay czero's, since the wiki records that
  CZ's radar named them that way and later versions swapped them. de_dust2_cz copies dust2;
  de_dust2_xmas is dust2 shifted by (-704, -448, -128). Maps without czero data get spawns, rescue
  and escape zones only.
- **DoD**: every `dod_control_point` under its localised name from `resource/dod_english.txt`
  ("POINT_ANZIO_PLAZA" is "the plaza"), every mapper-placed `dod_location` area name (8 maps), the
  spawn areas, and the extras the DoD wiki describes that the floor plan shows unambiguously (Anzio's
  beach and river, Charlie's beach and main complex, Donner's bunker, Kalt's sewers, Glider's castle).
- **TFC**: spawns by `team_no`, flag homes from `item_tfgoal`, and the TF Wiki's location sections
  for 2fort, well, badlands, dustbowl and avanti, placed from the floor plan; warpath's command
  points come from its capture goals. juno, magelli, palermo and koth_one have no wiki locations and
  spawns without teams, so they have nothing yet.
- **HLDM**: no wiki covers the maps. Players name spots by what spawns there, so each map gets its
  distinctive pickups (RPG, Gauss, Egon, crossbow, Hornet Gun, Magnum, shotgun, snarks, long jump,
  at most three of a kind), plus the named mechanisms: crossfire's bunker, towers, field, trench and
  pit, stalkyard's big guns, doublecross's towers and airstrike button.
- **Opposing Force**: the same pickup rule with Op4's weapons (Displacer, Sniper Rifle, M249, Shock
  Rifle, Spore Launcher, Deagle, Penguins, Grapple) and the CTF powerups; CTF maps add each team's
  base (`item_ctfflag` by `goal_no`: 1 Black Mesa, 2 Opposing Force) and spawns, op4cp_park its four
  control points from the `tcN_bm_control` triggers. No wiki covers the multiplayer maps.
- **DMC**: the maps are Quake's remade, and the Quake wiki names each room by the items in it. Where
  DMC kept Quake's item set (dm3, dm4, dm6: one of each), the rooms are anchored on those item
  entities ("Super Nailgun - top of computers in Computer Room"); dm2 was re-itemed (two Red Armors,
  two RLs), so only its unambiguous rooms are named. Every DMC map also names the items Quake players
  call out: Red/Yellow Armor, Quad, Pentagram, Ring, Mega Health, RL, LG, GL, SNG. Room lines come
  first in the file, so a position on the item reads as the room while "quad" still works as a target.

`scripts/place_map.ps1 -Map <map>` draws the walkable floor from navgen's `--dump-obj` top-down,
coloured by height, with a coordinate grid, the objective entities (`-Items` adds weapons), and the
places as labelled dots, into `nav_debug/places/<map>.png`; `-Crop "x0 y0 x1 y1"` zooms. It is how
the hand-placed points were found and is how to check a new one.

## brain_tools.cpp: what the models can do

`act`, `rules`, `alert`, `release`, `say_team`, `say`, `speak`, `radio` (one bot each) and `remember`.
Every tool also runs from the console through `crossbot_do`, on the same code.

- The JSON schemas the LLM sees are `prompts.tools` in `brain.json`. Renaming a tool or a parameter
  there breaks the executor.
- Targets are strings: a player name (exact, or a unique substring), `me` (the speaker), `aim` /
  `aim:<player>` (where they look), `site:<A|B|n>`, a place name or `place:<name>`, or `x y z`.
- Chat text loses `"`, `;` and `\`, collapses whitespace, and is capped to fit `FakeCmd`'s 128.

## brain_memory.cpp

Notes written by the `remember` tool go to `addons/crossbot/brain_memory.jsonl` (time, map, round,
players, text; newest 200 kept). Each context gets up to `memories` notes: ones naming the speaker
first, then this map's, then the newest.

## brain.json

`addons/crossbot/brain.json`; `build.bat` copies `brain.example.json` there once and never overwrites
it. The key lives only there, never in the repo. It is read at every map start and by
`crossbot_brain_reload`. Without a key the brain is off and says why.

The instructions the models get (system prompt, tool list, JEV questions, the gate line) are under
`prompts`; the context's section headings stay in code. A missing prompt turns the brain off and names
it, so an older `brain.json` needs the `prompts` block copied in from `brain.example.json`.

| key | meaning |
|---|---|
| `key` | OpenRouter key, used for both models |
| `llm.url`, `llm.model`, `llm.timeout`, `llm.max_tokens`, `llm.reasoning` | chat completions endpoint and model; `reasoning` is sent as the effort |
| `jev.url`, `jev.model`, `jev.timeout` | OpenRouter's decisions endpoint and the JEV model |
| `jev.act`, `jev.involve`, `jev.remember` | noul thresholds; the LLM is called when `act` or `remember` passes |
| `jev.ignore_levels`, `jev.ignore_min` | seconds per score level, and the least that ignores |
| `jev.recheck_levels`, `jev.recheck_min`, `jev.recheck_chain` | the same for re-checks, and the chain cap |
| `history.events`, `history.comms`, `history.comms_seconds`, `history.memories` | context sizes |
| `rate.lines_per_minute` | per-player trigger limit |
| `rate.radio_hold` | seconds after a radio call before its answer is carried out |
| `voice.enabled` | hear and speak on voice chat |
| `voice.stt_url`, `voice.stt_model`, `voice.stt_timeout` | speech to text |
| `voice.tts_url`, `voice.tts_model`, `voice.tts_voice`, `voice.tts_timeout` | text to speech; an empty voice is not sent |
| `voice.min_seconds`, `voice.max_seconds`, `voice.silence` | what counts as one utterance |
| `voice.send_rate` | Opus rate of the bots' voice: 8000, 12000, 16000, 24000 or 48000 |
| `prompts.system` | the LLM's system prompt, a string or an array of lines |
| `prompts.gate` | appended to the LLM context after JEV's verdict |
| `prompts.tools` | the tool list sent to the LLM, verbatim |
| `prompts.jev.act`, `.involve`, `.remember` | noul questions: `instructions`, `yes`, `no`; `{bot}` is the bot's name in `involve` |
| `prompts.jev.ignore`, `.recheck` | score questions: `instructions`, `none` for level 0, `level` for the rest with `{duration}`; ignore's `{why}` spreads the `why` list over the levels |

## Seeing what the models saw

`crossbot_brain_trace 1` makes the worker append one JSON line per HTTP exchange to
`addons/crossbot/trace/brain_<map>.jsonl`: time, job, brain, `jev` or `llm`, status, seconds, url, error,
the request body and the response (embedded as JSON when it parses, as a string when it does not). The
key travels in a header and never reaches the file. `crossbot_brain_debug 1` prints the same on the
console in 400-byte chunks, because the engine's print truncates longer strings into garbage.

**Reasoning eats the budget.** Nemotron 3 Super reasons by default; with `max_tokens` 600 it spent all
600 thinking and returned `finish_reason: length` with no tool call, so nothing happened. `reasoning:
"off"` sends `{"enabled": false}` and the same request answers in about 2s with the tool calls. A
`length` finish with no calls is reported as an error in `crossbot_brain_status`.

**JEV's scores are read at their most likely level**, not the weighted average: a plain "give me your
gun" came back almost certainly level 0, but the average leaked 17s of ignore. A re-check is only armed
when the LLM actually issued calls; otherwise JEV asked to look again after almost every line, "lol"
included.

## Testing

`scripts/test_directives.ps1` drives the executor through `crossbot_do` with no keys. `-Game cstrike`
(de_dust2): go, follow distance, a crouched hold, a C4 hand-off, plant on orders under
`no_auto_objective`, three shots at an `aim` point, a refused teammate target, melee only, and a walk to
the named place "goose". `-Game tfc` (casbah): ordered sentry / dispenser / detsentry, disguises (a spy
class is a bad argument), a 5s detpack, a heal on a real teammate, a medic refusing to build. `-Game dod`
(dod_anzio): a 30cal held prone deploys, a rifleman goes prone. `-Game valve` (crossfire): place and a
hand grenade throw resolve; `-Game valve -Map test2_doorway` must actually place the tripmine with
`crossbot_deploy 0`. Outcomes the game decides (TFC's `#Build_noroom`, enemies breaking off mining,
crossfire's airstrike) are accepted as long as the step reports them. CrossPlay takes one port per extra
game, so parallel runs need ports more than two apart.
`scripts/test_brain.ps1` runs the whole loop against `scripts/brain_mock.ps1`, a local HTTP listener
that answers like JEV and like a tool-calling LLM and rejects bodies that are not JSON; it swaps in a
mock `brain.json` and memory file and restores both. A bot stands in for the human through
`crossbot_brain_test_human <name>`; only that name counts as human, otherwise bots' own replies would
re-trigger the brain forever. It forces `sv_alltalk 0`, since the voice checks expect team voice, and sets
`radio_hold` to 4: every rcon waits out a 1s drain, so a status read would come after a 1s hold expired. `-Live` keeps the real `brain.json`.

Voice is tested through `crossbot_voice_say Lead follow me`. The mock's speech endpoint answers with
1.5s of a 300 Hz tone; its transcription endpoint measures what came back after the bot's packets were
paced, decoded and wrapped in a WAV. It returns the original text only if the rate is 16 kHz, the
length is within 0.3s, and the tone is still 300 Hz, and "garbled" otherwise. Live, real speech made
the same trip: "Ace, follow me to bombsite B, and cover me" came back as "Ace, follow me to Bumside B
and cover me", and Ace followed.
