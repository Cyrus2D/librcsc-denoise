---
applyTo: "rcsc/player/**,rcsc/common/**"
---

# Player Agent, World Model & Observation Pipeline (librcsc)

The player side of librcsc turns server messages (`see`, `sense_body`, `hear`, `fullstate`, `server_param`, `player_type`, `ok`) into a `WorldModel`, then lets a subclass decide and send one action batch per cycle.

## TL;DR

Each cycle, the library (not the subclass) parses server messages into a `WorldModel`, localizes objects, and composes the command batch.

- `PlayerAgent` (`rcsc/player/player_agent.h:67`) extends `SoccerAgent` and is driven by the online/offline client loop; a client subclass overrides `actionImpl()` to register actions.
- Each cycle: parse message -> update sensors -> `WorldModel::updateAfterSee/SenseBody` -> `action()` -> `ActionEffector::makeCommand` -> send.
- **Both see models reach the same parser.** `VisualSensor::parse` has no quantized/Gaussian branch; the only client-side switch is the `(gaussian_see)` setting command.
- The **noise-rate parameters are carried but never read** by librcsc's player code; the distance error model is the quantization-based object table.
- Denoising is split across deep-dive files: parsing/localization error bounds, fusion/ghost logic, and say/hear messaging (see **Deep Dives** below).
- **Open the full file when:** you need the lifecycle/dispatch map or the quantized-vs-Gaussian summary; open a deep-dive file for algorithms and thresholds.

## Deep Dives (denoising)

- [observation-parsing-localization](observation-parsing-localization.instructions.md): `see`/`sense_body` parsing, view timing, self/ball/player localization, `ObjectTable` error ranges, Gaussian mismatch and hook points.
- [observation-fusion-denoising](observation-fusion-denoising.instructions.md): forward models, the self position blend, ball replacement, player track matching, ghost removal, count thresholds.
- [communication-audio-denoising](communication-audio-denoising.instructions.md): all say message types with bit budgets, hear decoding, and how heard data corrects ball/player tracks.
- Summary of findings: the only blending filter is self position (`self_object.cpp:833-868`); ball/player are replace-and-age with reachability gates; gaussian_see is never modelled in error bounds.

## Overview

Server protocol: a player connects with `(init ...)`, receives `(server_param ...)`, `(player_param ...)`, `(player_type ...)`, and then per-cycle `(see ...)` and `(sense_body ...)` messages plus `(hear ...)`. A subclass overrides behavior; the library owns parsing, world state, and command composition.

## Architecture

**Lifecycle (virtuals from `SoccerAgent`, `rcsc/common/soccer_agent.h`):**
- `initImpl(CmdLineParser&)` (pure, :117) is implemented in `PlayerAgent::initImpl` (`rcsc/player/player_agent.cpp:787-847`): parses `--help`/`--player-config`, then `PlayerConfig` from file and command line, rejects client version `< 8.0` or above `MAX_PROTOCOL_VERSION`, sets object count thresholds, and installs `LocalizationDefault` on both world models.
- `handleStart()` (:854) connects (`connectTo`), sets the interval, and sends `(init ...)` via `Impl::sendInitCommand`.
- `handleMessage()` (`player_agent.cpp:913`) drains `M_client->receiveMessage()` and calls `parse()`. It calls `action()` when `(think)` arrived, or when not in synch mode and the see time equals the current cycle.
- `handleTimeout`, `handleExit`, `handleStartOffline`, `handleMessageOffline` are the other hooks.

**Dispatch (`PlayerAgent::parse`, `player_agent.cpp:1448-1516`):** prefix match on `(see `, `(sense_body `, `(hear `, `(think)`, `(fullstate `, `(change_player_type `, `(player_type `, `(player_param `, `(server_param `, `(ok `, `(error `, `(warning `, `(score `, `(init `/`(reconnect `.

**Per-cycle pipeline (`PlayerAgent::action()`, `player_agent.cpp:2361-2480`):**
1. `WorldModel::updateJustBeforeDecision`; `ActionEffector::reset`; `handleActionStart`.
2. `actionImpl()` (subclass), then `doArmAction`, `doViewAction`, `doNeckAction`, `doFocusAction`, `communicationImpl`.
3. `WorldModel::updateJustAfterDecision`; a view-change command updates `SeeState::setViewMode`.
4. `M_effector.makeCommand(ostr)` -> `M_client->sendMessage`; then `handleActionEnd`, `clearAllCommands`.

**Sensors (`rcsc/player/`):**
- `VisualSensor` (`visual_sensor.h:50`, `parse` at `visual_sensor.cpp:311`): markers, lines, ball, players (teammate/opponent/unknown lists).
- `BodySensor` (`body_sensor.h:47`, `parse` at `body_sensor.cpp:91`, `parse2` at :351): stamina, view width/quality, command counters, arm/tackle/collision/foul.
- `AudioSensor` (`audio_sensor.h:56`): player/coach/trainer/freeform/CLang audio.
- `FullstateSensor` (`fullstate_sensor.h:48`): debug/god-view state.
- `SeeState` (`see_state.h:44`): synch-see timing and view mode bookkeeping.

**World state:**
- `WorldModel` (`world_model.h`): `setLocalization` (:301), `setInterceptSimulator` (:307), `setServerParam` (:312), `setViewMode` (:433), `updateAfterSenseBody` (:379), `updateAfterSee` (:392), `updateAfterFullstate` (:405), `updateJustBeforeDecision` (:447), `updateJustAfterDecision` (:454). `update(...)` at :368.
- Object types: `SelfObject` (`self_object.h`), `BallObject` (`ball_object.h:61`), `PlayerObject` (`player_object.h`) on top of `AbstractPlayerObject` (`abstract_player_object.h:53`), and `BallState` (`ball_state.h:45`).
- `ObjectTable` (`object_table.h:48`) and `PlayerObjectUpdater` (`player_object_updater.h`) handle table-driven distance ranges and object updates.

**Localization:** `Localization` interface (`localization.h:50`) with `PlayerT` (carries `dist_error_`, :71). Default implementation `LocalizationDefault` (`localization_default.h:45`). `WorldModel::updateAfterSee` runs self, ball, then player localization (`world_model.cpp:1179-1187`).

**Command objects:** `ActionEffector` (`action_effector.h:56`) collects commands; `PlayerCommand` subclasses are in `player_command.h` (`PlayerKickCommand`, `PlayerDashCommand`, `PlayerTurnCommand`, `PlayerChangeViewCommand`, `PlayerCLangCommand`, ...). Action methods on `PlayerAgent`: `doKick`, `doDash`, `doTurn`, `doCatch`, `doMove`, `doTackle` (`player_agent.h:212-260`).

**Config / parameters (`rcsc/common/`):**
- `ServerParam` (`server_param.h:55`), `PlayerParam` (`player_param.h:51`), `PlayerType` (`player_type.h:54`) with `PlayerTypeSet` (`player_type.h:876`) for heterogeneous players.
- `StaminaModel` (`stamina_model.h:44`): members `M_stamina`, `M_effort`, `M_recovery`, `M_capacity`; `init(PlayerType)` (:66), `updateBySenseBody` (:129), `simulateWait` (:198), `simulateDash` (:213).
- `PlayerConfig` (`player/player_config.h`) holds `synch_see` (:85) and `gaussian_see` (:86, accessor `gaussianSee()` at :335).

**Interception:** `InterceptTable` (`intercept_table.h:55`) instantiates `InterceptSimulatorSelfV17` by default (`intercept_table.cpp:67`, :353). Other variants (`intercept_simulator_self_v7.h`, `_v13.h`) are present, but this file's default is V17. `InterceptSimulatorPlayer` (`intercept_simulator_player.h:49`) is the per-player simulator class.

## Patterns & Conventions

- **Subclass hook, library pipeline:** clients override virtuals and never reorder the parse/update steps.
- **Protocol gates by version:** `config().version()` controls behavior, e.g., `synch_see` is only sent when version `< 18.0` (`player_agent.cpp:1330`).
- **Time keeping:** `GameTime` cycle plus `stopped` counters; `analyzeCycle` (`player_agent.cpp:1523`) updates the current time from each message header.
- **Quality helpers:** `ViewQuality` (`view_mode.h:216`) has enum `HIGH`, `LOW`; `ViewWidth` (`view_mode.h:43`) has `NARROW=60`, `NORMAL=120`, `WIDE=180`.

## Observation Models: Quantized vs Gaussian

- **Server side:** quantized is the default. Gaussian is selected per player by sending `(gaussian_see)`, answered `(ok gaussian_see)`. Server: `Player::gaussian_see()` (`rcssserver-denoise/src/player.cpp:2387-2390`), `M_gaussian_see` default `false` (`player.cpp:187`), selection in `visualsenderplayer.cpp:377-379` (`GaussianObservation`). Manual: `manual/source/soccerserver/sensor-models-vison-sensor-model.rst` (note the file name spelling), lines 17-18 and 407-441.
- **Client config:** `gaussian_see` option (`player_config.cpp:219`, default `false` at :105). `sendSettingCommands` (`player_agent.cpp:1336-1339`) sends `(gaussian_see)` only when set. Its ack is logged and nothing else (`analyzeOK`, `player_agent.cpp:2257-2267`).
- **No mode branch in parsing:** `VisualSensor::parse` (`visual_sensor.cpp:311-473`) and `parsePlayer` read the same `<DIST> <DIR>` tokens for both models.
- **Noise parameters:** `dist_noise_rate`, `focus_dist_noise_rate`, `land_dist_noise_rate`, `land_focus_dist_noise_rate` are parsed (`server_param.cpp:997-1000`), stored in `ServerParam` (`server_param.cpp:681-684`) and `PlayerType` (`player_type.h:86-89`, accessors :444-462; parsed at `player_type.cpp:627-641`), then copied, printed, and transferred. **No file under `rcsc/player/` references them** (verified by search), and helios-base does not reference them either.
- **Distance error model:** `localization_default.cpp:59` defines `USE_OBJECT_TABLE` unconditionally. `localizePlayer` (:1566) calls `ObjectTable::getDistanceRange` (:1577-1580) and stores the result in `dist_error_` (:1613). The table is built with `quantize_dist` (`object_table.cpp:2476-2527`), so the error bounds encode the quantized model.
- **Legacy leftover:** `VisualSensor::Player_Low_Mode` (`visual_sensor.h:79`) is returned by `parsePlayer` when a player token does not match a known pattern (`visual_sensor.cpp:989`), and `parse()` discards it (:401-402). Helios does not reference it.

## Integration Points

- **Client loop:** `AbstractClient` / `OnlineClient` / `OfflineClient` (`rcsc/common/`), see [net-param-rcg](net-param-rcg.instructions.md).
- **Actions and skills:** helios's behaviors call `Body_*` skills; see [action-skills](action-skills.instructions.md).
- **Helios usage (verified by include scan):** `rcsc/player/player_agent.h` (76 includes), `soccer_action.h` (71), `intercept_table.h` (33), `debug_client.h` (32), `world_model.h` (21), `say_message_builder.h` (11), `soccer_intention.h` (7). The helios entry point is `helios-base-denoise/src/player/main_player.cpp`.
- **Helios view handling:** `helios-base-denoise/src/player/basic_actions/bhv_emergency.cpp:57` checks `viewQuality() != ViewQuality::HIGH`.

## Build & Test

- Compiled by both build systems: CMake target `rcsc` lists `rcsc_player` (`rcsc/CMakeLists.txt:21-37`); autotools `rcsc/Makefile.am` lists `player` in `SUBDIRS`.
- No unit tests cover this module in the build files.

## Important Notes

- **Gaussian mode is not validated by librcsc:** the error bounds in `dist_error_` come from the quantized table. Under `gaussian_see`, those bounds are an inference-level mismatch and were not runtime-tested here.
- `Impl::analyzeOK` logs `(ok gaussian_see)` but keeps no state, so the client cannot query which mode the server accepted.
- `PlayerAgent` keeps `M_worldmodel` and `M_fullstate_worldmodel`; the fullstate model's `updateJustBeforeDecision` runs only when `config().debugFullstate()` is set (`player_agent.cpp:2402-2407`).

## See Also

- [observation-parsing-localization](observation-parsing-localization.instructions.md)
- [observation-fusion-denoising](observation-fusion-denoising.instructions.md)
- [communication-audio-denoising](communication-audio-denoising.instructions.md)
- [action-skills](action-skills.instructions.md)
- [net-param-rcg](net-param-rcg.instructions.md)
- [formation-ann-geom](formation-ann-geom.instructions.md)
- [coach-trainer-clang](coach-trainer-clang.instructions.md)
- [build-install-tools](build-install-tools.instructions.md)
- [../copilot-instructions.md](../copilot-instructions.md)
