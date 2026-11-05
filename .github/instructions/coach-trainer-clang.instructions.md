---
applyTo: "rcsc/coach/**,rcsc/trainer/**,rcsc/clang/**,rcsc/monitor/**"
---

# Coach, Trainer, CLang & Monitor (librcsc)

Four non-player agent/protocol areas: `CoachAgent` (online coach), `TrainerAgent` (offline/trainer client), the CLang coach-language parser, and monitor command helpers.

## TL;DR

`CoachAgent` and `TrainerAgent` are privileged `SoccerAgent` clients, `CLangParser` handles coach-language advice, and monitor is library-only.

- `CoachAgent` and `TrainerAgent` both extend `SoccerAgent` (`rcsc/coach/coach_agent.h:59`, `rcsc/trainer/trainer_agent.h`), the same base as `PlayerAgent`.
- **Coach language (CLang):** `CLangParser` (`rcsc/clang/clang_parser.h`) parses `(info ...)`/`(advice ...)` messages into directives, actions, conditions and tokens; a grammar reference is at `rcsc/clang/coach_lang_grammer.txt` (sic).
- **Helios-base uses the coach and trainer agents** (`src/coach/sample_coach.h:42`, `src/trainer/sample_trainer.h`), but **not** `rcsc/monitor/` or the clang headers directly.
- **Monitor (`rcsc/monitor/`)** is a small command-builder library that depends only on `rcg`; nothing in helios or `src/` uses it.
- **Open the full file when:** you add a coach/trainer command, touch the CLang grammar, or change the coach world model.

## Overview

- **Coach** (online): receives audio/visual/`hear` data about the whole field and sends coach commands (`look`, `team_names`, `change_player_type`, `say`-style freeform messages, `team_graphic`, ...).
- **Trainer**: a privileged client that can move the ball and players, change the play mode, and set up scenarios.
- **CLang**: the coach-language used to give players advice; the coach sends `(clang ...)`, and players receive `(info ...)`/`(advice ...)` via audio.
- **Monitor**: command objects for a monitor client (kick-off, drop ball, move player, card, compression).

## Architecture

### Coach (`rcsc/coach/`)

- Agent: `CoachAgent` (`coach_agent.h:59`), config `CoachConfig` (`coach_config.h`).
- World modeling: `CoachWorldModel` (`coach_world_model.h`), `CoachWorldState`, `CoachBallObject`, `CoachPlayerObject`, `CycleData` (`cycle_data.h`), `CoachVisualSensor`, `CoachAudioSensor`, `CoachInterceptPredictor`.
- Analysis helpers: `PlayerTypeAnalyzer` (`player_type_analyzer.h`), `PlayerChangeState` (`player_change_state.h`), `SoccerAdviser` (`soccer_adviser.h`), `CLangManager` (`clang_manager.h`).
- Commands (`coach_command.h`): base `CoachCommand`, plus `CoachInitCommand`, `CoachByeCommand`, `CoachCheckBallCommand`, `CoachLookCommand`, `CoachTeamNamesCommand`, `CoachEyeCommand`, `CoachChangePlayerTypeCommand`, `CoachChangePlayerTypesCommand`, `CoachFreeformCommand`, `CoachTeamGraphicCommand`, `CoachCompressionCommand`, `CoachDoneCommand`.
- Debug: `CoachDebugClient` (`coach_debug_client.h`), default debug host `localhost`, port `6000 + 32` = 6032 (`coach_config.cpp:114-115`).
- Default server host `localhost` (`coach_config.cpp:81`).

### Trainer (`rcsc/trainer/`)

- Agent: `TrainerAgent` (`trainer_agent.h`), config `TrainerConfig` (`trainer_config.h`).
- Commands (`trainer_command.h`): base `TrainerCommand`, plus `TrainerInitCommand`, `TrainerCheckBallCommand`, `TrainerLookCommand`, `TrainerTeamNamesCommand`, `TrainerEarCommand`, `TrainerEyeCommand`, `TrainerKickOffCommand`, `TrainerChangeModeCommand`, `TrainerMoveBallCommand`, `TrainerMovePlayerCommand`, `TrainerRecoverCommand`, `TrainerChangePlayerTypeCommand`, `TrainerSayCommand`, `TrainerCompressionCommand`, `TrainerDoneCommand`.

### CLang (`rcsc/clang/`)

- Parser: `CLangParser` (`clang_parser.h`) with `parse()`, `message()`, `clear()` and a `handle*` family (for example `handleActMark`, `handleActHold`, `handleActBallTo`, `handleActHeteroType`, `handleConditionBool`, `handleDirectiveCommon`, `handleTokenRule`, `handleTokenTTL`, `handleTokenClear`, `handleUnumSet`, `handleInfoMessage`).
- Building blocks: `CLangMessage` (`clang_message.h`), `CLangInfoMessage` (`clang_info_message.h`), `CLangDirective`/`CLangDirectiveCommon` (`clang_directive.h`), `CLangAction` with `CLangActionMark`, `CLangActionHeteroType`, `CLangActionHold`, `CLangActionBallTo` (`clang_action.h`), `CLangCondition`/`CLangConditionBool` (`clang_condition.h`), `CLangToken`/`CLangTokenRule`/`CLangTokenClear` (`clang_token.h`), `CLangUnumSet` (`clang_unum.h`).
- Umbrella: `clang.h`; `types.h`.
- Grammar reference: `rcsc/clang/coach_lang_grammer.txt`, with example messages such as `(info (6000 (true) (do our {1} (mark {2 3}) ...)))` (lines 145-149).

### Monitor (`rcsc/monitor/`)

- `MonitorCommand` (`monitor_command.h`), with `MonitorInitCommand`, `MonitorByeCommand`, `MonitorKickOffCommand`, `MonitorDropBallCommand`, `MonitorFreeKickCommand`, `MonitorMovePlayerCommand`, `MonitorDiscardPlayerCommand`, `MonitorCardCommand`, `MonitorCompressionCommand`.

## Patterns & Conventions

- **Same agent base:** `CoachAgent` and `TrainerAgent` derive from `SoccerAgent` (`coach_agent.h:59`, `trainer_agent.h`), which declares the lifecycle hooks `PlayerAgent` implements (see [player-world-model](player-world-model.instructions.md)). Confirm each override in its own header before relying on it.
- **Command objects:** protocol commands are small classes. The player side is verified: `PlayerCLangCommand` is serialized with `toCommandString` (`player_agent.cpp:1364-1366`). I did not confirm the same for coach/trainer commands.
- **CLang version gate:** the player receives `(ok clang (ver <min> <max>))` (`player_agent.cpp:2285-2299`) and then sends `PlayerCLangCommand` with the configured range (`player_agent.cpp:1359-1367`).
- **Player-side CLang input:** `AudioSensor::parseCLangMessage` (`audio_sensor.cpp:573`) is the player's entry point for CLang audio; its body was not re-read for this file.

## Integration Points

- **Dependencies (module scan):** `coach -> clang, common, geom, net, param, rcg, time`; `trainer -> coach, common, geom, param`; `player -> clang, common, geom, net, param, time`; `monitor -> rcg`; `clang -> time`.
- **Helios-base (verified):** `src/coach/sample_coach.h:42` (`: public rcsc::CoachAgent`); `sample_coach.cpp` includes `coach/coach_command.h`, `coach_config.h`, `coach_debug_client.h`, `coach_world_model.h`, `common/*`; `src/coach/main_coach.cpp:80` uses `CmdLineParser`. `src/trainer/sample_trainer.h` references `TrainerAgent` (its base class was not re-read here).
- **Not used by helios or `src/`:** `rcsc/monitor/*` and the `rcsc/clang/*` headers (no include found in helios; `rcsc/clang` is used by `player` and `coach` inside librcsc).

## Build & Test

- **Coach, trainer, clang, monitor** are part of the shared library in CMake (`rcsc/CMakeLists.txt:21-37` links `rcsc_coach`, `rcsc_trainer`, `rcsc_clang`, `rcsc_monitor`) and in autotools (`rcsc/Makefile.am` `SUBDIRS` and `librcsc_la_LIBADD`).
- **CLang test:** `rcsc/clang/test_clang_parser.cpp`, listed in `TESTS`/`check_PROGRAMS` in `rcsc/clang/Makefile.am:60-69`, needs CppUnit (`--enable-unit-test`) and runs only with `make check`. It is not in CMake.
- **Color test:** `rcsc/color/Makefile.am:25-34` declares `noinst_PROGRAMS = thermo_color_test`; this is autotools-only, outside this scope.

## Important Notes

- **Spelling:** the grammar file is `coach_lang_grammer.txt` (no `a`); keep the name when referencing it.
- **Coach debug port** is `6000 + 32` (`coach_config.cpp:115`), not a separately configured value in code I read.
- **Legacy/unused:** the monitor module has no consumer in helios or `src/`. Treat it as library surface, not a current client path.
- The CLang parser and grammar were read, but I did not run `test_clang_parser` (needs CppUnit), so parser behavior beyond the examples in the test file is unverified.

## See Also

- [player-world-model](player-world-model.instructions.md)
- [net-param-rcg](net-param-rcg.instructions.md)
- [build-install-tools](build-install-tools.instructions.md)
- [../copilot-instructions.md](../copilot-instructions.md)
