---
applyTo: "rcsc/action/**"
---

# Action Skills (`rcsc/action`)

`rcsc/action` is librcsc's library of reusable body, neck, view, arm and behavior "skills", each a small class with `bool execute(PlayerAgent*)`. **It is not compiled by either build system**, and current helios-base uses its own copy of these skills under `helios-base-denoise/src/player/basic_actions/`.

## TL;DR

`rcsc/action` holds reusable `execute(PlayerAgent*)` skill classes, but neither build compiles them and helios-base uses its own copies.

- Skill pattern: a class with a constructor for its arguments and `bool execute( PlayerAgent * agent )` that registers commands through `PlayerAgent`/`ActionEffector` (verified in `body_intercept2009.h`, `arm_off.h`, `bhv_emergency.h`).
- Unversioned headers (`body_intercept.h`, `body_dribble.h`, `body_hold_ball.h`, ...) are thin wrappers over `*2008`/`*2009` implementations.
- **Not built:** neither `rcsc/CMakeLists.txt` nor `rcsc/Makefile.am` nor `configure.ac` includes `action/`.
- **helios-base does not include any `rcsc/action/` header**; it has its own `Body_*`, `KickTable` and `intention` copies.
- **Open the full file when:** you need an exact skill signature, change a `*2008`/`*2009` implementation, or decide whether to port a skill into helios-base.

## Overview

Skills group reusable movement decisions: go to a point, turn, dash, intercept, kick-to-relative, pass, dribble, neck scans, view changes. Behaviors (`bhv_*`) compose skills; intentions (`intention_*`) run multi-cycle plans through `SoccerIntention`.

## Architecture

**Main groups in `rcsc/action/`:**
- Body skills: `body_go_to_point.h` (+ `_dodge`), `body_intercept2009.h` (class `Body_Intercept2009`, :46), `body_pass.h` (class `Body_Pass`, :47-50), `body_turn_to_ball.h` (class `Body_TurnToBall`, :43), `body_turn_to_angle.h`, `body_turn_to_point.h`, `body_kick_one_step.h`, `body_kick_to_relative.h`, `body_smart_kick.h`, `body_stop_ball.h`, `body_stop_dash.h`, `body_tackle_to_point.h`, `body_clear_ball2009.h`, `body_advance_ball2009.h`, `body_hold_ball2008.h`, `body_dribble2008.h`.
- Neck/View: `neck_scan_field`, `neck_scan_players`, `neck_turn_to_ball(_or_scan)`, `neck_turn_to_player_or_scan`, `neck_turn_to_goalie_or_scan`, `neck_turn_to_low_conf_teammate`, `neck_turn_to_point.h`, `neck_turn_to_relative.h`, `view_normal.h`, `view_wide.h`, `view_synch.h`, `view_change_width.h`.
- Arm: `arm_off.h`, `arm_point_to_point.h`.
- Behaviors: `bhv_emergency.h`, `bhv_before_kick_off.h`, `bhv_scan_field.h`, `bhv_go_to_point_look_ball.h`, `bhv_body_neck_to_ball.h`, `bhv_body_neck_to_point.h`, `bhv_neck_body_to_ball.h`, `bhv_neck_body_to_point.h`.
- Intentions: `intention_time_limit_action.h`, `intention_dribble2008.h`.
- Kick data: `kick_table.h` (class `KickTable`, :52; singleton via `KickTable::instance()`, :359).
- Umbrella: `basic_actions.h` includes the arm, neck, view and body-turn skills in one header.

**Versioned skills and wrappers:**
- Unversioned wrappers that only include a versioned file: `body_intercept.h` -> `body_intercept2009.h` (:36; a `*2008` include is commented out at :35), `body_advance_ball.h` -> `body_advance_ball2009.h` (:35), `body_clear_ball.h` -> `body_clear_ball2009.h` (:35), `body_dribble.h` -> `body_dribble2008.h` (:35), `body_hold_ball.h` -> `body_hold_ball2008.h` (:35).
- There is no `class Body_Intercept` in librcsc; the class is `Body_Intercept2009`.

**Legacy (`rcsc/action/obsolete/`):** `bhv_shoot2008`, `body_advance_ball2007`, `body_clear_ball2007`, `body_dribble2006/2007`, `body_go_to_point2007/2009/2010`, `body_hold_ball2007`, `body_intercept2007/2008`, `body_kick_multi_step`, `body_kick_two_step`, `intention_dribble2006/2007`, `intention_kick2007`, `shoot_table2008`. Not referenced by any build file, and not by helios-base.

## Patterns & Conventions

- **Execute pattern:** `bool execute( PlayerAgent * agent )`. Examples: `arm_off.h:57`, `arm_point_to_point.h:64`, `bhv_before_kick_off.h:64`, `bhv_emergency.h:59`, `body_intercept2009` (see `Body_Intercept2009` at :46).
- **Dependencies:** `rcsc/action` depends on `common`, `geom`, and `player` (module dependency scan).
- **Commands only through the agent:** skills call `agent->doKick`, `doDash`, `doTurn`, `doMove`, or `effector()` methods, not raw socket writes.
- **Naming:** version suffix = year of the skill revision; wrappers keep the unsuffixed name.
- **`kick_table`:** a singleton `KickTable::instance()` used by kick-power and kick-range lookups (used by `body_smart_kick.h:35`).

## Integration Points

- **Agent contract:** skills run inside `PlayerAgent::actionImpl()` (pure virtual on `PlayerAgent`, `player_agent.h:473`). See [player-world-model](player-world-model.instructions.md).
- **Helios-base (verified):** `helios-base-denoise/src/player/basic_actions/` holds its own copies: `body_go_to_point.h` (class `Body_GoToPoint`, :43), `body_pass.h` (`Body_Pass`, :50), `body_turn_to_ball.h` (`Body_TurnToBall`, :41), `body_intercept2018.h`, `kick_table.h/.cpp`, `body_smart_kick`, `body_kick_one_step`, `body_advance_ball2009`, `intention_dribble2008`, `basic_actions.h`.
- **Helios callers (verified):** e.g., `bhv_basic_move.cpp:87-111` (`Body_Intercept`, `Body_GoToPoint`, `Body_TurnToBall`); `bhv_goalie_basic_move.cpp` (`Body_TurnToAngle`, `Body_StopDash`, `Body_GoToPoint`); `bhv_goalie_free_kick.cpp:211-254` (`Body_Pass::get_best_pass`, `Body_KickOneStep`, `Body_ClearBall`, `Body_TurnToPoint`); `bhv_penalty_kick.cpp:183-392`.
- **Helios includes:** the only skill-related include in helios is `rcsc/player/soccer_action.h` (71 includes), not `rcsc/action/*`.

## Build & Test

- `rcsc/CMakeLists.txt` `add_subdirectory` list and the `rcsc` target omit `action`; `rcsc/action/CMakeLists.txt` defines an `OBJECT` library `rcsc_action` that nothing consumes.
- `rcsc/Makefile.am` `SUBDIRS` omits `action`; the only reference is a commented-out `librcsc_agent.la` block that names `action/librcsc_action.la`.
- `configure.ac` `AC_CONFIG_FILES` does not list `rcsc/action/Makefile`.
- `rcsc/action/Makefile.am` declares `noinst_LTLIBRARIES = librcsc_action.la` but is not reached by `configure`.
- No unit tests exist for this directory in the build files.

## Important Notes

- **Do not assume `rcsc/action` matches helios behavior.** The copies differ by name and version (helios has `body_intercept2018.h` and its own `KickTable`; librcsc has `Body_Intercept2009`). Diff the files before porting.
- Treat `rcsc/action/obsolete/` and every `*2006`/`*2007`/`*2008` file as legacy unless a current build or helios file pulls it in (none does).
- Skill classes receive `PlayerAgent*`, so they can read `world()` and `config()` directly; avoid caching world state across cycles.

## See Also

- [player-world-model](player-world-model.instructions.md)
- [build-install-tools](build-install-tools.instructions.md)
- [../copilot-instructions.md](../copilot-instructions.md)
