# librcsc-denoise — Project Instructions

## TL;DR
librcsc: C++17 shared library (`librcsc`, SOVERSION 20) for building RoboCup 2D agents, coaches and trainers.
- `rcsc/` = library; `src/` = CLI tools (rcg converters etc.); `example/` = sample configs.
- Used by helios-base via `-lrcsc`; talks to rcssserver.
- `rcsc/action/` is not compiled by CMake/autotools; `obsolete/` is legacy.
- **Open the full file when:** you need build/install steps or the right scoped instruction.

## Project Overview
Client-side library: network, world model, sensors, localization, formations, geometry, RCG logs, coach/trainer, CLang. Server: `../rcssserver-denoise`; sample client: `../helios-base-denoise`. Protocol docs: `../manual/source/`.

## Architecture and Entry Points
- `rcsc/player/` PlayerAgent + WorldModel; `rcsc/common/` ServerParam/PlayerType/clients; `rcsc/net`, `rcsc/param`, `rcsc/rcg`, `rcsc/coach`, `rcsc/trainer`, `rcsc/clang`, `rcsc/formation`, `rcsc/geom`.
- Noise-rate params are stored in `rcsc/common/player_type.*`; the see parser has no quantized/Gaussian branch (details in player-world-model).

## Build, Test, Run
- Autotools: `./bootstrap && ./configure --disable-unit-test && make && sudo make install` (README.md; use `--prefix`).
- CMake: CMake 3.5+, C++17, Boost system; default prefix `$HOME/local`.
- No CTest. Known issues: `librcsc-config.in` lists stale `-lrcsc_*` names; `rcsc/rcsc.h` includes a missing `rcsc/bpn1.h`. See build-install-tools.

## Conventions
`.clang-format`: 4 spaces, no tabs, no column limit, spaces inside parentheses.

## Instruction Index
Open the entries relevant to the files you touch (on demand, not auto-loaded). For denoising work start with the observation/noise entries.

| Instruction | Covers (applyTo) | Summary | Path |
|---|---|---|---|
| Action Skills (`rcsc/action`) | `rcsc/action/**` | `rcsc/action` holds reusable `execute(PlayerAgent*)` skill classes, but neither build compiles them and helios-base uses its own copies. | [instructions/action-skills.instructions.md](instructions/action-skills.instructions.md) |
| Build, Install, CLI Tools & Tests (librcsc) | `CMakeLists.txt,**/CMakeLists.txt,configure.ac,**/Makefile.am,src/**,example/**,librcsc-config.in,librcsc.pc.in` | librcsc builds as one shared library (SOVERSION 20) through CMake or autotools, and the two systems disagree on tools, tests, and link names. | [instructions/build-install-tools.instructions.md](instructions/build-install-tools.instructions.md) |
| Coach, Trainer, CLang & Monitor (librcsc) | `rcsc/coach/**,rcsc/trainer/**,rcsc/clang/**,rcsc/monitor/**` | `CoachAgent` and `TrainerAgent` are privileged `SoccerAgent` clients, `CLangParser` handles coach-language advice, and monitor is library-only. | [instructions/coach-trainer-clang.instructions.md](instructions/coach-trainer-clang.instructions.md) |
| Inter-agent Messaging (say/hear) as a Denoiser (librcsc) | `rcsc/player/{audio_sensor,say_message_builder}*,rcsc/common/{audio_codec,audio_memory,audio_message,say_message,say_message_parser,freeform_message}*` | Players pack 1-22 typed messages into at most `say_msg_size` (10) characters of a 74-symbol alphabet; receivers decode into `AudioMemory`, and `WorldModel` uses | [instructions/communication-audio-denoising.instructions.md](instructions/communication-audio-denoising.instructions.md) |
| Formation, ANN & Geometry (librcsc) | `rcsc/formation/**,rcsc/ann/**,rcsc/geom/**` | Formations load through a format dispatcher, ANN code is legacy-only, and the geometry primitives are the heavily used current part. | [instructions/formation-ann-geom.instructions.md](instructions/formation-ann-geom.instructions.md) |
| Network, Params, RCG Logs, gzip & Timers (librcsc) | `rcsc/net/**,rcsc/param/**,rcsc/rcg/**,rcsc/gz/**,rcsc/time/**` | Clients talk UDP through `AbstractClient`/`OnlineClient`, configure via `ParamMap`/`CmdLineParser`, and read or write RCG logs through version-dispatched parser | [instructions/net-param-rcg.instructions.md](instructions/net-param-rcg.instructions.md) |
| Observation Fusion & Denoising in the World Model (librcsc) | `rcsc/player/{world_model,*_object,*_state,*_predictor,*_tracker,*_info,self_object,ball_object,player_object,interception,intercept_table}*` | Fusion is mostly replace-and-age: a forward model predicts each object, a fresh `see` overwrites it, and age counters (`*_count`) plus gating by speed and senso | [instructions/observation-fusion-denoising.instructions.md](instructions/observation-fusion-denoising.instructions.md) |
| Observation Parsing & Localization (librcsc) | `rcsc/player/{visual_sensor,localization,object_table,view_area,see_state,body_sensor,fullstate_sensor}*,rcsc/common/{server_param,player_type}*` | The parser keeps raw rounded values; `LocalizationDefault` converts them to positions with error bounds taken from a hard-coded quantization table, not from the | [instructions/observation-parsing-localization.instructions.md](instructions/observation-parsing-localization.instructions.md) |
| Player Agent, World Model & Observation Pipeline (librcsc) | `rcsc/player/**,rcsc/common/**` | Each cycle, the library (not the subclass) parses server messages into a `WorldModel`, localizes objects, and composes the command batch. | [instructions/player-world-model.instructions.md](instructions/player-world-model.instructions.md) |

## Related Areas
- Server: `../rcssserver-denoise/.github/copilot-instructions.md` (separate repo)
- Sample client: `../helios-base-denoise/.github/copilot-instructions.md` (separate repo)

## PR Review — Instruction Maintenance
GitHub Copilot PR reviewers: use the Instruction Index to read scoped guidance
relevant to the changed paths, respecting each repository/submodule's ownership.
When substantive changes to architecture, interfaces, workflows, build or
configuration behavior, or safety constraints make relevant instructions stale
or incomplete, report the maintenance need as a review finding/recommendation.
Identify the affected instruction and the claim or gap, with change evidence;
judge significance semantically, not by a changed-line threshold. This note does
not authorize reviewers to edit instructions and does not guarantee Copilot loads
all instruction files; follow relevant links on demand.
