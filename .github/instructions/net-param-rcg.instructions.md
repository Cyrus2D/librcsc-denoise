---
applyTo: "rcsc/net/**,rcsc/param/**,rcsc/rcg/**,rcsc/gz/**,rcsc/time/**"
---

# Network, Params, RCG Logs, gzip & Timers (librcsc)

Infrastructure modules under `rcsc/`: sockets and the client transport, command-line/config parsing, the RCG replay-log format, gzip streams, and timers. Current helios-base uses `net` indirectly (through `common`), `param` directly, and `time` indirectly; it does not use `rcg` or `gz` directly.

## TL;DR

Clients talk UDP through `AbstractClient`/`OnlineClient`, configure via `ParamMap`/`CmdLineParser`, and read or write RCG logs through version-dispatched parsers.

- **Transport:** `AbstractClient` (`rcsc/common/abstract_client.h:48`) with `OnlineClient` (UDP, `rcsc/common/online_client.h:52`) and `OfflineClient` (log playback, `offline_client.h:45`); sockets are in `rcsc/net/` (`UDPSocket`, `TCPSocket`, `HostAddress`).
- **Params:** `ParamMap` + `CmdLineParser` (command line, used by helios) and `ConfFileParser`, `IniParser`, `RCSSParamParser` (`rcsc/param/`).
- **RCG logs:** version dispatch in `rcsc/rcg/parser.cpp:83-119` and `serializer.cpp:94-100`; JSON logs use `ParserSimdJSON`; vendored `nlohmann/` and `simdjson/` sit under `rcsc/rcg/`.
- **gzip:** `GZCompressor`/`GZDecompressor` and `gzifstream`/`gzofstream` (`rcsc/gz/`), used by the client and by `src/resultprinter.cpp`.
- **Open the full file when:** you change the connection/handshake, a command-line option, log format handling, or compression.

## Overview

These modules are protocol- and file-level plumbing. The player/coach/trainer agents sit above them; the RCG tools in `src/` (see [build-install-tools](build-install-tools.instructions.md)) sit on top of `rcg` and `gz`.

## Architecture

### Net (`rcsc/net/`)

- `AbstractSocket` (`abstract_socket.h`), `UDPSocket` (`udp_socket.h`: `readDatagram`, `writeDatagram`), `TCPSocket` (`tcp_socket.h`), `HostAddress` (`host_address.h`). No internal module dependencies.

### Common client transport (`rcsc/common/`, uses `net` and `gz`)

- `AbstractClient` (`abstract_client.h:48`): `connectTo`, `receiveMessage`, `sendMessage`, `run`, `setIntervalMSec`, `intervalMSec`, `setServerAlive`, `isServerAlive`, `compress`/`decompress` with `GZCompressor`/`GZDecompressor` (`abstract_client.h:40-41`), `setCompressionLevel`, and the `handle*` hooks.
- `OnlineClient` (`online_client.h:52`): forward-declares `UDPSocket` (:41); methods `connectTo`, `receiveMessage`, `sendMessage`, `select`, `run`, `openOfflineLog`, `printOfflineThink`.
- `OfflineClient` (`offline_client.h:45`): log-driven replay client, same `AbstractClient` base.
- Agent wiring: `PlayerAgent::handleStart` calls `M_client->connectTo(config().host(), config().port())` (`player_agent.cpp:869-870`), then `setIntervalMSec` (:881) and `sendInitCommand`.
- Defaults: player host `localhost` (`player_config.cpp:89`), player port 6000 (`server_param.cpp:218`, `DEFAULT_PLAYER_PORT`); coach host `localhost` (`coach_config.cpp:81`).

### Param (`rcsc/param/`, no internal dependencies)

- `ParamMap` (`param_map.h`): registration via `add()`; `ParamEntity`, `Registrar`; switches `BoolSwitch` and `NegateSwitch`; `analyze()`.
- `CmdLineParser` (`cmd_line_parser.h`): `parse`, `parsePositional`, `print`, `printOptionNameArgs`, `positionalOptions`, `failed`.
- `ConfFileParser` (`conf_file_parser.h`), `IniParser` (`ini_parser.h`), `RCSSParamParser` (`rcss_param_parser.h`, parses server-style `(name value)` lists), base `ParamParser` (`param_parser.h`).
- Usage: `PlayerAgent::initImpl` builds a `ParamMap` with `help` and `player-config` options and runs `CmdLineParser` (`player_agent.cpp:793-799`), then `ConfFileParser` (:811) and `M_config.parse(...)` (:812-816). `PlayerConfig` registers its option names through `ParamMap` (`player_config.cpp:218-219`, e.g., `synch_see`, `gaussian_see`).

### RCG logs (`rcsc/rcg/`, no internal dependencies)

- `Handler` (`handler.h`): virtual `handle*` callbacks (`handleTeam`, `handleShow`, `handlePlayMode`, `handleServerParam`, `handlePlayerType`, ...).
- `Parser` (`parser.h`) with `create`/`parse`; dispatch in `parser.cpp:83-119`: `REC_VERSION_JSON` -> `ParserSimdJSON` (:119); `REC_VERSION_6/5/4` -> `ParserV4` (:112-114); `REC_VERSION_3` -> `ParserV3` (:115); `REC_VERSION_2` -> `ParserV2` (:116); `REC_OLD_VERSION` -> `ParserV1` (:117). The `ParserJSON` line is commented out (:118).
- `Serializer` (`serializer.h`) with `SerializerV1`..`SerializerV6` and `SerializerJSON` (dispatch `serializer.cpp:94-100`).
- Constants (`rcg/types.h`): `REC_OLD_VERSION = 1`, `REC_VERSION_2..6`, `REC_VERSION_JSON = -1`, `DEFAULT_LOG_VERSION = REC_VERSION_6`.
- Vendored third-party code: `rcsc/rcg/nlohmann/` and `rcsc/rcg/simdjson/`.
- Umbrella: `rcg.h` includes `rcg/types.h`, `util.h`, `handler.h`, `parser.h`, `serializer.h`.

### Gz (`rcsc/gz/`, no internal dependencies)

- `gzfilebuf::close()` (`gzfstream.cpp`) returns `nullptr` if the last flush (output mode) or `gzclose()` failed, so `gzofstream::close()` sets failbit; callers (e.g. the dataset logger) can detect a failed close.
- `GZCompressor`/`GZDecompressor` (`gzcompressor.h`), `gzfilebuf`/`gzifstream`/`gzofstream` (`gzfstream.h`), `gzfilterstreambuf`/`gzifilterstream`/`gzofilterstream` (`gzfilterstream.h`). Umbrella `gz.h` is installed.

### Time (`rcsc/time/`, no internal dependencies)

- `Timer` (`timer.h`): `elapsed`, `elapsedReal`, `restart`.
- `TimeStamp` (`time_stamp.h`): `setNow`, `elapsedSince`, `isValid`, `timePoint`.
- Top-level `rcsc/timer.h` is an installed forwarder that includes `rcsc/time/timer.h`.

## Patterns & Conventions

- **Protocol compression:** the client negotiates with `(compression N)`; the server confirms with `(ok compression N)` and `analyzeOK` calls `setCompressionLevel` (`player_agent.cpp:2273-2283`). The player sends `PlayerCompressionCommand` only for levels 1-9 (`player_agent.cpp:1369-1375`).
- **Message routing:** `PlayerAgent::parse` dispatches by message prefix (see [player-world-model](player-world-model.instructions.md)).
- **Version dispatch by number:** RCG parsers and serializers pick the implementation from the numeric version written in the log header; JSON is a separate sentinel (`-1`).
- **Option naming:** config options use snake_case names (`synch_see`), command-line options via `ParamMap`.

## Integration Points

- **Helios-base (verified by include scan):** `rcsc/param/cmd_line_parser.h` and `param_map.h` (`src/player/main_player.cpp:34`, `src/coach/sample_coach.cpp:45-46`, `main_coach.cpp:80`). `rcsc/common/abstract_client.h` (6 includes), `rcsc/common/logger.h`, `server_param.h`, `player_type.h`. Helios includes no `rcsc/net/*`, `rcsc/rcg/*`, or `rcsc/gz/*` header. Timers: `rcsc/timer.h` is included 12 times and `rcsc/time/timer.h` once.
- **Module dependency scan:** `common -> geom, gz, net, param, rcg`; `player -> net, param, time`; `coach -> net, param, rcg, time`.
- **Tools in `src/`:** `resultprinter.cpp:36` includes `rcsc/gz/gzfstream.h`; `rcgversion.cpp:37` includes `rcsc/rcg/types.h`; `rcg2csv.cpp:35-36` includes `rcsc/param/param_map.h` and `cmd_line_parser.h`.

## Build & Test

- All five modules are built into `librcsc` by CMake (`rcsc/CMakeLists.txt:21-37` lists `rcsc_net`, `rcsc_param`, `rcsc_rcg`, `rcsc_gz`, `rcsc_time`) and by autotools (`rcsc/Makefile.am`).
- `example/` tests (`param_main.cpp`, `gzifstream_main.cpp`, `gzofstream_main.cpp`, `loader_main.cpp`, `result_writer_main.cpp`) build only with `--enable-example`, and their link lines reference `-lrcsc_param`, `-lrcsc_gz`, `-lrcsc_rcg` (`example/Makefile.am:18-34`), which the build does not produce as shared libraries. Treat them as stale until verified.
- zlib: CMake runs `find_package(ZLIB)` and sets `HAVE_LIBZ` (`CMakeLists.txt:56-60`), with `add_definitions(-DHAVE_LIBZ)` commented out. Autotools checks `-lz` (`configure.ac:53-54`).

## Important Notes

- **Stale link names:** `librcsc-config.in:40` prints `-lrcsc_agent -lrcsc_clang -lrcsc_ann -lrcsc_net -lrcsc_color -lrcsc_time -lrcsc_param -lrcsc_rcg -lrcsc_gz -lrcsc_geom -lz -lm`. Neither build creates these shared libraries (only `librcsc`), so use `librcsc.pc` (`-lrcsc`) instead. See [build-install-tools](build-install-tools.instructions.md).
- **RCG v4-v6 share one parser:** `REC_VERSION_6/5/4` all map to `ParserV4` (`parser.cpp:112-114`). Check this before adding a v7 format.
- **JSON parser:** `ParserJSON` is commented out in favor of `ParserSimdJSON` (`parser.cpp:118-119`).
- The `rcg2*` and `rcgversion` tools in `src/` read RCG files through `rcg` and `gz`; they are covered in [build-install-tools](build-install-tools.instructions.md).

## See Also

- [player-world-model](player-world-model.instructions.md)
- [coach-trainer-clang](coach-trainer-clang.instructions.md)
- [build-install-tools](build-install-tools.instructions.md)
- [../copilot-instructions.md](../copilot-instructions.md)
