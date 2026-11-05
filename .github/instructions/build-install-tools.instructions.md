---
applyTo: "CMakeLists.txt,**/CMakeLists.txt,configure.ac,**/Makefile.am,src/**,example/**,librcsc-config.in,librcsc.pc.in"
---

# Build, Install, CLI Tools & Tests (librcsc)

librcsc builds one shared library, `librcsc` (SOVERSION 20), with two build systems: CMake (root `CMakeLists.txt`) and autotools (`bootstrap`, `configure.ac`, `Makefile.am`). Helios-base links it as `-lrcsc`.

## TL;DR

librcsc builds as one shared library (SOVERSION 20) through CMake or autotools, and the two systems disagree on tools, tests, and link names.

- **One library:** CMake target `rcsc` (`rcsc/CMakeLists.txt:21`) and autotools `librcsc.la` (`rcsc/Makefile.am`), both SOVERSION 20 (`CMakeLists.txt:5`, `-version-info 20:0:0`).
- **Build commands in-repo:** `./bootstrap && ./configure --disable-unit-test && make && sudo make install` (`README.md:28-35`). The CMake invocation is not documented in-repo.
- **Clients link `-lrcsc`:** verified in helios `configure.ac:67` and `CMakeLists.txt:21` (`find_library(... NAMES rcsc)`). `librcsc-config.in` lists stale `-lrcsc_*` names (see Important Notes).
- **CLI tools:** `src/` builds `rcg*` and `rclm*` tools; the two build systems disagree on the set (see below).
- **Open the full file when:** you change the library's module list, install layout, tool list, or test wiring.

## Overview

- Language: C++17 (`CMakeLists.txt:13-14`; `configure.ac:126` `AX_CXX_COMPILE_STDCXX_17`).
- Dependencies: Boost `system` (CMake `find_package(Boost 1.41.0 COMPONENTS system REQUIRED)`, `CMakeLists.txt:50`; autotools `AX_BOOST_BASE([1.41.0])`, `configure.ac:132-133`). zlib is optional in CMake (`:56-60`) and checked in autotools (`configure.ac:53-54`).
- Version metadata: CMake `project(librcsc VERSION 2026)` (:3); autotools `AC_INIT([librcsc],[2024],...)` (`configure.ac:6`). They disagree.

## Architecture

### CMake (root and `rcsc/`)

- `CMakeLists.txt`: `LIBRCSC_SOVERSION 20` (:5), `LIBRCSC_BUILDVERSION 20.0.0` (:6), `-DHAVE_CONFIG_H` (:63), `configure_file` for `cmake-config.h.in` -> `config.h` (:64-67), `add_subdirectory(rcsc)` (:75) and `add_subdirectory(src)` (:76), and generation of `Doxyfile`, `librcsc-config`, `librcscenv`, `librcsc.pc` (:89-92). Installs `librcsc-config` and `librcscenv` to `bin` (:94-98) and `librcsc.pc` to `lib/pkgconfig` (:100-103).
- `rcsc/CMakeLists.txt`: builds `rcsc` SHARED from object libraries `rcsc_util`, `rcsc_geom`, `rcsc_gz`, `rcsc_param`, `rcsc_rcg`, `rcsc_net`, `rcsc_time`, `rcsc_ann`, `rcsc_clang`, `rcsc_formation`, `rcsc_color`, `rcsc_common`, `rcsc_monitor`, `rcsc_player`, `rcsc_coach`, `rcsc_trainer` (:21-37). **`action` is absent.** Installs the library to `libdir` and umbrella headers (`types.h`, `geom.h`, `gz.h`, `rcg.h`, `factory.h`, `game_mode.h`, `game_time.h`, `math_util.h`, `random.h`, `soccer_math.h`, `timer.h`, `version.h`) to `include/rcsc` (:68-84). `rcsc.h` is not in that list.
- Per-module `CMakeLists.txt` each define an `OBJECT` library plus `install(FILES ...)` for their headers (for example `rcsc/clang/CMakeLists.txt:20`, `rcsc/color/CMakeLists.txt:17`, `rcsc/common/CMakeLists.txt:26`, `rcsc/formation/CMakeLists.txt:25`, `rcsc/coach/CMakeLists.txt:25`, `rcsc/action/CMakeLists.txt:42`).
- `src/CMakeLists.txt`: executables `rclmscheduler`, `rclmtableprinter` (from `resultprinter.cpp`), `rcg2csv`, `rcg2txt`, `rcgrenameteam`, `rcgresultprinter`, `rcgreverse`, `rcgverconv`, `rcgversion`; each links `rcsc` and `Boost::system` (plus `ZLIB::ZLIB` except `rclmscheduler`). Installed: all except `rcg2csv`.

### Autotools

- `bootstrap` -> `configure.ac` -> `Makefile.am` (`SUBDIRS = rcsc example src`, root `Makefile.am:4`). `doc:` target at `Makefile.am:15` (README says `make doc`).
- `configure.ac` options: `--enable-unit-test` (CppUnit >= 1.12.0, `:141-152`), `--enable-debug` (`-DDEBUG`, `:158-165`), `--enable-example` (`:173-180`). `LT_INIT([shared disable-static])` (:27).
- `AC_CONFIG_FILES` (`:197-223`) lists `librcsc.pc`, `librcsc.spec`, `librcsc-config`, `librcscenv`, and every module Makefile **except `rcsc/action`**.
- `rcsc/Makefile.am`: `lib_LTLIBRARIES = librcsc.la` whose `LIBADD` pulls in convenience libraries `librcsc_<module>.la` (each module's `Makefile.am` uses `noinst_LTLIBRARIES`). `librcsc_la_LDFLAGS = -version-info 20:0:0`. Installed headers are the same umbrella list, under `$(includedir)/rcsc`.
- A commented-out `librcsc_agent.la` block (`rcsc/Makefile.am`) is a leftover from a split-library design.

### Install layout

- Library: `libdir/librcsc.so.20` (CMake SOVERSION 20; autotools `-version-info 20:0:0`).
- Headers: `include/rcsc/...` (module headers via each module's install rules; umbrella headers listed above).
- `pkg-config`: `lib/pkgconfig/librcsc.pc`, with `Libs: -L${libdir} -lrcsc -lz -lm` and `Cflags: -I${includedir}` (`librcsc.pc.in:9-10`). This is the correct link line.
- Scripts: `librcsc-config`, `librcscenv` (`bin`, CMake install; autotools via `configure.ac`).

## Patterns & Conventions

**Code style (`.clang-format`, verified):** based on GNU (`# BasedOnStyle: GNU`); `IndentWidth: 4`; `TabWidth: 8`; `UseTab: Never` (spaces only); `ColumnLimit: 0` (no wrapping); `SpacesInParentheses: true` (`if ( cond )`); `PointerAlignment: Middle` (`Type * ptr`); `NamespaceIndentation: None`; `AccessModifierOffset: -4`; `ContinuationIndentWidth: 4`; `BreakBeforeBraces: Custom`; `SpaceBeforeRangeBasedForLoopColon: true`. The file declares `Standard: c++14` while the project builds with C++17. Observed code follows the same style (`namespace rcsc {` unindented; `#ifndef RCSC_PLAYER_AGENT_H` guards in `player_agent.h:32-33`; LGPL header block in every file).

## Integration Points

- **Helios-base:** `configure.ac:35-67` searches `$HOME/.local`, `$HOME/local`, `$HOME/rcss`, `/opt/robocup`, `/opt`, `/opt/local` for `include/rcsc/types.h`, then adds `-lrcsc`. Its CMake `find_library(LIBRCSC_LIB NAMES rcsc ...)` searches `LIBRCSC_INSTALL_DIR` (`CMakeLists.txt:19-36`).
- **CLI tools (`src/`):** `scheduler.cpp` -> `rclmscheduler`; `tableprinter.cpp` -> `rclmtableprinter` (autotools only); `resultprinter.cpp` -> `rcgresultprinter`, and also `rclmtableprinter` in CMake (divergence); `rcg2txt.cpp`, `rcg2csv.cpp`, `rcgreverse.cpp`, `rcgverconv.cpp`, `rcgversion.cpp`, `rcgrenameteam.cpp`. `rcgvalidator.cpp` and `object_table_printer.cpp` are in `src/` but CMake does not build them; autotools builds `rcgvalidator` (bin) and `object_table_printer` (noinst). `rcg3to4.in` and `rcg4to3.in` are template inputs, not built.
- **Examples:** `example/` programs (`param_main.cpp`, `gzifstream_main.cpp`, `gzofstream_main.cpp`, `loader_main.cpp`, `result_writer_main.cpp`) build only in autotools with `--enable-example`.

## Build & Test

- **Verified from README:** `./bootstrap`, `./configure --disable-unit-test`, `make`, `sudo make install`, `make doc` (`README.md:26-44`).
- **Verified from files (not executed here):** CMake project definition and targets (`CMakeLists.txt`, `rcsc/CMakeLists.txt`, `src/CMakeLists.txt`). I did not run either build.
- **Tests:** only autotools defines them, and they are not built by default.
  - Geometry unit tests: `rcsc/geom/Makefile.am:82-99` (`check_PROGRAMS`/`TESTS` `run_test_*`, `$(CPPUNIT_LIBS)`), run with `--enable-unit-test` and `make check`.
  - CLang parser test: `rcsc/clang/Makefile.am:60-69` (`test_clang_parser`), same condition.
  - `thermo_color_test`: `rcsc/color/Makefile.am:25-34` (`noinst_PROGRAMS`, built by plain `make`).
  - Example programs: `example/Makefile.am` under `BUILD_EXAMPLE`.
  - CMake has no `add_test`/`enable_testing` in any build file.

## Important Notes

- **Stale link names in `librcsc-config.in:40`:** `-lrcsc_agent -lrcsc_clang -lrcsc_ann -lrcsc_net -lrcsc_color -lrcsc_time -lrcsc_param -lrcsc_rcg -lrcsc_gz -lrcsc_geom -lz -lm`. Neither build produces these shared libraries. Use `librcsc.pc` (`-lrcsc`). The same stale names appear in `example/Makefile.am:18-34`, `rcsc/clang/Makefile.am:69`, and `rcsc/geom/Makefile.am:99`.
- **`rcsc/action` is not built** by either system (see [action-skills](action-skills.instructions.md)).
- **Version drift:** CMake `2026` vs autotools `2024` (`configure.ac:6`). SOVERSION 20 agrees.
- **CMake flags overwrite user flags:** `set(CMAKE_CXX_FLAGS "-W -Wall")` (`CMakeLists.txt:19`) replaces any `CXXFLAGS` set on the command line.
- **Broken umbrella include:** `rcsc/rcsc.h:38` includes `<rcsc/bpn1.h>` (real path `rcsc/ann/bpn1.h`). `rcsc.h` is not installed by CMake either.
- **Tool-set drift:** `rclmtableprinter` source differs between CMake (`resultprinter.cpp`) and autotools (`tableprinter.cpp`). `rcg2csv` is built by CMake but not installed. `rcgvalidator` exists only in autotools.
- **Unverified:** `rcsc/rcsc.pro` (qmake) exists; no build file was checked to use it. `librcsc.spec.in` is configured only by autotools (`configure.ac:199`).

## See Also

- [action-skills](action-skills.instructions.md)
- [net-param-rcg](net-param-rcg.instructions.md)
- [player-world-model](player-world-model.instructions.md)
- [../copilot-instructions.md](../copilot-instructions.md)
