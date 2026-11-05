---
applyTo: "rcsc/formation/**,rcsc/ann/**,rcsc/geom/**"
---

# Formation, ANN & Geometry (librcsc)

Three loosely coupled libraries: formation loading (static/CSV/JSON/versioned text), ANN models used only by legacy formations, and the 2D geometry primitives that current clients (helios-base) use heavily.

## TL;DR

Formations load through a format dispatcher, ANN code is legacy-only, and the geometry primitives are the heavily used current part.

- Current formation API: `FormationParser::parse(path)` picks a parser by the file's first line, returning `Formation::Ptr` (helios: `src/player/strategy.cpp:292`).
- **Current formation parsers:** V1/V2/V3 (versioned text), Static, CSV, JSON. The Delaunay (`DelaunayTriangulation`) formation is created by name.
- **ANN (`rcsc/ann`) is legacy:** it is built but reached only from `rcsc/formation/obsolete/`; helios-base does not include it.
- **Geometry (`rcsc/geom`) is current and heavily used:** `Vector2D` and `AngleDeg` lead by a wide margin in helios includes.
- **Open the full file when:** you add a formation format, change geometry primitives used by helios, or touch an ANN model.

## Overview

- **Formation** = a mapping from role number to a position in a field coordinate frame, optionally with role types, position pairs, marker flags, and set-play flags. Data is either static or trained/interpolated.
- **ANN** = neural network models (BPN, NGNet, RBF, SIRM) that once generated formations.
- **Geom** = 2D math: vectors, angles, lines, segments, rays, rectangles, polygons, circles, sectors, triangulation, Voronoi, convex hull.

## Architecture

### Formation (`rcsc/formation/`)

- `Formation` (`formation.h`): abstract base; carries `version()`/`setVersion()` (`formation.h:93`, `formation.cpp:50`) and `roleNames()`, `positionPairs()`.
- Implementations: `FormationStatic` (`formation_static.h`, `create()` at :68), `FormationDT` (`formation_dt.h`, `create()` at :72; `FormationDT::NAME = "DelaunayTriangulation"` at `formation_dt.cpp:43`, factory in `formation.cpp:243`), `FormationData` (`formation_data.h`), `RoleType` (`role_type.h`).
- Parser base: `FormationParser` (`formation_parser.h`), with static `parse(filepath)` (`formation_parser.h:118`; definition `formation_parser.cpp:45-58`).
- Concrete parsers: `FormationParserV1`/`V2`/`V3`, `FormationParserStatic`, `FormationParserCSV`, `FormationParserJSON`.

**Dispatch (`FormationParser::create`, `formation_parser.cpp:127-190`)**, applied to the first non-comment, non-empty line:
1. Starts with `{` -> `FormationParserJSON`.
2. `Method , <name>` -> `FormationParserCSV`.
3. `Formation <method> <ver>` -> `V3` (ver 3), `V2` (ver 2), `V1` (ver 1).
4. `Formation <name>` (one token) -> `FormationParserStatic` if the name starts with `Static`, otherwise falls back to `FormationParserV2` (`:181`).
5. Anything else -> returns null; `parse()` then fails with "could not create the formation parser instance".

### ANN (`rcsc/ann/`)

- `BPNetwork1` (`bpn1.h`, header only), `NGNet` (`ngnet.h`), `RBFNetwork` (`rbf.h`), `SIRM` (`sirm.h`), `SIRMsModel` (`sirms_model.h`).
- Built by CMake (`ann/CMakeLists.txt:2-6` lists `ngnet.cpp`, `rbf.cpp`, `sirm.cpp`, `sirms_model.cpp`) and by autotools (`noinst_LTLIBRARIES`).
- Consumers: only `rcsc/formation/obsolete/` (for example `formation_bpn.h:36` includes `rcsc/ann/bpn1.h`).

### Geometry (`rcsc/geom/`)

- Core: `Vector2D` (`vector_2d.h`), `AngleDeg` (`angle_deg.h`), `Line2D`, `Ray2D`, `Segment2D`, `Rect2D`, `Circle2D`, `Sector2D`, `Triangle2D`, `Polygon2D`, `Matrix2D`, `Size2D`, `Region2D`.
- Composite regions: `UnitedRegion2D`, `IntersectedRegion2D` (`composite_region_2d.h`).
- Computational: `ConvexHull` (`convex_hull.h`), `DelaunayTriangulation`, `ConstrainedDelaunayTriangulation`, `VoronoiDiagram`, `SegmentIntersection` detectors (`segment_intersection.h`).
- Umbrella: `rcsc/geom.h` includes the module headers and is installed.

## Patterns & Conventions

- **Value types with operators:** `Vector2D` and `AngleDeg` are copied by value and used in arithmetic (helios uses them everywhere).
- **Formation method overrides:** a formation subclass overrides `methodName()`, `getPosition(num, ...)`, `getPositions(...)`, `train(...)` and `toData()` (see `formation_dt.cpp:54-258`).
- **Parser helpers:** each parser checks structure in order (for example V3's `parseHeader`, `parseRoles`, `parseBeginRolesTag`, `parseData`, `parseEnd`; `formation_parser_v3.cpp:42-57`).
- **Config samples:** `rcsc/formation/test/example-*.conf` (`csv-dt`, `csv-static`, `json`, `static`, `v1`, `v2`, `v3`) are format references.

## Integration Points

**Helios-base (verified by include scan):**
- `formation/formation.h`: 5 includes; `formation/formation_parser.h`: 1 include (`src/player/strategy.cpp:60`, `sample_communication.cpp:40`, `keepaway_communication.cpp:40`, `role_sample.cpp:41`).
- Formation loading: `src/player/strategy.cpp:292` (`FormationParser::parse(filepath)`).
- Geometry includes (counts): `vector_2d.h` 60, `angle_deg.h` 10, `rect_2d.h` 13, `circle_2d.h` 7, `ray_2d.h` 7, `line_2d.h` 6, `segment_2d.h` 3, `sector_2d.h` 2, `matrix_2d.h` 1, `voronoi_diagram.h` 1.
- Helios does **not** include any `rcsc/ann/*` header.

**Module dependency scan (non-obsolete code):** `formation -> geom`, `geom -> time`. No non-obsolete module depends on `ann`.

## Build & Test

- Formation objects compile in both build systems (`rcsc/formation/CMakeLists.txt:2-14`, `formation/Makefile.am` via `rcsc/Makefile.am` `SUBDIRS`). The obsolete formation files are not in either list.
- Geometry tests (`rcsc/geom/Makefile.am:82-94`): `check_PROGRAMS` / `TESTS` of `run_test_*` targets, linked against CppUnit with `$(CPPUNIT_LIBS)`. They run only with `./configure --enable-unit-test` and `make check`. The list I read included `vector_2d`, `matrix_2d`, `segment_2d`, `triangle_2d`, `rect_2d`, `polygon_2d`, `voronoi_diagram`, `convex_hull`; I did not confirm the full list.
- No CMake test targets exist (no `add_test`/`enable_testing` in any build file).

## Important Notes

- **Legacy formations:** `rcsc/formation/obsolete/` contains `formation_bpn`, `_cdt`, `_knn`, `_ngnet`, `_rbf`, `_sbsp`, `_uva`. Each has a `readConf` method and an ANN dependency. None is compiled or used by helios-base.
- **Broken umbrella include:** `rcsc/rcsc.h:38` includes `<rcsc/bpn1.h>`, but the file is `rcsc/ann/bpn1.h`. Including `rcsc/rcsc.h` will fail. Nothing in helios-base includes `rcsc.h`.
- **Formation version numbers are data, not an API version:** `FormationParserV3`/`V2` check `version != 2` for the data block (`formation_parser_v3.cpp:356`, `formation_parser_v2.cpp:262`).
- **Typo in test file:** `rcsc/geom/test_rundom_convex_hull.cpp` (sic).

## See Also

- [player-world-model](player-world-model.instructions.md) (uses geometry types for world objects)
- [action-skills](action-skills.instructions.md)
- [build-install-tools](build-install-tools.instructions.md)
- [../copilot-instructions.md](../copilot-instructions.md)
