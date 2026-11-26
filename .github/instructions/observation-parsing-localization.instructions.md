---
applyTo: "rcsc/player/{visual_sensor,localization,object_table,view_area,see_state,body_sensor,fullstate_sensor}*,rcsc/common/{server_param,player_type}*"
---

# Observation Parsing & Localization (librcsc)

How `(see ...)` and `(sense_body ...)` become relative/global measurements with error bounds. All paths are under `rcsc/player/` unless noted.

## TL;DR

The parser preserves the reported values. The default localizer uses the quantization tables for quantized observations and the reported range with a Gaussian uncertainty interval after the server acknowledges Gaussian observations.

- `VisualSensor::parse` handles both wire formats. `PlayerAgent` sends `(gaussian_see)` when configured and activates the Gaussian range model after `(ok gaussian_see)`. The server exact-observation flags take precedence.
- Self position uses a grid-point sector-intersection filter over flags (not triangulation). Ball and players are a single polar-to-Cartesian conversion with table error ranges.
- `LocalizationDefault::Impl::distanceRange` uses the reported Gaussian distance for landmarks, ball and players. Its working half-width is twice the estimated sigma from the observing player type rates and the measured object/focus geometry. The ordinary paths still use `ObjectTable` and its exact-distance handling.
- Player velocity is computed with "Not consider noise"; the self error is NOT added to player position.
- **Open the full file when:** you change a parser field, an error bound, the see/sense_body timing logic, or add a Gaussian-aware error model.

## A. `see` parsing (`visual_sensor.cpp`)

- `parse` (311-473): skips a duplicate `M_time`, `clearAll`, dispatch by `getObjectTypeOf` (`visual_sensor.h:410-422`): `f`,`g` markers/goals; `F`,`G` behind markers (id Unknown, `M_behind_markers`); `p`/`P` players; `b`/`B` ball; `l` lines. Capital `(B)`/`(P)` are parsed by the same functions (visible-distance objects). Lists are sorted by distance (441-451); lines too (the localizer relies on it).
- `parseMarker` (480-570): dist, dir only; dist_chg/dir_chg not parsed (568). Marker-name map built in ctor (168-303; older map for v<6).
- `parseLine` (577-700): name char at index 2 (v>=6); returns false (line skipped) when the dir token is missing.
- `parseBall` (707-770): dist, dir, optional chg. `has_vel_` true only if chg tokens exist; otherwise `dist_chng_=dir_chng_=0`, `has_vel_=false` (`visual_sensor.h:110-124`).
- `parsePlayer` (777-1045): decides by token count.
  - `(p)` -> `Player_Unknown`; `(p "T")` -> `Unknown_Teammate/Opponent`; `(p "T" N)` -> `Teammate/Opponent` with unum; third token -> `goalie_`; no unum -> `Unum_Unknown`.
  - Missing body/face/arm stay `DIR_ERR=-360` (161-162); missing dist uses `DIST_ERR`. `k` -> `kicking_`, `t` -> `tackle_`. Opponent team name learned from the first opponent (813-818).
  - Fewer than 2 spaces -> `Player_Low_Mode`, dropped (401-402, 989).
- Far/too-far representation: nothing special; a far player simply arrives with fewer tokens, so unum/body/chg are absent (unknown-unum lists, no velocity).

## A2. View width/quality and timing

- `SeeState` (`see_state.cpp`): `M_last_timing` (TIME_0_00 / 50_0 / 22_5 / NOSYNCH), `M_see_count_history`; `getNextTiming` (72-138); `updateBySenseBody` (145-176) forces width/quality to the sensed values; `updateBySee` (183-242); `setNewCycle` (249-275).
- With `ServerParam::fullView()`: `getNextTiming` and `isSynch` take the synch branch, `canSendChangeView` returns true, `setViewMode` sets `M_cycles_till_next_see = 1` and `M_synch_type = SYNCH_EVERY`, and `ViewWidth::width()` returns 360 (`view_mode.cpp`).
- `setViewMode` (433-594): in synch mode cycles-till-next-see is 3/2/1 for wide/normal/narrow (439-468). `canSendChangeView` (352-416); `isSynch` (297-312); `S_synch_see_mode` is static (45).
- `ViewArea` (`view_area.h`): `{width, origin, angle, time}`; `contains(point, dir_thr, vis_dist2)` (152-168), used by ghost checks.
- `PlayerAgent::analyzeSee` (`player_agent.cpp:1550-1624`): `visual_.parse` (1582), `see_state_.updateBySee` (1589), then `WorldModel::updateAfterSee` only if the see time equals the current time. `analyzeSenseBody` (1631-1663): `body_.parse`, `see_state_.updateBySenseBody`, `updateAfterSenseBody`.
- `synch_see` is sent only for version < 18 (`player_agent.cpp:1330`).

## A3. `sense_body` (`body_sensor.cpp`)

- `parse` (91). Speed magnitude quantized 0.01 (207-208); `speed_dir_relative` integer degrees relative to face (213); `head_angle` integer (221); `focus_point` (814-827); stamina/effort/capacity.
- Used by `SelfObject::updateAfterSenseBody` (`self_object.cpp:482-770`): `vel = polar(speed, face + dir)` with dir rounded away from zero by 0.5 and error `dir_err=0.5` (1.0 if dir==0) (600-617); `vel_err = (maxcos-mincos, maxsin-minsin)*(speed+0.005)` using `faceError()+dir_err` (620-631). A reversed/too-small speed marks a collision estimate (644-669).
- `updateVelDirAfterSee` (944-1025) redoes the same velocity once the face is re-estimated by `see` (only if `faceCount()==0`).

## B. Localization (`localization_default.cpp`)

- `USE_OBJECT_TABLE` is defined at :59, so `inverseDistanceRange` (1018-1081) is dead code.
- **Self face** (`estimateSelfFace`, 1222-1249): `getFaceDirByLines` (294-354) first; fallback `getFaceDirByMarkers` (927-1014, nearest + farthest flag). `face_err = 0.5` (1245).
- **Self position** (`localizeSelf`, 1256-1346) is a point-set filter:
  1. `generatePoints` (700-832): up to 16x32 points in a sector around the nearest marker; `dist_inc=max(0.01, err/16)`, dist_loop in [2,16], dir_loop in [2,32]; distance `ave_dist ± dist_error` from the table; direction `seen_dir + face ± (0.5 + face_err)` via `getDirRange` (896-920).
  2. `updatePointsByMarkers` (361-390): intersect with each next marker (up to 30) - `updatePointsBy` (485-613) drops points outside the marker's `Sector2D`. `resamplePoints` (839-889) tops up to 50 with +-0.01 jitter, fixed-seed mt19937.
  3. Behind markers: `updatePointsByBehindMarker` (397-478) guesses the id with `getNearestMarker` (255) from the current estimate; re-filters up to 20 markers when the set is empty.
  4. `averagePoints` (620-693): `pos` = mean of points, `err` = half bounding-box extent (x, y).
- **Ball** (`localizeBallRelative`, 1353-1559): takes `balls().front()`. `rpos = avg_dist*(cos,sin)(avg_dir)`; `rpos_err` = half extent of the 4 dist/dir corners (1441-1450). If `has_vel_`: 16 corner combos of `dist_chng/dist ± 0.01` (hard-coded 0.02 step, 1467-1474) x min/max dist and `dir_chng ± 0.05` (1477-1478); `rvel` = average, `rvel_err` = half extent. The Gaussian case uses a half-step of 0.005 m/cycle divided by the reported range for the radial ratio. At reported range zero it supplies position without a direct velocity estimate.
- **Player** (`localizePlayer`, 1566-1671): `rpos` = polar of table-average distance; `dist_error_` = table error; `pos = self_pos + rpos` (self error NOT added); `vel = (dist_chng, rad(dir_chng)*avg_dist)` rotated by avg_dir plus `self_vel` (1622-1634, comment "Not consider noise" at 1621); body/face/arm are `+ self_face`; kicking/tackle copied.

## B2. `ObjectTable` (`object_table.cpp`, `object_table.h`)

- Hard-coded tables: static landmarks (166-~557; v18 narrow/normal/wide variants at 558/1167/1628 are unused) and movable (2003-2063; v18 at 2066/2175/2253).
- `getDistanceRange` / `getLandmarkDistanceRange` (`object_table.h:170-204`) ignore version and view width and call `getStaticObjInfo` (2322) / `getMovableObjInfo` (2353). Lookup is `lower_bound(see_dist - 0.001)`: first entry >= seen distance, no interpolation; returns false past the table end (movable max 148.4).
- `see_noise_free`: `getDistanceRange`/`getLandmarkDistanceRange` return early with `mean = seen distance`, `error = 0.01` (the server prints the exact distance with 6 significant digits); the table lookup is skipped.
- Generator `createTable(qstep)` (2490-2531) uses `quantize_dist` (2476) with unquantized min/max; `ObjectTable::create` (2453) is never called.
- Sample errors: movable ~5% of d (5.0->0.248, 10->0.499, 20.1->1.0, 60.3->3.02, 148.4->7.42); landmarks ~0.05 near, 0.1 @20, 0.25 @50, 0.5 @100. Movable seen-distance grid ~10% apart at large d.

## E. Mismatch vs server models

- **Quantized (default):** bounds match the server model (`Q(.,0.1)` on log distance); the hard-coded 0.02/0.1 chg steps match.
- **Gaussian:** the range center is the reported distance. The existing sector and corner samplers use a two-sigma working interval (estimated from the measurement and focus), with nonnegative lower distance limits. Angle handling and the existing WM fusion still apply.
- The Exp6 baseline used the old quantization lookup for Gaussian ranges. Its paired replay isolates the range error before the new live pilot also updates landmark localization.

## Not Found

- No per-object `pos_error` in `PlayerObject` (only `dist_error_` in `Localization::PlayerT`, `localization.h:71`).


## See Also

- [observation-fusion-denoising](observation-fusion-denoising.instructions.md)
- [communication-audio-denoising](communication-audio-denoising.instructions.md)
- [player-world-model](player-world-model.instructions.md)
- [../copilot-instructions.md](../copilot-instructions.md)
