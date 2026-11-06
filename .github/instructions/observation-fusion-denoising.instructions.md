---
applyTo: "rcsc/player/{world_model,*_object,*_state,*_predictor,*_tracker,*_info,self_object,ball_object,player_object,interception,intercept_table}*"
---

# Observation Fusion & Denoising in the World Model (librcsc)

How localized `see` data is merged with the internal forward model. Paths are under `rcsc/player/`.

## TL;DR

Fusion is mostly replace-and-age: a forward model predicts each object, a fresh `see` overwrites it, and age counters (`*_count`) plus gating by speed and sensor error act as the denoiser.

- There is no Kalman/averaging filter for ball or players; the ONLY blend is self position (`SelfObject::updatePosBySee`, `self_object.cpp:817-868`).
- Ball/player errors are additive bounds (`self.posError + rpos_error`), not variances.
- Players are matched to old tracks by unum, else by nearest distance under a reachability gate `speed_max*dash_noise*count + errors`.
- Ghosts (objects expected in view but not seen) are flagged and erased by `checkGhost`.
- **Open the full file when:** you change the update order, gating thresholds, count thresholds, ghost logic, or want to add real filtering.

## Pipeline (`world_model.cpp`)

- `updateAfterSenseBody` (863-905): `SelfObject::updateAfterSenseBody` then `update()` (733-856): predicts self (`M_self.update`), ball (`M_ball.update`), each player list (remove when pos count invalid); increments `M_dir_count`; clears at BeforeKickOff/AfterGoal; rotates the view-area deque.
- `updateAfterSee` (1110-1249): `localizeSelf` -> `localizeBall` -> `localizePlayers` -> `updatePlayerType`; then, if `self.posCount()<=10` and quality HIGH, builds a `ViewArea` and runs `checkGhost` + `updateDirCount`. With fullstate it only adds a view area. Under `full_view` (or a 360° view area) `updateDirCount` resets every direction count to 0 each cycle.
- `updateJustBeforeDecision` (1978-2000): `updateBallByHear`, `updateGoalieByHear`, `updatePlayerByHear`, `updatePlayerStaminaByHear` (see [communication-audio-denoising](communication-audio-denoising.instructions.md)).
- Count thresholds: set in `player_agent.cpp:828-836`; defaults `player_config.cpp:108-118`: self pos/vel/face 20/10/5, ball pos/rpos/vel 10/5/10, player pos/vel/face 30/5/2. `*Valid()` = count <= threshold.

## Self

- Forward model `SelfObject::update` (`self_object.cpp:200-408`): applies last body command (dash accel, turn moment, move, neck), `pos += vel`, `vel *= playerDecay`; errors: `pos_error/vel_error += playerRand*|vel|` (343-348), `vel_error *= decay`; all counts ++ (369-377).
- Sense-body correction (482-770): vel from speed/dir (see [observation-parsing-localization](observation-parsing-localization.instructions.md)); neck replaced if differs by > 0.5 (532); collision estimated when speed < 0.2 of predicted and direction reversed (661-669).
- **See blend** `updatePosBySee` (817-907): if `M_pos_count==1` (previous fix one cycle old), per axis: if `old_err < new_err` then `pos = new + (old - new) * new_err/(old_err+new_err)`, `err = (old_err+new_err)/2` (833-868); otherwise the new fix replaces (895-896). Then `pos_count=0`, face/body replaced, counts 0 (899-907). Position fix at `world_model.cpp:2177-2258` (`updateAngleBySee`, `updateVelDirAfterSee`, `Localization::localizeSelf`, `updatePosBySee`).

## Ball

- Forward model `BallObject::update` (`ball_object.cpp:128-258`): `vel += kick accel` (clamped to `ballAccelMax`, `accel_err` from `ActionEffector`), speed clamped `ballSpeedMax`; `vel_error += |v|*ballRand`; `pos += vel`, `pos_error += vel_error`; `vel *= ballDecay`, `vel_error *= ballDecay`; counts ++. `pos_count` rule: in PlayOn-like modes ++; otherwise set to 1000 (lost) when `posCount>=5` or `rposCount>=2 && dist*1.05 < visibleDistance`, else 1 (204-227).
- See update `WorldModel::localizeBall` (2265-2465): needs valid self face; if self pos invalid only `rpos` (and vel) are set (2309-2338); else `pos = self.pos + rpos`, `pos_error = self.posError + rpos_error`, `gvel = self.vel + rvel` (errors added). Stored via `updateAll`/`updatePos` (`ball_object.cpp:514-559`): plain replacement, `pos_count=0`, `lost/ghost=0`.
- No-velocity fallback `estimateBallVelByPosDiff` (2472-2719): rpos difference over 1/2/3 steps with decay formulas; gates dist < 3.15 and > 1.5, |dv| < 0.1 cancel, `ball_rand*2/3 + 0.1` consistency; if dist < 2.0 use pos diff (2380-2393); 2013 rule with `firstBallSpeed` (2396-2426).
- Ghost: `BallObject::setGhost` (106-120) pins `pos_count=1000`.

## Players (live path = `WorldModel::localizePlayers`, 2726-3091)

- `PlayerObjectUpdater` (`player_object_updater.cpp`, global matching with pair combinations) is **dead**: `#if 0` at `world_model.cpp:2728`. Constants there: `self_error 1.2`, sensor_error*3.5, <=3 candidates (add_matching_candidates 356-406).
- Requires valid self face+pos (2736). Order: opponents, unknown opponents, teammates, unknown teammates, unknown players, each localized via `Localization::localizePlayer`.
- `checkTeamPlayer` (3098-3356): (1) exact unum match -> `updateBySee`; (2) else candidates from the known list (unum compatible) and `old_unknown_players`; gate `d <= realSpeedMax*(1+playerRand)*count + heard_err(2.0 if heard fresher) + 1.0 + 2*dist_error_` with `count`/`pos` from the fresher of seen/heard; pick nearest (initial min 10*10 on an unsquared distance, effectively 100 m); (3) otherwise `emplace_back` a new object. The final `candidate != old_unknown_players.end()` check (3331) compares against the unknown list's end even for a known-list candidate (suspicious, not verified at runtime).
- `checkUnknownPlayer` (3364-3665): same gate over opponents, teammates, unknowns; nearest of the three wins; teammates use `heardPosCount <= seenPosCount` (3459) while others use `<` (asymmetric). No match -> new unknown player.
- `PlayerObject::updateBySee` (`player_object.cpp:184-310`): sets side, resets `ghost_count`; if see gave velocity use it; else if `0 < pos_count <= 2` and `rpos < 40`: `vel = last_seen_move/last_count`, clipped to speed_max, `*decay` (226-247); else `vel=0, vel_count=1000`. Then `pos = seen`, counts 0; body from see, else from movement (> 0.2) or vel (> 0.2).
- `PlayerObject::update` (151-176): `pos += vel` **without decay** (157), counts ++ (max 1000).
- Limits after update (3015-3070): teammates > 10 and opponents > 11 forget the worst by `PlayerPtrAccuracySorter`; unknowns removed when total > 25 (never one with `posCount==0`). Entries with invalid pos removed (3081-3082).
- Unknown unum resolution: only via exact unum match, nearest-track assignment above, or hear (`updatePlayerByHear`); no formation-based inference found.

## Ghost removal (`checkGhost`, 5331-5470)

- Ball: if `rposCount>0`, pos valid and inside the view area with `vis_dist = visibleDist - |self_vel|/decay*0.1 - |ball_vel|/ballDecay*0.05 - 0.12*min(4,posCount) - 0.25` and angle buffer 5 deg -> `setGhost`.
- Players: `vis_dist = visibleDist - |self_vel|/decay*0.1 - 0.25`; in-area and `posCount>0` -> `setGhost`; erase when unum unknown and `posCount>=10` and `ghostCount>=2` (teammates/opponents, angle buffer 5.0 / 1.0); unknown players erased when dist < 40*1.06 or already ghost.

## E. Hook points for denoising

- Replace ball replacement at `ball_object.cpp:514-559` and `world_model.cpp:2265-2465` with a gain-based filter using `pos_error` (inputs exist: `rpos_error`, `vel_error`).
- Reuse the self blend (`self_object.cpp:833-868`) as a template; extend to players (needs a new `pos_error` member in `PlayerObject`; none exists).
- Gate constants live in `world_model.cpp:3140-3141, 3183-3199, 3388-3389` (`dash_noise`, `self_error`, `dist_error_*2`).
- Add decay to `PlayerObject::update` (`player_object.cpp:156`) if a velocity model is wanted.

## Not Found

- No Kalman/EKF, no particle filter for ball/players (self uses a point set, see parsing file).
- No player position-error bound member; no formation- or team-based unknown-player identification.

## See Also

- [observation-parsing-localization](observation-parsing-localization.instructions.md)
- [communication-audio-denoising](communication-audio-denoising.instructions.md)
- [player-world-model](player-world-model.instructions.md)
- [../copilot-instructions.md](../copilot-instructions.md)
