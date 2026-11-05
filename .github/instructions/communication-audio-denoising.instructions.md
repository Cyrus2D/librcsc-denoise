---
applyTo: "rcsc/player/{audio_sensor,say_message_builder}*,rcsc/common/{audio_codec,audio_memory,audio_message,say_message,say_message_parser,freeform_message}*"
---

# Inter-agent Messaging (say/hear) as a Denoiser (librcsc)

How teammates exchange compact `say` strings and how heard data corrects the world model. Note: `audio_memory`, `audio_codec`, `say_message_parser`, `freeform_message*` live in `rcsc/common/`; builders and `AudioSensor` in `rcsc/player/`.

## TL;DR

Players pack 1-22 typed messages into at most `say_msg_size` (10) characters of a 74-symbol alphabet; receivers decode into `AudioMemory`, and `WorldModel` uses them only to refresh stale ball/player tracks.

- Message payload is quantized by hand (0.1-1.0 m steps), so heard data is no more precise than a close `see`, but it is noise-free and independent of distance and view cone.
- The heard direction is ignored by every parser (the `dir` parameter is unnamed, `say_message_parser.cpp:66`); no distance exists in `hear`.
- Heard info overrides state only when the local track is stale (`posCount >= 2/5`) or inconsistent; see thresholds below.
- Freeform messages are coach-only (`AudioSensor::parseCoachMessage`, 254-326); player audio uses `say_message_parser`.
- **Open the full file when:** you add/modify a message type, change bit budgets or precision, or tune how heard data corrects the world model.

## Receive path

- `PlayerAgent::analyzeHear` (`player_agent.cpp:1689`) -> `analyzeHearPlayer` -> `AudioSensor::parsePlayerMessage` (`audio_sensor.cpp:162-247`): `sscanf " (hear %ld %lf %7[^ ] %d "` = cycle, dir, sender (`our`/`opp`), unum; strips quotes; empty ignored. Teammate messages are cleared when `time != M_teammate_message_time` (225-233), then `parseTeammateMessage` (398-433): loop on the first character, look up `M_say_message_parsers[tag]`, `parse(unum, dir, msg, time)` returns consumed length, `<0` aborts the rest; unknown tag aborts. Opponent messages are stored but not parsed (237-246); `hearOpponentAudio()` gates (`player_agent.cpp:1343`).
- Parsers call `AudioMemory::set*` (`rcsc/common/audio_memory.cpp:74+`): each list is cleared the first time it is written in a new cycle (e.g. 85-91), so memory holds only this cycle's messages.
- Server limit (ground truth): one heard message per group per cycle (`hear_max/inc/decay 1/1/1`), so extra teammates' messages may be dropped before reaching librcsc.

## Alphabet and codec (`rcsc/common/audio_codec.cpp`)

- 74 characters: `a-zA-Z` + ` ().+-*/?<>_` + `0-9` (141-146). `encodeInt64ToStr(v, n)` = base-74 with n digits (179-210).
- Position codecs: `posToBit18` x 9 bit over [-52,52], y 9 bit over [-34,34] (335-358, steps 0.2035/0.133); `posToBit19` x 10 bit [-52.5,52.5], y 9 bit (387-410, steps 0.1026/0.133); `posVelToBit31` = pos19 + sign+5-bit speed per axis, step `ballSpeedMax/31` (437-461). `encodePercentageToChar` = `rint(v*73)` (270-300). Consts: `X_NORM 57.5`, `Y_NORM 39`, `SPEED_NORM 3` (49-51).

## All say message types (builder `say_message_builder.h/.cpp`, parser `say_message_parser.h/.cpp`; length = header + payload chars, `slength()`)

| Hdr | Class | Len | Payload / precision |
|---|---|---|---|
| b | Ball | 6 | pos19+vel signed 5bit/axis (pos ~0.1/0.13, vel ~0.097) via `encodePosVelToStr5` |
| p | Pass | 10 | receiver unum + receive pos (4 chars, pos19) + ball pos/vel (5 chars) |
| i | Intercept | 3 | unum char + cycle char (builder 599-635) |
| g | Goalie | 5 | x in [37,52.9] 0.1, y [-19.9,19.9] 0.1, body 1 deg (204-250) |
| e | GoalieAndPlayer | 8 | goalie as 'g' + player unum 1..22, x,y step 0.555 (295-353) |
| o | OffsideLine | 2 | x in [10,52], 1 percentage char (~0.58 m) |
| d | DefenseLine | 2 | x in [-52,-10], 1 char |
| w | WaitRequest | 1 | header only |
| F | Setplay | 2 | wait-step char |
| h | PassRequest | 4 | pos18 (3 chars) |
| s | Stamina | 2 | rate char (1/73) |
| r | Recovery | 2 | rate char |
| c | StaminaCapacity | 2 | rate char |
| D | Dribble | 4 | target 0.5 m, queue count 1..10 (870-902) |
| G | BallGoalie | 10 | ball 0.1 m, vel 63 levels/axis (or none), goalie 0.1 m + 1 deg (947-1019) |
| P | OnePlayer | 4 | unum 22, x,y step 0.63 (1073-1111) |
| Q | TwoPlayer | 7 | 2 x (unum, x,y 0.63) (1176-1218) |
| R | ThreePlayer | 10 | 3 x (unum, x,y 0.63) (1302-1344) |
| S | Self | 5 | x,y 0.4 m, body 6 deg, stamina 10% (1412-1448) |
| T | Teammate | 5 | unum 11, x,y 0.7 m, body 2 deg (1498-1540) |
| O | Opponent | 5 | same layout as T (1575+) |
| B | BallPlayer | 10 | ball pos/vel (5) + player unum 22, x,y 1.0 m, body 2 deg (4 chars, 1711-1751) |

Header chars and lengths: parser header file `say_message_parser.h` (`sheader()`/`slength()` per class, e.g. b:132/145, S:1250/1263). Builders reject a message when `to.length()+slength() > playerSayMsgSize()` (`say_message_builder.cpp:54`); `ActionEffector::makeSayCommand` (`action_effector.cpp:1723-1760`) appends in insertion order (the priority sort is commented out, 1733) and `addSayMessage` (1539) does no size check. `PlayerSayCommand` carries the string.

## Heard data -> world model (`WorldModel::updateJustBeforeDecision`, `world_model.cpp:1988-1991`)

All skipped when `M_fullstate_time == time()`.

- **Ball** `updateBallByHear` (1486-1563): among this-cycle ball messages (b, p, B, G) choose the sender whose known teammate position is nearest to the current ball estimate (unknown senders only if none known); then `BallObject::updateByHear` (`ball_object.cpp:566-695`): stores `M_heard_pos/vel` (count 0); no change if last command was kick; a pass with valid vel and `seenVelCount>0` replaces vel (and pos when `seenPosCount>0`); ghost-confirmed ball is replaced; replace when `posCount>=5` or (`posCount>=2` and (`dist_diff > sender_dist*0.05+1.0` or `sender_dist < dist_from_self*0.95`)); or when `posCount>0`, ball beyond visible distance and sender within `visibleDistance-1`. Replace sets `pos_count=1, vel_count=1`.
- **Goalie** `updateGoalieByHear` (1570-1719): averages heard goalie data; skip if the goalie was just seen (`posCount==0 && bodyCount==0`); else match the known goalie, else nearest unknown in the penalty area within `posCount*speedMax + dist*0.06`, else create a new goalie.
- **Players** `updatePlayerByHear` (1726-1925): unum 1..11 = our side, 12..22 = theirs; skip self; match by unum, else nearest same-side/unknown-unum track with `d < posCount*1.2 + distFromSelf*0.06` (also among unknown players, which are promoted); else add new player. `PlayerObject::updateByHear` (`player_object.cpp:374-436`): sets `heard_pos` count 0, `ghost_count=0`, assigns unum if `unumCount>0`, caps `unum_count` at 2; replaces pos only if `seenPosCount>=2` or (`>0` and dist > 20), then `pos_count=1`; body only if `bodyCount>=2` and given (`!= -360`).
- **Stamina** `updatePlayerStaminaByHear` (1930-1961): recovery and capacity per teammate.
- **Lines**: offside line replaced when `new_line < heard_x - 1.0` (`world_model.cpp:4363-4382`, count 30); defense line similarly (4510-4524).
- **Intercepts**: `InterceptTable::hearTeammate/hearOpponent` (`intercept_table.cpp:234, 276`) from `world_model.cpp:5285-5298`. Pass sender excluded/marked at 4203-4205.
- Dribble, wait, setplay, pass request: stored in `AudioMemory` for team logic; no WorldModel update found.

## E. Denoising value and hooks

- Heard positions have fixed quantization error (up to ~0.35-0.5 m for 0.7-1.0 m steps) but zero sensor noise and unlimited range: useful to bound drift of unseen/far tracks (`posCount` large) and to identify unknown players (unum 1..22).
- Server noise models do not touch hear, so Gaussian vs quantized see does not change decoding; only the self-pose quality of the sender matters (a sender's own noisy pose is relayed).
- Hooks: gates in `ball_object.cpp:651-693` and `player_object.cpp:406-412` (replace vs ignore; could weight by `pos_error`), codec steps in `say_message_builder.cpp` (e.g. T/O 0.7 m, B 1.0 m: raise precision to cut relayed quantization), message choice in client code (helios `communicationImpl`).

## Not Found

- No use of the `hear` direction; no distance-based hear filtering; no checksum or redundancy; no use of heard data to estimate self pose.
- No `pos_error` stored from heard values in the ball/player objects.

## See Also

- [observation-fusion-denoising](observation-fusion-denoising.instructions.md)
- [observation-parsing-localization](observation-parsing-localization.instructions.md)
- [player-world-model](player-world-model.instructions.md)
- [../copilot-instructions.md](../copilot-instructions.md)
