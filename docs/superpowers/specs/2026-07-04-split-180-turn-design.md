# Split 180° turns into two 90° turns with a scan pause

**Date:** 2026-07-04
**Context:** SuperTeams Challenge
**Status:** Approved — implement directly

## Goal

Replace a single in-place 180° turn with two 90° turns separated by a 500 ms
pause. Purpose: give the left/right cameras a stable, still window at the
intermediate 90° heading to scan the maze walls that are otherwise only ever
"front"/"back" walls (never swept by the side cameras during straight driving).

## Constraint that drives the design

The 500 ms pause **must not** block. `cyclicMainTask()` and `cyclicRunTask()`
(the latter runs `cam.Update()` and can fire the ejectors) execute once per
main-loop iteration, *before* the `RunState` switch. A blocking `delay(500)`
would freeze the camera scan this feature exists for. The pause is therefore a
new **non-blocking timed state**, not a delay.

## Approach

Implement in the `main.cpp` state machine, reusing the existing
`StartTurn` / `ControlTurn` / `EndTurn` primitives unchanged in spirit.

### Mapping-correctness invariant

`EndTurn()` is the only thing that calls `p_mapSys->Turn()`. It is called
**only on the final leg — never at the intermediate**. From the mapper's view
exactly one `Turn(final)` fires — byte-for-byte identical to today's unsplit
180°. The split is purely "current 180° behaviour + a still pause inserted mid
rotation." Zero mapping risk.

### Leg target vs. robot target

`robotTargetAngle` (the `Orientations` the robot wants to face) is **also** read
by `ControlDrive` (main.cpp:211, 411) as the post-turn wall-following heading, so
it must stay the *final* orientation throughout. The turn leg uses a **separate**
`float _turnLegTargetAngle`; the `TURN` state drives `ControlTurn` from that, not
from `robotTargetAngle`.

### Flow (example North → South)

```
GET_INSTRUCTIONS  StartTurnToOrientation(South): detects 180°,
                  leg target = East (+90° CW), _turnSplitPending = true
TURN              ControlTurn(East) ... TURNED   (ControlTurn Stop()'s motors; NO EndTurn)
TURN_PAUSE        hold 500 ms                     (motors stay stopped; cameras + ejectors live)
TURN (leg 2)      StartTurn(South); ControlTurn(South) ... TURNED
                  EndTurn()  -> single map Turn(South) + wall-align  -> SETTILE
```

During `TURN_PAUSE` the robot keeps the mid-turn sensor state (bumpers off,
color frozen, ToF off — inherited from the first `StartTurn`, restored only by
the final `EndTurn`), so `cyclicRunTask()` behaves exactly as it does mid-turn:
no motion commanded, cameras and ejectors active.

### 180° detection

`((int)target - (int)current + 4) % 4 == 2`, where
`current = gyro.GetOrientationFromAngle()`. Intermediate = `(current + 1) % 4`
(+90° CW). Direction is arbitrary for scan coverage (both neighbours scan the
same two walls); CW chosen for simplicity.

### Turn speed on the legs

Both 90° legs are forced to the `TURN_180_SPEED` (60) cap — the "camera
frame-rate limit" — so cameras also get usable frames *during* rotation, not
only in the pause. Implemented via a defaulted `bool force180Speed = false`
parameter on `StartTurn` that OR's into the existing `_TURN_180_DEGREE` flag
(`_TURN_180_DEGREE` is read only for the speed cap, Driving.cpp:96).

## Changes

| File | Change |
|---|---|
| `lib/CustomDatatypes/src/CustomDatatypes.h` | Add `TURN_PAUSE` to `RunState` (additive; no `switch` on RunState exists). |
| `lib/Driving/src/Driving.h` | `StartTurn(float angle, bool force180Speed = false)`. |
| `lib/Driving/src/Driving.cpp` | Honor `force180Speed` when setting `_TURN_180_DEGREE`. |
| `src/main.cpp` | `TURN_SPLIT_PAUSE_MS` const; `_turnLegTargetAngle`, `_turnSplitPending`, `ts_turnPauseStart` vars; `StartTurnToOrientation()` helper (dedups the four `T_*` cases); leg logic in `TURN`; new `TURN_PAUSE` state. |

## Verification

`pio run` must succeed. Full behaviour verification requires hardware:
confirm a 180° command produces two 90° turns with a ~500 ms still hold between
them, cameras active during the hold, and the map ends in the correct final
orientation.
