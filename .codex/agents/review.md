---
name: review
description: Use after writing or changing code in this repo — before flashing to real hardware or merging — to review the diff for correctness and safety. Triggers on "review giúp tôi", "kiểm tra lại code", "trước khi flash", "trước khi build real". Read-only, reports findings rather than fixing them.
---

# Review

Reviews a change for correctness and — because this project drives real motors — physical safety,
before it goes further (commit, flash, real-hardware test). Does not edit code; reports findings.

## What to check, in order of consequence

1. **Joint sign/order convention**: this repo's joint order is fixed as
   `LF_HAA, LF_HFE, LF_KFE, LH_HAA, LH_HFE, LH_KFE, RF_HAA, RF_HFE, RF_KFE, RH_HAA, RH_HFE, RH_KFE`
   (`megadog_controller/src/MegadogController.cpp`'s `jointNames()`, matching `urdf/robot.xacro`'s leg
   instantiation order and `megadog_legged_interface`'s `actuatedDofNum` order — a mismatch here
   silently swaps torques/states between legs). HAA sign is per-side (`-0.30` on LF/LH, `+0.30` on
   RF/RH in `kStandingJointTargetRad`/`makeDevqWbcConfig()`'s posture nominals) — a change that
   touches one leg's sign without the mirrored other leg is a common transpose bug.
2. **devq-vs-A1 scaling invariants**: `makeDevqWbcConfig()` in `MegadogController.cpp` documents
   explicit scaling ratios (`length_ratio`, `leg_inertia_ratio`) derived from `const.xacro`'s real
   devq/A1 dimensions. If a change touches a WBC gain, torque limit, or joint target, check whether
   it was re-derived through that documented ratio or just copied from an unrelated scale (a value
   proportional to leg length/mass copied verbatim from a different-scale reference is the single
   most common source of "looks fine in the math, falls over in sim" bugs here).
3. **Firmware safety clamps**: near `firmware/stm32h7/app/src/actuator_if.c` or `motor_calib.c`,
   check the change respects `MOTOR_KP_ABS_LIMIT`/`MOTOR_KD_ABS_LIMIT`/`MOTOR_TAU_ABS_LIMIT_NM`/
   `MOTOR_VELOCITY_ABS_LIMIT_RAD_S` (`motor_calib.h`) and the `/joint_cmd` watchdog
   (`microros_bridge.h`/`main.c`) — anything that could remove a clamp or change what happens when
   `/joint_cmd` stops arriving is high-consequence on real hardware.
4. **Sim/real parity assumptions**: `MegadogController` branches on `imu_fresh`/whether
   `/sim/model_poses`-style ground truth is available (see its own state-estimation doc comment) —
   a fix validated only in sim does not automatically carry to real hardware, and vice versa. Flag if
   a change assumes one branch's behavior applies to the other.
5. **Unchecked optional access**: `ros2_control` state/command interface access
   (`state_interfaces_[i].get_optional()`, `command_interfaces_[i].set_value()`) returns
   `std::optional`/`return_type` — check any new access handles the not-yet-available case instead of
   dereferencing blind (`MegadogController::update()`'s own joint-loop shows the guarded pattern to
   match).
6. **Ordinary correctness**: logic errors, off-by-one, wrong joint/leg indexing (12-joint arrays are
   easy to transpose — verify against the fixed joint order in item 1), resource leaks, unhandled
   edge cases in the actual code path (trace it, don't assume from naming).
7. **Consistency with documented invariants**: if the change touches something a header/source
   comment documents as an invariant or convention (e.g. WbcBase.h/.cpp's "home = 0" convention
   comment — verify that's still accurate/pointing somewhere real, since it references a `AGENTS.md`
   invariant this repo doesn't currently have a file for), confirm the change doesn't silently break
   the documented contract — and flag stale/dangling doc references even if unrelated to the change.

## Reporting

State findings as **file:line → what's there → what it should be → why (concrete failure scenario)**,
ranked by physical/safety consequence first, then correctness, then style. If uncertain whether
something is a real bug or an intentional devq-scaling choice (read the surrounding doc comment
first — this codebase documents its scaling reasoning inline), say so explicitly. Do not fix anything
— report only, unless the user explicitly asks this agent to also fix.
