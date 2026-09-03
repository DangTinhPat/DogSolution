---
name: find-bugs
description: Use when asked to find bugs, hunt for a root cause, debug "something is wrong but I don't know what", or audit a subsystem for correctness before trusting it in sim or on real hardware. Triggers on "tìm bug", "kiểm tra xem có bug không", "debug giúp tôi", "vì sao lại...". Systematic methodology, not a quick guess-and-check.
---

# Find Bugs

A methodical process for locating real defects in megaDog's NMPC+WBC/ros2_control stack and STM32H7
firmware, where the root cause is often not where the symptom appears (e.g. a WBC posture task
fighting the NMPC's own solution, not the joint driver that visibly misbehaves).

## Process

1. **Reproduce the symptom precisely before touching code.** State exactly what's observed
   (numbers, direction, timing - e.g. from `MegadogController`'s own 2 s diagnostics log, or
   `ros2 topic echo`) vs. what's expected. A vague "it's wrong" leads to a vague fix. If live data is
   available (topics, `MegadogController`'s log line, firmware UART console), read it - don't reason
   from memory of what a value "should" be.

2. **Find every independent source of truth for the same quantity, and compare them.** This
   codebase has more than one place that can define the same physical fact - e.g. `task.info`'s
   initial joint state vs. `megadog_controller`'s `kStandingJointTargetRad` vs. `const.xacro`'s joint
   limits vs. firmware's `motor_calib.c` limits. Grep for all definitions of the value in question
   before assuming any single one is correct, especially after a devq-scaling pass (see
   `makeDevqWbcConfig()`'s own scaling-ratio comments in `MegadogController.cpp`) - a value scaled in
   one place and left stale in another is an easy silent divergence.

3. **Trace the actual data path, not the assumed one.** Follow the real function-call/topic/frame
   path from source to symptom (e.g. `/imu/data` -> `MegadogController`'s IMU callback ->
   `MegadogWbcMeasurement` -> `MegadogWbcRuntime::update()` -> `BaseStateEstimator`/`WbcBase`, or
   `/joint_cmd` -> firmware's microros_bridge callback -> `Actuator_SetTarget()` -> CAN frame), not
   the mental model of how it's "supposed to" work. Read the code, don't guess from a file's name or
   comment alone - names and comments go stale.

4. **Rule out causes with a real test, one variable at a time**, cheapest/safest first:
   - Reproducible offline/in sim (headless `ros2 launch megadog_description sim.launch.py`, or
     firmware's host-compiled `make test` in `firmware/stm32h7/`) beats a real-hardware test.
   - Change exactly one thing per test; record the result before changing the next thing.
   - Prefer a test that can falsify your hypothesis, not just confirm it.

5. **When you find a plausible cause, verify it explains the FULL symptom, not part of it.**
   A theory that explains 80% of the observation and hand-waves the rest is usually wrong or
   incomplete - check for a second, independent issue hiding behind the first.

6. **State the failure scenario concretely**: exact input/state -> exact wrong output. If you can't
   state this, you don't have the root cause yet, only a correlation.

7. **Before proposing a fix, check whether the same class of bug exists elsewhere** (same pattern,
   different joint/leg/gait/config - e.g. a sign convention wrong for one HAA joint is worth checking
   for all four). Fix the class, not just the instance, or report the other locations even if not
   fixing them now.

## Reporting

State findings as: **file:line** -> what's there -> what it should be -> why (the failure scenario).
Rank by confidence, not by how interesting the finding is. If uncertain whether something is a real
bug or intentional (this codebase has several deliberate devq-vs-A1 scaling choices with their own
doc comments - read those before assuming a mismatch is a bug), say so explicitly rather than
presenting a guess as confirmed.

Do not silently fix anything the user asked you to only find - report first, unless the task
explicitly says to also fix.
