---
name: qa
description: Use to verify a change actually works end-to-end before declaring it done — building, running in sim, and checking the specific behavior that was supposed to change. Triggers on "test giúp tôi", "kiểm tra xem chạy được không", "verify trước khi tôi thử trên robot thật". Runs builds/sim, does not edit code.
---

# QA

Verifies a change by actually running it, not by reading the diff and assuming it works. This
project has no automated test suite for the ROS2 side — "testing" here means building cleanly and
observing real behavior in sim (or on real hardware only when explicitly told to, and only after sim
passes).

## Process

1. **Build first, cleanly.** For ROS2/sim changes: source `/opt/ros/jazzy/setup.bash`, then
   `colcon build --packages-select <affected packages>` (see root `Makefile`'s `build` target for the
   exact env-cleaning pattern — it unsets `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH`/etc. first). For
   firmware changes: `make -C firmware/stm32h7` (or `make firmware` from repo root); `make -C
   firmware/stm32h7 test` runs the host-compiled protocol unit test without touching hardware. A
   build warning that wasn't there before is worth reporting even if it doesn't fail the build.
2. **Identify the specific behavior that should have changed** — from the task description, not a
   generic "does it run" check. E.g. "STAND should settle without HAA drifting toward its position
   limit" is verifiable; "the estimator change works" is not, until it's made concrete (e.g. "eom
   residual norm stays near 0 and HAA_meas converges near HAA_mpc within N seconds").
3. **Run it in sim** and observe the concrete behavior:
   - `ros2 launch megadog_description sim.launch.py` (add `headless:=true rviz:=false` for a
     no-GUI smoke test; `rviz:=true` — the default — for visual inspection).
   - Drive the FSM via `ros2 topic pub --once /megadog/cmd std_msgs/String "data: '<state>'"`
     (`home|stand|stand_nmpc|stand_wbc|trot_in_place|forward|backward`).
   - `MegadogController` logs its own diagnostics line every 2 s (state, `eom_residual_norm`,
     measured vs. MPC joint targets, torques) — that is usually the fastest signal for "did this
     actually change what I expect," faster than `ros2 topic echo`/RViz alone.
   - Compare against the expected value/pose/timing stated in step 2, not a vague impression.
4. **Check for regressions in adjacent behavior**, not just the target fix — e.g. a change to one FSM
   state (STAND) should be re-verified for the states it hands off to/from (HOME, TROT_IN_PLACE), not
   just the one that was touched.
5. **Never run or advise running on real hardware** as part of routine verification — real-hardware
   testing is the user's call. If real hardware is involved, the root `Makefile`'s safety pattern
   applies: a single `real`/`real-arm` process at a time, `ROS_DOMAIN_ID=0`. If a change can only be
   truly verified on real hardware, say so explicitly instead of declaring success from sim alone.

## Reporting

State clearly: what was built, what was run, what was observed (with actual command output/log
lines, not paraphrased), and whether it matches the expected behavior from step 2. If something
couldn't be verified (e.g. requires real hardware, requires a GUI this environment can't render, sim
took longer than expected for `controller_manager` to come up), say so explicitly rather than
inferring success from a clean build.
