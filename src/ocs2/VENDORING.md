# OCS2 vendoring record

This directory (`src/ocs2/`) and the sibling `src/ocs2_robotic_assets/`
vendor a subset of [leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2)
and [leggedrobotics/ocs2_robotic_assets](https://github.com/leggedrobotics/ocs2_robotic_assets),
so babyDog can build the real OCS2 SQP/MPC stack on ROS2 Jazzy without ROS1.

See `../../CLAUDE.md` and `/home/dvt/.claude/plans/purrfect-imagining-hartmanis.md`
for why this exists. This file records exactly what was pulled in and why, so
a future re-vendor onto a newer upstream commit is a rebase against this list,
not archaeology.

## Sources

- `ocs2`: branch `ros2`, commit `cc73189408d4931d4d9b467a5e280a56ad7ab492`
  (2026-07-20). This branch is already `ament_cmake` and CI-tested on Jazzy
  upstream — no catkin-to-ament rewrite was needed, only local build fixes if
  any (see `JAZZY_LOCAL_PATCHES.diff` if present).
- `ocs2_robotic_assets`: default branch, commit
  `b126d00d55f1e67905c3c1516995466df6873c5c` (2022-04-07).

## Packages vendored, and why

Vendored in dependency order (each package's `<depend>` list in its
`package.xml` was read directly to build this order — not assumed):

| Package | Path | Needed because |
|---|---|---|
| `ocs2_thirdparty` | `ocs2_thirdparty/` | header-only base dep of `ocs2_core`; also bundles CppAD + CppADCodeGen (`include/cppad/`), so neither needs a separate apt package or clone |
| `ocs2_core` | `ocs2_core/` | base algorithmic package everything else depends on |
| `ocs2_oc` | `ocs2_oc/` | optimal-control base types, depended on by mpc/sqp/ddp/ipm/qp_solver |
| `ocs2_qp_solver` | `ocs2_qp_solver/` | **discovered during vendoring, not in the original plan draft**: real `<depend>` of `ocs2_sqp`, `ocs2_ddp`, `ocs2_ipm` (originally under upstream's `ocs2_test_tools/`, despite the folder name it is a build dependency, not test-only) |
| `ocs2_mpc` | `ocs2_mpc/` | `MPC_MRT_Interface`, needed by both SqpMpc and babyDog's future in-process wiring |
| `blasfeo_catkin` | `ocs2_sqp/blasfeo_catkin/` | **discovered during vendoring**: real `<depend>` of `ocs2_sqp`/`ocs2_ipm`; FetchContents BLASFEO (github.com/giaf/blasfeo @ `ae6e2d1dea015862a09990b95905038a756ffc7d`) at CMake configure time — no separate vendoring needed, just network access during build |
| `hpipm_catkin` | `ocs2_sqp/hpipm_catkin/` | **discovered during vendoring**: real `<depend>` of `ocs2_sqp`/`ocs2_ipm`; FetchContents HPIPM (github.com/giaf/hpipm @ `255ffdf38d3a5e2c3285b29568ce65ae286e5faf`) |
| `ocs2_sqp` | `ocs2_sqp/ocs2_sqp/` | the actual `SqpMpc` solver qm_control/babyDog use |
| `ocs2_ddp` | `ocs2_ddp/` | hard `<depend>` of `ocs2_legged_robot`'s `package.xml` even though never instantiated as a solver — confirmed both upstream and in qm_control (dead config there too) |
| `ocs2_ipm` | `ocs2_ipm/` | same as `ocs2_ddp` — hard transitive dep, unused solver |
| `ocs2_robotic_tools` | `ocs2_robotic_tools/` | rotation/robotic math helpers used by the WBC/centroidal layers |
| `ocs2_pinocchio_interface` | `ocs2_pinocchio/ocs2_pinocchio_interface/` | Pinocchio wrapper layer OCS2's dynamics/kinematics code is built on |
| `ocs2_centroidal_model` | `ocs2_pinocchio/ocs2_centroidal_model/` | `PinocchioCentroidalDynamicsAD`, `CentroidalModelRbdConversions` — the SRBD dynamics qm_control's `QMDynamicsAD` wraps directly |
| `ocs2_legged_robot` | `ocs2_robotic_examples/ocs2_legged_robot/` | the actual quadruped OCP definition babyDog will stand up in Milestone 2 (babyDog has no arm, so this stock package is used directly — no `qm::QMInterface`-style fork needed) |
| `ocs2_robotic_assets` | `../ocs2_robotic_assets/` (sibling of `ocs2/`) | hard `<depend>` of `ocs2_centroidal_model`'s `package.xml`; babyDog uses its own URDF from `main_bot`, this is vendored only to satisfy the colcon dependency graph |
| `ocs2_msgs` | `ocs2_msgs/` | added later (debug-visualization pass, not Milestone 1/2/3/4): hard `<depend>` of `ocs2_ros_interfaces`'s `package.xml`. Still not needed for actual control - see below |
| `ocs2_ros_interfaces` | `ocs2_ros_interfaces/` | added later (debug-visualization pass): hard `<depend>` of `ocs2_legged_robot_ros`'s `package.xml` (needed for `DummyObserver`/`VisualizationColors`). **Still not used for control** - `MPC_ROS_Interface`/`RosReferenceManager` (the ROS-topic policy transport this package also provides) remain unused; babyDog's `Ocs2WbcRuntime` still runs `MPC_MRT_Interface` fully in-process |
| `ocs2_legged_robot_ros` | `ocs2_robotic_examples/ocs2_legged_robot_ros/` | added later (debug-visualization pass): needed for `ocs2::legged_robot::LeggedRobotVisualizer`, called directly in-process from `Ocs2WbcRuntime` (constructor takes `PinocchioInterface`/`CentroidalModelInfo`/`PinocchioEndEffectorKinematics`/`rclcpp::Node::SharedPtr`, no ROS message transport involved) to publish RViz `Marker`/`MarkerArray` trajectory debugging aids. Vendored as a complete, unmodified package including its unused node executables (`legged_robot_ddp_mpc`, `legged_robot_sqp_mpc`, `legged_robot_dummy`, etc.) - only the library target (`LeggedRobotVisualizer.cpp`) is actually linked into `controller`; the executables are never invoked |

## Explicitly NOT vendored

| Package | Why not |
|---|---|
| `ocs2_perceptive`, `ocs2_python_interface`, `ocs2_frank_wolfe`, `ocs2_slp` | not transitive deps of `ocs2_legged_robot`; no perception/Python/manipulation need here |
| `ocs2_pinocchio/ocs2_self_collision*`, `ocs2_pinocchio/ocs2_sphere_approximation` | not transitive deps of `ocs2_legged_robot` (self-collision is declared-but-unused even in qm_control) |
| `ocs2_raisim*`, `ocs2_mpcnet*` | separate simulator/learning integrations, unrelated to babyDog's Gazebo sim |
| `ocs2_robotic_examples/{ocs2_ballbot,ocs2_cartpole,ocs2_double_integrator,ocs2_mobile_manipulator,ocs2_quadrotor}*` | other example robots, not needed |
| `ocs2_doc`, `ocs2/ocs2` (meta), `ocs2_ocs2` | documentation/meta packages |

If a future build step surfaces a genuine transitive need for any of these,
add it here with the reason before vendoring it — don't vendor speculatively.

## Gotchas found while bringing up Milestone 2

- **`SqpMpc`/`SqpSolver` do NOT automatically use the `ReferenceManager` built
  by `LeggedRobotInterface`.** `SqpSolver`'s constructor leaves `SolverBase`'s
  `referenceManagerPtr_` at its default no-op `ReferenceManager`. Skipping
  `mpc.getSolverPtr()->setReferenceManager(interface->getReferenceManagerPtr())`
  before the first `mpc.run()` means `SwitchedModelReferenceManager::modifyReferences()`
  (which populates `SwingTrajectoryPlanner`'s per-leg spline trajectories)
  never runs, leaving those trajectories empty — the first constraint
  evaluation then indexes an empty `std::vector<SplineCpg>` and segfaults
  inside `SplineCpg::velocity()`. This reproduces identically with vanilla,
  unmodified ANYmal config (verified with a standalone repro against
  `ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf` +
  `ocs2_legged_robot/config/mpc/task.info`) — it is a required wiring step
  for any caller of `SqpMpc`, not a babyDog-specific bug or an upstream
  defect. See `src/babydog_legged_interface/test/test_babydog_legged_interface.cpp`
  for the fix. **Milestone 4's `StateTrot::applyOcs2Wbc()` must do the same
  call** when it constructs its own `MPC_MRT_Interface`/solver.

## Gotchas found while bringing up Milestone 4

- **The very first `MPC_MRT_Interface::advanceMpc()`/`SqpMpc::run()` call must
  happen on the thread that constructed the `LeggedRobotInterface`/`SqpMpc`,
  not on a freshly spawned background thread.** `controller::hwbc::Ocs2WbcRuntime`
  (`src/controller/src/control/Ocs2WbcRuntime.cpp`) owns a background thread
  that repeatedly calls `advanceMpc()`, following qm_control's own
  `QMController` threading pattern. Spawning that thread and letting it make
  the *first* `advanceMpc()` call segfaulted deterministically inside the
  CppAD-generated dynamics library (`rk2SensitivityDiscretization` calling
  into a `dlopen`'d `dynamics_systemFlowMap_lib.so`) — reproduced with
  `sqp.nThreads` at both `1` and `3` in `task.info`, which rules out a race
  inside `SqpSolver`'s own internal `ThreadPool` (confirmed by reading
  `ThreadPool`/`SqpSolver::SqpSolver` directly: `nThreads=1` gives that pool
  zero worker threads, so the whole solve runs serially on whichever thread
  calls it — and it still crashed there). The likely cause is a `dlopen`/TLS
  interaction: the generated `.so` is loaded on the constructing thread, and
  its first-ever evaluation apparently needs to happen there too. **Fix**:
  `Ocs2WbcRuntime`'s constructor does one synchronous, blocking
  `mrt_->setCurrentObservation(...)` + `mrt_->advanceMpc()` warm-up call
  itself, *before* spawning the background MPC thread — this exactly mirrors
  qm_control's own `QMController::starting()`, which blocks on `advanceMpc()`
  from the control thread and only *then* sets `mpcRunning_ = true`, the flag
  gating whether its background thread is allowed to call `advanceMpc()` at
  all. Skipping this warm-up is a silent, deterministic crash the first time
  any caller's background MPC thread runs — not something that shows up as a
  flaky/rare race.

## Local patches

None yet. If Jazzy-specific build fixes are needed (e.g. Pinocchio discovery:
upstream's `installation.md` assumes `robotpkg-pinocchio` under
`/opt/openrobots`, this machine instead has apt's `ros-jazzy-pinocchio` under
`/opt/ros/jazzy`), they will be recorded in `JAZZY_LOCAL_PATCHES.diff`
alongside this file rather than as silent hand-edits.
