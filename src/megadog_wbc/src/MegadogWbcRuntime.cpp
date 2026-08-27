#include "megadog_wbc/MegadogWbcRuntime.h"

#include "megadog_legged_interface/MegadogA1Interface.h"

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/common/utils.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace megadog
{
namespace hwbc
{
using namespace ocs2;
using namespace ocs2::legged_robot;

namespace
{
scalar_t wrapAngleNear(const scalar_t reference, const scalar_t value)
{
    return reference + std::atan2(std::sin(value - reference), std::cos(value - reference));
}

// Same layout WbcBase::updateMeasured()/test_hierarchical_wbc.cpp use:
// [orientation(zyx), position, joints] then the same for velocities.
vector_t buildRbdState(const MegadogWbcMeasurement& measurement, size_t generalizedCoordinatesNum, size_t actuatedDofNum)
{
    vector_t rbdState = vector_t::Zero(2 * generalizedCoordinatesNum);
    vector3_t baseEulerZyx;
    baseEulerZyx << measurement.base_euler_zyx_rad[0], measurement.base_euler_zyx_rad[1], measurement.base_euler_zyx_rad[2];
    vector3_t baseEulerZyxRate;
    baseEulerZyxRate << measurement.base_euler_zyx_rate_rad_s[0], measurement.base_euler_zyx_rate_rad_s[1],
        measurement.base_euler_zyx_rate_rad_s[2];
    const vector3_t baseAngularVelocityWorld =
        getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(baseEulerZyx, baseEulerZyxRate);

    rbdState(0) = measurement.base_euler_zyx_rad[0];
    rbdState(1) = measurement.base_euler_zyx_rad[1];
    rbdState(2) = measurement.base_euler_zyx_rad[2];
    rbdState(3) = measurement.base_pos_m[0];
    rbdState(4) = measurement.base_pos_m[1];
    rbdState(5) = measurement.base_pos_m[2];
    for (size_t i = 0; i < actuatedDofNum; ++i) {
        rbdState(6 + i) = measurement.joint_pos_rad[i];
    }
    const size_t velocityOffset = generalizedCoordinatesNum;
    rbdState(velocityOffset + 0) = baseAngularVelocityWorld(0);
    rbdState(velocityOffset + 1) = baseAngularVelocityWorld(1);
    rbdState(velocityOffset + 2) = baseAngularVelocityWorld(2);
    rbdState(velocityOffset + 3) = measurement.base_linear_vel_m_s[0];
    rbdState(velocityOffset + 4) = measurement.base_linear_vel_m_s[1];
    rbdState(velocityOffset + 5) = measurement.base_linear_vel_m_s[2];
    for (size_t i = 0; i < actuatedDofNum; ++i) {
        rbdState(velocityOffset + 6 + i) = measurement.joint_vel_rad_s[i];
    }
    return rbdState;
}

bool isFiniteVector(const vector_t& value)
{
    return value.allFinite();
}

size_t safeModeAtTimeOrStance(const ModeSchedule& schedule, const scalar_t time)
{
    const size_t stanceMode = static_cast<size_t>(ModeNumber::STANCE);
    if (!std::isfinite(time)) {
        std::cerr << "[MegadogWbcRuntime] non-finite mode query time, falling back to STANCE" << std::endl;
        return stanceMode;
    }
    if (schedule.modeSequence.empty()) {
        std::cerr << "[MegadogWbcRuntime] empty mode schedule, falling back to STANCE" << std::endl;
        return stanceMode;
    }
    if (schedule.modeSequence.size() != schedule.eventTimes.size() + 1) {
        std::cerr << "[MegadogWbcRuntime] inconsistent mode schedule: eventTimes="
                  << schedule.eventTimes.size() << " modeSequence=" << schedule.modeSequence.size()
                  << ", falling back to STANCE" << std::endl;
        return stanceMode;
    }
    try {
        return schedule.modeAtTime(time);
    } catch (const std::exception& e) {
        std::cerr << "[MegadogWbcRuntime] modeAtTime(" << time << ") failed: " << e.what()
                  << ", falling back to STANCE" << std::endl;
        return stanceMode;
    }
}
}  // namespace

MegadogWbcRuntime::MegadogWbcRuntime(HierarchicalWbcConfig wbc_config, bool visualize_enabled)
    : wbc_config_(wbc_config), visualize_enabled_(visualize_enabled)
{
    try {
        interface_ = megadog_legged_interface::createInterface();

        mpc_ = std::make_unique<SqpMpc>(interface_->mpcSettings(), interface_->sqpSettings(),
                                        interface_->getOptimalControlProblem(), interface_->getInitializer());
        // Required wiring - see babyDog's src/ocs2/VENDORING.md "Gotchas found
        // while bringing up Milestone 2" for the segfault this avoids
        // (SwingTrajectoryPlanner never populated without it).
        mpc_->getSolverPtr()->setReferenceManager(interface_->getReferenceManagerPtr());

        mrt_ = std::make_unique<MPC_MRT_Interface>(*mpc_);
        mrt_->initRollout(&interface_->getRollout());

        rbd_conversions_ =
            std::make_unique<CentroidalModelRbdConversions>(interface_->getPinocchioInterface(), interface_->getCentroidalModelInfo());

        CentroidalModelPinocchioMapping mapping(interface_->getCentroidalModelInfo());
        PinocchioEndEffectorKinematics eeKinematics(interface_->getPinocchioInterface(), mapping,
                                                    interface_->modelSettings().contactNames3DoF);
        wbc_ = std::make_unique<HierarchicalWbc>(interface_->getPinocchioInterface(), interface_->getCentroidalModelInfo(), eeKinematics,
                                                 wbc_config_);

        if (visualize_enabled_) {
            // A standalone rclcpp::Node, not the ros2_control controller's own
            // rclcpp_lifecycle::LifecycleNode (an incompatible type
            // LeggedRobotVisualizer's constructor cannot accept) - same as
            // babyDog's Ocs2WbcRuntime. Namespaced so the visualizer's own
            // (never-published-to, see publishVisualization()) "joint_states"
            // publisher resolves away from the real robot's; the
            // /legged_robot/... marker topics are absolute in the vendored
            // source, so this namespace has no effect on them.
            visualizer_node_ = rclcpp::Node::make_shared("ocs2_wbc_visualizer", "ocs2_debug");
            visualizer_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(visualizer_node_);
            visualizer_ = std::make_unique<ocs2::legged_robot::LeggedRobotVisualizer>(
                interface_->getPinocchioInterface(), interface_->getCentroidalModelInfo(), eeKinematics,
                visualizer_node_, /*maxUpdateFrequency=*/20.0);
            // megaDog has no persistent odom/world TF frame (Gazebo's ground-
            // truth model pose feeds MegadogWbcMeasurement directly);
            // publishVisualization() bridges this frame to "base" each tick.
            visualizer_->frameId_ = "ocs2_world";
            // Deliberately NOT "/legged_robot/currentState" -
            // ocs2_legged_robot_ros::LeggedRobotVisualizer's own constructor
            // already advertises a (permanently unused - see
            // publishContactMarkers()) currentStatePublisher_ on that exact
            // name; a second publisher on the same topic from this same
            // process/node showed up as "Publisher count: 2" in `ros2 topic
            // info`, and RViz's MarkerArray display for it (and, seemingly
            // via some related QoS-matching confusion, the unrelated
            // optimizedStateTrajectory display too) showed a persistent
            // error status - see megadog_description's rviz config, which
            // now points at this own topic name instead.
            contact_markers_publisher_ =
                visualizer_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                    "/legged_robot/contactState", 1);
        }

        const vector_t initState = interface_->getInitialState();
        const contact_flag_t allStance{true, true, true, true};
        const vector_t desiredInput = weightCompensatingInput(interface_->getCentroidalModelInfo(), allStance);
        const scalar_t horizon = interface_->mpcSettings().timeHorizon_;
        const TargetTrajectories targetTrajectories({0.0, horizon}, {initState, initState}, {desiredInput, desiredInput});
        interface_->getReferenceManagerPtr()->setTargetTrajectories(targetTrajectories);

        // Warm-up solve on THIS (constructing) thread before mpc_thread_ ever
        // touches the solver - see babyDog's Ocs2WbcRuntime.cpp for why
        // (CppAD-generated dynamics libraries appear to require their first
        // evaluation on the thread that loaded them).
        SystemObservation warmupObservation;
        warmupObservation.time = 0.0;
        warmupObservation.state = initState;
        warmupObservation.input = desiredInput;
        warmupObservation.mode = ModeNumber::STANCE;
        mrt_->setCurrentObservation(warmupObservation);
        mrt_->advanceMpc();

        const double configuredMpcFrequencyHz =
            interface_->mpcSettings().mpcDesiredFrequency_ > 0.0 ? interface_->mpcSettings().mpcDesiredFrequency_ : 100.0;
        mpc_desired_frequency_hz_ = configuredMpcFrequencyHz;
    } catch (const std::exception& e) {
        std::cerr << "[MegadogWbcRuntime] initialization failed: " << e.what() << std::endl;
        return;
    }

    ready_ = true;
    thread_running_ = true;
    mpc_thread_ = std::thread(&MegadogWbcRuntime::mpcThreadFunction, this);
}

MegadogWbcRuntime::~MegadogWbcRuntime()
{
    thread_running_ = false;
    if (mpc_thread_.joinable()) {
        mpc_thread_.join();
    }
}

void MegadogWbcRuntime::mpcThreadFunction()
{
    const auto period = std::chrono::duration<double>(1.0 / mpc_desired_frequency_hz_);
    while (thread_running_) {
        const auto tickStart = std::chrono::steady_clock::now();
        if (mpc_worker_enabled_.load(std::memory_order_relaxed)) {
            try {
                mrt_->advanceMpc();
            } catch (const std::exception& e) {
                std::cerr << "[MegadogWbcRuntime] MPC thread error, stopping: " << e.what() << std::endl;
                thread_running_ = false;
                break;
            }
        }
        const auto elapsed = std::chrono::steady_clock::now() - tickStart;
        const auto sleepDuration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period) - elapsed;
        if (sleepDuration > std::chrono::steady_clock::duration::zero()) {
            std::this_thread::sleep_for(sleepDuration);
        }
    }
}

bool MegadogWbcRuntime::setGaitTemplateIfNeeded(const std::string& gait_name, const double time_s)
{
    const std::string requested = gait_name.empty() ? std::string("stance") : gait_name;
    const scalar_t horizon = interface_->mpcSettings().timeHorizon_;
    // Re-inserting only on a gait *name* change (the old condition here) left
    // the gait schedule's mode sequence valid only up to the insertion
    // time + 4*horizon (~4s at this task.info's timeHorizon=1.0s) - a gait
    // held longer than that without ever changing name (e.g. FORWARD left
    // running) ran the switched-model reference manager's GaitSchedule out
    // of defined modes, and ocs2::ModeSchedule::modeAtTime() querying past
    // the end segfaulted instead of erroring. Refresh whenever time_s gets
    // within one horizon of running out, regardless of whether the name
    // changed, so a long-held gait's window keeps sliding forward.
    if (requested == active_gait_name_ && time_s + horizon < gait_schedule_valid_until_s_) {
        return true;
    }
    try {
        const ModeSequenceTemplate gait_template =
            loadModeSequenceTemplate(megadog_legged_interface::getConfigPath("gait.info"), requested, false);
        const scalar_t validUntil = time_s + 4.0 * horizon;
        interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule()->insertModeSequenceTemplate(
            gait_template, time_s, validUntil);
        active_gait_name_ = requested;
        gait_schedule_valid_until_s_ = validUntil;
    } catch (const std::exception& e) {
        std::cerr << "[MegadogWbcRuntime] failed to set gait '" << requested << "': " << e.what() << std::endl;
        return false;
    }
    return true;
}

void MegadogWbcRuntime::setTargetTrajectories(const double time_s, const vector_t& observation_state,
                                              const MegadogWbcCommand& command)
{
    const auto& info = interface_->getCentroidalModelInfo();
    const vector_t defaultJointState = interface_->getInitialState().tail(info.actuatedDofNum);
    const scalar_t timeToTarget = std::max<scalar_t>(0.5, interface_->mpcSettings().timeHorizon_);

    vector_t baseCurrentPose = observation_state.segment(6, 6);
    if (std::isfinite(command.com_height_m) && command.com_height_m > 0.02) {
        baseCurrentPose(2) = command.com_height_m;
    }
    baseCurrentPose(4) = 0.0;
    baseCurrentPose(5) = 0.0;

    const vector3_t zyx = baseCurrentPose.tail<3>();
    vector3_t commandVelocityBody;
    commandVelocityBody << command.base_velocity_x_m_s, command.base_velocity_y_m_s, 0.0;
    const vector3_t commandVelocityWorld = getRotationMatrixFromZyxEulerAngles(zyx) * commandVelocityBody;

    vector_t baseTargetPose(6);
    baseTargetPose = baseCurrentPose;
    baseTargetPose(0) = std::isfinite(command.base_x_reference_m)
        ? command.base_x_reference_m + commandVelocityWorld(0) * timeToTarget
        : baseCurrentPose(0) + commandVelocityWorld(0) * timeToTarget;
    baseTargetPose(1) = std::isfinite(command.base_y_reference_m)
        ? command.base_y_reference_m + commandVelocityWorld(1) * timeToTarget
        : baseCurrentPose(1) + commandVelocityWorld(1) * timeToTarget;
    baseTargetPose(2) =
        std::isfinite(command.com_height_m) && command.com_height_m > 0.02 ? command.com_height_m : baseCurrentPose(2);
    const scalar_t yawReference = std::isfinite(command.base_yaw_reference_rad)
        ? wrapAngleNear(baseCurrentPose(3), command.base_yaw_reference_rad)
        : baseCurrentPose(3);
    baseTargetPose(3) = wrapAngleNear(baseCurrentPose(3), yawReference + command.base_yaw_rate_rad_s * timeToTarget);
    baseTargetPose(4) = 0.0;
    baseTargetPose(5) = 0.0;

    vector_t startState = vector_t::Zero(info.stateDim);
    vector_t finalState = vector_t::Zero(info.stateDim);
    startState.segment(6, 6) = baseCurrentPose;
    finalState.segment(6, 6) = baseTargetPose;
    startState.tail(info.actuatedDofNum) = centroidal_model::getJointAngles(observation_state, info);
    finalState.tail(info.actuatedDofNum) = defaultJointState;
    startState.head<3>() = commandVelocityWorld;
    finalState.head<3>() = commandVelocityWorld;

    const vector_t desiredInput = vector_t::Zero(info.inputDim);
    const TargetTrajectories targetTrajectories({time_s, time_s + timeToTarget}, {startState, finalState},
                                                {desiredInput, desiredInput});
    interface_->getReferenceManagerPtr()->setTargetTrajectories(targetTrajectories);
}

bool MegadogWbcRuntime::update(const double time_s, const double dt_s, const rclcpp::Time& ros_time,
                               const MegadogWbcMeasurement& measurement,
                               const MegadogWbcCommand& command, MegadogWbcResult& result)
{
    if (!ready_ || !thread_running_ || !std::isfinite(time_s) || !std::isfinite(dt_s) || dt_s <= 0.0) {
        return false;
    }

    const auto& info = interface_->getCentroidalModelInfo();
    const vector_t rbdState = buildRbdState(measurement, info.generalizedCoordinatesNum, info.actuatedDofNum);
    if (!isFiniteVector(rbdState)) {
        std::cerr << "[MegadogWbcRuntime] measured RBD state is not finite" << std::endl;
        return false;
    }

    SystemObservation observation;
    observation.time = time_s;
    try {
        observation.state = rbd_conversions_->computeCentroidalStateFromRbdModel(rbdState);
    } catch (const std::exception& e) {
        std::cerr << "[MegadogWbcRuntime] computeCentroidalStateFromRbdModel failed: " << e.what() << std::endl;
        return false;
    }
    observation.input = vector_t::Zero(info.inputDim);

    vector_t optimizedState;
    vector_t optimizedInput;
    size_t plannedMode = 0;
    const bool stanceHoldCommand =
        command.gait_name == "stance" && std::abs(command.base_velocity_x_m_s) < 1e-6 &&
        std::abs(command.base_velocity_y_m_s) < 1e-6 && std::abs(command.base_yaw_rate_rad_s) < 1e-6;
    if (stanceHoldCommand) {
        mpc_worker_enabled_.store(false, std::memory_order_relaxed);

        optimizedState = vector_t::Zero(info.stateDim);
        optimizedState.segment(6, 6) = observation.state.segment(6, 6);
        if (std::isfinite(command.base_x_reference_m)) {
            optimizedState(6) = command.base_x_reference_m;
        }
        if (std::isfinite(command.base_y_reference_m)) {
            optimizedState(7) = command.base_y_reference_m;
        }
        if (std::isfinite(command.com_height_m) && command.com_height_m > 0.02) {
            optimizedState(8) = command.com_height_m;
        }
        if (std::isfinite(command.base_yaw_reference_rad)) {
            optimizedState(9) = wrapAngleNear(observation.state(9), command.base_yaw_reference_rad);
        }
        optimizedState(10) = 0.0;
        optimizedState(11) = 0.0;
        optimizedState.tail(info.actuatedDofNum) = interface_->getInitialState().tail(info.actuatedDofNum);

        const contact_flag_t allStance{true, true, true, true};
        optimizedInput = weightCompensatingInput(info, allStance);
        plannedMode = ModeNumber::STANCE;
    } else {
        mpc_worker_enabled_.store(true, std::memory_order_relaxed);
        if (!setGaitTemplateIfNeeded(command.gait_name, time_s)) {
            return false;
        }
        setTargetTrajectories(time_s, observation.state, command);
        const auto& activeModeSchedule = interface_->getReferenceManagerPtr()->getModeSchedule();
        observation.mode = safeModeAtTimeOrStance(activeModeSchedule, time_s);

        try {
            mrt_->setCurrentObservation(observation);
            mrt_->updatePolicy();
        } catch (const std::exception& e) {
            std::cerr << "[MegadogWbcRuntime] MPC policy update failed: " << e.what() << std::endl;
            return false;
        }
        if (!mrt_->initialPolicyReceived()) {
            return false;
        }

        try {
            mrt_->evaluatePolicy(observation.time, observation.state, optimizedState, optimizedInput, plannedMode);
        } catch (const std::exception& e) {
            std::cerr << "[MegadogWbcRuntime] MPC policy evaluation failed: " << e.what() << std::endl;
            return false;
        }
    }
    if (!isFiniteVector(optimizedState) || !isFiniteVector(optimizedInput)) {
        std::cerr << "[MegadogWbcRuntime] MPC policy produced a non-finite state/input" << std::endl;
        return false;
    }

    vector_t cmd;
    try {
        cmd = wbc_->update(optimizedState, optimizedInput, rbdState, plannedMode, dt_s, time_s);
    } catch (const std::exception& e) {
        std::cerr << "[MegadogWbcRuntime] HierarchicalWbc::update failed: " << e.what() << std::endl;
        return false;
    }

    const size_t nq = info.generalizedCoordinatesNum;
    const size_t numDecisionVars = nq + 3 * info.numThreeDofContacts;
    if (static_cast<size_t>(cmd.size()) != numDecisionVars + info.actuatedDofNum) {
        return false;
    }
    for (long i = 0; i < cmd.size(); ++i) {
        if (!std::isfinite(cmd(i))) {
            return false;
        }
    }

    const vector_t vDotAll = cmd.head(nq);
    const vector_t forces = cmd.segment(nq, 3 * info.numThreeDofContacts);
    const vector_t torque = cmd.tail(info.actuatedDofNum);
    const vector_t jointAngles = centroidal_model::getJointAngles(optimizedState, info);
    const vector_t jointVelocities = centroidal_model::getJointVelocities(optimizedInput, info);

    // Independently recompute the floating-base EOM residual - same
    // technique test_hierarchical_wbc.cpp uses.
    auto pinocchioInterface = interface_->getPinocchioInterface();
    const auto& model = pinocchioInterface.getModel();
    auto& data = pinocchioInterface.getData();
    vector_t qMeasured = vector_t::Zero(nq);
    qMeasured.head<3>() = rbdState.segment<3>(3);
    qMeasured.segment<3>(3) = rbdState.segment<3>(0);
    qMeasured.tail(info.actuatedDofNum) = rbdState.segment(6, info.actuatedDofNum);
    vector_t vMeasured = vector_t::Zero(nq);
    vMeasured.head<3>() = rbdState.segment<3>(nq + 3);
    vMeasured.segment<3>(3) =
        getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(qMeasured.segment<3>(3), rbdState.segment<3>(nq));
    vMeasured.tail(info.actuatedDofNum) = rbdState.segment(nq + 6, info.actuatedDofNum);

    pinocchio::forwardKinematics(model, data, qMeasured, vMeasured);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, qMeasured);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model, data, qMeasured, vMeasured);

    matrix_t jStacked(3 * info.numThreeDofContacts, nq);
    for (size_t i = 0; i < info.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, nq);
        pinocchio::getFrameJacobian(model, data, info.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        jStacked.block(3 * i, 0, 3, nq) = jac.topRows<3>();
    }
    const matrix_t Mb = data.M.topRows(6);
    const vector_t hb = data.nle.topRows(6);
    const matrix_t JbT = jStacked.transpose().topRows(6);
    const vector_t residual = Mb * vDotAll - JbT * forces + hb;

    for (size_t i = 0; i < info.actuatedDofNum && i < 12; ++i) {
        result.torque_nm[i] = torque(static_cast<long>(i));
        result.position_rad[i] = jointAngles(static_cast<long>(i));
        result.velocity_rad_s[i] = jointVelocities(static_cast<long>(i));
    }
    result.eom_residual_norm = residual.norm();
    result.valid = true;

    if (visualize_enabled_) {
        publishVisualization(time_s, ros_time, measurement);

        const contact_flag_t contactFlags = modeNumber2StanceLeg(plannedMode);
        std::vector<vector3_t> feetPositions(info.numThreeDofContacts);
        std::vector<vector3_t> feetForces(info.numThreeDofContacts);
        for (size_t i = 0; i < info.numThreeDofContacts; ++i) {
            feetPositions[i] = data.oMf[info.endEffectorFrameIndices[i]].translation();
            feetForces[i] = forces.segment<3>(3 * i);
        }
        publishContactMarkers(time_s, ros_time, contactFlags, feetPositions, feetForces);
    }
    return true;
}

void MegadogWbcRuntime::publishVisualization(const double time_s, const rclcpp::Time& ros_time,
                                             const MegadogWbcMeasurement& measurement)
{
    if (!visualizer_ || !visualizer_node_ || !visualizer_tf_broadcaster_) {
        return;
    }
    if (time_s - last_visualization_time_s_ < visualization_min_period_s_) {
        return;
    }
    last_visualization_time_s_ = time_s;

    // Broadcast base -> ocs2_world as the inverse of the robot's current
    // measured world pose, so RViz (whose Fixed Frame is "base" - megaDog has
    // no persistent odom/world TF frame) renders the visualizer's world-frame
    // trajectory markers positioned correctly relative to the robot body.
    const vector3_t eulerZyx(measurement.base_euler_zyx_rad[0], measurement.base_euler_zyx_rad[1],
                             measurement.base_euler_zyx_rad[2]);
    const Eigen::Quaterniond q_world_base = getQuaternionFromEulerAnglesZyx<scalar_t>(eulerZyx);
    const Eigen::Quaterniond q_base_world = q_world_base.conjugate();
    const vector3_t p_world_base(measurement.base_pos_m[0], measurement.base_pos_m[1], measurement.base_pos_m[2]);
    const vector3_t p_base_world = q_base_world * (-p_world_base);

    geometry_msgs::msg::TransformStamped tf_msg;
    const auto timeStamp = ros_time;
    tf_msg.header.stamp = timeStamp;
    tf_msg.header.frame_id = "base";
    tf_msg.child_frame_id = "ocs2_world";
    tf_msg.transform.translation.x = p_base_world.x();
    tf_msg.transform.translation.y = p_base_world.y();
    tf_msg.transform.translation.z = p_base_world.z();
    tf_msg.transform.rotation.x = q_base_world.x();
    tf_msg.transform.rotation.y = q_base_world.y();
    tf_msg.transform.rotation.z = q_base_world.z();
    tf_msg.transform.rotation.w = q_base_world.w();
    visualizer_tf_broadcaster_->sendTransform(tf_msg);

    // Pure Marker/MarkerArray publishers only - deliberately not calling
    // visualizer_->update()/publishObservation(), which would also broadcast
    // a competing "base" TF and a "joint_states" topic with hardcoded ANYmal
    // joint names, colliding with the real robot_state_publisher's own.
    try {
        visualizer_->publishDesiredTrajectory(timeStamp, mrt_->getCommand().mpcTargetTrajectories_);
        const auto& primalSolution = mrt_->getPolicy();
        visualizer_->publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_,
                                                     primalSolution.stateTrajectory_, primalSolution.modeSchedule_);
    } catch (const std::exception&) {
        // Stance hold can run without an active MRT policy. Visualization is
        // debug-only, so it must never make the realtime controller fail.
    }
}

void MegadogWbcRuntime::publishContactMarkers(const double /*time_s*/, const rclcpp::Time& ros_time,
                                              const contact_flag_t& contactFlags,
                                              const std::vector<vector3_t>& feetPositions,
                                              const std::vector<vector3_t>& feetForces)
{
    if (!contact_markers_publisher_ || !visualizer_node_) {
        return;
    }

    visualization_msgs::msg::MarkerArray markerArray;
    markerArray.markers.reserve(feetPositions.size() * 2 + 2);
    for (size_t i = 0; i < feetPositions.size(); ++i) {
        markerArray.markers.emplace_back(
            getFootMarker(feetPositions[i], contactFlags[i], visualizer_->feetColorMap_[i],
                         visualizer_->footMarkerDiameter_, visualizer_->footAlphaWhenLifted_));
        markerArray.markers.emplace_back(
            getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green,
                          visualizer_->forceScale_));
    }
    markerArray.markers.emplace_back(getCenterOfPressureMarker(
        feetForces.begin(), feetForces.end(), feetPositions.begin(), contactFlags.begin(), Color::green,
        visualizer_->copMarkerDiameter_));
    markerArray.markers.emplace_back(getSupportPolygonMarker(
        feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black,
        visualizer_->supportPolygonLineWidth_));

    const auto timeStamp = ros_time;
    assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
                getHeaderMsg(visualizer_->frameId_, timeStamp));
    assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
    contact_markers_publisher_->publish(markerArray);
}

}  // namespace hwbc
}  // namespace megadog
