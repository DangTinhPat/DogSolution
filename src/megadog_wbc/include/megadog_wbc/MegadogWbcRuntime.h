#pragma once

// Wires megadog_legged_interface's OCS2 SqpMpc + this package's
// HierarchicalWbc into a single runtime object a ros2_control controller can
// own. Trimmed from babyDog's Ocs2WbcRuntime (src/controller/control/
// Ocs2WbcRuntime.h): no joint-order remapping - A1's URDF joint order
// (LF,LH,RF,RH x HAA,HFE,KFE, see megadog_description/urdf/robot.xacro)
// already matches megadog_legged_interface's own actuatedDofNum order, unlike
// babyDog (whose ros2_control order differs from qm_control's), so no
// Ocs2JointOrder-equivalent conversion is needed here. RViz visualization
// (visualize_enabled) IS ported, same mechanism as babyDog: publishes OCS2's
// own Marker/MarkerArray desired/optimized trajectory via
// ocs2_legged_robot_ros::LeggedRobotVisualizer, called directly in-process.

#include "megadog_wbc/BaseStateEstimator.h"
#include "megadog_wbc/HierarchicalWbc.h"

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <thread>

namespace megadog
{
namespace hwbc
{

struct MegadogWbcResult
{
    std::array<double, 12> torque_nm{};
    std::array<double, 12> position_rad{};
    std::array<double, 12> velocity_rad_s{};
    // Independently recomputed (not read back from the QP), same technique
    // test_hierarchical_wbc.cpp uses - a nonzero residual means the WBC's own
    // hard constraint slipped, distinct from "OCS2 gave us a policy at all".
    double eom_residual_norm = 0.0;
    // Base-frame lateral foot offsets in contact order [LF, RF, LH, RH],
    // plus derived stance-width metrics. These are diagnostics first: they
    // make forward-trot footprint changes measurable before any stronger
    // swing-target shaping is attempted.
    std::array<double, 4> foot_lateral_y_m{};
    double front_width_m = 0.0;
    double hind_width_m = 0.0;
    double width_mean_m = 0.0;
    double width_skew_m = 0.0;
    double left_fore_hind_skew_m = 0.0;
    double right_fore_hind_skew_m = 0.0;
    double active_diagonal_width_m = std::numeric_limits<double>::quiet_NaN();
    bool valid = false;
};

struct MegadogWbcCommand
{
    // Name loaded from megadog_legged_interface/config/gait.info
    // (stance/trot/standing_trot/...).
    std::string gait_name = "stance";
    double base_velocity_x_m_s = 0.0;
    double base_velocity_y_m_s = 0.0;
    double base_yaw_rate_rad_s = 0.0;
    double base_x_reference_m = std::numeric_limits<double>::quiet_NaN();
    double base_y_reference_m = std::numeric_limits<double>::quiet_NaN();
    double base_yaw_reference_rad = std::numeric_limits<double>::quiet_NaN();
    double com_height_m = 0.3;
    bool use_mpc_for_stance_hold = false;
};

// Measured feedback this runtime needs each control tick. base_euler_zyx_rad/
// base_euler_zyx_rate_rad_s/base_linear_accel_local_m_s2 come from /imu/data
// (same in sim and on real hardware, see MegadogController's
// state-estimation comment). base_pos_m/base_linear_vel_m_s are the
// CALLER's best guess (MegadogController currently pins them at a fixed
// local origin with zero velocity) - MegadogWbcRuntime::update() does not
// use them directly; it overwrites both with BaseStateEstimator's own
// leg-odometry estimate (IMU + joint encoders + gait-schedule contact,
// see BaseStateEstimator.h) before building the RBD state the base_height/
// base_linear Cartesian tasks in WbcBase.cpp see. They stay in this struct
// so a caller with a real absolute-position source (none exists yet) has
// somewhere to put one.
struct MegadogWbcMeasurement
{
    std::array<double, 12> joint_pos_rad{};
    std::array<double, 12> joint_vel_rad_s{};
    std::array<double, 3> base_pos_m{};
    std::array<double, 3> base_euler_zyx_rad{};
    std::array<double, 3> base_linear_vel_m_s{};
    std::array<double, 3> base_euler_zyx_rate_rad_s{};
    std::array<double, 3> base_linear_accel_local_m_s2{};
    // True when base_pos_m/base_linear_vel_m_s above already come from a
    // trustworthy absolute source (sim ground truth) rather than a
    // placeholder - see MegadogController's base_fresh/imu_fresh branches.
    // When true, MegadogWbcRuntime::update() uses base_pos_m/
    // base_linear_vel_m_s exactly as given, bypassing BaseStateEstimator
    // entirely (matching ultraDog's ground-truth-only pipeline, proven
    // stable). When false (no ground truth - real hardware, or sim without
    // /sim/model_poses bridged), BaseStateEstimator's own leg-odometry
    // estimate is used instead, and this struct's base_pos_m/
    // base_linear_vel_m_s are ignored (only base_euler_zyx_rad/
    // base_euler_zyx_rate_rad_s/base_linear_accel_local_m_s2 - IMU-derived -
    // matter in that case).
    bool base_pose_is_ground_truth = false;
};

class MegadogWbcRuntime
{
public:
    // visualize_enabled publishes OCS2's own RViz Marker/MarkerArray desired/
    // optimized trajectory visualization via
    // ocs2_legged_robot_ros::LeggedRobotVisualizer (see publishVisualization()
    // below) - sim-only debug tooling, same as babyDog's Ocs2WbcRuntime.
    explicit MegadogWbcRuntime(HierarchicalWbcConfig wbc_config = {}, bool visualize_enabled = false);
    ~MegadogWbcRuntime();

    MegadogWbcRuntime(const MegadogWbcRuntime&) = delete;
    MegadogWbcRuntime& operator=(const MegadogWbcRuntime&) = delete;

    // True once construction (interface/MPC/WBC/thread) succeeded. If false,
    // update() always returns false without touching result.
    bool ready() const { return ready_; }

    // Call once per control tick. Never blocks: if the background MPC thread
    // has not produced a policy yet, this returns false immediately.
    bool update(double time_s, double dt_s, const rclcpp::Time& ros_time, const MegadogWbcMeasurement& measurement,
                const MegadogWbcCommand& command, MegadogWbcResult& result);

    // Call when the caller (MegadogController) is about to start a fresh
    // locomotion segment (e.g. re-entering TROT_IN_PLACE after a STAND
    // hold) - see MegadogWbcRuntime.cpp's implementation and update()/
    // mpcThreadFunction() for the full story of the "frozen gait on
    // re-entry" bug this exists to fix (found via stress-testing, took two
    // failed fix attempts before the actual root cause - a stale
    // GaitSchedule validity window, not just the MPC solver's warm-start -
    // was found by reading setGaitTemplateIfNeeded() rather than guessing
    // again). Does two things, both safe to call from the caller's own
    // (control) thread:
    //  1. Requests an MPC solver reset - lightweight, thread-safe: only
    //     sets an atomic flag that the MPC thread itself picks up and acts
    //     on at the start of its own next loop iteration (see
    //     mpcThreadFunction()) - the actual solver reset (mpc_->reset(),
    //     NOT thread-safe on its own) never runs concurrently with
    //     advanceMpc(). update() then holds (returns false, caller falls
    //     back to its own last-valid-effort) until a policy computed via a
    //     solve that ran AFTER this call is confirmed via generation
    //     counter (not a raw timestamp comparison - see
    //     mpc_advance_count_'s doc comment for why that didn't work).
    //  2. Clears active_gait_name_/resets gait_schedule_valid_until_s_ -
    //     both control-thread-only state, no synchronization needed -
    //     forcing the next setGaitTemplateIfNeeded() call to fully retile
    //     the gait schedule from the new segment's time_s even if the
    //     requested gait name happens to be unchanged from before the
    //     hold, instead of short-circuiting on a validity window left over
    //     from the old, now-irrelevant epoch.
    void beginNewLocomotionSegment();

private:
    void mpcThreadFunction();
    bool setGaitTemplateIfNeeded(const std::string& gait_name, double time_s);
    void setTargetTrajectories(double time_s, const ocs2::vector_t& observation_state,
                               const MegadogWbcCommand& command);
    // Broadcasts base -> ocs2_world (inverse of the robot's current measured
    // world pose - megadog_description's A1 URDF has no persistent odom/world
    // TF frame either, see MegadogWbcRuntime.cpp) then publishes the
    // visualizer's desired/optimized trajectory markers. No-op if
    // visualize_enabled was false at construction.
    void publishVisualization(double time_s, const rclcpp::Time& ros_time, const MegadogWbcMeasurement& measurement);
    // Support polygon / center-of-pressure / per-foot force+contact markers -
    // called directly with the free functions from VisualizationHelpers.h,
    // same as babyDog's Ocs2WbcRuntime.
    void publishContactMarkers(double time_s, const rclcpp::Time& ros_time, const ocs2::legged_robot::contact_flag_t& contactFlags,
                               const std::vector<Eigen::Vector3d>& feetPositions,
                               const std::vector<Eigen::Vector3d>& feetForces);

    HierarchicalWbcConfig wbc_config_;

    std::unique_ptr<ocs2::legged_robot::LeggedRobotInterface> interface_;
    std::unique_ptr<ocs2::SqpMpc> mpc_;
    std::unique_ptr<ocs2::MPC_MRT_Interface> mrt_;
    std::unique_ptr<ocs2::CentroidalModelRbdConversions> rbd_conversions_;
    std::unique_ptr<HierarchicalWbc> wbc_;
    std::unique_ptr<BaseStateEstimator> base_state_estimator_;

    std::thread mpc_thread_;
    std::atomic<bool> thread_running_{false};
    std::atomic<bool> mpc_worker_enabled_{false};
    // See requestMpcReset()'s doc comment.
    //
    // A first version of this "is the policy fresh after a reset" check
    // compared policy.timeTrajectory_.front() against the reset's target
    // time_s directly and was WRONG - verified by isolated sim to still
    // reproduce the frozen-gait bug 3/3 times. Reason: the STALE policy's
    // own front() is a large, perfectly ordinary-looking value from its own
    // (now-irrelevant) prior locomotion segment's local clock (e.g. 163.2s,
    // if that segment ran that long before STAND suspended the MPC worker)
    // - segment-local time values aren't comparable across a reset the way
    // a raw "old vs new" comparison assumes, so "front() >= reset_time(0.0)"
    // was true for the STALE policy too, defeating the check on its very
    // first evaluation.
    //
    // Fixed with a generation counter instead of a time-value comparison:
    // mpc_advance_count_ increments (MPC thread only, after each completed
    // advanceMpc()) every solve cycle. requestMpcReset() (control thread)
    // records the count AT THE MOMENT of the request. update() then holds
    // until the count has advanced by at least kMpcResetSettleAdvances
    // cycles past that recorded value - i.e. until a solve has actually run
    // AFTER the reset was issued (not merely until some timestamp looks
    // numerically plausible) - with a small margin so the buffered result
    // has had a control tick to be swapped in via updatePolicy() before
    // being trusted.
    std::atomic<bool> mpc_reset_requested_{false};
    std::atomic<uint64_t> mpc_advance_count_{0};
    bool mpc_awaiting_fresh_policy_ = false;
    uint64_t mpc_reset_request_advance_count_ = 0;
    static constexpr uint64_t kMpcResetSettleAdvances = 2;
    bool ready_ = false;
    double mpc_desired_frequency_hz_ = 100.0;
    std::string active_gait_name_ = "stance";
    // How far (in MPC/observation time) the currently active gait's mode
    // schedule has been extended - see setGaitTemplateIfNeeded()'s doc
    // comment for why this needs periodic refreshing, not just refreshing
    // on a gait *name* change.
    double gait_schedule_valid_until_s_ = -std::numeric_limits<double>::infinity();

    bool visualize_enabled_ = false;
    rclcpp::Node::SharedPtr visualizer_node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> visualizer_tf_broadcaster_;
    std::unique_ptr<ocs2::legged_robot::LeggedRobotVisualizer> visualizer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr contact_markers_publisher_;
    // publishDesiredTrajectory()/publishOptimizedStateTrajectory() are called
    // directly (bypassing the visualizer's own update()-internal throttle) -
    // this replicates that same throttling (MPC time, not wall-clock).
    double last_visualization_time_s_ = std::numeric_limits<double>::lowest();
    double visualization_min_period_s_ = 0.05;  // 20 Hz
};

}  // namespace hwbc
}  // namespace megadog
