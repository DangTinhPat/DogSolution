#pragma once

// Single ros2_control ControllerInterface wiring megadog_wbc's
// MegadogWbcRuntime (OCS2 SqpMpc + HierarchicalWbc) to A1's "effort" command
// interface, modeled on qiayuanl/legged_control's own LeggedController: one
// controller owns the whole MPC+WBC pipeline. Only "effort" is claimed as a
// command interface (see megadog_description/urdf/common/gazebo.xacro) -
// there is no separate low-level PD controller layer, so the joint-level PD
// (kp=0, kd=3, matching legged_control's own LeggedController.cpp
// hybridJointHandles_[j].setCommand(pos,vel,kp=0,kd=3,ff) call) is computed
// directly here: effort = torque_ff + kd * (velocity_desired - velocity_measured).
//
// State estimation: sim uses Gazebo's own ground-truth model pose for base
// position/orientation/velocity (/sim/model_poses) - matching ultraDog's
// real-A1-scale sibling exactly, which is proven stable over long trot runs
// this way. Real hardware has no such ground truth, so it falls back to
// megadog_wbc's BaseStateEstimator (IMU + joint encoders + gait-schedule
// contact leg odometry) - see MegadogWbcMeasurement::base_pose_is_ground_truth
// in MegadogWbcRuntime.h for how MegadogWbcRuntime picks between the two.
// (An earlier revision of this file replaced ground truth with the estimator
// unconditionally, in sim too - a rigorous multi-agent investigation traced
// megaDog's ~15-25s trot instability directly to that: dead-reckoning drift
// with no absolute-position correction, that ultraDog's ground-truth path
// never has to contend with. Restoring ground truth for sim, and reserving
// the estimator for when it's genuinely needed (no ground truth available),
// fixed it.)

#include <controller_interface/controller_interface.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "megadog_wbc/MegadogWbcRuntime.h"

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace megadog_controller
{

// Commands accepted on the "/megadog/cmd" (std_msgs/String) topic - see
// MegadogController::on_configure()'s subscription callback for the exact
// string values. HOME is the state on activation: zero effort, no WBC/MPC
// engaged, so the robot rests under gravity/contact in its prone sim pose.
// Normal commands go directly from HOME to STAND/TROT/FORWARD/BACKWARD -
// there is no intermediate joint-space recovery state (A1's own
// LeggedController has none either: it hands straight to MPC/WBC from the
// first control tick, using task.info's initialState purely as the
// optimizer's cold-start guess, not a PD target - see the port notes this
// replaces). A prior revision had an INIT crouch-recovery PD state here;
// removed since HOME->WBC direct handoff proved sufficient and INIT was
// never actually reachable in practice.
enum class MegadogFsmState
{
    HOME,
    STAND,
    STAND_NMPC,
    STAND_WBC,
    TROT_IN_PLACE,
    FORWARD,
    BACKWARD,
    // Genuine sideways translation (crab-walk) - base_velocity_y_m_s only,
    // heading held fixed. NOT the robot turning to face a new direction
    // then walking forward - see MegadogWbcCommand's base_velocity_y_m_s
    // field and setTargetTrajectories()'s handling of it.
    STRAFE_LEFT,
    STRAFE_RIGHT,
    // Yaw-rate steering (rotate in place) - base_yaw_rate_rad_s only.
    TURN_LEFT,
    TURN_RIGHT,
};

class MegadogController : public controller_interface::ControllerInterface
{
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    struct SimBaseSample
    {
        std::array<double, 3> position_m{};
        std::array<double, 3> euler_zyx_rad{};
        // Raw normalized quaternion (w,x,y,z), kept alongside euler_zyx_rad
        // so publishOdomTf() can broadcast "odom" -> "base" directly without
        // a lossy euler round-trip - defaults to identity.
        std::array<double, 4> orientation_wxyz{1.0, 0.0, 0.0, 0.0};
        std::array<double, 3> linear_velocity_m_s{};
        std::array<double, 3> euler_zyx_rate_rad_s{};
        std::chrono::steady_clock::time_point stamp{};
        bool has_data = false;
    };

    struct RealImuSample
    {
        std::array<double, 3> euler_zyx_rad{};
        std::array<double, 3> euler_zyx_rate_rad_s{};
        std::array<double, 4> orientation_wxyz{1.0, 0.0, 0.0, 0.0};
        std::array<double, 3> angular_velocity_body_rad_s{};
        std::array<double, 3> linear_acceleration_m_s2{};
        std::chrono::steady_clock::time_point stamp{};
        bool has_data = false;
    };

    static const std::vector<std::string>& jointNames();

    std::unique_ptr<megadog::hwbc::MegadogWbcRuntime> runtime_;
    // A MutuallyExclusive callback group serializes invocations in arrival
    // order, so consecutive callback calls never race on
    // latest_sim_base_state_ writes.
    rclcpp::CallbackGroup::SharedPtr sim_base_callback_group_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sim_base_subscription_;
    rclcpp::CallbackGroup::SharedPtr real_imu_callback_group_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr real_imu_subscription_;
    // Broadcasts "odom" -> "base" every control tick from the same
    // ground-truth pose sim_base_subscription_ already receives (or, on real
    // hardware with no ground truth, the IMU sample's orientation only), so
    // RViz can set Fixed Frame to "odom" and see the robot actually walk
    // through the world instead of always rendering pinned at the origin.
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
    std::mutex sim_base_mutex_;
    SimBaseSample latest_sim_base_state_;
    std::mutex imu_mutex_;
    RealImuSample latest_real_imu_state_;
    SimBaseSample last_control_base_sample_;
    bool last_control_base_sample_valid_ = false;
    std::array<double, 3> filtered_base_linear_velocity_m_s_{};
    std::array<double, 3> filtered_base_euler_zyx_rate_rad_s_{};

    rclcpp::CallbackGroup::SharedPtr cmd_callback_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscription_;
    // Plain enum behind atomic<int> - MegadogFsmState is trivially copyable,
    // so relaxed load/store from update() (real-time loop) racing the
    // subscription callback (its own executor thread) is safe: worst case a
    // command takes one extra control tick to be observed.
    std::atomic<int> fsm_state_{static_cast<int>(MegadogFsmState::HOME)};
    // Real-time-thread-only (update() is the sole reader/writer) - detects
    // the tick the FSM state changes, to reset wbc_time_s_/
    // state_entered_wbc_time_s_ appropriately.
    MegadogFsmState last_fsm_state_seen_ = MegadogFsmState::HOME;
    std::array<double, 3> latched_base_position_reference_m_{};
    double latched_base_yaw_reference_rad_ = 0.0;
    bool base_reference_latched_ = false;
    double state_entered_wbc_time_s_ = 0.0;
    std::array<double, 12> standup_start_joint_pos_{};
    bool standup_start_latched_ = false;
    double locomotion_runtime_epoch_wbc_time_s_ = 0.0;
    // Ramped toward the FSM state's target velocity each tick (see update())
    // instead of being commanded as a step - see kVelocityRampMps2's doc
    // comment in MegadogController.cpp for why. Runs unconditionally every
    // tick regardless of fsm_state, so switching FROM a state that was
    // driving one of these axes always ramps it back toward 0 instead of
    // leaving it stuck nonzero - see the target_velocity_x/y/target_yaw_rate
    // ternary chain in update() for the shared mechanism all three axes use.
    double smoothed_velocity_x_m_s_ = 0.0;
    double smoothed_velocity_y_m_s_ = 0.0;
    double smoothed_yaw_rate_rad_s_ = 0.0;
    std::array<double, 12> last_valid_effort_{};
    bool has_last_valid_effort_ = false;
    // Blends effort from last_valid_effort_ toward the freshly-computed WBC
    // torque over kEffortBlendDurationS once runtime_->update() resumes
    // succeeding after a hold (see kEffortBlendDurationS's doc comment in
    // MegadogController.cpp) - turns the hold-release instant into a ramp
    // instead of a step.
    bool effort_hold_active_ = false;
    double effort_blend_elapsed_s_ = 0.0;

    double start_time_s_ = -1.0;
    double elapsed_s_ = 0.0;
    double wbc_time_s_ = 0.0;
    double diagnostics_elapsed_s_ = 0.0;
    bool runtime_failure_reported_ = false;
    bool time_jump_reported_ = false;
};

}  // namespace megadog_controller
