#include "megadog_controller/MegadogController.h"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

namespace megadog_controller
{
namespace
{
using config_type = controller_interface::interface_configuration_type;

// Matches megadog_legged_interface's actuatedDofNum order exactly (see
// megadog_description/urdf/robot.xacro's leg instantiation order LF, LH, RF, RH).
// devq vs. A1 scaling factors (see const.xacro's real mass/length numbers):
//   length_ratio = devq's thigh/calf_length (0.15) / A1's (0.2) = 0.75
//   leg_inertia_ratio = per-leg mass ratio (1.112684/1.694 = 0.657) *
//                        length_ratio^2 (0.5625) = 0.3696 - approximates how
//                        much less reflected inertia a devq leg joint sees
//                        vs. A1's, for gains that bypass WBC's mass matrix.
// kJointKd runs on top of WbcBase's QP output as a direct joint-space torque
// term (torque += kJointKd * velocity_error) - it does NOT go through
// Pinocchio's mass matrix like WbcBase's own Cartesian kp/kd do, so it scales
// with leg_inertia_ratio (was legged_control's stock kd=3 for A1-scale legs).
constexpr double kJointKd = 2.0;
constexpr double kTorqueLimitNm = 80.0;  // sim debug limit; intentionally above devq's physical rating.
constexpr double kHaaTorqueLimitNm = 80.0;
// Comfortably under reference.info's targetDisplacementVelocity (0.5 m/s) -
// the reference manager's own upper bound for how fast a single MPC horizon
// is allowed to ask the base to move (devq/babyDog's own reference.info
// value, see the megaDog port plan breezy-purring-moler.md).
constexpr double kWalkSpeedMps = 0.18;
// Max |d(velocity)/dt| applied to the FSM's target velocity before it reaches
// MegadogWbcCommand::base_velocity_x_m_s - see update()'s ramp toward
// target_velocity_x. Without this, switching TROT_IN_PLACE -> FORWARD (or the
// reverse) stepped the commanded velocity from 0 to kWalkSpeedMps in a single
// control tick; setTargetTrajectories() folds that straight into a stepped
// target *position* timeToTarget seconds out, which the MPC then has to
// "catch up to" abruptly - visible as a jerky stutter rather than a
// smooth speed-up/slow-down. At this rate, a full forward<->backward reversal
// (2*kWalkSpeedMps) takes 2*kWalkSpeedMps/kVelocityRampMps2 = 1.2s.
constexpr double kVelocityRampMps2 = 0.8;
constexpr double kComHeightM = 0.22;
constexpr double kStandupDurationS = 3.0;
constexpr double kStandupKp = 35.0;
constexpr double kStandupKd = 1.8;
constexpr double kWbcHfePostureKp = 0.0;
constexpr double kWbcKfePostureKp = 0.0;
constexpr double kDiagnosticsPeriodS = 2.0;

constexpr std::array<double, 12> kStandingJointTargetRad{
    -0.30, 0.574027, -1.37275,
    -0.30, 0.574027, -1.37275,
     0.30, 0.574027, -1.37275,
     0.30, 0.574027, -1.37275,
};

bool isHaaJointIndex(const std::size_t joint_index)
{
    return joint_index % 3 == 0;
}

double smoothStep(const double x)
{
    const double t = std::clamp(x, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double smoothStepDerivative(const double x)
{
    const double t = std::clamp(x, 0.0, 1.0);
    return 6.0 * t * (1.0 - t);
}

double clampJointTorque(const std::size_t joint_index, const double torque)
{
    const double limit = isHaaJointIndex(joint_index) ? kHaaTorqueLimitNm : kTorqueLimitNm;
    return std::isfinite(torque) ? std::clamp(torque, -limit, limit) : 0.0;
}

// WbcBase's Cartesian task gains (swing_kp/kd, base_*_kp/kd in WbcBase.h)
// compute a *desired acceleration* (b = accDesired + kp*posError + kd*velError)
// that WbcBase's QP then maps to torque through Pinocchio's real mass matrix
// - so unlike kJointKd above, these do NOT need mass scaling (the QP
// already uses devq's real, ported mass/inertia). What they DO need is
// length scaling: kp/kd here set a closed-loop natural frequency
// (omega_n = sqrt(kp), independent of mass - same as a pendulum's period,
// which famously doesn't depend on the bob's mass, only its length). devq's
// shorter legs (0.15 vs A1's 0.2, length_ratio=0.75) have inherently faster
// natural dynamics, so the gains scale UP to keep pace:
//   kp_new = kp_old / length_ratio        (= kp_old * 1.3333)
//   kd_new = kd_old / sqrt(length_ratio)  (= kd_old * 1.1547, preserves the
//                                           same damping ratio zeta)
// WbcBase.h's own defaults (350/37/400/140/400/100/400/140) are qm_control's
// Aliengo-scale starting point, left as the library's generic baseline -
// megaDog's devq-scaled profile is built explicitly here instead.
megadog::hwbc::HierarchicalWbcConfig makeDevqWbcConfig()
{
    megadog::hwbc::HierarchicalWbcConfig config;
    config.swing_kp = 260.0;
    config.swing_kd = 28.0;
    config.swing_task_weight = 55.0;
    config.base_height_kp = 400.0;
    config.base_height_kd = 140.0;
    config.base_linear_kp = 400.0;
    config.base_linear_kd = 100.0;
    config.base_angular_kp = 300.0;
    config.base_angular_kd = 105.0;
    config.haa_posture_kp = 120.0;
    config.haa_posture_kd = 20.0;
    config.haa_posture_task_weight = 80.0;
    config.haa_posture_nominal_rad = {-0.30, -0.30, 0.30, 0.30};
    config.leg_posture_kp = 65.0;
    config.leg_posture_kd = 10.0;
    config.leg_posture_task_weight = 22.0;
    config.leg_posture_nominal_rad = {
        -0.30, 0.574027, -1.37275,
        -0.30, 0.574027, -1.37275,
         0.30, 0.574027, -1.37275,
         0.30, 0.574027, -1.37275,
    };
    config.leg_torque_limits_nm = {80.0, 80.0, 80.0};
    // Keeping qm_control's 10 s WBC warm-up here leaves STAND/TROT running
    // without base height/angular or swing-leg tasks for several seconds
    // after handoff, so the optimizer can settle into visually
    // impossible-looking leg poses before the real posture tasks ever
    // engage.
    config.init_task_seconds = 0.0;
    return config;
}

double quatToYaw(double w, double x, double y, double z)
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
double quatToPitch(double w, double x, double y, double z)
{
    const double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    return std::asin(sinp);
}
double quatToRoll(double w, double x, double y, double z)
{
    return std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
}

std::array<double, 3> bodyAngularVelocityToEulerZyxRate(const std::array<double, 3>& euler_zyx_rad,
                                                        const std::array<double, 3>& omega_body_rad_s)
{
    const double roll = euler_zyx_rad[2];
    const double pitch = euler_zyx_rad[1];
    const double wx = omega_body_rad_s[0];
    const double wy = omega_body_rad_s[1];
    const double wz = omega_body_rad_s[2];
    const double cos_pitch = std::cos(pitch);
    if (std::abs(cos_pitch) < 0.05 || !std::isfinite(cos_pitch)) {
        return {0.0, 0.0, 0.0};
    }
    const double sin_roll = std::sin(roll);
    const double cos_roll = std::cos(roll);
    const double tan_pitch = std::tan(pitch);
    const double roll_dot = wx + sin_roll * tan_pitch * wy + cos_roll * tan_pitch * wz;
    const double pitch_dot = cos_roll * wy - sin_roll * wz;
    const double yaw_dot = (sin_roll * wy + cos_roll * wz) / cos_pitch;
    return {yaw_dot, pitch_dot, roll_dot};
}

bool finiteArray(const std::array<double, 3>& values)
{
    return std::all_of(values.begin(), values.end(), [](const double value) { return std::isfinite(value); });
}

const char* fsmStateName(const MegadogFsmState state)
{
    switch (state) {
        case MegadogFsmState::HOME:
            return "HOME";
        case MegadogFsmState::STAND:
            return "STAND";
        case MegadogFsmState::STAND_NMPC:
            return "STAND_NMPC";
        case MegadogFsmState::STAND_WBC:
            return "STAND_WBC";
        case MegadogFsmState::TROT_IN_PLACE:
            return "TROT_IN_PLACE";
        case MegadogFsmState::FORWARD:
            return "FORWARD";
        case MegadogFsmState::BACKWARD:
            return "BACKWARD";
    }
    return "UNKNOWN";
}

bool isWbcState(const MegadogFsmState state)
{
    return state == MegadogFsmState::STAND || state == MegadogFsmState::STAND_NMPC ||
           state == MegadogFsmState::STAND_WBC || state == MegadogFsmState::TROT_IN_PLACE || state == MegadogFsmState::FORWARD ||
           state == MegadogFsmState::BACKWARD;
}

bool isLocomotionState(const MegadogFsmState state)
{
    return state == MegadogFsmState::TROT_IN_PLACE || state == MegadogFsmState::FORWARD ||
           state == MegadogFsmState::BACKWARD;
}
}  // namespace

const std::vector<std::string>& MegadogController::jointNames()
{
    static const std::vector<std::string> names = {
        "LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE",
    };
    return names;
}

controller_interface::InterfaceConfiguration MegadogController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(jointNames().size());
    for (const auto& joint_name : jointNames()) {
        conf.names.push_back(joint_name + "/effort");
    }
    return conf;
}

controller_interface::InterfaceConfiguration MegadogController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(jointNames().size() * 2);
    for (const auto& joint_name : jointNames()) {
        conf.names.push_back(joint_name + "/position");
        conf.names.push_back(joint_name + "/velocity");
    }
    return conf;
}

controller_interface::CallbackReturn MegadogController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MegadogController::on_configure(const rclcpp_lifecycle::State&)
{
    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

    real_imu_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions real_imu_options;
    real_imu_options.callback_group = real_imu_callback_group_;
    real_imu_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            const double qw = msg->orientation.w;
            const double qx = msg->orientation.x;
            const double qy = msg->orientation.y;
            const double qz = msg->orientation.z;
            const double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            const std::array<double, 3> omega_body{
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z,
            };
            const std::array<double, 3> linear_accel{
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
            };
            const bool finite_imu = std::isfinite(q_norm) && q_norm > 0.5 &&
                finiteArray(omega_body) && finiteArray(linear_accel);
            if (!finite_imu) {
                return;
            }
            const double inv_norm = 1.0 / q_norm;
            const double nw = qw * inv_norm, nx = qx * inv_norm, ny = qy * inv_norm, nz = qz * inv_norm;

            RealImuSample next;
            next.euler_zyx_rad = {quatToYaw(nw, nx, ny, nz), quatToPitch(nw, nx, ny, nz), quatToRoll(nw, nx, ny, nz)};
            next.orientation_wxyz = {nw, nx, ny, nz};
            next.angular_velocity_body_rad_s = omega_body;
            next.linear_acceleration_m_s2 = linear_accel;
            next.euler_zyx_rate_rad_s = bodyAngularVelocityToEulerZyxRate(next.euler_zyx_rad, omega_body);
            next.stamp = std::chrono::steady_clock::now();
            next.has_data = finiteArray(next.euler_zyx_rad) && finiteArray(next.euler_zyx_rate_rad_s);
            if (!next.has_data) {
                return;
            }

            std::lock_guard<std::mutex> lock(imu_mutex_);
            latest_real_imu_state_ = next;
        },
        real_imu_options);

    cmd_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cmd_options;
    cmd_options.callback_group = cmd_callback_group_;
    cmd_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
        "/megadog/cmd", rclcpp::QoS(10),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(get_node()->get_logger(), "MegadogController: received /megadog/cmd '%s'", msg->data.c_str());
            MegadogFsmState next;
            if (msg->data == "home") {
                next = MegadogFsmState::HOME;
            } else if (msg->data == "stand") {
                next = MegadogFsmState::STAND;
            } else if (msg->data == "stand_nmpc") {
                next = MegadogFsmState::STAND_NMPC;
            } else if (msg->data == "stand_wbc") {
                next = MegadogFsmState::STAND_WBC;
            } else if (msg->data == "trot_in_place") {
                next = MegadogFsmState::TROT_IN_PLACE;
            } else if (msg->data == "forward" || msg->data == "trot" || msg->data == "trot_forward") {
                next = MegadogFsmState::FORWARD;
            } else if (msg->data == "backward") {
                next = MegadogFsmState::BACKWARD;
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Unknown /megadog/cmd '%s' (expected home|stand|stand_nmpc|stand_wbc|trot_in_place|forward|trot|trot_forward|backward)",
                             msg->data.c_str());
                return;
            }
            // HOME's settled prone pose is the requested init pose, so normal
            // commands hand directly to WBC/MPC.
            fsm_state_.store(static_cast<int>(next), std::memory_order_relaxed);
        },
        cmd_options);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MegadogController::on_activate(const rclcpp_lifecycle::State&)
{
    // OCS2/WBC runtime is created lazily on the first non-HOME command. This
    // keeps opening the simulator in the passive HOME pose from touching CppAD/
    // MPC at all, which avoids startup aborts before the user asks the robot to
    // stand or walk.
    runtime_.reset();
    start_time_s_ = -1.0;
    elapsed_s_ = 0.0;
    wbc_time_s_ = 0.0;
    diagnostics_elapsed_s_ = 0.0;
    runtime_failure_reported_ = false;
    // Always start at rest - see MegadogFsmState::HOME's doc comment. A
    // previous activation's state (if any) is deliberately not preserved.
    fsm_state_.store(static_cast<int>(MegadogFsmState::HOME), std::memory_order_relaxed);
    last_fsm_state_seen_ = MegadogFsmState::HOME;
    last_valid_effort_.fill(0.0);
    has_last_valid_effort_ = false;
    latched_base_position_reference_m_ = {};
    latched_base_yaw_reference_rad_ = 0.0;
    base_reference_latched_ = false;
    state_entered_wbc_time_s_ = 0.0;
    locomotion_runtime_epoch_wbc_time_s_ = 0.0;
    smoothed_velocity_x_m_s_ = 0.0;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MegadogController::on_deactivate(const rclcpp_lifecycle::State&)
{
    runtime_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MegadogController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    const double dt = period.seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
        for (auto& command_interface : command_interfaces_) {
            std::ignore = command_interface.set_value(0.0);
        }
        return controller_interface::return_type::OK;
    }
    elapsed_s_ += dt;
    if (start_time_s_ < 0.0) {
        start_time_s_ = 0.0;
    }

    RealImuSample imu_sample;
    bool imu_fresh;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_sample = latest_real_imu_state_;
        const auto now = std::chrono::steady_clock::now();
        imu_fresh = imu_sample.has_data && std::chrono::duration<double>(now - imu_sample.stamp).count() < 0.2;
    }

    megadog::hwbc::MegadogWbcMeasurement measurement;
    std::array<double, 12> measured_velocity{};
    for (std::size_t j = 0; j < jointNames().size(); ++j) {
        const auto position = state_interfaces_[2 * j].get_optional();
        const auto velocity = state_interfaces_[2 * j + 1].get_optional();
        const double pos = position && std::isfinite(*position) ? *position : 0.0;
        const double vel = velocity && std::isfinite(*velocity) ? *velocity : 0.0;
        measurement.joint_pos_rad[j] = pos;
        measurement.joint_vel_rad_s[j] = vel;
        measured_velocity[j] = vel;
    }

    if (imu_fresh) {
        // Same on sim and real hardware: attitude/angular rate/linear
        // acceleration all come from /imu/data. base_pos_m/base_linear_vel_m_s
        // here are just placeholders - MegadogWbcRuntime's BaseStateEstimator
        // overwrites both every tick with its own leg-odometry estimate
        // (IMU + joint encoders + gait-schedule contact), so what's set here
        // is never actually used by WBC/NMPC.
        measurement.base_pos_m = {0.0, 0.0, kComHeightM};
        measurement.base_euler_zyx_rad = imu_sample.euler_zyx_rad;
        measurement.base_linear_vel_m_s = {0.0, 0.0, 0.0};
        measurement.base_euler_zyx_rate_rad_s = imu_sample.euler_zyx_rate_rad_s;
        measurement.base_linear_accel_local_m_s2 = imu_sample.linear_acceleration_m_s2;

        if (odom_tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header.stamp = time;
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id = "base";
            odom_tf.transform.translation.x = measurement.base_pos_m[0];
            odom_tf.transform.translation.y = measurement.base_pos_m[1];
            odom_tf.transform.translation.z = measurement.base_pos_m[2];
            odom_tf.transform.rotation.w = imu_sample.orientation_wxyz[0];
            odom_tf.transform.rotation.x = imu_sample.orientation_wxyz[1];
            odom_tf.transform.rotation.y = imu_sample.orientation_wxyz[2];
            odom_tf.transform.rotation.z = imu_sample.orientation_wxyz[3];
            odom_tf_broadcaster_->sendTransform(odom_tf);
        }
    } else {
        measurement.base_pos_m = {0.0, 0.0, kComHeightM};
    }

    const auto fsm_state = static_cast<MegadogFsmState>(fsm_state_.load(std::memory_order_relaxed));
    const auto previous_fsm_state = last_fsm_state_seen_;
    if (previous_fsm_state == MegadogFsmState::HOME && fsm_state != MegadogFsmState::HOME) {
        wbc_time_s_ = 0.0;
    } else if (fsm_state != MegadogFsmState::HOME) {
        wbc_time_s_ += dt;
    }
    if (fsm_state != previous_fsm_state) {
        state_entered_wbc_time_s_ = wbc_time_s_;
        if (previous_fsm_state == MegadogFsmState::HOME && fsm_state != MegadogFsmState::HOME) {
            standup_start_joint_pos_ = measurement.joint_pos_rad;
            standup_start_latched_ = true;
        }
        if (isLocomotionState(fsm_state) && !isLocomotionState(previous_fsm_state)) {
            locomotion_runtime_epoch_wbc_time_s_ =
                std::max(state_entered_wbc_time_s_, standup_start_latched_ ? kStandupDurationS : 0.0);
        }
    }

    if (fsm_state == MegadogFsmState::HOME) {
        base_reference_latched_ = false;
        standup_start_latched_ = false;
        locomotion_runtime_epoch_wbc_time_s_ = 0.0;
    } else if (isWbcState(fsm_state) && (!base_reference_latched_ || fsm_state != previous_fsm_state)) {
        latched_base_position_reference_m_ = measurement.base_pos_m;
        latched_base_yaw_reference_rad_ = measurement.base_euler_zyx_rad[0];
        base_reference_latched_ = true;
    }

    // Ramp toward the FSM state's target velocity every tick (including
    // HOME/STAND, so it decays back to 0 smoothly instead of snapping - see
    // kVelocityRampMps2's doc comment for why this matters).
    const double target_velocity_x = fsm_state == MegadogFsmState::FORWARD    ? kWalkSpeedMps
                                      : fsm_state == MegadogFsmState::BACKWARD ? -kWalkSpeedMps
                                                                                : 0.0;
    const double max_delta = kVelocityRampMps2 * dt;
    smoothed_velocity_x_m_s_ += std::clamp(target_velocity_x - smoothed_velocity_x_m_s_, -max_delta, max_delta);

    std::array<double, 12> effort{};
    bool ok = false;
    megadog::hwbc::MegadogWbcResult result;
    if (fsm_state == MegadogFsmState::HOME) {
        // HOME is the prone/resting init pose: literal zero effort, no WBC/MPC
        // call. The next locomotion command first uses the short joint-space
        // stand-up ramp below, then hands control to WBC/MPC.
        runtime_failure_reported_ = false;
    } else if (standup_start_latched_ && wbc_time_s_ < kStandupDurationS) {
        const double phase = wbc_time_s_ / kStandupDurationS;
        const double alpha = smoothStep(phase);
        const double alpha_dot = smoothStepDerivative(phase) / kStandupDurationS;
        for (std::size_t j = 0; j < jointNames().size(); ++j) {
            const double delta = kStandingJointTargetRad[j] - standup_start_joint_pos_[j];
            const double pos_des = standup_start_joint_pos_[j] + alpha * delta;
            const double vel_des = alpha_dot * delta;
            const double torque =
                kStandupKp * (pos_des - measurement.joint_pos_rad[j]) + kStandupKd * (vel_des - measured_velocity[j]);
            effort[j] = clampJointTorque(j, torque);
            result.position_rad[j] = pos_des;
            result.velocity_rad_s[j] = vel_des;
            result.torque_nm[j] = effort[j];
        }
        result.valid = true;
        ok = true;
        runtime_failure_reported_ = false;
    } else {
        megadog::hwbc::MegadogWbcCommand command;
        command.com_height_m = kComHeightM;
        switch (fsm_state) {
            case MegadogFsmState::STAND:
            case MegadogFsmState::STAND_NMPC:
            case MegadogFsmState::STAND_WBC:
                command.gait_name = "stance";
                command.use_mpc_for_stance_hold = fsm_state == MegadogFsmState::STAND_NMPC;
                if (fsm_state != MegadogFsmState::STAND_NMPC && base_reference_latched_) {
                    command.base_x_reference_m = latched_base_position_reference_m_[0];
                    command.base_y_reference_m = latched_base_position_reference_m_[1];
                    command.base_yaw_reference_rad = latched_base_yaw_reference_rad_;
                }
                break;
            case MegadogFsmState::TROT_IN_PLACE:
            case MegadogFsmState::FORWARD:
            case MegadogFsmState::BACKWARD:
                // Match ultraDog/A1: all trot-family states share the same
                // gait template. Switching between in-place and moving then
                // only changes the ramped base velocity, avoiding a mid-stride
                // gait-template rewrite that can feel like a periodic stutter.
                command.gait_name = "trot";
                command.base_velocity_x_m_s = smoothed_velocity_x_m_s_;
                if (base_reference_latched_) {
                    command.base_y_reference_m = latched_base_position_reference_m_[1];
                    command.base_yaw_reference_rad = latched_base_yaw_reference_rad_;
                }
                break;
            case MegadogFsmState::HOME:
                break;  // unreachable (handled above)
        }

        const double runtime_epoch_s =
            (fsm_state == MegadogFsmState::STAND || fsm_state == MegadogFsmState::STAND_NMPC ||
             fsm_state == MegadogFsmState::STAND_WBC) &&
                    standup_start_latched_
                ? kStandupDurationS
                : locomotion_runtime_epoch_wbc_time_s_;
        const double runtime_time_s = std::max(0.0, wbc_time_s_ - runtime_epoch_s);
        if (!imu_fresh) {
            if (!runtime_failure_reported_) {
                RCLCPP_WARN(
                    get_node()->get_logger(),
                    "MegadogController has no fresh /imu/data; WBC/NMPC held");
                runtime_failure_reported_ = true;
            }
            ok = false;
        } else if (!runtime_) {
            runtime_ = std::make_unique<megadog::hwbc::MegadogWbcRuntime>(makeDevqWbcConfig(), true);
            if (!runtime_->ready()) {
                RCLCPP_ERROR(get_node()->get_logger(), "MegadogWbcRuntime failed to initialize");
                runtime_.reset();
            }
        }
        if (imu_fresh) {
            ok = runtime_ && runtime_->ready() && runtime_->update(runtime_time_s, dt, time, measurement, command, result);
        }
        if (ok && result.valid) {
            for (std::size_t j = 0; j < jointNames().size(); ++j) {
                double posture_torque = 0.0;
                if (j % 3 == 1) {
                    posture_torque = kWbcHfePostureKp * (kStandingJointTargetRad[j] - measurement.joint_pos_rad[j]);
                } else if (j % 3 == 2) {
                    posture_torque = kWbcKfePostureKp * (kStandingJointTargetRad[j] - measurement.joint_pos_rad[j]);
                }
                const double torque = result.torque_nm[j] + kJointKd * (result.velocity_rad_s[j] - measured_velocity[j]) +
                                      posture_torque;
                effort[j] = clampJointTorque(j, torque);
            }
            last_valid_effort_ = effort;
            has_last_valid_effort_ = true;
            runtime_failure_reported_ = false;
        } else {
            if (!runtime_failure_reported_) {
                if (imu_fresh) {
                    RCLCPP_WARN(get_node()->get_logger(), "MegadogWbcRuntime produced no valid sample; holding last valid effort");
                }
                runtime_failure_reported_ = true;
            }
            effort = has_last_valid_effort_ ? last_valid_effort_ : effort;
        }
    }

    for (std::size_t j = 0; j < jointNames().size(); ++j) {
        std::ignore = command_interfaces_[j].set_value(effort[j]);
    }

    diagnostics_elapsed_s_ += dt;
    if (diagnostics_elapsed_s_ >= kDiagnosticsPeriodS) {
        diagnostics_elapsed_s_ = 0.0;
        RCLCPP_INFO(
            get_node()->get_logger(),
            "MegadogController: t=%.2f wbc_t=%.2f state=%s valid=%d imu_fresh=%d vx=%.3f eom=%.4f "
            "base=[z %.3f yaw %.3f pitch %.3f roll %.3f vz %.3f wyaw %.3f wpitch %.3f wroll %.3f] "
            "HAA_meas=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "HFE_meas=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "KFE_meas=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "HAA_mpc=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "HFE_mpc=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "KFE_mpc=[LF %.3f LH %.3f RF %.3f RH %.3f] "
            "HAA_tau=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "HFE_tau=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "KFE_tau=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "HAA_eff=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "HFE_eff=[LF %.2f LH %.2f RF %.2f RH %.2f] "
            "KFE_eff=[LF %.2f LH %.2f RF %.2f RH %.2f]",
            elapsed_s_, wbc_time_s_, fsmStateName(fsm_state), ok && result.valid ? 1 : 0,
            imu_fresh ? 1 : 0,
            smoothed_velocity_x_m_s_, result.eom_residual_norm,
            measurement.base_pos_m[2], measurement.base_euler_zyx_rad[0], measurement.base_euler_zyx_rad[1],
            measurement.base_euler_zyx_rad[2], measurement.base_linear_vel_m_s[2],
            measurement.base_euler_zyx_rate_rad_s[0], measurement.base_euler_zyx_rate_rad_s[1],
            measurement.base_euler_zyx_rate_rad_s[2],
            measurement.joint_pos_rad[0], measurement.joint_pos_rad[3], measurement.joint_pos_rad[6],
            measurement.joint_pos_rad[9],
            measurement.joint_pos_rad[1], measurement.joint_pos_rad[4], measurement.joint_pos_rad[7],
            measurement.joint_pos_rad[10],
            measurement.joint_pos_rad[2], measurement.joint_pos_rad[5], measurement.joint_pos_rad[8],
            measurement.joint_pos_rad[11],
            result.position_rad[0], result.position_rad[3], result.position_rad[6], result.position_rad[9],
            result.position_rad[1], result.position_rad[4], result.position_rad[7], result.position_rad[10],
            result.position_rad[2], result.position_rad[5], result.position_rad[8], result.position_rad[11],
            result.torque_nm[0], result.torque_nm[3], result.torque_nm[6], result.torque_nm[9],
            result.torque_nm[1], result.torque_nm[4], result.torque_nm[7], result.torque_nm[10],
            result.torque_nm[2], result.torque_nm[5], result.torque_nm[8], result.torque_nm[11],
            effort[0], effort[3], effort[6], effort[9], effort[1], effort[4], effort[7], effort[10], effort[2],
            effort[5], effort[8], effort[11]);
    }

    last_fsm_state_seen_ = fsm_state;

    return controller_interface::return_type::OK;
}

}  // namespace megadog_controller

PLUGINLIB_EXPORT_CLASS(megadog_controller::MegadogController, controller_interface::ControllerInterface)
