// Copyright 2024 salhus
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file pto_controller_node.cpp
 * @brief ROS 2 node that runs the pluggable PTO control strategy in the HIL loop.
 *
 * @details
 * This node is the "PTO control law" in the HIL architecture. Its responsibilities:
 *
 *   1. Read the `strategy` parameter and instantiate the corresponding PtoStrategyBase
 *      implementation via the factory function.
 *
 *   2. Subscribe to /joint_states to obtain the PTO joint velocity and position.
 *      The joint states come from the ros2_control JointStateBroadcaster, which reads
 *      directly from the ODrive encoder via the hardware interface.
 *
 *   3. Run a control timer at configurable rate (default 100 Hz).
 *
 *   4. Each tick: call strategy->compute_torque(ω, x, dt) and publish:
 *      - /pto_effort_controller/commands (Float64MultiArray) — command to ros2_control
 *      - /pto/commanded_torque (Float64) — what the control law computed
 *      - /pto/measured_torque (Float64) — what the ODrive actually delivered (from joint_states effort)
 *
 * The separation between "commanded" and "measured" torque is the key HIL signal:
 *   τ_cmd = what the control law asked for
 *   τ_meas = what the motor actually delivered (including electrical dynamics, friction, latency)
 *   error = τ_cmd − τ_meas → quantifies actuator fidelity
 *
 * Topic summary:
 *   Subscribes:
 *     /joint_states  (sensor_msgs/JointState) — position, velocity, effort from ODrive
 *
 *   Publishes:
 *     /pto_effort_controller/commands  (std_msgs/Float64MultiArray) — to ros2_control
 *     /pto/commanded_torque            (std_msgs/Float64) — τ_cmd for logging
 *     /pto/measured_torque             (std_msgs/Float64) — τ_meas for HIL feedback to Chrono
 *
 * References:
 *   Ringwood, J.V. et al. (2014). IEEE Control Systems, 34(5), 30–55.
 *   Penalba, M. et al. (2019). Renewable Energy, 139, 1077–1089.
 *
 * @author salhus
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Strategy headers — all concrete implementations
#include "pto_controller/pto_strategy_base.hpp"
#include "pto_controller/passive_damping.hpp"
#include "pto_controller/optimal_passive.hpp"
#include "pto_controller/reactive_control.hpp"
#include "pto_controller/latching_control.hpp"
#include "pto_controller/declutching_control.hpp"

namespace pto_controller
{

// ─────────────────────────────────────────────────────────────────────────────
// Factory function implementation
// (declared in pto_strategy_base.hpp, defined here to avoid link-order issues)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Instantiate a PTO strategy by name with the given parameters.
 *
 * @param strategy_name  Strategy identifier string.
 * @param params         Parameters bundle.
 * @return Owning pointer to the strategy.
 * @throws std::invalid_argument for unknown strategy names.
 */
std::unique_ptr<PtoStrategyBase> create_pto_strategy(
  const std::string & strategy_name,
  const PtoStrategyParams & params)
{
  if (strategy_name == "passive_damping") {
    return std::make_unique<PassiveDamping>(params.damping_coefficient_b);
  } else if (strategy_name == "optimal_passive") {
    return std::make_unique<OptimalPassive>(
      params.radiation_damping,
      params.body_mass_kg,
      params.added_mass_kg,
      params.hydrostatic_stiffness,
      params.wave_omega);
  } else if (strategy_name == "reactive") {
    return std::make_unique<ReactiveControl>(
      params.damping_coefficient_b,
      params.spring_k);
  } else if (strategy_name == "latching") {
    return std::make_unique<LatchingControl>(
      params.latch_velocity_threshold,
      params.unlatch_delay_s,
      params.max_latch_torque,
      params.damping_coefficient_b);
  } else if (strategy_name == "declutching") {
    return std::make_unique<DeclutchingControl>(
      params.engage_position_threshold,
      params.engage_duration_s,
      params.damping_when_engaged);
  } else {
    throw std::invalid_argument(
      "Unknown PTO strategy: '" + strategy_name +
      "'. Valid: passive_damping, optimal_passive, reactive, latching, declutching");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Node class
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief ROS 2 node implementing the pluggable PTO control strategy.
 */
class PtoControllerNode : public rclcpp::Node
{
public:
  explicit PtoControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pto_controller_node", options),
    pto_velocity_(0.0),
    pto_position_(0.0),
    pto_effort_measured_(0.0),
    joint_state_received_(false),
    last_control_time_(this->now())
  {
    // ── Parameter declarations ────────────────────────────────────────────
    declare_parameter("strategy", std::string("passive_damping"));
    declare_parameter("joint_name", std::string("pto_joint"));
    declare_parameter("rate_hz", 100.0);
    declare_parameter("torque_limit_nm", 100.0);

    // Strategy parameters — all declared even if not used by the active strategy,
    // so they can be inspected/set via `ros2 param` without changing strategy.
    declare_parameter("damping_coefficient_b", 200.0);
    declare_parameter("spring_k", 0.0);
    declare_parameter("radiation_damping", 200.0);
    declare_parameter("body_mass_kg", 1000.0);
    declare_parameter("added_mass_kg", 500.0);
    declare_parameter("hydrostatic_stiffness", 3000.0);
    declare_parameter("wave_omega", 1.0);
    declare_parameter("latch_velocity_threshold", 0.05);
    declare_parameter("unlatch_delay_s", 0.5);
    declare_parameter("max_latch_torque", 500.0);
    declare_parameter("engage_position_threshold", 0.1);
    declare_parameter("engage_duration_s", 0.3);
    declare_parameter("damping_when_engaged", 300.0);

    // ── Read parameters ───────────────────────────────────────────────────
    const std::string strategy_name = get_parameter("strategy").as_string();
    joint_name_ = get_parameter("joint_name").as_string();
    const double rate_hz = get_parameter("rate_hz").as_double();
    torque_limit_ = get_parameter("torque_limit_nm").as_double();

    PtoStrategyParams params;
    params.damping_coefficient_b = get_parameter("damping_coefficient_b").as_double();
    params.spring_k = get_parameter("spring_k").as_double();
    params.radiation_damping = get_parameter("radiation_damping").as_double();
    params.body_mass_kg = get_parameter("body_mass_kg").as_double();
    params.added_mass_kg = get_parameter("added_mass_kg").as_double();
    params.hydrostatic_stiffness = get_parameter("hydrostatic_stiffness").as_double();
    params.wave_omega = get_parameter("wave_omega").as_double();
    params.latch_velocity_threshold = get_parameter("latch_velocity_threshold").as_double();
    params.unlatch_delay_s = get_parameter("unlatch_delay_s").as_double();
    params.max_latch_torque = get_parameter("max_latch_torque").as_double();
    params.engage_position_threshold = get_parameter("engage_position_threshold").as_double();
    params.engage_duration_s = get_parameter("engage_duration_s").as_double();
    params.damping_when_engaged = get_parameter("damping_when_engaged").as_double();

    // ── Instantiate strategy ──────────────────────────────────────────────
    strategy_ = create_pto_strategy(strategy_name, params);

    // ── Publishers ────────────────────────────────────────────────────────
    pub_effort_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pto_effort_controller/commands",
      rclcpp::QoS(10));
    pub_commanded_torque_ = create_publisher<std_msgs::msg::Float64>(
      "/pto/commanded_torque",
      rclcpp::QoS(10));
    pub_measured_torque_ = create_publisher<std_msgs::msg::Float64>(
      "/pto/measured_torque",
      rclcpp::QoS(10).reliable());

    // ── Subscriber ────────────────────────────────────────────────────────
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        handle_joint_states(msg);
      });

    // ── Control timer ─────────────────────────────────────────────────────
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    control_timer_ = create_wall_timer(
      period,
      std::bind(&PtoControllerNode::control_step, this));

    // ── Startup log ───────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "PtoControllerNode started");
    RCLCPP_INFO(get_logger(), "  Strategy   : %s", strategy_->name().c_str());
    RCLCPP_INFO(get_logger(), "  Joint name : %s", joint_name_.c_str());
    RCLCPP_INFO(get_logger(), "  Rate       : %.0f Hz", rate_hz);
    RCLCPP_INFO(get_logger(), "  Torque lim : ±%.1f N·m", torque_limit_);
  }

  ~PtoControllerNode()
  {
    // Publish zero torque on shutdown to bring the PTO motor to rest safely.
    RCLCPP_INFO(get_logger(), "Shutting down — publishing zero PTO torque");
    publish_torque_command(0.0);
  }

private:
  /**
   * @brief JointState subscriber callback.
   *
   * Extracts PTO joint position, velocity, and effort from the message.
   * The effort value is the measured torque reported by the ODrive current controller.
   */
  void handle_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == joint_name_) {
        if (i < msg->position.size()) {
          pto_position_ = msg->position[i];
        }
        if (i < msg->velocity.size()) {
          pto_velocity_ = msg->velocity[i];
        }
        if (i < msg->effort.size()) {
          pto_effort_measured_ = msg->effort[i];
          // Publish measured torque (HIL feedback to Chrono simulation node).
          auto meas_msg = std_msgs::msg::Float64();
          meas_msg.data = pto_effort_measured_;
          pub_measured_torque_->publish(meas_msg);
        }
        joint_state_received_ = true;
        break;
      }
    }
  }

  /**
   * @brief Control timer callback — the main PTO control loop.
   *
   * Computes dt since last call, invokes the strategy, saturates the torque
   * to the hardware limit, and publishes to ros2_control and logging topics.
   */
  void control_step()
  {
    if (!joint_state_received_) {
      // No joint state received yet — do not command the motor.
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /joint_states from joint '%s'...", joint_name_.c_str());
      return;
    }

    // Compute dt since last control step (use actual elapsed time, not nominal,
    // to handle timer jitter gracefully in time-based strategies).
    const rclcpp::Time now = this->now();
    const double dt = (now - last_control_time_).seconds();
    last_control_time_ = now;

    // Protect against unreasonably large dt (e.g., first call, timer restart).
    const double dt_clamped = std::min(dt, 0.1);

    // Compute desired torque from strategy.
    double tau_cmd = strategy_->compute_torque(pto_velocity_, pto_position_, dt_clamped);

    // Saturate torque to protect hardware.
    if (std::abs(tau_cmd) > torque_limit_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Torque saturated: requested %.2f N·m, limited to ±%.2f N·m",
        tau_cmd, torque_limit_);
      tau_cmd = std::copysign(torque_limit_, tau_cmd);
    }

    publish_torque_command(tau_cmd);
  }

  /**
   * @brief Publish the torque command to ros2_control and the commanded_torque topic.
   */
  void publish_torque_command(double tau)
  {
    // Publish to ros2_control effort controller.
    auto effort_msg = std_msgs::msg::Float64MultiArray();
    effort_msg.data.push_back(tau);
    pub_effort_cmd_->publish(effort_msg);

    // Publish for logging / tracking error computation.
    auto cmd_msg = std_msgs::msg::Float64();
    cmd_msg.data = tau;
    pub_commanded_torque_->publish(cmd_msg);
  }

  // ── Member variables ─────────────────────────────────────────────────────

  std::unique_ptr<PtoStrategyBase> strategy_;  ///< Active PTO control strategy

  std::string joint_name_;  ///< Name of the PTO joint in /joint_states
  double torque_limit_;     ///< Hardware torque limit [N·m]

  // Latest joint state values (updated by subscriber callback).
  double pto_velocity_;           ///< PTO joint velocity [rad/s]
  double pto_position_;           ///< PTO joint position [rad]
  double pto_effort_measured_;    ///< Measured PTO torque from ODrive [N·m]
  bool joint_state_received_;     ///< Guard flag — don't command until state is known

  rclcpp::Time last_control_time_;  ///< Timestamp of last control step (for dt)

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_effort_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_commanded_torque_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_measured_torque_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

  // Control timer
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace pto_controller

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pto_controller::PtoControllerNode>());
  rclcpp::shutdown();
  return 0;
}
