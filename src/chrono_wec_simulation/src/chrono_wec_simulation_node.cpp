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
 * @file chrono_wec_simulation_node.cpp
 * @brief ROS 2 node that drives the WEC dynamics model and publishes motion commands.
 *
 * @details
 * This node is the "wave-body solver" in the HIL loop. Its responsibilities are:
 *
 *   1. Maintain a real-time WEC dynamics model (currently LinearWecDynamics placeholder;
 *      will be replaced by HydroChrono/SeaStack when available).
 *
 *   2. Subscribe to /pto/measured_torque — the actual torque delivered by the physical
 *      PTO motor (measured via ODrive current feedback). This is the HIL feedback signal
 *      that closes the loop between the numerical model and the physical hardware.
 *
 *   3. Step the dynamics at a high rate (simulation_rate_hz, default 1000 Hz) using
 *      the measured PTO torque.
 *
 *   4. Publish motion setpoints at a configurable output rate (publish_rate_hz, default
 *      100 Hz) to avoid overwhelming the motor controller with data.
 *
 * Topic summary:
 *   Subscribes:
 *     /pto/measured_torque     (std_msgs/Float64) — HIL feedback from physical PTO motor
 *
 *   Publishes:
 *     /wec/velocity_command    (std_msgs/Float64) — ẋ(t) → hydro motor velocity setpoint
 *     /wec/position_command    (std_msgs/Float64) — x(t) → for position-based PTO strategies
 *     /wec/wave_elevation      (std_msgs/Float64) — η(t) → for logger wave power calculation
 *     /wec/body_position       (std_msgs/Float64) — x(t) → duplicate for logging convenience
 *
 * Architecture note:
 *   The simulation_rate_hz and publish_rate_hz are intentionally decoupled. The dynamics
 *   integrate at 1000 Hz for accuracy, but the motor controller only needs new setpoints
 *   at 100 Hz (it interpolates between setpoints). This reduces network load on the
 *   Simulation PC → Motor PC DDS link from ~1 MB/s to ~100 kB/s for Float64 topics.
 *
 * References:
 *   Falnes, J. (2002). Ocean Waves and Oscillating Systems. Cambridge University Press.
 *   Penalba, M. et al. (2019). Renewable Energy, 139, 1077–1089.
 *
 * @author salhus
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "chrono_wec_simulation/wec_body_dynamics.hpp"

namespace chrono_wec_simulation
{

/**
 * @brief ROS 2 node implementing the WEC wave-body simulation in the HIL loop.
 *
 * Runs a WecBodyDynamics model in real time, subscribes to the physical PTO torque
 * feedback, and publishes velocity/position commands to the motor control PC.
 */
class ChronoWecSimulationNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the node, declare parameters, create publishers/subscribers,
   *        instantiate the dynamics model, and start the simulation timer.
   *
   * @param options  Node options (passed through from main).
   */
  explicit ChronoWecSimulationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("chrono_wec_simulation_node", options),
    latest_pto_torque_(0.0),
    sim_step_count_(0),
    publish_step_count_(0)
  {
    // ── Parameter declarations ────────────────────────────────────────────
    // Each parameter is declared with a description so that
    // `ros2 param describe` and rqt_reconfigure show meaningful information.

    declare_parameter("simulation_rate_hz", 1000.0);
    declare_parameter("publish_rate_hz", 100.0);
    declare_parameter("body_mass_kg", 1000.0);
    declare_parameter("added_mass_inf_kg", 500.0);
    declare_parameter("radiation_damping_ns_m", 200.0);
    declare_parameter("hydrostatic_stiffness_n_m", 3000.0);
    declare_parameter("excitation_force_amplitude_n", 500.0);
    declare_parameter("wave_omega_rad_s", 1.0);
    declare_parameter("wave_height_m", 1.0);

    // ── Read and validate parameters ──────────────────────────────────────
    const double sim_rate = get_parameter("simulation_rate_hz").as_double();
    const double pub_rate = get_parameter("publish_rate_hz").as_double();
    const double mass = get_parameter("body_mass_kg").as_double();
    const double added_mass = get_parameter("added_mass_inf_kg").as_double();
    const double b_rad = get_parameter("radiation_damping_ns_m").as_double();
    const double k_hs = get_parameter("hydrostatic_stiffness_n_m").as_double();
    const double f_exc_amp = get_parameter("excitation_force_amplitude_n").as_double();
    const double omega = get_parameter("wave_omega_rad_s").as_double();
    const double wave_h = get_parameter("wave_height_m").as_double();

    if (sim_rate <= 0.0 || pub_rate <= 0.0) {
      throw std::invalid_argument("simulation_rate_hz and publish_rate_hz must be positive");
    }
    if (pub_rate > sim_rate) {
      RCLCPP_WARN(
        get_logger(),
        "publish_rate_hz (%.0f) > simulation_rate_hz (%.0f) — clamping to simulation rate",
        pub_rate, sim_rate);
    }

    // Compute how many simulation steps between each publish
    publish_every_n_steps_ = static_cast<int>(std::round(sim_rate / pub_rate));
    if (publish_every_n_steps_ < 1) {
      publish_every_n_steps_ = 1;
    }

    // ── Instantiate dynamics model ────────────────────────────────────────
    // This is the only place where the concrete dynamics implementation is named.
    // To switch to HydroChrono, replace this line with:
    //   dynamics_ = std::make_unique<HydroChronoDynamics>(...);
    dynamics_ = std::make_unique<LinearWecDynamics>(
      mass, added_mass, b_rad, k_hs, f_exc_amp, omega, wave_h);

    dt_ = 1.0 / sim_rate;

    // ── Publishers ────────────────────────────────────────────────────────
    // QoS: Best-effort, keep-last(10) — suitable for continuous real-time streams.
    // The motor controller can tolerate an occasional dropped message because the
    // dynamics evolve smoothly; a missed setpoint is interpolated by the controller.
    auto qos = rclcpp::QoS(10);

    pub_velocity_cmd_ = create_publisher<std_msgs::msg::Float64>(
      "/wec/velocity_command", qos);
    pub_position_cmd_ = create_publisher<std_msgs::msg::Float64>(
      "/wec/position_command", qos);
    pub_wave_elevation_ = create_publisher<std_msgs::msg::Float64>(
      "/wec/wave_elevation", qos);
    pub_body_position_ = create_publisher<std_msgs::msg::Float64>(
      "/wec/body_position", qos);

    // ── Subscriber ────────────────────────────────────────────────────────
    // The PTO torque subscription uses reliable QoS to ensure that the HIL
    // feedback torque is never silently dropped. This is the most safety-critical
    // topic in the system — if τ_pto is stale, the simulation diverges from reality.
    sub_pto_torque_ = create_subscription<std_msgs::msg::Float64>(
      "/pto/measured_torque",
      rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        latest_pto_torque_ = msg->data;
      });

    // ── Simulation timer ──────────────────────────────────────────────────
    // Uses a wall timer. On the Simulation PC, a real-time kernel is not required
    // because small timing jitter (< 1 ms) in the dynamics integration introduces
    // negligible error at 1000 Hz (the WEC dynamics have time constants of seconds).
    const auto period = std::chrono::duration<double>(dt_);
    sim_timer_ = create_wall_timer(period, std::bind(&ChronoWecSimulationNode::sim_step, this));

    // ── Startup log ───────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "ChronoWecSimulationNode started");
    RCLCPP_INFO(get_logger(), "  Dynamics model : LinearWecDynamics (placeholder)");
    RCLCPP_INFO(get_logger(), "  Simulation rate: %.0f Hz (dt = %.4f s)", sim_rate, dt_);
    RCLCPP_INFO(get_logger(), "  Publish rate   : %.0f Hz (every %d steps)", pub_rate,
      publish_every_n_steps_);
    RCLCPP_INFO(get_logger(), "  Body mass      : %.0f kg (+ %.0f kg added = %.0f kg total)",
      mass, added_mass, mass + added_mass);
    RCLCPP_INFO(get_logger(), "  B_rad          : %.1f N·s/m", b_rad);
    RCLCPP_INFO(get_logger(), "  K_hs           : %.1f N/m", k_hs);
    RCLCPP_INFO(get_logger(), "  Wave ω         : %.3f rad/s (T = %.2f s)",
      omega, 2.0 * M_PI / omega);
    RCLCPP_INFO(get_logger(), "  F_exc amplitude: %.1f N", f_exc_amp);
    RCLCPP_INFO(get_logger(), "  Wave height    : %.2f m", wave_h);
  }

  /**
   * @brief Destructor — publishes a zero velocity command before shutdown.
   *
   * This prevents the hydro motor from holding the last commanded velocity
   * after the simulation node exits. Publishing zero causes the motor
   * controller to bring the shaft to rest safely.
   */
  ~ChronoWecSimulationNode()
  {
    RCLCPP_INFO(get_logger(), "Shutting down — publishing zero velocity command");
    auto zero = std_msgs::msg::Float64();
    zero.data = 0.0;
    pub_velocity_cmd_->publish(zero);
  }

private:
  /**
   * @brief Timer callback — the main simulation loop.
   *
   * Called at simulation_rate_hz. Each call:
   *   1. Steps the dynamics model with the latest measured PTO torque.
   *   2. Every publish_every_n_steps_ calls, publishes all output topics.
   */
  void sim_step()
  {
    // Step the dynamics with the current (zero-order held) PTO torque.
    dynamics_->step(dt_, latest_pto_torque_);
    ++sim_step_count_;

    // Publish at the (decimated) publish rate.
    if (sim_step_count_ % publish_every_n_steps_ == 0) {
      const double vel = dynamics_->get_velocity();
      const double pos = dynamics_->get_position();
      // Compute simulation time from step count to avoid floating-point drift.
      const double t = static_cast<double>(sim_step_count_) * dt_;
      const double eta = dynamics_->get_wave_elevation(t);

      publish_float("/wec/velocity_command", vel, pub_velocity_cmd_);
      publish_float("/wec/position_command", pos, pub_position_cmd_);
      publish_float("/wec/wave_elevation", eta, pub_wave_elevation_);
      publish_float("/wec/body_position", pos, pub_body_position_);

      ++publish_step_count_;

      // Periodic status log (every 10 s at publish rate).
      if (publish_step_count_ % static_cast<int>(10.0 * 1.0 / (dt_ * publish_every_n_steps_)) ==
        0)
      {
        RCLCPP_DEBUG(
          get_logger(),
          "t=%.2f s  x=%.4f m  ẋ=%.4f m/s  τ_pto=%.2f N·m  η=%.3f m",
          t, pos, vel, latest_pto_torque_, eta);
      }
    }
  }

  /**
   * @brief Convenience helper — publish a double value to a Float64 topic.
   *
   * @param topic_name  For debug only (not used in publish itself).
   * @param value       The value to publish.
   * @param publisher   The publisher to use.
   */
  void publish_float(
    [[maybe_unused]] const std::string & topic_name,
    double value,
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher)
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = value;
    publisher->publish(msg);
  }

  // ── Member variables ─────────────────────────────────────────────────────

  /// The WEC body dynamics model. Owning pointer enables swap of implementation.
  std::unique_ptr<WecBodyDynamics> dynamics_;

  /// Most recently received PTO torque [N·m]. Thread-safe for single-threaded node.
  double latest_pto_torque_;

  /// Integration timestep [s] = 1 / simulation_rate_hz.
  double dt_;

  /// Publish every Nth simulation step to achieve publish_rate_hz.
  int publish_every_n_steps_;

  /// Total simulation steps taken (used for timing).
  uint64_t sim_step_count_;

  /// Total publish events (used for periodic log).
  uint64_t publish_step_count_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_velocity_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_position_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wave_elevation_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_body_position_;

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_pto_torque_;

  // Simulation timer
  rclcpp::TimerBase::SharedPtr sim_timer_;
};

}  // namespace chrono_wec_simulation

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<chrono_wec_simulation::ChronoWecSimulationNode>());
  rclcpp::shutdown();
  return 0;
}
