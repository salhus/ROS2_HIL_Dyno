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
 * @file wec_dyno_logger_node.cpp
 * @brief ROS 2 node that computes and logs WEC HIL performance metrics.
 *
 * @details
 * This node is a passive observer in the HIL loop. It subscribes to all relevant
 * topics, computes standardised performance metrics over a sliding time window,
 * publishes them at a configurable rate, and optionally exports to CSV.
 *
 * Computed metrics (Ringwood et al., 2014; Penalba et al., 2019):
 *
 *   1. Instantaneous power  P(t) = τ_pto · ω                       [W]
 *   2. Average absorbed power  P_avg = (1/T)∫P(t)dt                [W]
 *   3. Capture width ratio  CWR = P_avg / (P_wave · D)             [dimensionless]
 *   4. Peak-to-average ratio  PAPR = max|P| / P_avg                [dimensionless]
 *   5. Torque tracking error  RMS(τ_cmd − τ_meas)                  [N·m]
 *   6. Reactive power fraction  fraction of time P(t) < 0          [dimensionless]
 *
 * Wave power per unit crest width (linear wave theory, Falnes, 2002):
 *   P_wave = (1/8) · ρ · g² · H² · T / (4π)    [W/m]
 * where H is wave height [m], T is wave period [s], ρ = 1025 kg/m³.
 *
 * The CWR is a device-size-independent efficiency metric that facilitates
 * comparison across different WEC scales and geometries.
 *
 * Topic summary:
 *   Subscribes:
 *     /joint_states              (sensor_msgs/JointState)
 *     /pto/commanded_torque      (std_msgs/Float64)
 *     /pto/measured_torque       (std_msgs/Float64)
 *     /wec/velocity_command      (std_msgs/Float64)
 *     /wec/wave_elevation        (std_msgs/Float64)
 *
 *   Publishes (all Float64, namespace ~/):
 *     ~/instantaneous_power
 *     ~/average_power
 *     ~/capture_width_ratio
 *     ~/peak_to_average_ratio
 *     ~/torque_tracking_error
 *     ~/reactive_power_fraction
 *
 * References:
 *   Falnes, J. (2002). Ocean Waves and Oscillating Systems. Cambridge University Press.
 *   Penalba, M. et al. (2019). Renewable Energy, 139, 1077–1089.
 *   Ringwood, J.V. et al. (2014). IEEE Control Systems, 34(5), 30–55.
 *
 * @author salhus
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <memory>
#include <numeric>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace wec_dyno_logger
{

/// Sample record for the sliding window buffer.
struct Sample
{
  double t;             ///< Timestamp [s]
  double velocity;      ///< PTO velocity [rad/s]
  double tau_cmd;       ///< Commanded PTO torque [N·m]
  double tau_meas;      ///< Measured PTO torque [N·m]
  double power;         ///< Instantaneous power P = τ_meas · ω [W]
  double wave_elevation;  ///< Wave surface elevation η [m]
};

/**
 * @brief WEC HIL performance metrics logger node.
 */
class WecDynoLoggerNode : public rclcpp::Node
{
public:
  explicit WecDynoLoggerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("wec_dyno_logger_node", options),
    pto_velocity_(0.0),
    tau_cmd_(0.0),
    tau_meas_(0.0),
    wave_elevation_(0.0),
    start_time_(this->now())
  {
    // ── Parameter declarations ────────────────────────────────────────────
    declare_parameter("pto_joint_name", std::string("pto_joint"));
    declare_parameter("averaging_window_s", 30.0);
    declare_parameter("device_diameter_m", 1.0);
    declare_parameter("wave_height_m", 1.0);
    declare_parameter("wave_omega_rad_s", 1.0);
    declare_parameter("publish_rate_hz", 10.0);
    declare_parameter("csv_output_path", std::string(""));

    // ── Read parameters ───────────────────────────────────────────────────
    pto_joint_name_ = get_parameter("pto_joint_name").as_string();
    window_s_ = get_parameter("averaging_window_s").as_double();
    device_diameter_ = get_parameter("device_diameter_m").as_double();
    wave_height_ = get_parameter("wave_height_m").as_double();
    wave_omega_ = get_parameter("wave_omega_rad_s").as_double();
    const double pub_rate = get_parameter("publish_rate_hz").as_double();
    const std::string csv_path = get_parameter("csv_output_path").as_string();

    // ── Open CSV file if requested ────────────────────────────────────────
    if (!csv_path.empty()) {
      csv_file_.open(csv_path);
      if (!csv_file_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_path.c_str());
      } else {
        // Write CSV header.
        csv_file_ << "t_s,velocity_rad_s,tau_cmd_Nm,tau_meas_Nm,"
                  << "inst_power_W,avg_power_W,cwr,papr,tracking_error_Nm,"
                  << "reactive_fraction,wave_elevation_m\n";
        RCLCPP_INFO(get_logger(), "Logging to CSV: %s", csv_path.c_str());
      }
    }

    // ── Publishers (all in node namespace ~/...) ──────────────────────────
    auto qos = rclcpp::QoS(10);
    pub_inst_power_ = create_publisher<std_msgs::msg::Float64>("~/instantaneous_power", qos);
    pub_avg_power_ = create_publisher<std_msgs::msg::Float64>("~/average_power", qos);
    pub_cwr_ = create_publisher<std_msgs::msg::Float64>("~/capture_width_ratio", qos);
    pub_papr_ = create_publisher<std_msgs::msg::Float64>("~/peak_to_average_ratio", qos);
    pub_tracking_err_ = create_publisher<std_msgs::msg::Float64>("~/torque_tracking_error", qos);
    pub_reactive_frac_ = create_publisher<std_msgs::msg::Float64>("~/reactive_power_fraction", qos);

    // ── Subscribers ───────────────────────────────────────────────────────
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == pto_joint_name_) {
            if (i < msg->velocity.size()) {
              pto_velocity_ = msg->velocity[i];
            }
            break;
          }
        }
      });

    sub_tau_cmd_ = create_subscription<std_msgs::msg::Float64>(
      "/pto/commanded_torque", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        tau_cmd_ = msg->data;
      });

    sub_tau_meas_ = create_subscription<std_msgs::msg::Float64>(
      "/pto/measured_torque", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        tau_meas_ = msg->data;
        // Record each measured torque update as a sample for the sliding window.
        record_sample();
      });

    sub_wave_elevation_ = create_subscription<std_msgs::msg::Float64>(
      "/wec/wave_elevation", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        wave_elevation_ = msg->data;
      });

    // ── Publish timer ─────────────────────────────────────────────────────
    const auto period = std::chrono::duration<double>(1.0 / pub_rate);
    publish_timer_ = create_wall_timer(period, std::bind(&WecDynoLoggerNode::publish_metrics, this));

    RCLCPP_INFO(get_logger(), "WecDynoLoggerNode started");
    RCLCPP_INFO(get_logger(), "  PTO joint      : %s", pto_joint_name_.c_str());
    RCLCPP_INFO(get_logger(), "  Window         : %.0f s", window_s_);
    RCLCPP_INFO(get_logger(), "  Device diameter: %.2f m", device_diameter_);
    RCLCPP_INFO(get_logger(), "  Wave height    : %.2f m", wave_height_);
    RCLCPP_INFO(get_logger(), "  Wave ω         : %.3f rad/s", wave_omega_);
    RCLCPP_INFO(get_logger(), "  Publish rate   : %.0f Hz", pub_rate);
  }

private:
  /**
   * @brief Record a new sample into the sliding window buffer and evict old samples.
   */
  void record_sample()
  {
    const double t = (this->now() - start_time_).seconds();
    const double power = tau_meas_ * pto_velocity_;

    window_.push_back({t, pto_velocity_, tau_cmd_, tau_meas_, power, wave_elevation_});

    // Evict samples older than the averaging window.
    while (!window_.empty() && (t - window_.front().t) > window_s_) {
      window_.pop_front();
    }
  }

  /**
   * @brief Timer callback — compute and publish all metrics.
   */
  void publish_metrics()
  {
    if (window_.empty()) {
      return;
    }

    // ── Instantaneous power ───────────────────────────────────────────────
    // Use the most recent sample.
    const double p_inst = window_.back().power;

    // ── Average absorbed power ─────────────────────────────────────────────
    // P_avg = (1/N) Σ P_i  (discrete approximation of time average).
    double p_sum = 0.0;
    for (const auto & s : window_) {
      p_sum += s.power;
    }
    const double p_avg = p_sum / static_cast<double>(window_.size());

    // ── Wave power per unit crest width (Falnes, 2002) ─────────────────────
    // P_wave = (1/8) · ρ · g² · H² · T / (4π)  [W/m]
    // where T = 2π/ω, ρ = 1025 kg/m³, g = 9.81 m/s².
    constexpr double rho = 1025.0;
    constexpr double g = 9.81;
    const double T_wave = 2.0 * M_PI / wave_omega_;
    const double p_wave_per_m = (1.0 / 8.0) * rho * g * g * wave_height_ * wave_height_ *
      T_wave / (4.0 * M_PI);

    // ── Capture width ratio ────────────────────────────────────────────────
    // CWR = P_avg / (P_wave · D)
    // CWR > 1.0 is theoretically possible for point absorbers (Falnes, 2002, §5.5).
    const double p_wave_total = p_wave_per_m * device_diameter_;
    const double cwr = (p_wave_total > 0.0) ? (p_avg / p_wave_total) : 0.0;

    // ── Peak-to-average ratio ──────────────────────────────────────────────
    // PAPR = max|P| / P_avg  (important for PTO sizing).
    double p_peak = 0.0;
    for (const auto & s : window_) {
      p_peak = std::max(p_peak, std::abs(s.power));
    }
    const double papr = (std::abs(p_avg) > 1e-6) ? (p_peak / std::abs(p_avg)) : 0.0;

    // ── Torque tracking error (RMS) ────────────────────────────────────────
    // RMS(τ_cmd − τ_meas) — primary HIL actuator fidelity metric.
    double err_sq_sum = 0.0;
    for (const auto & s : window_) {
      const double err = s.tau_cmd - s.tau_meas;
      err_sq_sum += err * err;
    }
    const double tracking_err_rms = std::sqrt(err_sq_sum / static_cast<double>(window_.size()));

    // ── Reactive power fraction ────────────────────────────────────────────
    // Fraction of samples where P(t) < 0 (power flowing back to wave).
    // High RPF indicates the PTO is frequently acting as a motor → bidirectional
    // power flow. Important for electrical hardware sizing.
    int negative_power_count = 0;
    for (const auto & s : window_) {
      if (s.power < 0.0) {
        ++negative_power_count;
      }
    }
    const double reactive_frac = static_cast<double>(negative_power_count) /
      static_cast<double>(window_.size());

    // ── Publish ───────────────────────────────────────────────────────────
    publish_float(pub_inst_power_, p_inst);
    publish_float(pub_avg_power_, p_avg);
    publish_float(pub_cwr_, cwr);
    publish_float(pub_papr_, papr);
    publish_float(pub_tracking_err_, tracking_err_rms);
    publish_float(pub_reactive_frac_, reactive_frac);

    // ── CSV logging ───────────────────────────────────────────────────────
    if (csv_file_.is_open()) {
      const auto & s = window_.back();
      csv_file_ << s.t << ","
                << s.velocity << ","
                << s.tau_cmd << ","
                << s.tau_meas << ","
                << p_inst << ","
                << p_avg << ","
                << cwr << ","
                << papr << ","
                << tracking_err_rms << ","
                << reactive_frac << ","
                << s.wave_elevation << "\n";
      csv_file_.flush();
    }
  }

  /// Convenience publisher helper.
  void publish_float(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub,
    double value)
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = value;
    pub->publish(msg);
  }

  // ── Member variables ─────────────────────────────────────────────────────

  std::string pto_joint_name_;
  double window_s_;          ///< Averaging window duration [s]
  double device_diameter_;   ///< WEC characteristic dimension D [m] for CWR
  double wave_height_;       ///< Wave height H [m] for wave power calculation
  double wave_omega_;        ///< Wave angular frequency ω [rad/s]

  // Latest values updated by subscriber callbacks
  double pto_velocity_;
  double tau_cmd_;
  double tau_meas_;
  double wave_elevation_;

  // Sliding window sample buffer
  std::deque<Sample> window_;

  // Reference time for elapsed time computation
  rclcpp::Time start_time_;

  // Optional CSV output
  std::ofstream csv_file_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_inst_power_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_avg_power_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cwr_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_papr_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_tracking_err_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_reactive_frac_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_tau_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_tau_meas_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_wave_elevation_;

  // Publish timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace wec_dyno_logger

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wec_dyno_logger::WecDynoLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
