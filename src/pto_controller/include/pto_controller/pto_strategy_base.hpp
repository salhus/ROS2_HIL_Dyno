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
 * @file pto_strategy_base.hpp
 * @brief Abstract interface for PTO control strategies + factory function.
 *
 * @details
 * This header defines the Strategy pattern interface used by pto_controller_node.
 * All PTO control laws (passive damping, reactive, latching, etc.) derive from
 * PtoStrategyBase and are interchangeable at runtime via the `strategy` parameter.
 *
 * Design rationale (Strategy pattern):
 *   - Separation of concerns: the control node handles ROS communication; strategies
 *     handle the control mathematics. This enables clean unit testing of each strategy.
 *   - Extensibility: adding a new strategy requires only a new header file and a
 *     single line in the factory function. No node changes are needed.
 *   - Runtime reconfigurability: the node can swap strategies by destroying the old
 *     unique_ptr and constructing a new one via the factory.
 *
 * Interface contract:
 *   - compute_torque() is called at the control rate (default 100 Hz). The returned
 *     torque is the desired PTO torque in [N·m]. Positive torque resists positive
 *     velocity (extracting energy). Negative torque adds energy (reactive strategies).
 *   - reset() is called when the strategy is re-initialised (e.g., after a parameter
 *     change). Implementations must clear all internal state variables.
 *   - name() returns a human-readable identifier for logging.
 *
 * Thread safety:
 *   The strategy is accessed only from the control timer callback (single-threaded
 *   executor). Thread safety is not required.
 *
 * References:
 *   Ringwood, J.V., Bacelli, G., & Fusco, F. (2014). IEEE Control Systems, 34(5), 30–55.
 *
 * @author salhus
 */

#ifndef PTO_CONTROLLER__PTO_STRATEGY_BASE_HPP_
#define PTO_CONTROLLER__PTO_STRATEGY_BASE_HPP_

#include <memory>
#include <string>
#include <stdexcept>

namespace pto_controller
{

/**
 * @brief Abstract base class for all PTO control strategies.
 *
 * Implementations compute the desired PTO torque given the current joint state
 * (velocity, position) and the elapsed time since the last control step.
 */
class PtoStrategyBase
{
public:
  /**
   * @brief Compute the desired PTO torque command.
   *
   * @param velocity  Current PTO joint velocity ω [rad/s].
   *                  For a linear heaving WEC connected via a rack-and-pinion,
   *                  this maps to heave velocity via the transmission ratio.
   * @param position  Current PTO joint position x [rad or m depending on joint].
   *                  Used by reactive, latching, and declutching strategies.
   * @param dt        Elapsed time since last control step [s]. Used by strategies
   *                  with internal timers (latching, declutching).
   *
   * @return Commanded PTO torque [N·m]. Sign convention: positive opposes positive
   *         velocity (resistive / energy-extracting). Negative is reactive / motoring.
   */
  virtual double compute_torque(double velocity, double position, double dt) = 0;

  /**
   * @brief Return a human-readable name for this strategy.
   *
   * Used in log messages and the `/pto/active_strategy` topic.
   */
  virtual std::string name() const = 0;

  /**
   * @brief Reset all internal state (timers, latches, integrators, etc.).
   *
   * Called when the strategy is re-initialised. After reset(), the strategy must
   * behave as if freshly constructed.
   */
  virtual void reset() = 0;

  /// Virtual destructor for correct polymorphic cleanup.
  virtual ~PtoStrategyBase() = default;
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Parameters bundle passed to the strategy factory.
 *
 * All strategy-specific parameters are collected here so that the factory
 * function has a single, extensible argument. Parameters not used by a
 * particular strategy are ignored.
 */
struct PtoStrategyParams
{
  // Passive damping / optimal passive
  double damping_coefficient_b{200.0};   ///< Damping B [N·m·s/rad]

  // Optimal passive — additional radiation model parameters
  double radiation_damping{200.0};       ///< B_rad [N·m·s/rad]
  double body_mass_kg{1000.0};           ///< m [kg]
  double added_mass_kg{500.0};           ///< m_∞ [kg]
  double hydrostatic_stiffness{3000.0};  ///< K_hs [N/m]
  double wave_omega{1.0};                ///< ω [rad/s]

  // Reactive control
  double spring_k{0.0};                  ///< Spring coefficient K [N·m/rad]

  // Latching control
  double latch_velocity_threshold{0.05};  ///< |ω| threshold to trigger latch [rad/s]
  double unlatch_delay_s{0.5};            ///< Time to hold latch before releasing [s]
  double max_latch_torque{500.0};         ///< Maximum holding torque [N·m]

  // Declutching control
  double engage_position_threshold{0.1};  ///< |x| threshold to engage damper [rad]
  double engage_duration_s{0.3};          ///< Duration to hold damper engaged [s]
  double damping_when_engaged{300.0};     ///< Damping while engaged [N·m·s/rad]
};

/**
 * @brief Factory function — create a PTO strategy by name.
 *
 * @param strategy_name  One of: "passive_damping", "optimal_passive", "reactive",
 *                       "latching", "declutching".
 * @param params         Strategy parameters bundle.
 *
 * @return Owning pointer to the constructed strategy.
 * @throws std::invalid_argument if strategy_name is not recognised.
 *
 * To register a new strategy:
 *   1. Add a new header under include/pto_controller/
 *   2. Include it in pto_controller_node.cpp (or here, if preferred)
 *   3. Add an `else if (strategy_name == "my_strategy")` branch below
 */
std::unique_ptr<PtoStrategyBase> create_pto_strategy(
  const std::string & strategy_name,
  const PtoStrategyParams & params);

}  // namespace pto_controller

#endif  // PTO_CONTROLLER__PTO_STRATEGY_BASE_HPP_
