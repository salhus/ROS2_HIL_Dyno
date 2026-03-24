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
 * @file declutching_control.hpp
 * @brief Declutching (phase-control) PTO control strategy.
 *
 * @details
 * Declutching is conceptually the dual of latching: instead of locking the shaft
 * near zero velocity, the shaft is periodically disconnected ("declutched") from
 * the PTO damper, allowing it to oscillate freely. The damper is engaged only
 * during the phase when the wave excitation force is aligned with the velocity,
 * maximising the product P = F · v.
 *
 * Algorithm (Babarit et al., 2009):
 *   - The shaft cycles between two modes:
 *     DECLUTCHED: damper is disconnected; shaft oscillates freely under wave excitation.
 *                 τ = 0 (no torque).
 *     ENGAGED:    damper applies passive force; τ = −B · ω.
 *   - Engagement timing: engage when |x| exceeds a threshold (near position extremes,
 *     where velocity is decreasing and the body is decelerating into the damper load).
 *     Alternatively, a timer-based engagement cycle can be used.
 *   - Duration: remain engaged for engage_duration_s, then declutch again.
 *
 * Comparison with latching:
 *   - Latching locks the shaft (applies large opposing torque near zero velocity).
 *   - Declutching frees the shaft (applies zero torque during part of the cycle).
 *   - Both achieve phase alignment with the excitation force.
 *   - Declutching requires a simpler actuator (on/off clutch), while latching
 *     requires a brake/lock mechanism.
 *   - Neither requires bidirectional power flow (unlike reactive control).
 *
 * State machine:
 *   DECLUTCHED → ENGAGED: when |x| >= engage_position_threshold
 *   ENGAGED → DECLUTCHED: when engage_timer >= engage_duration_s
 *
 * References:
 *   Babarit, A., Guglielmi, M., & Clément, A.H. (2009). Declutching control of a
 *     wave energy converter. Ocean Engineering, 36(12–13), 1015–1024.
 *
 * @author salhus
 */

#ifndef PTO_CONTROLLER__DECLUTCHING_CONTROL_HPP_
#define PTO_CONTROLLER__DECLUTCHING_CONTROL_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

#include "pto_controller/pto_strategy_base.hpp"

namespace pto_controller
{

/**
 * @brief Declutching (free-shaft) phase-control PTO strategy.
 *
 * Alternates between free-shaft (no torque) and engaged-damper phases to align
 * energy extraction with the wave excitation force peaks.
 */
class DeclutchingControl : public PtoStrategyBase
{
public:
  /// State machine states.
  enum class DeclutchState
  {
    DECLUTCHED,  ///< Shaft is free; no PTO torque applied.
    ENGAGED      ///< Damper is engaged; τ = −B · ω.
  };

  /**
   * @brief Construct declutching control.
   *
   * @param engage_position_threshold  |x| above which the damper engages [rad or m].
   *                                   Tune to the expected position amplitude.
   * @param engage_duration_s          How long the damper remains engaged [s].
   *                                   Typically T/4 for regular waves.
   * @param damping_when_engaged       Damping applied during ENGAGED phase [N·m·s/rad].
   *                                   Can be set to B_opt for maximum extraction.
   */
  DeclutchingControl(
    double engage_position_threshold,
    double engage_duration_s,
    double damping_when_engaged)
  : pos_threshold_(engage_position_threshold),
    engage_duration_(engage_duration_s),
    b_engaged_(damping_when_engaged),
    state_(DeclutchState::DECLUTCHED),
    engage_timer_(0.0)
  {
    if (pos_threshold_ <= 0.0) {
      throw std::invalid_argument(
        "DeclutchingControl: engage_position_threshold must be positive");
    }
    if (engage_duration_ <= 0.0) {
      throw std::invalid_argument("DeclutchingControl: engage_duration_s must be positive");
    }
    if (b_engaged_ < 0.0) {
      throw std::invalid_argument("DeclutchingControl: damping_when_engaged must be >= 0");
    }
  }

  /**
   * @brief Compute torque using the declutching state machine.
   *
   * @param velocity  Joint velocity [rad/s].
   * @param position  Joint position [rad or m] (used to trigger engagement).
   * @param dt        Elapsed time since last call [s] (advances engage timer).
   * @return Torque [N·m]. Zero during DECLUTCHED phase.
   */
  double compute_torque(double velocity, double position, double dt) override
  {
    double torque = 0.0;

    switch (state_) {
      case DeclutchState::DECLUTCHED:
        // No torque — shaft is free.
        torque = 0.0;
        // Engage when the body position exceeds the threshold.
        // This corresponds to the body being near its displacement extremes,
        // where the wave is doing maximum work on the body.
        if (std::abs(position) >= pos_threshold_) {
          state_ = DeclutchState::ENGAGED;
          engage_timer_ = 0.0;
        }
        break;

      case DeclutchState::ENGAGED:
        // Apply passive damping.
        torque = -b_engaged_ * velocity;
        engage_timer_ += dt;
        // Declutch after the engage duration has elapsed.
        if (engage_timer_ >= engage_duration_) {
          state_ = DeclutchState::DECLUTCHED;
          engage_timer_ = 0.0;
        }
        break;
    }

    return torque;
  }

  std::string name() const override {return "declutching";}

  void reset() override
  {
    state_ = DeclutchState::DECLUTCHED;
    engage_timer_ = 0.0;
  }

  /// @brief Return current declutch state for diagnostics.
  DeclutchState get_state() const {return state_;}

private:
  double pos_threshold_;  ///< Position threshold to trigger engagement [rad or m]
  double engage_duration_;  ///< Duration to stay engaged [s]
  double b_engaged_;      ///< Damping coefficient during ENGAGED phase [N·m·s/rad]

  DeclutchState state_;   ///< Current state
  double engage_timer_;   ///< Time elapsed since last engagement [s]
};

}  // namespace pto_controller

#endif  // PTO_CONTROLLER__DECLUTCHING_CONTROL_HPP_
