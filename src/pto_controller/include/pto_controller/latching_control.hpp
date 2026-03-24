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
 * @file latching_control.hpp
 * @brief Latching (phase-control) PTO control strategy.
 *
 * @details
 * Latching control periodically locks the WEC shaft at velocity zero-crossings,
 * holding it stationary until the wave excitation force is in phase with the
 * (would-be) velocity. This phase-shifts the body response to align with the
 * excitation force, increasing energy absorption without requiring reactive power.
 *
 * Algorithm (Budal & Falnes, 1980):
 *   - Monitor the PTO joint velocity ω.
 *   - When |ω| drops below a threshold near zero (indicating a velocity zero-crossing),
 *     LATCH: apply a large holding torque τ_hold = −sign(ω) · τ_max to lock the shaft.
 *   - After a calculated unlatch delay τ_delay, RELEASE: return to free-damping mode.
 *   - In FREE mode, apply passive damping: τ = −B_free · ω.
 *
 * Unlatch delay optimisation (Babarit & Clément, 2006):
 *   The optimal unlatch delay positions the release such that the wave excitation
 *   force is maximum at the instant of release. For a regular wave of frequency ω:
 *     τ_delay_opt = T_wave/4 − t_latch_duration
 *   This can be tuned empirically on the bench.
 *
 * State machine:
 *   FREE → LATCHED: when |ω| < latch_velocity_threshold
 *   LATCHED → FREE: when latch_timer >= unlatch_delay_s
 *
 * Advantages over reactive control:
 *   - No negative power flow — the PTO only extracts energy.
 *   - Simpler hardware requirements (unidirectional power electronics).
 *   - Near-optimal for regular waves.
 *
 * Disadvantages:
 *   - The holding torque can be very large (equivalent to locking the shaft rigidly).
 *     This places mechanical stress on the drivetrain. Tune max_latch_torque carefully.
 *   - Performance degrades for irregular waves (sub-optimal unlatch timing).
 *   - Requires reliable velocity zero-crossing detection.
 *
 * References:
 *   Budal, K., & Falnes, J. (1980). Interacting point absorbers with controlled motion.
 *     In B. Count (Ed.), Power from Sea Waves (pp. 381–399). Academic Press.
 *   Babarit, A., & Clément, A.H. (2006). Optimal latching control of a wave energy
 *     device in regular and irregular waves. Applied Ocean Research, 28(2), 77–91.
 *
 * @author salhus
 */

#ifndef PTO_CONTROLLER__LATCHING_CONTROL_HPP_
#define PTO_CONTROLLER__LATCHING_CONTROL_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

#include "pto_controller/pto_strategy_base.hpp"

namespace pto_controller
{

/**
 * @brief Latching phase-control strategy.
 *
 * Implements a two-state (FREE / LATCHED) controller that locks the shaft at
 * velocity zero-crossings and releases after an optimised delay.
 */
class LatchingControl : public PtoStrategyBase
{
public:
  /// State machine states.
  enum class LatchState
  {
    FREE,    ///< Shaft is free; passive damping is applied.
    LATCHED  ///< Shaft is locked; holding torque is applied.
  };

  /**
   * @brief Construct latching control.
   *
   * @param latch_velocity_threshold  |ω| below which latching is triggered [rad/s].
   *                                  Too high → premature latching; too low → missed crossings.
   *                                  Typical: 5–10% of peak velocity.
   * @param unlatch_delay_s           Time to hold the latch before releasing [s].
   *                                  For regular waves at ω: unlatch_delay ≈ T/4.
   * @param max_latch_torque          Maximum holding torque [N·m].
   *                                  Must exceed the peak wave excitation torque on the shaft.
   * @param free_damping_b            Passive damping applied during FREE phase [N·m·s/rad].
   */
  LatchingControl(
    double latch_velocity_threshold,
    double unlatch_delay_s,
    double max_latch_torque,
    double free_damping_b = 0.0)
  : vel_threshold_(latch_velocity_threshold),
    unlatch_delay_(unlatch_delay_s),
    max_torque_(max_latch_torque),
    b_free_(free_damping_b),
    state_(LatchState::FREE),
    latch_timer_(0.0),
    prev_velocity_(0.0)
  {
    if (vel_threshold_ <= 0.0) {
      throw std::invalid_argument("LatchingControl: latch_velocity_threshold must be positive");
    }
    if (unlatch_delay_ < 0.0) {
      throw std::invalid_argument("LatchingControl: unlatch_delay_s must be non-negative");
    }
    if (max_torque_ <= 0.0) {
      throw std::invalid_argument("LatchingControl: max_latch_torque must be positive");
    }
  }

  /**
   * @brief Compute torque using the latching state machine.
   *
   * @param velocity  Joint velocity [rad/s].
   * @param position  Not used in base latching (available for extensions).
   * @param dt        Elapsed time since last call [s] (advances latch timer).
   * @return Torque [N·m].
   */
  double compute_torque(double velocity, double /*position*/, double dt) override
  {
    double torque = 0.0;

    switch (state_) {
      case LatchState::FREE:
        // Apply passive damping in free phase.
        torque = -b_free_ * velocity;
        // Detect zero-crossing: latch when |ω| falls below threshold.
        if (std::abs(velocity) < vel_threshold_) {
          state_ = LatchState::LATCHED;
          latch_timer_ = 0.0;
        }
        break;

      case LatchState::LATCHED:
        // Apply large holding torque opposing the last known direction of motion.
        // The sign of prev_velocity_ holds the direction at the moment of latching.
        torque = -max_torque_ * (prev_velocity_ >= 0.0 ? 1.0 : -1.0);
        latch_timer_ += dt;
        // Release when the unlatch delay has elapsed.
        if (latch_timer_ >= unlatch_delay_) {
          state_ = LatchState::FREE;
          latch_timer_ = 0.0;
        }
        break;
    }

    prev_velocity_ = velocity;
    return torque;
  }

  std::string name() const override {return "latching";}

  void reset() override
  {
    state_ = LatchState::FREE;
    latch_timer_ = 0.0;
    prev_velocity_ = 0.0;
  }

  /// @brief Return current latch state for diagnostics.
  LatchState get_state() const {return state_;}

private:
  double vel_threshold_;  ///< Velocity threshold to trigger latching [rad/s]
  double unlatch_delay_;  ///< Duration to hold latch before releasing [s]
  double max_torque_;     ///< Maximum holding torque [N·m]
  double b_free_;         ///< Passive damping during FREE phase [N·m·s/rad]

  LatchState state_;      ///< Current state machine state
  double latch_timer_;    ///< Time elapsed since latching [s]
  double prev_velocity_;  ///< Velocity at the time of last call (for sign detection)
};

}  // namespace pto_controller

#endif  // PTO_CONTROLLER__LATCHING_CONTROL_HPP_
