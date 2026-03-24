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
 * @file passive_damping.hpp
 * @brief Passive linear damping PTO control strategy.
 *
 * @details
 * The simplest PTO control law: apply a torque proportional to (and opposing) velocity.
 *
 *   τ = −B · ω
 *
 * where B is the damping coefficient [N·m·s/rad].
 *
 * Physical interpretation:
 *   This emulates a linear mechanical or hydraulic damper with no spring element.
 *   The power extracted is P = τ · ω = −B · ω².
 *   Since ω² ≥ 0 always, power flow is always from wave to PTO (unidirectional).
 *
 * Optimality condition (Evans, 1976; Falnes, 2002 §5.3):
 *   For a single sinusoidal wave of frequency ω, the passive damping coefficient
 *   that maximises average absorbed power is:
 *
 *     B_opt = sqrt( B_rad² + (ω·(m + m_∞) − K_hs/ω)² )
 *
 *   This requires knowing the wave frequency and radiation coefficients. When these
 *   are unknown, a fixed B tuned near B_rad is a practical baseline. Use the
 *   OptimalPassive strategy when the wave frequency is known.
 *
 * References:
 *   Evans, D.V. (1976). J. Fluid Mechanics, 77(1), 1–25.
 *   Falnes, J. (2002). Ocean Waves and Oscillating Systems. Cambridge University Press. §5.3.
 *
 * @author salhus
 */

#ifndef PTO_CONTROLLER__PASSIVE_DAMPING_HPP_
#define PTO_CONTROLLER__PASSIVE_DAMPING_HPP_

#include "pto_controller/pto_strategy_base.hpp"

namespace pto_controller
{

/**
 * @brief Passive linear damping: τ = −B · ω.
 *
 * This is the simplest baseline PTO strategy and the reference point against which
 * all other strategies are compared. For regular waves at the optimal frequency,
 * it achieves at most 50% of the theoretical maximum power absorption.
 */
class PassiveDamping : public PtoStrategyBase
{
public:
  /**
   * @brief Construct with the specified damping coefficient.
   *
   * @param damping_b  Damping coefficient B [N·m·s/rad]. Must be ≥ 0.
   *                   B = 0 → zero torque (free-spinning shaft, no power extraction).
   *                   B → ∞ → locked shaft (no motion, no power extraction).
   *                   B = B_opt → maximum passive power extraction.
   */
  explicit PassiveDamping(double damping_b)
  : b_(damping_b)
  {
    if (b_ < 0.0) {
      throw std::invalid_argument("PassiveDamping: damping_b must be >= 0");
    }
  }

  /**
   * @brief Compute torque: τ = −B · ω.
   *
   * @param velocity  Joint velocity ω [rad/s].
   * @param position  Not used (damping-only law).
   * @param dt        Not used (memoryless law).
   * @return Torque [N·m]. Always opposes velocity (resistive).
   */
  double compute_torque(double velocity, double /*position*/, double /*dt*/) override
  {
    return -b_ * velocity;
  }

  std::string name() const override {return "passive_damping";}

  void reset() override {}  // Stateless — nothing to reset.

private:
  double b_;  ///< Damping coefficient [N·m·s/rad]
};

}  // namespace pto_controller

#endif  // PTO_CONTROLLER__PASSIVE_DAMPING_HPP_
