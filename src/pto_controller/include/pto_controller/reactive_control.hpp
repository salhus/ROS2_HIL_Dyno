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
 * @file reactive_control.hpp
 * @brief Complex-conjugate (reactive) PTO control strategy.
 *
 * @details
 * Reactive control applies both a damping and a spring torque:
 *
 *   τ = −B · ω − K · x
 *
 * This is the "complex-conjugate" control law (Salter, 1974; Falnes, 2002, §5.4).
 *
 * Physical interpretation:
 *   - The −B·ω term extracts energy (resistive damping).
 *   - The −K·x term modifies the resonant frequency of the WEC, shifting it to
 *     match the wave frequency (reactive stiffness).
 *   - Together, they ensure that the WEC velocity is in phase with the excitation
 *     force, which is the condition for maximum power absorption (Falnes, 2002).
 *
 * Optimal control parameters (regular wave at frequency ω):
 *   B_opt = B_rad                          (absorb exactly as much as radiated)
 *   K_opt = K_hs − ω²·(m + m_∞)           (cancel the reactive impedance)
 *
 * With these values, the absorbed power reaches the theoretical maximum:
 *   P_max = |F_exc|² / (8·B_rad)
 *
 * Bidirectional power flow:
 *   The spring term K·x can be negative (spring adds energy when K > K_hs/ω²),
 *   meaning the PTO motor must temporarily inject energy into the shaft. This is
 *   the key disadvantage of reactive control: it requires a bidirectional PTO
 *   (motor–generator pair or bidirectional hydraulics). On an ODrive, this is
 *   achievable but requires careful current limit management.
 *
 * Caution for HIL testing:
 *   When K is large, the reactive power fraction can be very high. Monitor
 *   ~/reactive_power_fraction from wec_dyno_logger. Set torque_limit_nm in
 *   pto_params.yaml appropriately to protect the hardware.
 *
 * References:
 *   Salter, S.H. (1974). Wave power. Nature, 249(5459), 720–724.
 *   Falnes, J. (2002). Ocean Waves and Oscillating Systems. §5.4.
 *   Ringwood, J.V. et al. (2014). IEEE Control Systems, 34(5), 30–55.
 *
 * @author salhus
 */

#ifndef PTO_CONTROLLER__REACTIVE_CONTROL_HPP_
#define PTO_CONTROLLER__REACTIVE_CONTROL_HPP_

#include <stdexcept>
#include <string>

#include "pto_controller/pto_strategy_base.hpp"

namespace pto_controller
{

/**
 * @brief Complex-conjugate reactive control: τ = −B·ω − K·x.
 *
 * Maximum power absorption strategy for regular waves. Requires bidirectional
 * PTO (the motor must occasionally inject energy into the shaft).
 */
class ReactiveControl : public PtoStrategyBase
{
public:
  /**
   * @brief Construct reactive control with damping and spring coefficients.
   *
   * @param damping_b  Damping coefficient B [N·m·s/rad].
   *                   Set to B_rad for maximum power absorption.
   * @param spring_k   Spring coefficient K [N·m/rad].
   *                   Negative K means the PTO is motoring (injecting power).
   *                   Set to K_hs − ω²·m_total for optimal reactive control.
   */
  ReactiveControl(double damping_b, double spring_k)
  : b_(damping_b), k_(spring_k)
  {
    if (b_ < 0.0) {
      throw std::invalid_argument("ReactiveControl: damping_b must be >= 0");
    }
    // Note: spring_k CAN be negative (reactive/motoring) — this is intentional.
  }

  /**
   * @brief Compute torque: τ = −B·ω − K·x.
   *
   * @param velocity  Joint velocity ω [rad/s].
   * @param position  Joint position x [rad or m].
   * @param dt        Not used (memoryless law).
   * @return Torque [N·m]. Can be positive (motoring) when K·x > B·ω.
   */
  double compute_torque(double velocity, double position, double /*dt*/) override
  {
    return -b_ * velocity - k_ * position;
  }

  std::string name() const override {return "reactive";}

  void reset() override {}  // Stateless.

private:
  double b_;  ///< Damping coefficient B [N·m·s/rad]
  double k_;  ///< Spring coefficient K [N·m/rad] (can be negative)
};

}  // namespace pto_controller

#endif  // PTO_CONTROLLER__REACTIVE_CONTROL_HPP_
