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
 * @file optimal_passive.hpp
 * @brief Frequency-optimal passive damping PTO control strategy.
 *
 * @details
 * Computes the theoretically optimal passive (resistive) damping coefficient
 * B_opt for a known wave frequency ω (Evans, 1976):
 *
 *   B_opt = sqrt( B_rad² + (ω·(m + m_∞) − K_hs/ω)² )
 *
 * and then applies: τ = −B_opt · ω
 *
 * Derivation (Falnes, 2002, §5.3):
 *   The absorbed power from a passive damper is:
 *     P_abs = (1/2) · |V|² · B  (complex amplitude analysis)
 *   where |V|² is the velocity amplitude squared.
 *   This is maximised subject to the WEC equation of motion constraint, yielding B_opt.
 *
 * Key insight:
 *   B_opt depends on the wave angular impedance Z(ω) = B_rad + j·(ω·m_total − K_hs/ω).
 *   Specifically, B_opt = |Z(ω)| = |Im(Z)| when B_rad is known.
 *   At resonance (ω = ω_n = sqrt(K_hs/m_total)), the imaginary part vanishes and
 *   B_opt = B_rad — confirming the intuitive result.
 *
 * Comparison with passive_damping:
 *   - passive_damping uses a fixed B (set by the user).
 *   - optimal_passive computes B_opt from the wave model parameters.
 *   - For irregular waves, B_opt is frequency-dependent; this strategy uses the
 *     dominant wave frequency (single-frequency approximation).
 *
 * References:
 *   Evans, D.V. (1976). A theory for wave-power absorption by oscillating bodies.
 *     J. Fluid Mechanics, 77(1), 1–25.
 *   Falnes, J. (2002). Ocean Waves and Oscillating Systems. Cambridge University Press.
 *
 * @author salhus
 */

#ifndef PTO_CONTROLLER__OPTIMAL_PASSIVE_HPP_
#define PTO_CONTROLLER__OPTIMAL_PASSIVE_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

#include "pto_controller/pto_strategy_base.hpp"

namespace pto_controller
{

/**
 * @brief Frequency-optimal passive damping: τ = −B_opt · ω.
 *
 * B_opt is computed once at construction from the radiation model parameters
 * and the (known) wave frequency.
 */
class OptimalPassive : public PtoStrategyBase
{
public:
  /**
   * @brief Construct and compute the optimal damping coefficient.
   *
   * @param radiation_damping   B_rad [N·m·s/rad] — from radiation analysis or experiment.
   * @param body_mass_kg        m [kg] — dry body mass.
   * @param added_mass_kg       m_∞ [kg] — added mass at infinite frequency.
   * @param hydrostatic_stiff   K_hs [N/m] — hydrostatic restoring stiffness.
   * @param wave_omega          ω [rad/s] — dominant wave angular frequency.
   */
  OptimalPassive(
    double radiation_damping,
    double body_mass_kg,
    double added_mass_kg,
    double hydrostatic_stiff,
    double wave_omega)
  {
    if (wave_omega <= 0.0) {
      throw std::invalid_argument("OptimalPassive: wave_omega must be positive");
    }
    const double m_total = body_mass_kg + added_mass_kg;
    // Imaginary part of mechanical impedance at frequency ω:
    //   Im(Z) = ω·m_total − K_hs/ω
    const double im_z = wave_omega * m_total - hydrostatic_stiff / wave_omega;
    // Optimal damping = magnitude of impedance:
    //   B_opt = sqrt(B_rad² + Im(Z)²)
    b_opt_ = std::sqrt(radiation_damping * radiation_damping + im_z * im_z);
  }

  /**
   * @brief Compute torque: τ = −B_opt · ω.
   *
   * @param velocity  Joint velocity ω [rad/s].
   * @param position  Not used.
   * @param dt        Not used.
   * @return Torque [N·m]. Always resistive.
   */
  double compute_torque(double velocity, double /*position*/, double /*dt*/) override
  {
    return -b_opt_ * velocity;
  }

  std::string name() const override {return "optimal_passive";}

  void reset() override {}  // Stateless.

  /// @brief Return the computed optimal damping coefficient for diagnostics.
  double get_b_opt() const {return b_opt_;}

private:
  double b_opt_;  ///< Optimal damping coefficient [N·m·s/rad]
};

}  // namespace pto_controller

#endif  // PTO_CONTROLLER__OPTIMAL_PASSIVE_HPP_
