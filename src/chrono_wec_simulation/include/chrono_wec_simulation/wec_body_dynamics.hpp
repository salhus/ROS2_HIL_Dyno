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
 * @file wec_body_dynamics.hpp
 * @brief Abstract dynamics model interface and LinearWecDynamics placeholder implementation.
 *
 * @details
 * This header defines the swappable dynamics interface used by chrono_wec_simulation_node.
 * The design follows the Strategy pattern: the node holds a pointer to WecBodyDynamics and
 * calls step() / get_velocity() etc., without knowing the concrete implementation.
 *
 * Two implementations are provided:
 *
 *   1. WecBodyDynamics  — abstract base class (the interface contract)
 *   2. LinearWecDynamics — 1-DOF linearised heaving buoy (placeholder until HydroChrono)
 *
 * The placeholder model implements the standard linearised WEC equation of motion
 * (Falnes, 2002, eq. 5.3):
 *
 *   m_total · ẍ = F_exc(t) − B_rad · ẋ − K_hs · x + τ_pto
 *
 * where:
 *   m_total = body mass + added mass at infinite frequency (Falnes, 2002, eq. 5.15)
 *   F_exc(t) = excitation force: F̂_exc · sin(ω_wave · t)  [N]
 *   B_rad    = radiation damping coefficient               [N·s/m]
 *   K_hs     = hydrostatic restoring stiffness             [N/m]
 *   τ_pto    = measured PTO torque (HIL feedback)          [N·m]
 *
 * Linearisation assumptions (valid for small amplitude oscillations, kA ≪ 1):
 *   - The radiation force is approximated as a memoryless damper: F_rad = -B_rad · ẋ.
 *     This omits the Cummins (1962) convolution integral that captures the frequency-dependent
 *     radiation memory. The full HydroChrono node will restore this via a state-space
 *     approximation (Taghipour et al., 2008).
 *   - Excitation is a single sinusoidal component (regular wave). Irregular sea states require
 *     superposition of frequency components or an IFFT approach.
 *   - Hydrostatic restoring is linear: F_hs = -K_hs · x (valid for small heave displacement).
 *
 * Integration scheme: 4th-order Runge-Kutta (RK4).
 *   RK4 achieves O(h⁵) local truncation error with only 4 function evaluations per step.
 *   This is the standard choice for WEC time-domain simulation (Penalba et al., 2019) because:
 *     a) The oscillatory dynamics require tight integration to avoid phase drift.
 *     b) The method is explicit (no solver iteration required), enabling real-time operation.
 *
 * Replacement path:
 *   When HydroChrono/SeaStack is available, create a class:
 *     class HydroChronoDynamics : public WecBodyDynamics { ... }
 *   and pass it to the node constructor. Zero changes to the node itself are needed.
 *
 * References:
 *   Cummins, W.E. (1962). The impulse response function and ship motions. Schiffstechnik, 9.
 *   Falnes, J. (2002). Ocean Waves and Oscillating Systems. Cambridge University Press.
 *   Penalba, M. et al. (2019). Renewable Energy, 139, 1077–1089.
 *   Taghipour, R. et al. (2008). Ocean Engineering, 35(7), 685–705.
 *
 * @author salhus
 */

#ifndef CHRONO_WEC_SIMULATION__WEC_BODY_DYNAMICS_HPP_
#define CHRONO_WEC_SIMULATION__WEC_BODY_DYNAMICS_HPP_

#include <cmath>
#include <stdexcept>

namespace chrono_wec_simulation
{

/**
 * @brief Abstract interface for a WEC body dynamics model.
 *
 * All dynamics implementations (placeholder linear model, HydroChrono, etc.) must
 * satisfy this interface. The chrono_wec_simulation_node depends only on this interface,
 * enabling seamless swap of the underlying physics engine without any node changes.
 */
class WecBodyDynamics
{
public:
  /**
   * @brief Advance the dynamics model by one timestep.
   *
   * @param dt         Timestep size [s]. Must be > 0. Typically 1 ms (1000 Hz).
   * @param pto_torque PTO torque feedback from the physical motor [N·m].
   *                   Positive torque is in the direction of positive velocity.
   *                   This is the measured (actual) torque, not the commanded value —
   *                   the difference is the HIL signal of interest.
   */
  virtual void step(double dt, double pto_torque) = 0;

  /**
   * @brief Get the current body velocity ẋ [m/s].
   *
   * This is the heave velocity of the WEC body relative to the sea floor.
   * Published as /wec/velocity_command to drive the hydro emulator motor.
   */
  virtual double get_velocity() const = 0;

  /**
   * @brief Get the current body position x [m].
   *
   * Positive direction is upward (heave). Published as /wec/position_command
   * for PTO strategies that require position information (e.g., reactive control,
   * latching, declutching).
   */
  virtual double get_position() const = 0;

  /**
   * @brief Compute the wave surface elevation η(t) at the given simulation time [m].
   *
   * For the linear placeholder: η(t) = (A/2) · sin(ω·t), where A is wave height.
   * This is published as /wec/wave_elevation for the logger's wave power computation.
   *
   * @param t  Current simulation time [s].
   */
  virtual double get_wave_elevation(double t) const = 0;

  /**
   * @brief Compute the wave excitation force F_exc(t) at the given simulation time [N].
   *
   * @param t  Current simulation time [s].
   */
  virtual double get_excitation_force(double t) const = 0;

  /// Virtual destructor — required for correct polymorphic cleanup.
  virtual ~WecBodyDynamics() = default;
};

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Linearised 1-DOF heaving buoy dynamics model.
 *
 * Implements the placeholder WEC model:
 *
 *   m_total · ẍ = F̂_exc · sin(ω·t) − B_rad · ẋ − K_hs · x + τ_pto
 *
 * State vector: [x, ẋ]
 * Integrator:   4th-order Runge-Kutta
 *
 * This class is designed to be straightforward to validate against the known
 * analytical solution for a damped harmonic oscillator driven at resonance:
 *
 *   At resonance (ω = sqrt(K_hs / m_total)):
 *   x_ss(t) ≈ (F̂_exc / (B_rad · ω)) · cos(ω · t)    [if τ_pto ≈ 0]
 *
 * This provides a built-in sanity check for the integration accuracy.
 */
class LinearWecDynamics : public WecBodyDynamics
{
public:
  /**
   * @brief Construct the linear WEC dynamics model with physical parameters.
   *
   * @param mass_kg                Body mass [kg]. Physical mass of the WEC float.
   * @param added_mass_inf_kg      Added mass at infinite frequency m_∞ [kg].
   *                               Falnes (2002): m_total = m + m_∞.
   * @param radiation_damping      Radiation damping B_rad [N·s/m]. At resonance,
   *                               optimal passive damping equals B_rad (Evans, 1976).
   * @param hydrostatic_stiffness  Hydrostatic stiffness K_hs [N/m]. For a heaving
   *                               cylinder: K_hs = ρ · g · A_wp, where A_wp is the
   *                               waterplane area.
   * @param excitation_amplitude   Excitation force amplitude F̂_exc [N].
   * @param wave_omega             Wave angular frequency ω [rad/s].
   * @param wave_height_m          Wave height H [m], used to compute η(t) = H/2 · sin(ω·t).
   *
   * @throws std::invalid_argument if any physical parameter is non-positive.
   */
  LinearWecDynamics(
    double mass_kg,
    double added_mass_inf_kg,
    double radiation_damping,
    double hydrostatic_stiffness,
    double excitation_amplitude,
    double wave_omega,
    double wave_height_m)
  : m_total_(mass_kg + added_mass_inf_kg),
    b_rad_(radiation_damping),
    k_hs_(hydrostatic_stiffness),
    f_exc_amp_(excitation_amplitude),
    omega_(wave_omega),
    wave_height_(wave_height_m),
    x_(0.0),
    x_dot_(0.0),
    t_(0.0)
  {
    if (m_total_ <= 0.0) {
      throw std::invalid_argument("Total mass must be positive");
    }
    if (b_rad_ < 0.0) {
      throw std::invalid_argument("Radiation damping must be non-negative");
    }
    if (k_hs_ <= 0.0) {
      throw std::invalid_argument("Hydrostatic stiffness must be positive");
    }
    if (omega_ <= 0.0) {
      throw std::invalid_argument("Wave angular frequency must be positive");
    }
  }

  /**
   * @brief Advance the dynamics by dt seconds using RK4 integration.
   *
   * RK4 evaluates the derivative function f(t, [x, ẋ]) four times per step:
   *   k1 = f(t,         [x,         ẋ        ])
   *   k2 = f(t + h/2,   [x + k1·h/2, ẋ + k1·h/2])
   *   k3 = f(t + h/2,   [x + k2·h/2, ẋ + k2·h/2])
   *   k4 = f(t + h,     [x + k3·h,   ẋ + k3·h  ])
   *   [x, ẋ]_{n+1} = [x, ẋ]_n + (h/6)·(k1 + 2·k2 + 2·k3 + k4)
   *
   * The derivative function (state-space form):
   *   d/dt [x]   = [ẋ                                                          ]
   *   d/dt [ẋ]  = [(F̂·sin(ω·t) − B_rad·ẋ − K_hs·x + τ_pto) / m_total       ]
   *
   * @param dt         Timestep [s].
   * @param pto_torque Measured PTO torque [N·m] (held constant across RK4 sub-steps).
   */
  void step(double dt, double pto_torque) override
  {
    // RK4 integration. The PTO torque is treated as a zero-order hold across the step
    // (constant during the dt interval), which is valid for typical dt = 1 ms and
    // CAN update rates of ~1 kHz.
    auto deriv = [this, pto_torque](double t, double x, double xd) -> std::pair<double, double> {
        const double f_exc = f_exc_amp_ * std::sin(omega_ * t);
        const double xdd = (f_exc - b_rad_ * xd - k_hs_ * x + pto_torque) / m_total_;
        return {xd, xdd};
      };

    auto [k1_x, k1_v] = deriv(t_, x_, x_dot_);
    auto [k2_x, k2_v] = deriv(t_ + dt / 2.0, x_ + k1_x * dt / 2.0, x_dot_ + k1_v * dt / 2.0);
    auto [k3_x, k3_v] = deriv(t_ + dt / 2.0, x_ + k2_x * dt / 2.0, x_dot_ + k2_v * dt / 2.0);
    auto [k4_x, k4_v] = deriv(t_ + dt, x_ + k3_x * dt, x_dot_ + k3_v * dt);

    x_ += (dt / 6.0) * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x);
    x_dot_ += (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
    t_ += dt;
  }

  /// @brief Current heave position x [m].
  double get_position() const override {return x_;}

  /// @brief Current heave velocity ẋ [m/s].
  double get_velocity() const override {return x_dot_;}

  /**
   * @brief Wave surface elevation η(t) = (H/2) · sin(ω·t) [m].
   *
   * This uses the small-amplitude (linear) wave theory assumption. The wave amplitude
   * A = H/2 where H is the significant wave height.
   */
  double get_wave_elevation(double t) const override
  {
    return (wave_height_ / 2.0) * std::sin(omega_ * t);
  }

  /// @brief Wave excitation force F_exc(t) = F̂ · sin(ω·t) [N].
  double get_excitation_force(double t) const override
  {
    return f_exc_amp_ * std::sin(omega_ * t);
  }

  /// @brief Current simulation time [s].
  double get_time() const {return t_;}

private:
  // Physical parameters — set at construction, immutable thereafter.
  double m_total_;    ///< Total mass m + m_∞ [kg]
  double b_rad_;      ///< Radiation damping coefficient [N·s/m]
  double k_hs_;       ///< Hydrostatic stiffness [N/m]
  double f_exc_amp_;  ///< Excitation force amplitude [N]
  double omega_;      ///< Wave angular frequency [rad/s]
  double wave_height_;  ///< Wave height H [m]

  // State variables — updated by step().
  double x_;      ///< Heave position [m]
  double x_dot_;  ///< Heave velocity [m/s]
  double t_;      ///< Simulation time [s]
};

}  // namespace chrono_wec_simulation

#endif  // CHRONO_WEC_SIMULATION__WEC_BODY_DYNAMICS_HPP_
