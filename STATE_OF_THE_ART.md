# Why This Project Is State of the Art

## What Exists Today

The WEC HIL landscape has roughly three tiers:

### Tier 1 — National Lab / Large University Benches

- **Aalborg University** (Têtu et al., 2018) — dSPACE real-time target, custom hydraulic PTO, closed-source MATLAB/Simulink models
- **Mutriku / IDOM** (Garrido et al., 2015) — Speedgoat target, turbine-based PTO, proprietary control stack
- **Edinburgh / FloWave** (Forehand et al., 2016) — NI PXI real-time, custom wave tank integration
- **Sandia National Lab** (Bacelli et al., 2017) — MATLAB xPC target, linear test bed

These are all **closed-source, proprietary, and expensive** (dSPACE/Speedgoat licenses alone are \$30k–\$100k+). None of them publish their full control software stack. None are reproducible by another research group without buying the same proprietary toolchain.

### Tier 2 — Simulation-Only Control Comparisons

- **Ringwood et al. (2014)** — comprehensive survey, but entirely numerical
- **Penalba & Ringwood (2019)** — wave-to-wire modeling, simulation only
- **Most MPC papers** (Cretel, Li & Belmont, etc.) — validate in MATLAB, never on hardware

These are valuable theory contributions but they cannot capture the real-world effects that matter for PTO design: friction, cogging, bandwidth limits, communication delays.

### Tier 3 — What Doesn't Exist

- An open-source, reproducible WEC HIL bench
- A standardised framework for comparing control strategies on identical physical hardware
- A ROS 2-based anything in the WEC space

---

## Five Specific Novelties of This Project

### 1. First Open-Source WEC HIL Platform

No one has published a complete, reproducible WEC HIL system — hardware bill of materials, control software, simulation coupling, and benchmarking methodology — that another research group can replicate. Every existing bench is a one-off.

This project is fully open-source: ROS 2 nodes, URDF, launch files, control laws, metrics computation. The hardware is commodity (ODrive ~\$250, two motors ~\$100 each). A graduate student with \$500 and a weekend can build this. That is a fundamentally different accessibility level than a dSPACE bench.

**Open-source research infrastructure papers are highly cited** (see: ROS itself, Gazebo, Drake, MoveIt). This project follows that model for the marine energy domain.

### 2. Closed-Loop HIL with Measured Torque Feedback

The critical technical contribution is the **bidirectional coupling** where the simulation sees the real motor's torque output, not the idealized command:

```
Chrono computes:  m·ẍ = F_exc(t) + F_rad(ẋ) + F_hs(x) + τ_pto_measured
                                                           ↑
                                             not τ_commanded, not -B·ω
                                             the actual motor output including
                                             friction, cogging, saturation,
                                             CAN latency, current loop bandwidth
```

The gap between `τ_commanded` and `τ_measured` is **the entire point of HIL**. For passive damping, this gap is small. For reactive control (bidirectional power flow), latching (instantaneous torque transitions), and MPC (aggressive optimal trajectories), the gap becomes significant and directly affects absorbed power.

No existing open-source WEC tool captures this. WEC-Sim, Capytaine, NEMOH — they all assume the PTO delivers exactly what the control law requests.

### 3. Fair, Standardised Control Strategy Comparison on Identical Hardware

Existing literature compares WEC control strategies in simulation, where each paper uses different wave conditions, different body geometries, different hydrodynamic codes, and different performance metrics. It is nearly impossible to draw fair conclusions across papers.

This project enforces apples-to-apples:

- **Same hardware** — identical motors, identical shaft, identical ODrive, identical CAN bus
- **Same wave input** — same Chrono model, same excitation force, same wave spectrum
- **Same metrics** — standardised computation of power, CWR, peak-to-average, reactive fraction, torque tracking error
- **Same code** — switching between strategies is a single parameter change

An example results table from this framework would look like:

| Strategy | P\_avg (W) | CWR | P\_peak / P\_avg | Reactive Fraction | RMS Torque Error (N·m) |
|---|---|---|---|---|---|
| Passive | 2.1 | 0.18 | 3.2 | 0% | 0.02 |
| Optimal passive | 3.4 | 0.29 | 3.8 | 0% | 0.03 |
| Reactive | 5.1 | 0.44 | 8.7 | 34% | 0.15 |
| Latching | 4.8 | 0.41 | 12.1 | 0% | 0.41 |
| Declutching | 3.9 | 0.33 | 6.2 | 0% | 0.08 |

The last column — **torque tracking error** — is the novel insight. It quantifies which control strategies are *actually realisable* on physical hardware versus which ones look good in simulation but fall apart when the motor cannot track the commanded torque. Nobody has published this comparison on real hardware with open-source tools.

### 4. Torque Tracking Fidelity as a PTO Design Metric

This is potentially the most interesting academic contribution. The existing literature treats the PTO as ideal — you command a torque, you get it. This project reveals the frequency-dependent tracking bandwidth of a real electric PTO:

- At low frequencies (slow wave motion), all strategies track well
- At high frequencies (latching transients, MPC aggressive trajectories), the motor physically cannot follow
- The **cross-spectral analysis** between `τ_cmd` and `τ_meas` gives a transfer function of the real PTO actuator

This has direct engineering implications: it tells WEC developers what actuator bandwidth they actually need for each control strategy, grounded in measurement rather than assumption. This is a result that simulation cannot provide.

### 5. ROS 2 + Project Chrono Integration for Marine Energy

ROS 2 is standard in robotics but virtually unused in marine energy. Project Chrono (via HydroChrono/SeaStack) is gaining traction for WEC simulation but has no real-time hardware coupling. This project bridges these two ecosystems.

This positions the work at the intersection of:

- **Marine renewable energy** — the application domain
- **Robotics middleware** — the software infrastructure
- **Real-time simulation** — the numerical method
- **Hardware-in-the-Loop testing** — the validation methodology

That intersection is empty right now. This project occupies a unique niche.

---

## Summary

| Dimension | Existing State of the Art | This Project's Advance |
|---|---|---|
| **Accessibility** | Proprietary (\$50k+ toolchains) | Open-source, \$500 hardware |
| **Reproducibility** | One-off benches, closed code | Fully public, anyone can replicate |
| **Control comparison** | Simulation-only, inconsistent conditions | Same physical hardware, standardised metrics |
| **PTO fidelity** | Assumed ideal | Measured, frequency-resolved tracking error |
| **Software stack** | MATLAB / dSPACE / Speedgoat | ROS 2 + Project Chrono (modern, extensible) |
| **Coupling** | Open-loop or proprietary closed-loop | Bidirectional HIL with measured torque feedback |

---

## Relationship to Phase 1

<a href="https://github.com/salhus/hil_odrive_ros2_control">Phase 1</a> builds the passive dyno — a standard motor test bench with a sine-driven motor and a passive resistive load. This is **not** state of the art by itself. It serves three purposes:

1. **Validated hardware** — confirms both ODrive axes work, CAN is stable, velocity PID tracks cleanly, and shaft coupling is sound before trusting any HIL results.
2. **Baseline measurement** — passive damping on a regular wave has a known analytical solution (Falnes, 2002). The HIL result can be compared against this baseline to quantify what the closed-loop simulation coupling adds.
3. **De-risked hardware** — if something goes wrong in Phase 2, you can isolate hardware problems from software/control problems.

Phase 1 is scaffolding. Phase 2 — this repository — is the research contribution.

---

## Target Publication Venues

- **EWTEC** — European Wave and Tidal Energy Conference
- **Renewable Energy** (Elsevier)
- **IEEE Transactions on Sustainable Energy**
- **OMAE** — International Conference on Offshore Mechanics and Arctic Engineering

---

## References

- Bacelli, G., Coe, R.G., Patterson, D., & Wilson, D. (2017). System identification of a wave energy converter. *Proc. 12th European Wave and Tidal Energy Conference (EWTEC)*.
- Babarit, A. & Clément, A.H. (2006). Optimal latching control of a wave energy device in regular and irregular waves. *Applied Ocean Research*, 28(2), 77–91.
- Babarit, A., Guglielmi, M., & Clément, A.H. (2009). Declutching control of a wave energy converter. *Ocean Engineering*, 36(12–13), 1015–1024.
- Cretel, J.A.M., Lightbody, G., Thomas, G.P., & Lewis, A.W. (2011). Maximisation of energy capture by a wave-energy point absorber using model predictive control. *Proc. 18th IFAC World Congress*.
- Evans, D.V. (1976). A theory for wave-power absorption by oscillating bodies. *Journal of Fluid Mechanics*, 77(1), 1–25.
- Falnes, J. (2002). *Ocean Waves and Oscillating Systems: Linear Interactions Including Wave-Energy Extraction*. Cambridge University Press.
- Forehand, D.I.M., Kiprakis, A.E., Nambiar, A.J., & Wallace, A.R. (2016). A fully coupled wave-to-wire model of an array of wave energy converters. *IEEE Transactions on Sustainable Energy*, 7(1), 118–128.
- Garrido, A.J., Garrido, I., Amundarain, M., Alberdi, M., & De la Sen, M. (2015). Sliding-mode control of wave power generation plants. *IEEE Transactions on Industry Applications*, 48(6), 2372–2381.
- Li, G. & Belmont, M.R. (2014). Model predictive control of sea wave energy converters — Part I: A convex approach for the case of a single device. *Renewable Energy*, 69, 453–463.
- Penalba, M., Sell, N.P., Ringwood, J.V., & Hillis, A.J. (2019). Validating a wave-to-wire model for a wave energy converter — Part I: The hydraulic transmission system. *Renewable Energy*, 139, 1077–1089.
- Penalba, M. & Ringwood, J.V. (2019). A review of wave-to-wire models for wave energy converters. *Energies*, 9(7), 506.
- Ringwood, J.V., Bacelli, G., & Fusco, F. (2014). Energy-maximizing control of wave-energy converters: The development of control system technology to optimize their operation. *IEEE Control Systems Magazine*, 34(5), 30–55.
- Têtu, A., Ferri, F., Kramer, M.B., & Todalshaug, J.H. (2018). Physical and mathematical modeling of a wave energy converter equipped with a negative spring mechanism for phase control. *Energies*, 11(10), 2595.
