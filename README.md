# ROS2 HIL Dynamometer — Phase 2: WEC Hardware-in-the-Loop Dynamometer with Pluggable PTO Control Framework

![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![License](https://img.shields.io/badge/license-Apache--2.0-green)
![Status](https://img.shields.io/badge/status-scaffolding-yellow)

---

## Table of Contents

1. [Problem Statement & Motivation](#1-problem-statement--motivation)
2. [System Architecture](#2-system-architecture)
3. [The HIL Feedback Loop](#3-the-hil-feedback-loop--why-this-is-real-hil)
4. [Placeholder Dynamics Model](#4-placeholder-dynamics-model)
5. [PTO Control Strategies](#5-pto-control-strategies)
6. [Performance Metrics](#6-performance-metrics)
7. [Repository Structure](#7-repository-structure)
8. [Build & Run Instructions](#8-build--run-instructions)
9. [Deployment Modes](#9-deployment-modes)
10. [Network & DDS Configuration](#10-network--dds-configuration)
11. [Latency Considerations](#11-latency-considerations)
12. [Phase 1 Reference](#12-phase-1-reference)
13. [References](#13-references)
14. [License](#14-license)

---

## 1. Problem Statement & Motivation

Numerical simulation remains the dominant method for evaluating Wave Energy Converter (WEC) control
strategies (Ringwood et al., 2014). While computationally convenient, simulation alone cannot capture
the nonlinear, rate-dependent, and stochastic phenomena inherent in physical power take-off (PTO)
systems — cogging torque, friction, driver bandwidth limitations, communication latency, and thermal
effects (Bacelli et al., 2017). Hardware-in-the-Loop (HIL) testing bridges this gap by coupling a
real-time numerical wave-body model to a physical PTO drivetrain, enabling high-fidelity evaluation
of control algorithms before open-water deployment.

Existing WEC HIL benches rely on proprietary real-time targets (dSPACE, Speedgoat, NI PXI) and
closed-source software stacks:

- **Aalborg University** — Têtu et al. (2018): Wavestar-inspired bench using dSPACE
- **Mutriku / IDOM** — Garrido et al. (2015): OWC column with NI PXI real-time target
- **Edinburgh / FloWave** — Forehand et al. (2016): Programmable wave flume with Speedgoat

There is no open-source, reproducible WEC HIL platform available to the research community. This
project provides one — built on **ROS 2 (Jazzy)**, **Project Chrono** (via HydroChrono/SeaStack),
and **commodity ODrive motor controllers**.

---

## 2. System Architecture

Two physical machines connected via a dedicated gigabit Ethernet link. DDS (FastDDS) handles all
inter-node communication transparently, enabling seamless distributed or single-machine operation.

```
┌─ Simulation PC ──────────────────────────────────────────────────┐
│                                                                  │
│  ┌──────────────────────────┐    ┌─────────────────────────────┐ │
│  │ chrono_wec_simulation    │    │ wec_dyno_logger_node        │ │
│  │                          │    │                             │ │
│  │ Subscribes:              │    │ Subscribes:                 │ │
│  │  /pto/measured_torque    │    │  /joint_states              │ │
│  │                          │    │  /pto/commanded_torque      │ │
│  │ Publishes:               │    │  /pto/measured_torque       │ │
│  │  /wec/velocity_command   │    │  /wec/velocity_command      │ │
│  │  /wec/position_command   │    │  /wec/wave_elevation        │ │
│  │  /wec/wave_elevation     │    │                             │ │
│  │  /wec/body_position      │    │ Publishes:                  │ │
│  │                          │    │  ~/instantaneous_power      │ │
│  │ Dynamics:                │    │  ~/average_power            │ │
│  │  Placeholder:            │    │  ~/capture_width_ratio      │ │
│  │   m·ẍ = F_exc(t)        │    │  ~/peak_to_average_ratio    │ │
│  │       + F_rad(ẋ)        │    │  ~/torque_tracking_error    │ │
│  │       + F_hs(x)         │    │  ~/reactive_power_fraction  │ │
│  │       + τ_pto           │    │  → CSV export               │ │
│  └──────────┬───────────────┘    └─────────────────────────────┘ │
│             │                                                    │
└─────────────┼────────────────────────────────────────────────────┘
              │ DDS / gigabit Ethernet
┌─────────────┼──── Motor Control PC (PREEMPT_RT optional) ────────┐
│             │                                                    │
│  ┌──────────┴───────────┐        ┌─────────────────────────────┐ │
│  │ hydro_motor_ctrl     │        │ pto_controller_node         │ │
│  │                      │        │                             │ │
│  │ Subscribes:          │        │ Subscribes:                 │ │
│  │  /wec/velocity_cmd   │        │  /joint_states (ω, x)      │ │
│  │                      │        │                             │ │
│  │ PID + FF tracks      │        │ Strategy (param):           │ │
│  │ v_ref from Chrono    │        │  passive_damping            │ │
│  │                      │        │  optimal_passive            │ │
│  │ Publishes:           │        │  reactive                   │ │
│  │  /motor_effort_      │        │  latching                   │ │
│  │   controller/cmds    │        │  declutching                │ │
│  │                      │        │  mpc                        │ │
│  └──────────┬───────────┘        │                             │ │
│             │                    │ Publishes:                  │ │
│             ▼                    │  /pto_effort_controller/cmd │ │
│        ┌─────────┐               │  /pto/commanded_torque      │ │
│        │ ODrive  │               │  /pto/measured_torque       │ │
│        │ axis0 ══╪══ shaft ══ axis1                            │ │
│        └─────────┘               └─────────────────────────────┘ │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

**Simulation PC** — runs the wave-body dynamics solver and data logger. No real-time kernel required.

**Motor Control PC** — runs the velocity tracking controller (hydro emulator), PTO control law, and
`ros2_control` hardware interface to the ODrive. Benefits significantly from a PREEMPT_RT kernel to
achieve deterministic torque commands; target jitter < 200 µs.

---

## 3. The HIL Feedback Loop — Why This Is Real HIL

The critical distinction between this bench and open-loop replay is **bidirectional coupling**:

1. Chrono solves the WEC equation of motion: `m·ẍ = F_exc(t) + F_rad(ẋ) + F_hs(x) + τ_pto`
2. Publishes the resulting velocity `ẋ(t)` → the hydro motor tracks it physically on the shaft
3. The PTO motor applies its control torque to the shared shaft
4. The **actual, measured** PTO torque (including real-world losses) feeds back to Chrono
5. Chrono incorporates this measured torque into the next timestep → repeat from step 1

The PTO torque that Chrono sees is **not** the idealized `τ = -B·ω` that the control law commanded —
it is what the motor **actually delivered**, filtered through electrical dynamics, mechanical friction,
CAN bus latency (~0.5 ms per frame), and current controller bandwidth (~1 kHz for ODrive). This is
precisely what HIL testing is for: exposing the gap between idealized control design and physical
reality (Penalba et al., 2019).

---

## 4. Placeholder Dynamics Model

Until HydroChrono/SeaStack is integrated, `chrono_wec_simulation_node` implements a **1-DOF heaving
buoy** described by the linearised WEC model (Falnes, 2002):

```
m_total · ẍ = F_exc(t) − B_rad · ẋ − K_hs · x + τ_pto
```

where:

| Symbol | Parameter | Default | Units |
|--------|-----------|---------|-------|
| `m_total` | body mass + added mass at infinite frequency (`m + m_∞`) | 1500 | kg |
| `F_exc(t)` | excitation force = `F̂_exc · sin(ω·t)` | amplitude 500 | N |
| `B_rad` | radiation damping coefficient | 200 | N·s/m |
| `K_hs` | hydrostatic restoring stiffness | 3000 | N/m |
| `τ_pto` | measured PTO torque feedback | — | N·m |

**Integration**: 4th-order Runge-Kutta at the node's timer rate (default 1000 Hz). RK4 is chosen
over simpler Euler or Verlet methods because it achieves O(h⁵) local error with only four function
evaluations per step, providing excellent accuracy for the oscillatory nature of WEC dynamics.

**Linearisation assumptions**: This model omits the radiation convolution integral (Cummins, 1962),
which represents the frequency-dependent memory effect of wave radiation. The full HydroChrono node
will include this via a state-space approximation (Taghipour et al., 2008). The linear model is
valid for small wave amplitudes (kA ≪ 1) and single wave-frequency excitation.

---

## 5. PTO Control Strategies

All strategies implement the `PtoStrategyBase` interface, making them fully interchangeable via the
`strategy` parameter of `pto_controller_node`. Adding a new strategy requires only a new header file
and a factory registration line.

| Strategy | Control Law | Reference | Notes |
|----------|-------------|-----------|-------|
| `passive_damping` | τ = −B·ω | Falnes (2002) | Simplest baseline. Optimal resistive when B = B_rad for regular waves. |
| `optimal_passive` | τ = −B_opt·ω, B_opt = √(B_rad² + (ω(m+m_a) − K_hs/ω)²) | Evans (1976) | B_opt computed from known wave frequency and radiation coefficients. |
| `reactive` | τ = −B·ω − K·x | Salter (1974); Falnes (2002) | Complex-conjugate control. Maximises absorbed power but requires bidirectional power flow. |
| `latching` | Lock shaft at velocity zero-crossing, release after optimal delay | Budal & Falnes (1980); Babarit & Clément (2006) | Near-optimal for regular waves. No reactive power required. |
| `declutching` | Free shaft, engage damper at optimal phase | Babarit et al. (2009) | Dual of latching. Unidirectional power flow. Simpler to implement. |
| `mpc` | Minimise J = −∫P_pto dt subject to constraints | Cretel et al. (2011); Li & Belmont (2014) | Requires wave prediction horizon. State-of-the-art. Placeholder stub only. |

---

## 6. Performance Metrics

`wec_dyno_logger_node` computes and publishes the following metrics in real time. All metrics are
computed over a configurable sliding window (default 30 s).

| Metric | Formula | Why It Matters |
|--------|---------|----------------|
| Instantaneous power | P(t) = τ_pto · ω | Raw time-series power signal |
| Average absorbed power | P_avg = (1/T) ∫ τ_pto · ω dt | Primary WEC performance metric |
| Capture width ratio | CWR = P_avg / (P_wave · D) | Normalised efficiency, device-size independent |
| Peak-to-average power ratio | PAPR = P_peak / P_avg | PTO sizing criterion; high PAPR → large, expensive converter |
| Reactive power fraction | RPF = P_reactive / P_avg | Fraction of energy flowing back; penalises reactive strategies |
| Torque tracking error | RMS(τ_cmd − τ_meas) | Actuator fidelity — the primary HIL validation metric |
| Torque tracking bandwidth | Cross-spectral analysis | Frequency-dependent fidelity of the physical PTO |

---

## 7. Repository Structure

```
ROS2_HIL_Dyno/
├── src/
│   ├── chrono_wec_simulation/          # WEC dynamics solver (Simulation PC)
│   │   ├── src/
│   │   │   └── chrono_wec_simulation_node.cpp
│   │   ├── include/
│   │   │   └── chrono_wec_simulation/
│   │   │       └── wec_body_dynamics.hpp    # Abstract dynamics interface + LinearWecDynamics
│   │   ├── config/
│   │   │   └── wec_params.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── pto_controller/                 # Pluggable PTO control strategies (Motor PC)
│   │   ├── src/
│   │   │   └── pto_controller_node.cpp
│   │   ├── include/
│   │   │   └── pto_controller/
│   │   │       ├── pto_strategy_base.hpp    # Abstract strategy interface + factory
│   │   │       ├── passive_damping.hpp
│   │   │       ├── optimal_passive.hpp
│   │   │       ├── reactive_control.hpp
│   │   │       ├── latching_control.hpp
│   │   │       └── declutching_control.hpp
│   │   ├── config/
│   │   │   └── pto_params.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── wec_dyno_logger/               # Metrics and data recording (Simulation PC)
│   │   ├── src/
│   │   │   └── wec_dyno_logger_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── wec_dyno_bringup/              # Launch files, URDF, controller config
│       ├── launch/
│       │   ├── dyno_single_machine.launch.py
│       │   └── dyno_distributed.launch.py
│       ├── config/
│       │   ├── controllers.yaml
│       │   └── dds_profile.xml
│       ├── description/
│       │   └── urdf/
│       │       └── dyno.urdf.xacro
│       ├── CMakeLists.txt
│       └── package.xml
│
└── README.md
```

---

## 8. Build & Run Instructions

### Prerequisites

- ROS 2 Jazzy installed (`source /opt/ros/jazzy/setup.bash`)
- `ros2_control` stack: `sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers`
- `robot_state_publisher`: `sudo apt install ros-jazzy-robot-state-publisher`
- `xacro`: `sudo apt install ros-jazzy-xacro`
- ODrive ROS2 hardware interface: see [salhus/hil_odrive_ros2_control](https://github.com/salhus/hil_odrive_ros2_control)

### Build

```bash
cd ~/ros2_ws
# Clone into src/
git clone https://github.com/salhus/ROS2_HIL_Dyno.git src/ROS2_HIL_Dyno

# Install dependencies
rosdep update
rosdep install --from-paths src/ROS2_HIL_Dyno/src --ignore-src -r -y

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

### Run — Single Machine Mode

Launches all nodes on one machine (ideal for development and testing without hardware):

```bash
ros2 launch wec_dyno_bringup dyno_single_machine.launch.py
```

Optional arguments:

```bash
ros2 launch wec_dyno_bringup dyno_single_machine.launch.py \
    controllers_file:=/path/to/controllers.yaml \
    wec_params_file:=/path/to/wec_params.yaml \
    pto_params_file:=/path/to/pto_params.yaml
```

### Run — Distributed Mode

**Motor Control PC** (start first):

```bash
ros2 launch wec_dyno_bringup dyno_distributed.launch.py
```

**Simulation PC** (start after motor PC is ready):

```bash
ros2 run chrono_wec_simulation chrono_wec_simulation_node \
    --ros-args --params-file /path/to/wec_params.yaml
ros2 run wec_dyno_logger wec_dyno_logger_node
```

### Change PTO Strategy at Runtime

```bash
# At launch:
ros2 launch wec_dyno_bringup dyno_single_machine.launch.py strategy:=reactive

# Or reconfigure live (for strategies that support it):
ros2 param set /pto_controller_node strategy optimal_passive
```

---

## 9. Deployment Modes

### Single Machine

All nodes run on one computer. The `dyno_single_machine.launch.py` launch file starts every node.
Useful for simulation-only testing, algorithm development, and CI validation. No ODrive hardware
required in this mode — the `ros2_control` mock hardware interface can be used.

### Distributed (Two-Machine HIL)

- **Simulation PC**: `chrono_wec_simulation_node` + `wec_dyno_logger_node`
- **Motor Control PC**: `ros2_control_node` + `pto_controller_node` + controller spawners

`dyno_distributed.launch.py` launches only the motor-side nodes. The simulation-side nodes are
started separately on the simulation PC. Both machines must be on the same DDS domain
(`ROS_DOMAIN_ID` must match) and on the same network segment, or bridged via the
`dds_profile.xml` discovery configuration.

---

## 10. Network & DDS Configuration

### FastDDS Profile

`src/wec_dyno_bringup/config/dds_profile.xml` provides a tuned FastDDS profile for cross-machine
HIL operation:

- **UDP unicast transport** with direct peer discovery (no multicast required)
- **Tuned socket buffer sizes** (8 MB send/receive) for sustained high-rate data streams
- **Heartbeat and acknowledgement tuning** for reliable delivery at 1 kHz
- **Best-effort QoS** for high-rate sensor data; **reliable QoS** for commands

### ROS Domain

Set the same domain ID on both machines before launching:

```bash
export ROS_DOMAIN_ID=42   # same value on both machines
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/dds_profile.xml
```

### Network Setup

```bash
# On both machines — assign static IPs on the dedicated Ethernet link:
sudo ip addr add 192.168.100.1/24 dev eth1   # Simulation PC
sudo ip addr add 192.168.100.2/24 dev eth1   # Motor PC

# Verify connectivity:
ping 192.168.100.2   # from Simulation PC
```

---

## 11. Latency Considerations

HIL fidelity degrades when the round-trip latency (Chrono → motor → Chrono) is a significant
fraction of the wave period. For a 1 Hz wave, the period is 1000 ms; the target total loop latency
is < 10 ms.

| Hop | Typical Latency | Mitigation |
|-----|----------------|------------|
| DDS serialisation + UDP | 0.1–0.5 ms | Tuned profile, pre-allocated buffers |
| Network transmission (direct Ethernet) | 0.05–0.2 ms | Dedicated gigabit link, no switch |
| DDS deserialisation | 0.1–0.5 ms | Same |
| `ros2_control` update cycle jitter | 0–5 ms | PREEMPT_RT kernel on Motor PC |
| ODrive CAN frame | ~0.5 ms per frame | 1 Mbit/s CAN bus |
| Total round-trip | ~1–7 ms | — |

**Extrapolation**: When latency introduces non-trivial phase error, the simulation node can apply
a first-order predictor: `x̂(t+τ) ≈ x(t) + τ·ẋ(t)` to compensate for the round-trip delay `τ`.

---

## 12. Phase 1 Reference

Phase 1 of this project established the basic ODrive HIL bench with a single passive damper. The
Phase 1 codebase (single-motor velocity tracking + passive PTO) is available at:

**[salhus/hil_odrive_ros2_control](https://github.com/salhus/hil_odrive_ros2_control)**

This Phase 2 repository extends that work to a full two-motor WEC HIL bench with:

- Bidirectional coupling between the numerical WEC model and the physical PTO
- Pluggable PTO control strategy framework
- Standardised performance metrics and data logging
- Distributed deployment support (Simulation PC + Motor Control PC)

---

## 13. References

Babarit, A., & Clément, A. H. (2006). Optimal latching control of a wave energy device in regular
and irregular waves. *Applied Ocean Research*, 28(2), 77–91.
https://doi.org/10.1016/j.apor.2006.05.002

Babarit, A., Guglielmi, M., & Clément, A. H. (2009). Declutching control of a wave energy
converter. *Ocean Engineering*, 36(12–13), 1015–1024.
https://doi.org/10.1016/j.oceaneng.2009.05.006

Bacelli, G., Coe, R. G., Patterson, D., & Wilson, D. (2017). System identification of a wave
energy converter. *Proceedings of the 12th European Wave and Tidal Energy Conference (EWTEC)*.

Budal, K., & Falnes, J. (1980). Interacting point absorbers with controlled motion. In B. Count
(Ed.), *Power from Sea Waves* (pp. 381–399). Academic Press.

Cretel, J. A. M., Lightbody, G., Thomas, G. P., & Lewis, A. W. (2011). Maximisation of energy
capture by a wave-energy point absorber using model predictive control. *Proceedings of the 18th
IFAC World Congress*, 18(1), 3714–3721.

Cummins, W. E. (1962). The impulse response function and ship motions. *Schiffstechnik*, 9, 101–109.

Evans, D. V. (1976). A theory for wave-power absorption by oscillating bodies. *Journal of Fluid
Mechanics*, 77(1), 1–25. https://doi.org/10.1017/S0022112076001109

Falnes, J. (2002). *Ocean Waves and Oscillating Systems: Linear Interactions Including Wave-Energy
Extraction*. Cambridge University Press.
https://doi.org/10.1017/CBO9780511754630

Forehand, D. I. M., Kiprakis, A. E., Nambiar, A. J., & Wallace, A. R. (2016). A fully coupled
wave-to-wire model of an array of wave energy converters. *IEEE Transactions on Sustainable
Energy*, 7(1), 118–128.

Garrido, A. J., Garrido, I., Amundarain, M., Alberdi, M., & De la Sen, M. (2015). Sliding-mode
control of wave power generation plants. *IEEE Transactions on Industry Applications*, 48(6),
2372–2381.

Li, G., & Belmont, M. R. (2014). Model predictive control of sea wave energy converters — Part I:
A convex approach for the case of a single device. *Renewable Energy*, 69, 453–463.
https://doi.org/10.1016/j.renene.2014.03.070

Penalba, M., Sell, N. P., Ringwood, J. V., & Hillis, A. J. (2019). Validating a wave-to-wire model
for a wave energy converter — Part I: The hydraulic transmission system. *Renewable Energy*, 139,
1077–1089. https://doi.org/10.1016/j.renene.2019.02.067

Ringwood, J. V., Bacelli, G., & Fusco, F. (2014). Energy-maximizing control of wave-energy
converters: The development of control system technology to optimize their operation. *IEEE Control
Systems Magazine*, 34(5), 30–55. https://doi.org/10.1109/MCS.2014.2333253

Salter, S. H. (1974). Wave power. *Nature*, 249(5459), 720–724.
https://doi.org/10.1038/249720a0

Taghipour, R., Pérez, T., & Moan, T. (2008). Hybrid frequency–time domain models for dynamic
response analysis of marine structures. *Ocean Engineering*, 35(7), 685–705.
https://doi.org/10.1016/j.oceaneng.2007.11.002

Têtu, A., Ferri, F., Kramer, M. B., & Todalshaug, J. H. (2018). Physical and mathematical modeling
of a wave energy converter equipped with a negative spring mechanism for phase control. *Energies*,
11(10), 2576. https://doi.org/10.3390/en11102576

---

## 14. License

Copyright 2024 salhus

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
in compliance with the License. You may obtain a copy of the License at:

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License
is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
or implied. See the License for the specific language governing permissions and limitations under
the License.
