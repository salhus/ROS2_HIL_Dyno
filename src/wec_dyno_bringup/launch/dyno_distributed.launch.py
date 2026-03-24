# Copyright 2024 salhus
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
dyno_distributed.launch.py
═══════════════════════════════════════════════════════════════════════════════
Launch file for the WEC HIL Dynamometer in DISTRIBUTED (two-machine) mode.

This launch file starts ONLY the motor-side nodes on the Motor Control PC.
The simulation-side nodes (chrono_wec_simulation, wec_dyno_logger) are started
separately on the Simulation PC:

  Motor Control PC (this file):
    - ros2_control_node           (ODrive hardware interface)
    - robot_state_publisher
    - joint_state_broadcaster
    - hydro_effort_controller
    - pto_effort_controller
    - pto_controller_node         (runs HERE for lowest latency to ODrive)

  Simulation PC (start separately):
    ros2 run chrono_wec_simulation chrono_wec_simulation_node \\
        --ros-args --params-file /path/to/wec_params.yaml
    ros2 run wec_dyno_logger wec_dyno_logger_node

Rationale for this split:
  The pto_controller_node runs on the Motor Control PC (not the Simulation PC)
  to minimise the latency between the control law output and the ODrive command.
  On a PREEMPT_RT kernel, this latency can be < 500 µs (ODrive CAN frame).

  If pto_controller_node ran on the Simulation PC, the control command would
  traverse the DDS/Ethernet link (1–3 ms additional round-trip), degrading the
  torque tracking bandwidth from ~100 Hz to ~30 Hz. For MPC strategies that
  require wave prediction, the control node is allowed on the Simulation PC.

Network prerequisites:
  Both machines must:
    1. Share the same ROS_DOMAIN_ID (e.g., export ROS_DOMAIN_ID=42)
    2. Be on the same network segment, or have the dds_profile.xml applied:
       export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/dds_profile.xml
    3. Have static IPs on the dedicated Ethernet link (see README.md §10)

Example (Motor Control PC):
  export ROS_DOMAIN_ID=42
  export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/dds_profile.xml
  ros2 launch wec_dyno_bringup dyno_distributed.launch.py strategy:=latching

Architecture reference:
  See README.md §9 for full distributed deployment instructions.
═══════════════════════════════════════════════════════════════════════════════
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package share directories ─────────────────────────────────────────
    bringup_share = get_package_share_directory('wec_dyno_bringup')
    pto_share = get_package_share_directory('pto_controller')

    # ── Default file paths ────────────────────────────────────────────────
    default_controllers_file = os.path.join(bringup_share, 'config', 'controllers.yaml')
    default_pto_params_file = os.path.join(pto_share, 'config', 'pto_params.yaml')
    xacro_file = os.path.join(bringup_share, 'description', 'urdf', 'dyno.urdf.xacro')

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_controllers_file = DeclareLaunchArgument(
        'controllers_file',
        default_value=default_controllers_file,
        description='Full path to ros2_control controllers YAML file')

    declare_pto_params_file = DeclareLaunchArgument(
        'pto_params_file',
        default_value=default_pto_params_file,
        description='Full path to pto_controller parameters YAML file')

    declare_strategy = DeclareLaunchArgument(
        'strategy',
        default_value='passive_damping',
        description='PTO control strategy: passive_damping | optimal_passive | '
                    'reactive | latching | declutching')

    # ── URDF / robot description ──────────────────────────────────────────
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
    ])
    robot_description = {'robot_description': robot_description_content}

    # ── Motor-side nodes ──────────────────────────────────────────────────

    # robot_state_publisher — needed for TF even on the motor PC.
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description])

    # ros2_control_node — ODrive hardware interface.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            LaunchConfiguration('controllers_file'),
        ],
        output='screen')

    # Controller spawners.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    hydro_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hydro_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    pto_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pto_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    # pto_controller_node — runs on the Motor PC for lowest latency to ODrive.
    # The PTO control law executes close to the hardware, reducing torque command
    # latency from ~3 ms (cross-machine DDS) to ~0.5 ms (local process + CAN).
    pto_controller_node = Node(
        package='pto_controller',
        executable='pto_controller_node',
        name='pto_controller_node',
        parameters=[
            LaunchConfiguration('pto_params_file'),
            {'strategy': LaunchConfiguration('strategy')},
        ],
        output='screen')

    # ── NOTE ──────────────────────────────────────────────────────────────
    # chrono_wec_simulation_node and wec_dyno_logger_node are NOT launched here.
    # They run on the Simulation PC. Start them with:
    #
    #   ros2 run chrono_wec_simulation chrono_wec_simulation_node \
    #       --ros-args --params-file <wec_params.yaml>
    #
    #   ros2 run wec_dyno_logger wec_dyno_logger_node
    #
    # Both machines must share the same ROS_DOMAIN_ID and DDS profile.

    return LaunchDescription([
        declare_controllers_file,
        declare_pto_params_file,
        declare_strategy,

        robot_state_pub_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        hydro_controller_spawner,
        pto_controller_spawner,
        pto_controller_node,
    ])
