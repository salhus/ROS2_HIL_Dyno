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
dyno_single_machine.launch.py
═══════════════════════════════════════════════════════════════════════════════
Launch file for the WEC HIL Dynamometer in SINGLE-MACHINE mode.

Launches ALL nodes on one computer. This is the primary mode for:
  - Development and algorithm testing (no hardware required)
  - System integration testing before distributed deployment
  - CI validation

Nodes launched:
  1. robot_state_publisher       — publishes TF from URDF
  2. ros2_control_node           — hardware interface to ODrive (or mock)
  3. joint_state_broadcaster     — spawned after controller_manager is ready
  4. hydro_effort_controller     — effort controller for Motor 0 (hydro)
  5. pto_effort_controller       — effort controller for Motor 1 (PTO)
  6. chrono_wec_simulation_node  — WEC dynamics solver
  7. pto_controller_node         — pluggable PTO control strategy
  8. wec_dyno_logger_node        — metrics and data recording

Launch arguments:
  controllers_file    Path to controllers.yaml  (default: package default)
  wec_params_file     Path to wec_params.yaml   (default: package default)
  pto_params_file     Path to pto_params.yaml   (default: package default)
  strategy            PTO strategy name          (default: passive_damping)
  use_mock_hardware   'true' to use mock hardware interface (default: false)

Example:
  ros2 launch wec_dyno_bringup dyno_single_machine.launch.py strategy:=reactive

Architecture reference:
  See README.md §2 for the full system architecture diagram.
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
    chrono_share = get_package_share_directory('chrono_wec_simulation')
    pto_share = get_package_share_directory('pto_controller')

    # ── Default file paths ────────────────────────────────────────────────
    default_controllers_file = os.path.join(bringup_share, 'config', 'controllers.yaml')
    default_wec_params_file = os.path.join(chrono_share, 'config', 'wec_params.yaml')
    default_pto_params_file = os.path.join(pto_share, 'config', 'pto_params.yaml')
    xacro_file = os.path.join(bringup_share, 'description', 'urdf', 'dyno.urdf.xacro')

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_controllers_file = DeclareLaunchArgument(
        'controllers_file',
        default_value=default_controllers_file,
        description='Full path to ros2_control controllers YAML file')

    declare_wec_params_file = DeclareLaunchArgument(
        'wec_params_file',
        default_value=default_wec_params_file,
        description='Full path to chrono_wec_simulation parameters YAML file')

    declare_pto_params_file = DeclareLaunchArgument(
        'pto_params_file',
        default_value=default_pto_params_file,
        description='Full path to pto_controller parameters YAML file')

    declare_strategy = DeclareLaunchArgument(
        'strategy',
        default_value='passive_damping',
        description='PTO control strategy: passive_damping | optimal_passive | '
                    'reactive | latching | declutching')

    declare_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware interface (true for testing without ODrive)')

    # ── URDF / robot description ──────────────────────────────────────────
    # Process the xacro file to obtain the URDF string.
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
    ])
    robot_description = {'robot_description': robot_description_content}

    # ── Nodes ─────────────────────────────────────────────────────────────

    # 1. robot_state_publisher — publishes TF transforms from URDF joint states.
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description])

    # 2. ros2_control_node — the controller manager + hardware interface.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            LaunchConfiguration('controllers_file'),
        ],
        output='screen')

    # 3. joint_state_broadcaster — spawned after controller_manager is ready.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    # 4. hydro_effort_controller spawner.
    hydro_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hydro_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    # 5. pto_effort_controller spawner.
    pto_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pto_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    # 6. chrono_wec_simulation_node — WEC dynamics solver.
    #    Runs the LinearWecDynamics placeholder model at 1000 Hz.
    chrono_wec_simulation_node = Node(
        package='chrono_wec_simulation',
        executable='chrono_wec_simulation_node',
        name='chrono_wec_simulation_node',
        parameters=[LaunchConfiguration('wec_params_file')],
        output='screen')

    # 7. pto_controller_node — pluggable PTO control strategy.
    #    Strategy is overridden from the launch argument if provided.
    pto_controller_node = Node(
        package='pto_controller',
        executable='pto_controller_node',
        name='pto_controller_node',
        parameters=[
            LaunchConfiguration('pto_params_file'),
            {'strategy': LaunchConfiguration('strategy')},
        ],
        output='screen')

    # 8. wec_dyno_logger_node — metrics and data recording.
    wec_dyno_logger_node = Node(
        package='wec_dyno_logger',
        executable='wec_dyno_logger_node',
        name='wec_dyno_logger_node',
        output='screen')

    # ── Assemble LaunchDescription ────────────────────────────────────────
    return LaunchDescription([
        # Launch arguments
        declare_controllers_file,
        declare_wec_params_file,
        declare_pto_params_file,
        declare_strategy,
        declare_mock_hardware,

        # Nodes
        robot_state_pub_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        hydro_controller_spawner,
        pto_controller_spawner,
        chrono_wec_simulation_node,
        pto_controller_node,
        wec_dyno_logger_node,
    ])
