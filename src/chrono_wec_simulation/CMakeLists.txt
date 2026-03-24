cmake_minimum_required(VERSION 3.16)
project(chrono_wec_simulation LANGUAGES CXX)

# ── C++20 standard ────────────────────────────────────────────────────────────
# C++20 is required for structured bindings (auto [k1_x, k1_v] = ...) used in
# the RK4 integrator and for std::numbers::pi.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Find dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(chrono_wec_simulation_node
  src/chrono_wec_simulation_node.cpp
)

target_include_directories(chrono_wec_simulation_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(chrono_wec_simulation_node
  rclcpp
  std_msgs
)

# ── Installation ──────────────────────────────────────────────────────────────
install(TARGETS chrono_wec_simulation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Linting / Testing ─────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
