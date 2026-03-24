cmake_minimum_required(VERSION 3.16)
project(wec_dyno_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files, configuration files, and URDF description.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
