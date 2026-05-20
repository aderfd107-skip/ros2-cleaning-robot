cmake_minimum_required(VERSION 3.8)
project(cleaning_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch config maps docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/auto_explore_and_save.py
  scripts/phase5_world_smoke.sh
  scripts/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
