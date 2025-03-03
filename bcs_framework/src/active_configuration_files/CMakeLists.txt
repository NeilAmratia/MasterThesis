cmake_minimum_required(VERSION 3.8)
project(active_configuration_files)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Set source and include directories
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Gather all source files automatically
file(GLOB SOURCE_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  ${SRC_DIR}/*.cpp
)

add_executable(run_framework ${SOURCE_FILES})

target_include_directories(run_framework PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${SRC_DIR}
)

ament_target_dependencies(run_framework
  rclcpp
  std_msgs
)

install(TARGETS run_framework
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
