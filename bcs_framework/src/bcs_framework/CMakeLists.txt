cmake_minimum_required(VERSION 3.8)
project(bcs_framework)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add option for default config path
set(DEFAULT_CONFIG_PATH "input_config/input_configuration.xml" CACHE STRING "Default configuration file path")

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(LLVM REQUIRED CONFIG)
find_package(Clang REQUIRED CONFIG)
find_package(tomlplusplus REQUIRED)
find_package(TinyXML2 REQUIRED)

include_directories(${LLVM_INCLUDE_DIRS})
include_directories(${CLANG_INCLUDE_DIRS})
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Main framework executable
add_executable(create_framework 
  main.cpp
  framework_core/delta_processor.cpp
  framework_core/feature_config.cpp
  framework_core/config_parser.cpp
  framework_core/framework_manager.cpp
)

# Add compile definition for default config path
target_compile_definitions(create_framework PRIVATE 
  DEFAULT_CONFIG_PATH="${DEFAULT_CONFIG_PATH}"
)


target_link_libraries(create_framework 
    clangTooling
    clangFrontend
    clangAST
    clangRewrite
    tomlplusplus::tomlplusplus
    tinyxml2
    ${LLVM_LIBRARIES}
)

ament_target_dependencies(create_framework
  rclcpp
)

# Feature Model Generator executable
add_executable (feature_model_generator
    framework_core/feature_model_generator_main.cpp
    framework_core/feature_model_generator.cpp
)

target_link_libraries(feature_model_generator
    tinyxml2
)

add_executable(feature_analyzer_node framework_core/feature_model_generator_using_AST.cpp)

target_link_libraries(feature_analyzer_node
  clangTooling
  clangBasic
  clangASTMatchers
  clangAST
  clangFrontend
)


install(TARGETS
  create_framework
  feature_model_generator
  feature_analyzer_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
