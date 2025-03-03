cmake_minimum_required(VERSION 3.8)
project(bcs_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pugixml REQUIRED)
find_package(Qt5 COMPONENTS
    Core
    Gui
    Qml
    Quick
    QuickControls2
REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(bcs_simulator
    src/main.cpp
    src/MessageModel.cpp
    src/ROS2Bridge.cpp
    qml/qml.qrc
)

target_include_directories(bcs_simulator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(bcs_simulator
    Qt5::Core
    Qt5::Gui
    Qt5::Qml
    Qt5::Quick
    Qt5::QuickControls2
    rclcpp::rclcpp
)

ament_target_dependencies(bcs_simulator
    rclcpp
    std_msgs
)

# Test executable
add_executable(bcs_test_suite
    test_cases/ConfigParser.cpp
    test_cases/test_main.cpp
    test_cases/TestManager.cpp
    test_cases/PowerWindowTests.cpp
    test_cases/ExteriorMirrorTests.cpp
    test_cases/HMITests.cpp
    test_cases/SecurityTests.cpp
    #test_cases/InteractionTests.cpp
)

# Test executable configuration
target_include_directories(bcs_test_suite PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/test_cases
)

# Test executable links
target_link_libraries(bcs_test_suite
    rclcpp::rclcpp
    pugixml
)

ament_target_dependencies(bcs_test_suite
    rclcpp
    std_msgs
)

install(TARGETS 
    bcs_simulator
    bcs_test_suite
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY qml
    DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
