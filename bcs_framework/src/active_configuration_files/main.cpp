#include <rclcpp/rclcpp.hpp>
#include "src/HMI.hpp"        
#include "src/Security.hpp"   
#include "src/DoorSystem.hpp" 

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create shared pointers for each node
    auto human_machine_interface = std::make_shared<HMI>();
    auto security_system = std::make_shared<Security>();
    auto door_system = std::make_shared<DoorSystem>();

    // Use a MultiThreadedExecutor to spin multiple nodes concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(human_machine_interface);
    executor.add_node(security_system);
    executor.add_node(door_system);

    // Spin the executor
    executor.spin();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}