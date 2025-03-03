#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "HMI.hpp"

HMI::HMI() : Node("human_machine_interface") {
    HMI_status.initializeROS(this);
    LED_status.initializeROS(this);
}

void HMI::HMIStatus::initializeROS(rclcpp::Node* node) {

    // Standard Components
    pw_but_dn = node->create_publisher<std_msgs::msg::Bool>("pw_but_dn", 10);
    pw_but_up = node->create_publisher<std_msgs::msg::Bool>("pw_but_up", 10);
    em_but_right = node->create_publisher<std_msgs::msg::Bool>("em_but_right", 10);
    em_but_down = node->create_publisher<std_msgs::msg::Bool>("em_but_down", 10);
    em_but_up = node->create_publisher<std_msgs::msg::Bool>("em_but_up", 10);
    em_but_left = node->create_publisher<std_msgs::msg::Bool>("em_but_left", 10);

    pw_but_mv_dn = node->create_subscription<std_msgs::msg::Bool>(
        "pw_but_mv_dn", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            pw_but_dn->publish(new_msg);
        }
    );
    pw_but_mv_up = node->create_subscription<std_msgs::msg::Bool>(
        "pw_but_mv_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            pw_but_up->publish(new_msg);
        }
    );
    em_but_mv_left = node->create_subscription<std_msgs::msg::Bool>(
        "em_but_mv_left", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            em_but_left->publish(new_msg);
        }
    );
    em_but_mv_right = node->create_subscription<std_msgs::msg::Bool>(
        "em_but_mv_right", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            em_but_right->publish(new_msg);
        }
    );
    em_but_mv_up = node->create_subscription<std_msgs::msg::Bool>(
        "em_but_mv_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            em_but_up->publish(new_msg);
        }
    );
    em_but_mv_dn = node->create_subscription<std_msgs::msg::Bool>(
        "em_but_mv_dn", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            em_but_down->publish(new_msg);
        }
    );
    
    // DELTA:add:2:AS

    // DELTA:add:6:LED_AS

    // DELTA:add:8:LED_PW

}

void HMI::StatusLED::initializeROS(rclcpp::Node* node) {

    // DELTA:add:3:LED_AS
    // DELTA:add:9:LED_AS

    // DELTA:add:3:LED_FP
    
    // DELTA:add:3:LED_CLS

    // LED Power Window
    // DELTA:add:4:LED_PW
    // DELTA:add:5:LED_PW
    // DELTA:add:10:LED_PW

    // DELTA:add:3:LED_EM

    // DELTA:add:3:LED_Heatable
}

// DELTA:add:4:LED_AS
// DELTA:add:10:LED_AS

// DELTA:add:4:LED_FP

// DELTA:add:4:LED_CLS

// DELTA:add:6:LED_PW
// DELTA:add:11:LED_PW

// DELTA:add:4:LED_EM

// DELTA:add:4:LED_Heatable
