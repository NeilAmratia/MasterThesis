#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class HMI : public rclcpp::Node {
public:
    HMI();

private:
    class HMIStatus {
    public:
        void initializeROS(rclcpp::Node* node);
        
    private:

        // Standard Components
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_but_mv_dn;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_but_mv_up;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_but_mv_left;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_but_mv_right;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_but_mv_up;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_but_mv_dn;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pw_but_dn;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pw_but_up;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr em_but_right;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr em_but_down;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr em_but_up;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr em_but_left;

        // DELTA:add:1:AS

        // DELTA:add:5:LED_AS

        // DELTA:add:7:LED_PW

    };


    class StatusLED {
    public:
        void initializeROS(rclcpp::Node* node);
        // DELTA:add:1:LED_AS
        // DELTA:add:7:LED_AS
        // DELTA:add:1:LED_FP
        // DELTA:add:1:LED_CLS
        // DELTA:add:2:LED_PW
        // DELTA:add:1:LED_EM
        // DELTA:add:1:LED_Heatable
        
    private:

        // DELTA:add:2:LED_AS
        // DELTA:add:8:LED_AS

        // DELTA:add:2:LED_FP

        // DELTA:add:2:LED_CLS

        // LED Power Window
        // DELTA:add:1:LED_PW
        // DELTA:add:3:LED_PW
        // DELTA:add:9:LED_PW
        
        // DELTA:add:2:LED_EM
        
        // DELTA:add:2:LED_Heatable
    };
    HMIStatus HMI_status;
    StatusLED LED_status;
};