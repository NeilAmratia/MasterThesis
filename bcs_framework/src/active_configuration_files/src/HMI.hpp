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
        void LED_FP();
// DELTA:add:1:LED_FP
        // DELTA:add:1:LED_CLS
        void LED_AutoPW_up();
        void LED_AutoPW_dn();
// DELTA:add:2:LED_PW
        // DELTA:add:1:LED_EM
        // DELTA:add:1:LED_Heatable
        
    private:

        // DELTA:add:2:LED_AS
        // DELTA:add:8:LED_AS

        // LED Finger Protection
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fp_on;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fp_off;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_fp_on;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_fp_off;
        bool fp_on_value = false;
        bool fp_off_value = false;
        enum class FingerProtectionState {
            FP_LED_OFF,
            FP_LED_ON
        };
        FingerProtectionState current_led_fp_state = FingerProtectionState::FP_LED_OFF;
// DELTA:add:2:LED_FP

        // DELTA:add:2:LED_CLS

        // LED Power Window
        // DELTA:add:1:LED_PW
        // DELTA:add:3:LED_PW
        // Automatic Window with CLS
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_auto_mv_dn;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_auto_mv_up;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_auto_mv_stop;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pw_up_on;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pw_up_off;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pw_dn_on;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pw_dn_off;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cls_lock_2;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cls_unlock_2;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pw_cls_up_on;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pw_cls_up_off;
        bool pw_auto_mv_dn_value = false;
        bool pw_auto_mv_up_value = false;
        bool pw_auto_mv_stop_value = false;
        bool cls_lock_2_value = false;
        bool cls_unlock_2_value = true;
        enum class AutoPWUpState {
            AUTOPW_UP_LED_OFF,
            AUTOPW_UP_LED_ON,
            AUTOPW_UP_CLS_LED_ON
        };
        AutoPWUpState current_led_AutoPW_up_state = AutoPWUpState::AUTOPW_UP_LED_OFF;
        enum class AutoPWDnState {
            AUTOPW_DN_LED_OFF,
            AUTOPW_DN_LED_ON
        };
        AutoPWDnState current_led_AutoPW_dn_state = AutoPWDnState::AUTOPW_DN_LED_OFF;
// DELTA:add:9:LED_PW
        
        // DELTA:add:2:LED_EM
        
        // DELTA:add:2:LED_Heatable
    };
    HMIStatus HMI_status;
    StatusLED LED_status;
};