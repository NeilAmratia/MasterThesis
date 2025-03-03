#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class DoorSystem : public rclcpp::Node {
public:
    DoorSystem();

private:
    class PW {
    public:
        void initializeROS(rclcpp::Node* node);
        // DELTA:change:2:Automatic_PW
        void processManual();
        // DELTA:change:2:Automatic_PW:END
        void processFingerProtection();
        
    private:
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_but_up;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_but_dn;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_pos_up;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_pos_dn;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fp_on_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fp_off_sub;
        bool pw_but_up_value = false;
        bool pw_but_dn_value = false;
        bool pw_pos_up_value = false;
        bool pw_pos_dn_value = false;
        bool fp_on_value = false;
        bool fp_off_value = false;

        // DELTA:delete:1:CLS
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cls_lock;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cls_unlock;
        bool pw_enabled_state_ = true;
        // DELTA:delete:1:CLS:END
        
        // DELTA:change:1:Automatic_PW
        // Manual window
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pw_mv_up;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pw_mv_dn;
        // DELTA:change:1:Automatic_PW:END
        
        enum class WindowState {
            PW_UP,
            PW_PEND,
            PW_DN
        };
        WindowState current_window_state = WindowState::PW_UP;

        // Finger protection
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fp_on;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fp_off;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finger_detected;

        bool finger_detected_value = false;
        enum class FingerProtectionState {
            FP_OFF,
            FP_ON
        };
        FingerProtectionState current_fp_state = FingerProtectionState::FP_OFF;
    };

    class EM {
    public:
        void initializeROS(rclcpp::Node* node);
        void processElectric();
        // DELTA:add:1:Heatable
        
    private:

        // Electric mirror
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr em_mv_left;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr em_mv_right;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr em_mv_up;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr em_mv_down;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_but_right;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_but_down;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_but_up;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_but_left;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_pos_right;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_pos_bottom;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_pos_top;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr em_pos_left;

        // DELTA:add:5:LED_EM

        bool em_but_right_value = false;
        bool em_but_down_value = false;
        bool em_but_up_value = false;
        bool em_but_left_value = false;
        bool em_pos_right_value = false;
        bool em_pos_bottom_value = false;
        bool em_pos_top_value = false;
        bool em_pos_left_value = false;

        enum class ElectricMirrorState {
            EM_TOP,
            EM_TOP_LEFT,
            EM_LEFT,
            EM_BOTTOM_LEFT,
            EM_BOTTOM,
            EM_BOTTOM_RIGHT,
            EM_RIGHT,
            EM_TOP_RIGHT,
            EM_PENDING
        };
        ElectricMirrorState current_em_state = ElectricMirrorState::EM_PENDING;
        
        // DELTA:add:2:Heatable
        
    };

    PW power_window;
    EM exterior_mirror;
};