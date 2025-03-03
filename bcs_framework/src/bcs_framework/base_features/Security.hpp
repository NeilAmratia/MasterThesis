#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class Security : public rclcpp::Node {
public:
    Security();

private:

    // DELTA:delete:3:AS
    class AS {
    public:
        void initializeROS(rclcpp::Node* node);
        void alarmSystem();
        
    private:
        // Standard Components
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr key_pos_lock_2;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr key_pos_unlock_2;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr as_alarm_detected;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr time_alarm_elapsed;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr as_activated;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr as_deactivated;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr as_active_on;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr as_active_off;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr as_alarm_off;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr as_alarm_on;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr as_alarm_was_detected;
        bool key_pos_lock_2_value = false;
        bool key_pos_unlock_2_value = false;
        bool as_alarm_detected_value = false;
        bool as_activated_value = true;
        bool time_alarm_elapsed_value = false;
        enum class AlarmSystemState {
            AS_OFF,
            AS_ON,
            AS_ALARM
        };
        AlarmSystemState current_alarm_system_state = AlarmSystemState::AS_OFF;

        // DELTA:add:1:Control_AS

        // DELTA:add:1:Interior_Monitoring

    };
    AS alarm_system;
    // DELTA:delete:3:AS:END

    // DELTA:delete:3:CLS
    class CLS {
    public:
        void initializeROS(rclcpp::Node* node);
        void cls();
        
    private:
        // Standard Components
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr key_pos_lock;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr key_pos_unlock;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cls_lock;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cls_unlock;
        bool key_pos_lock_value = false;
        bool key_pos_unlock_value = false;

        // DELTA:add:1:RCK

        // DELTA:add:1:Automatic_Locking

        enum class CentrolLockingSystemState {
            CLS_UNLOCK,
            CLS_LOCK
        };
        CentrolLockingSystemState current_cls_state = CentrolLockingSystemState::CLS_UNLOCK;
    };
    CLS cls;
    // DELTA:delete:3:CLS:END

    // DELTA:delete:7:RCK
    class RCK {
    public:
        void initializeROS(rclcpp::Node* node);
        // DELTA:add:4:Safety_Function
        
    private:
        // Standard Components
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rck_but_lock;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rck_but_unlock;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rck_lock;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rck_unlock;
        bool rck_but_lock_value = false;
        bool rck_but_unlock_value = false;

        // DELTA:add:1:Control_Automatic_PW

        // DELTA:add:1:Safety_Function
        
    };
    RCK rck;
    // DELTA:delete:7:RCK:END
};