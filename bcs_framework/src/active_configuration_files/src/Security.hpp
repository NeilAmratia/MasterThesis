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

        // Remote Control Key Components
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rck_lock_sub_2;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rck_unlock_sub_2;
        bool rck_lock_sub_2_value = false;
        bool rck_unlock_sub_2_value = false;
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
        void rck_sf();
// DELTA:add:4:Safety_Function
        
    private:
        // Standard Components
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rck_but_lock;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rck_but_unlock;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rck_lock;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rck_unlock;
        bool rck_but_lock_value = false;
        bool rck_but_unlock_value = false;

        // Automatic Power Window Components
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_rm_up;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pw_rm_dn;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pw_but_up;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pw_but_dn;
// DELTA:add:1:Control_Automatic_PW

        // Safety Functions Components
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_open_2;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr time_rck_sf_elapsed;
        bool door_open_value_2 = false;
        bool time_rck_sf_elapsed_value = false;
        
        enum class RemoteSafetyFunctionsState {
            SF_OFF,
            SF_ON
        };
        RemoteSafetyFunctionsState current_rck_sf_state = RemoteSafetyFunctionsState::SF_OFF;
// DELTA:add:1:Safety_Function
        
    };
    RCK rck;
    // DELTA:delete:7:RCK:END
};