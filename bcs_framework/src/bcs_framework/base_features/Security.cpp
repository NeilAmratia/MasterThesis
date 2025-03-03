#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "Security.hpp"

Security::Security() : Node("security_feature") {
    // DELTA:delete:4:AS
    alarm_system.initializeROS(this);
    // DELTA:delete:4:AS:END
    // DELTA:delete:4:CLS
    cls.initializeROS(this);
    // DELTA:delete:4:CLS:END
    // DELTA:delete:8:RCK
    rck.initializeROS(this);
    // DELTA:delete:8:RCK:END
}

// DELTA:delete:5:AS
void Security::AS::initializeROS(rclcpp::Node* node) {

    // Standard Components
    key_pos_lock_2 = node->create_subscription<std_msgs::msg::Bool>(
        "key_pos_lock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            key_pos_lock_2_value = msg->data;
            alarmSystem();
            key_pos_lock_2_value = false;
        }
    );
    key_pos_unlock_2 = node->create_subscription<std_msgs::msg::Bool>(
         "key_pos_unlock", 10,
         [this](const std_msgs::msg::Bool::SharedPtr msg) {
             key_pos_unlock_2_value = msg->data;
             alarmSystem();
             key_pos_unlock_2_value = false;
         }
    );
    as_alarm_detected = node->create_subscription<std_msgs::msg::Bool>(
        "as_alarm_detected", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            as_alarm_detected_value = msg->data;
            alarmSystem();
            as_alarm_detected_value = false;
        }
    );
    time_alarm_elapsed = node->create_subscription<std_msgs::msg::Bool>(
        "time_alarm_elapsed", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            time_alarm_elapsed_value = msg->data;
            alarmSystem();
            time_alarm_elapsed_value = false;
        }
    );
    as_activated = node->create_subscription<std_msgs::msg::Bool>(
        "as_activated", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            as_activated_value = msg->data;
            as_activated_value = true;
            alarmSystem();
        }
    );
    as_deactivated = node->create_subscription<std_msgs::msg::Bool>(
        "as_deactivated", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            as_activated_value = msg->data;
            as_activated_value = false;
            alarmSystem();
        }
    );
    as_active_on = node->create_publisher<std_msgs::msg::Bool>("as_active_on", 10);
    as_active_off = node->create_publisher<std_msgs::msg::Bool>("as_active_off", 10);
    as_alarm_off = node->create_publisher<std_msgs::msg::Bool>("as_alarm_off", 10);
    as_alarm_on = node->create_publisher<std_msgs::msg::Bool>("as_alarm_on", 10);
    as_alarm_was_detected = node->create_publisher<std_msgs::msg::Bool>("as_alarm_was_detected", 10);

    // DELTA:add:2:Control_AS

    // DELTA:add:2:Interior_Monitoring
     
}

void Security::AS::alarmSystem() {
    std_msgs::msg::Bool msg;
    msg.data = true;

    switch (current_alarm_system_state) {
        case AlarmSystemState::AS_OFF:
            if (as_activated_value && key_pos_lock_2_value) {
                as_active_on->publish(msg);
                current_alarm_system_state = AlarmSystemState::AS_ON;
            } 
            // DELTA:add:3:Control_AS  
            break;

        case AlarmSystemState::AS_ON:
            if (key_pos_unlock_2_value) {
                as_active_off->publish(msg);
                current_alarm_system_state = AlarmSystemState::AS_OFF;
            } else if (as_alarm_detected_value) {
                as_alarm_on->publish(msg);
                current_alarm_system_state = AlarmSystemState::AS_ALARM;
            } // DELTA:add:3:Interior_Monitoring 
            // DELTA:add:4:Control_AS
            break;

        case AlarmSystemState::AS_ALARM:
            if (key_pos_unlock_2_value) {
                as_active_off->publish(msg);
                as_alarm_off->publish(msg);
                // DELTA:add:4:Interior_Monitoring
                current_alarm_system_state = AlarmSystemState::AS_OFF;
            }   else if (time_alarm_elapsed_value) {
                as_alarm_was_detected->publish(msg);
                as_alarm_off->publish(msg);
                // DELTA:add:5:Interior_Monitoring
                current_alarm_system_state = AlarmSystemState::AS_ALARM;
            }  // DELTA:add:5:Control_AS 
            // DELTA:add:6:Control_AS
            break;
    }
}
// DELTA:delete:5:AS:END

// DELTA:delete:5:CLS
void Security::CLS::initializeROS(rclcpp::Node* node) {

    // Standard Components
    key_pos_lock = node->create_subscription<std_msgs::msg::Bool>(
        "key_pos_lock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            key_pos_lock_value = msg->data;
            cls();
            key_pos_lock_value = false;
        }
    );
    key_pos_unlock = node->create_subscription<std_msgs::msg::Bool>(
        "key_pos_unlock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            key_pos_unlock_value = msg->data;
            cls();
            key_pos_unlock_value = false;
        }
    );
    cls_lock = node->create_publisher<std_msgs::msg::Bool>("cls_lock", 10);
    cls_unlock = node->create_publisher<std_msgs::msg::Bool>("cls_unlock", 10);

    // DELTA:add:2:RCK

    // DELTA:add:2:Automatic_Locking
}

void Security::CLS::cls() {
    std_msgs::msg::Bool msg;
    msg.data = true;

    switch (current_cls_state) {
        case CentrolLockingSystemState::CLS_UNLOCK:
            if (key_pos_lock_value) {
                cls_lock->publish(msg);
                current_cls_state = CentrolLockingSystemState::CLS_LOCK;
            }  // DELTA:add:3:RCK
            // DELTA:add:3:Automatic_Locking
            break;

        case CentrolLockingSystemState::CLS_LOCK:
            if (key_pos_unlock_value) {
                cls_unlock->publish(msg);
                current_cls_state = CentrolLockingSystemState::CLS_UNLOCK;
            } // DELTA:add:4:RCK
            // DELTA:add:4:Automatic_Locking
            break;
    }
}
// DELTA:delete:5:CLS:END

// DELTA:delete:9:RCK
void Security::RCK::initializeROS(rclcpp::Node* node) {

    // Standard Components
    rck_but_lock = node->create_subscription<std_msgs::msg::Bool>(
        "rck_but_lock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            rck_lock->publish(new_msg);
            rck_but_lock_value = msg->data;
        }
    );
    rck_but_unlock = node->create_subscription<std_msgs::msg::Bool>(
        "rck_but_unlock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            rck_unlock->publish(new_msg);
            rck_but_unlock_value = msg->data;
            // DELTA:add:5:Safety_Function
        }
    );
    rck_lock = node->create_publisher<std_msgs::msg::Bool>("rck_lock", 10);
    rck_unlock = node->create_publisher<std_msgs::msg::Bool>("rck_unlock", 10);

    // DELTA:add:2:Control_Automatic_PW
    
    // DELTA:add:2:Safety_Function
    
}

// DELTA:add:3:Safety_Function
// DELTA:delete:9:RCK:END
