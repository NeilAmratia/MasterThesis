#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "Security.hpp"

Security::Security() : Node("security_feature") {
    
    // DELTA:delete:4:CLS
    cls.initializeROS(this);
    // DELTA:delete:4:CLS:END
    // DELTA:delete:8:RCK
    rck.initializeROS(this);
    // DELTA:delete:8:RCK:END
}



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

    // Remote Control Key Components
    rck_lock_sub_2 = node->create_subscription<std_msgs::msg::Bool>(
        "rck_lock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            rck_lock_sub_2_value = msg->data;
            cls();
            rck_lock_sub_2_value = false;
        }
    );
    rck_unlock_sub_2 = node->create_subscription<std_msgs::msg::Bool>(
        "rck_unlock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            rck_unlock_sub_2_value = msg->data;
            cls();
            rck_unlock_sub_2_value = false;
        }
    );
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
            }  else if (rck_lock_sub_2_value) {
                   cls_lock->publish(msg);
                   current_cls_state = CentrolLockingSystemState::CLS_LOCK;
               }
// DELTA:add:3:RCK
            // DELTA:add:3:Automatic_Locking
            break;

        case CentrolLockingSystemState::CLS_LOCK:
            if (key_pos_unlock_value) {
                cls_unlock->publish(msg);
                current_cls_state = CentrolLockingSystemState::CLS_UNLOCK;
            } else if (rck_unlock_sub_2_value) {
                  std::cout << "ERROR 9" << std::endl;
                  cls_unlock->publish(msg);
                  current_cls_state = CentrolLockingSystemState::CLS_UNLOCK;
              } 
// DELTA:add:4:RCK
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
            rck_sf();
            rck_but_unlock_value = false;
// DELTA:add:5:Safety_Function
        }
    );
    rck_lock = node->create_publisher<std_msgs::msg::Bool>("rck_lock", 10);
    rck_unlock = node->create_publisher<std_msgs::msg::Bool>("rck_unlock", 10);

    // Automatic Power Window Components
    pw_rm_up = node->create_subscription<std_msgs::msg::Bool>(
        "pw_rm_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std::cout << "ERROR 7" << std::endl;
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            pw_but_up->publish(new_msg);
        }
    );
    pw_rm_dn = node->create_subscription<std_msgs::msg::Bool>(
        "pw_rm_dn", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std_msgs::msg::Bool new_msg;
            new_msg.data = msg->data;
            pw_but_dn->publish(new_msg);
        }
    );
    pw_but_up = node->create_publisher<std_msgs::msg::Bool>("pw_but_up", 10);
    pw_but_dn = node->create_publisher<std_msgs::msg::Bool>("pw_but_dn", 10);
// DELTA:add:2:Control_Automatic_PW
    
    // Safety Functions Components
    door_open_2 = node->create_subscription<std_msgs::msg::Bool>(
        "door_open", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            door_open_value_2 = msg->data;
            rck_sf();
        }
    );
    time_rck_sf_elapsed = node->create_subscription<std_msgs::msg::Bool>(
        "time_rck_sf_elapsed", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            time_rck_sf_elapsed_value = msg->data;
            rck_sf();
        }
    );
// DELTA:add:2:Safety_Function
    
}

void Security::RCK::rck_sf() {
    std_msgs::msg::Bool msg;
    msg.data = true;

    switch (current_rck_sf_state) {
        case RemoteSafetyFunctionsState::SF_OFF:
            if (rck_but_unlock_value) {
                rck_unlock->publish(msg);
                current_rck_sf_state = RemoteSafetyFunctionsState::SF_ON;
            }
            break;

        case RemoteSafetyFunctionsState::SF_ON:
            if (door_open_value_2) {
		        std::cout << "ERROR 8" << std::endl;
                rck_lock->publish(msg);
                current_rck_sf_state = RemoteSafetyFunctionsState::SF_OFF;
            } else if (time_rck_sf_elapsed_value) {
                rck_lock->publish(msg);
                current_rck_sf_state = RemoteSafetyFunctionsState::SF_OFF;
            }
            break;
    }
}
// DELTA:add:3:Safety_Function
// DELTA:delete:9:RCK:END
