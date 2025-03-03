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

    // LED Finger Protection
    fp_on = node->create_subscription<std_msgs::msg::Bool>(
        "fp_on", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            fp_on_value = msg->data;
            LED_FP();
            fp_on_value = false;
        }
    );
    fp_off = node->create_subscription<std_msgs::msg::Bool>(
        "fp_off", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            fp_off_value = msg->data;
            LED_FP();
            fp_off_value = false;
        }
    );
    led_fp_on = node->create_publisher<std_msgs::msg::Bool>("led_fp_on", 10);
    led_fp_off = node->create_publisher<std_msgs::msg::Bool>("led_fp_off", 10);
// DELTA:add:3:LED_FP
    
    // DELTA:add:3:LED_CLS

    // LED Power Window
    // DELTA:add:4:LED_PW
    // Automatic Window
    pw_auto_mv_dn = node->create_subscription<std_msgs::msg::Bool>(
        "pw_auto_mv_dn", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            pw_auto_mv_dn_value = msg->data;
            LED_AutoPW_dn();
        }
    );
    pw_auto_mv_up = node->create_subscription<std_msgs::msg::Bool>(
        "pw_auto_mv_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            pw_auto_mv_up_value = msg->data;
            LED_AutoPW_up();
        }
    );
    pw_auto_mv_stop = node->create_subscription<std_msgs::msg::Bool>(
        "pw_auto_mv_stop", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            pw_auto_mv_stop_value = msg->data;
            LED_AutoPW_up();
            LED_AutoPW_dn();
        }
    );
    
    led_pw_up_on = node->create_publisher<std_msgs::msg::Bool>("led_pw_up_on", 10);
    led_pw_up_off = node->create_publisher<std_msgs::msg::Bool>("led_pw_up_off", 10);
    led_pw_dn_on = node->create_publisher<std_msgs::msg::Bool>("led_pw_dn_on", 10);
    led_pw_dn_off = node->create_publisher<std_msgs::msg::Bool>("led_pw_dn_off", 10);
// DELTA:add:5:LED_PW
    cls_lock_2 = node->create_subscription<std_msgs::msg::Bool>(
        "cls_lock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            cls_lock_2_value = msg->data;
            cls_lock_2_value = true;
            cls_unlock_2_value = false;
            LED_AutoPW_up();
        }
    );
    cls_unlock_2 = node->create_subscription<std_msgs::msg::Bool>(
        "cls_unlock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            cls_unlock_2_value = msg->data;
            cls_lock_2_value = false;
            cls_unlock_2_value = true;
            LED_AutoPW_up();
        }
    );
    led_pw_cls_up_on = node->create_publisher<std_msgs::msg::Bool>("led_pw_cls_up_on", 10);
    led_pw_cls_up_off = node->create_publisher<std_msgs::msg::Bool>("led_pw_cls_up_off", 10);
// DELTA:add:10:LED_PW

    // DELTA:add:3:LED_EM

    // DELTA:add:3:LED_Heatable
}

// DELTA:add:4:LED_AS
// DELTA:add:10:LED_AS

void HMI::StatusLED::LED_FP() {
    std_msgs::msg::Bool msg;
    msg.data = true;

    switch (current_led_fp_state) {
        case FingerProtectionState::FP_LED_OFF:
            if (fp_on_value) {
                led_fp_on->publish(msg);
                current_led_fp_state = FingerProtectionState::FP_LED_ON;
            } 
            break;

        case FingerProtectionState::FP_LED_ON:
            if (fp_off_value) {
                led_fp_off->publish(msg);
                current_led_fp_state = FingerProtectionState::FP_LED_OFF;
            } 
            break;
    }
}
// DELTA:add:4:LED_FP

// DELTA:add:4:LED_CLS

// DELTA:add:6:LED_PW
void HMI::StatusLED::LED_AutoPW_up() {
    std_msgs::msg::Bool msg;
    msg.data = true;

    switch (current_led_AutoPW_up_state) {
        case AutoPWUpState::AUTOPW_UP_LED_OFF:
            if (pw_auto_mv_up_value && cls_unlock_2_value) {
                led_pw_up_on->publish(msg);
                current_led_AutoPW_up_state = AutoPWUpState::AUTOPW_UP_LED_ON;
            } else if (pw_auto_mv_up_value && cls_lock_2_value) {
                std::cout << "ERROR 11" << std::endl;
                led_pw_cls_up_on->publish(msg);
                current_led_AutoPW_up_state = AutoPWUpState::AUTOPW_UP_CLS_LED_ON;
            }
            break;

        case AutoPWUpState::AUTOPW_UP_LED_ON:
            if (pw_auto_mv_stop_value) {
                led_pw_up_off->publish(msg);
                current_led_AutoPW_up_state = AutoPWUpState::AUTOPW_UP_LED_OFF;
            } 
            break;
            
        case AutoPWUpState::AUTOPW_UP_CLS_LED_ON:
            if (pw_auto_mv_stop_value) {
                led_pw_cls_up_off->publish(msg);
                current_led_AutoPW_up_state = AutoPWUpState::AUTOPW_UP_LED_OFF;
            } 
            break;
    }
}

void HMI::StatusLED::LED_AutoPW_dn() {
    std_msgs::msg::Bool msg;
    msg.data = true;

    switch (current_led_AutoPW_dn_state) {
        case AutoPWDnState::AUTOPW_DN_LED_OFF:
            if (pw_auto_mv_dn_value) {
                led_pw_dn_on->publish(msg);
                current_led_AutoPW_dn_state = AutoPWDnState::AUTOPW_DN_LED_ON;
            } 
            break;

        case AutoPWDnState::AUTOPW_DN_LED_ON:
            if (pw_auto_mv_stop_value) {
                led_pw_dn_off->publish(msg);
                current_led_AutoPW_dn_state = AutoPWDnState::AUTOPW_DN_LED_OFF;
            } 
            break;
    }
}
// DELTA:add:11:LED_PW

// DELTA:add:4:LED_EM

// DELTA:add:4:LED_Heatable
