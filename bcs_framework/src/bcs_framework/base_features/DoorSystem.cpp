#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "DoorSystem.hpp"

DoorSystem::DoorSystem() : Node("door_system") {
    power_window.initializeROS(this);
    exterior_mirror.initializeROS(this);
}

void DoorSystem::PW::initializeROS(rclcpp::Node* node) {

    pw_but_up = node->create_subscription<std_msgs::msg::Bool>(
        "pw_but_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            pw_but_up_value = msg->data;
            // DELTA:change:5:Automatic_PW
            processManual();
            // DELTA:change:5:Automatic_PW:END
	        pw_but_up_value = false;
        }
    );
    pw_but_dn = node->create_subscription<std_msgs::msg::Bool>(
        "pw_but_dn", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            pw_but_dn_value = msg->data;
            processFingerProtection();
            // DELTA:change:6:Automatic_PW
            processManual();
            // DELTA:change:6:Automatic_PW:END
            pw_but_dn_value = false;
        }
    );
    pw_pos_up = node->create_subscription<std_msgs::msg::Bool>(
        "pw_pos_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            pw_pos_up_value = msg->data;
            // DELTA:add:10:Automatic_PW
        }
    );
    pw_pos_dn = node->create_subscription<std_msgs::msg::Bool>(
        "pw_pos_dn", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            pw_pos_dn_value = msg->data;
            // DELTA:add:11:Automatic_PW
        }
    );
    fp_on_sub = node->create_subscription<std_msgs::msg::Bool>(
        "fp_on", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            fp_on_value = msg->data;
            fp_on_value = true;
            // DELTA:add:13:Automatic_PW
        }
    );
    fp_off_sub = node->create_subscription<std_msgs::msg::Bool>(
        "fp_off", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            fp_off_value = msg->data;
            fp_on_value = false;
            // DELTA:add:14:Automatic_PW
        }
    );
    // DELTA:delete:2:CLS
    cls_lock = node->create_subscription<std_msgs::msg::Bool>(
        "cls_lock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            pw_enabled_state_ = msg->data;
            pw_enabled_state_ = false;
            // DELTA:change:8:Automatic_PW
            processManual();
            // DELTA:change:8:Automatic_PW:END
        }
    );
    cls_unlock = node->create_subscription<std_msgs::msg::Bool>(
        "cls_unlock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            pw_enabled_state_ = msg->data;
            pw_enabled_state_ = true;
            // DELTA:change:9:Automatic_PW
            processManual();
            // DELTA:change:9:Automatic_PW:END
        }
    );
    // DELTA:delete:2:CLS:END

    // DELTA:change:3:Automatic_PW
    // Manual window
    pw_mv_up = node->create_publisher<std_msgs::msg::Bool>("pw_mv_up", 10);
    pw_mv_dn = node->create_publisher<std_msgs::msg::Bool>("pw_mv_dn", 10);
    // DELTA:change:3:Automatic_PW:END

    // Finger protection
    fp_on = node->create_publisher<std_msgs::msg::Bool>("fp_on", 10);
    fp_off = node->create_publisher<std_msgs::msg::Bool>("fp_off", 10);
    finger_detected = node->create_subscription<std_msgs::msg::Bool>(
        "finger_detected", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            finger_detected_value = msg->data;
            processFingerProtection();
        }
    );
        

}

// DELTA:change:4:Automatic_PW
// DELTA:change:12:Automatic_PW
void DoorSystem::PW::processManual() {
    std_msgs::msg::Bool msg;
    msg.data = true;

    switch (current_window_state) {
        case WindowState::PW_UP:
            if (pw_but_dn_value // DELTA:add:6:CLS
            ) {
                current_window_state = WindowState::PW_PEND;
                pw_mv_dn->publish(msg);
            }
            break;

        case WindowState::PW_PEND:
            if (pw_but_up_value && pw_pos_up_value) {
                current_window_state = WindowState::PW_UP;
            } else if (pw_but_dn_value && !pw_pos_dn_value // DELTA:add:7:CLS
            ) {
                std::cout << "ERROR 1" << std::endl;
                pw_mv_dn->publish(msg);
            } else if (pw_but_up_value && !fp_on_value && !pw_pos_up_value) {
                std::cout << "ERROR 13" << std::endl;
                pw_mv_up->publish(msg);
            } else if (pw_but_dn_value && pw_pos_dn_value) {
                current_window_state = WindowState::PW_DN;
            }
            break;

        case WindowState::PW_DN:
            if (pw_but_up_value && !fp_on_value) {
                current_window_state = WindowState::PW_PEND;
                pw_mv_up->publish(msg);
            }
            break;
    }
}
// DELTA:change:12:Automatic_PW:END
// DELTA:change:4:Automatic_PW:END

void DoorSystem::PW::processFingerProtection() {
    std_msgs::msg::Bool msg;
    msg.data = true;

    switch (current_fp_state) {
        case FingerProtectionState::FP_OFF:
            // DELTA:change:7:Automatic_PW
            if (finger_detected_value) {
                current_fp_state = FingerProtectionState::FP_ON;
                msg.data = true;
                fp_on->publish(msg);
                msg.data = false;
                fp_off->publish(msg);
            } 
            // DELTA:change:7:Automatic_PW:END
            break;

        case FingerProtectionState::FP_ON:
            if (pw_but_dn_value ) {
                current_fp_state = FingerProtectionState::FP_OFF;
                msg.data = false;
                fp_on->publish(msg);
                msg.data = true;
                fp_off->publish(msg);
            } 
            break;
    }
}

void DoorSystem::EM::initializeROS(rclcpp::Node* node) {

    // Electric mirror
    em_mv_left = node->create_publisher<std_msgs::msg::Bool>("em_mv_left", 10);
    em_mv_right = node->create_publisher<std_msgs::msg::Bool>("em_mv_right", 10);
    em_mv_up = node->create_publisher<std_msgs::msg::Bool>("em_mv_up", 10);
    em_mv_down = node->create_publisher<std_msgs::msg::Bool>("em_mv_down", 10);
    em_but_right = node->create_subscription<std_msgs::msg::Bool>(
        "em_but_right", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            em_but_right_value = msg->data;
            processElectric();
            em_but_right_value = false;
        }
    );
    em_but_down = node->create_subscription<std_msgs::msg::Bool>(
        "em_but_down", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            em_but_down_value = msg->data;
            processElectric();
            em_but_down_value = false;
        }
    );
    em_but_up = node->create_subscription<std_msgs::msg::Bool>(
        "em_but_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            em_but_up_value = msg->data;
            processElectric();
            em_but_up_value = false;
        }
    );
    em_but_left = node->create_subscription<std_msgs::msg::Bool>(
        "em_but_left", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            em_but_left_value = msg->data;
            processElectric();
            em_but_left_value = false;
        }
    );
    em_pos_right = node->create_subscription<std_msgs::msg::Bool>(
        "em_pos_right", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            em_pos_right_value = msg->data;
            processElectric();
            em_pos_right_value = false;
        }
    );
    em_pos_bottom = node->create_subscription<std_msgs::msg::Bool>(
        "em_pos_bottom", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            em_pos_bottom_value = msg->data;
            processElectric();
            em_pos_bottom_value = false;
        }
    );
    em_pos_top = node->create_subscription<std_msgs::msg::Bool>(
        "em_pos_top", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            em_pos_top_value = msg->data;
            processElectric();
            em_pos_top_value = false;
        }
    );
    em_pos_left = node->create_subscription<std_msgs::msg::Bool>(
        "em_pos_left", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            em_pos_left_value = msg->data;
            processElectric();
            em_pos_left_value = false;
        }
    );

    // DELTA:add:3:Heatable
    // DELTA:add:6:LED_EM

}

// DELTA:change:7:LED_EM
void DoorSystem::EM::processElectric() {
    std_msgs::msg::Bool msg;
    msg.data = true;

    switch (current_em_state) {
        case ElectricMirrorState::EM_TOP:
            if (em_but_right_value) {
                em_mv_right->publish(msg);
            } else if (em_but_left_value) {
                em_mv_left->publish(msg);
            } else if (em_pos_left_value) {
                current_em_state = ElectricMirrorState::EM_TOP_LEFT;
            } else if (em_pos_right_value) {
                current_em_state = ElectricMirrorState::EM_TOP_RIGHT;
            } else if (em_but_down_value) {
                em_mv_down->publish(msg);
                current_em_state = ElectricMirrorState::EM_PENDING;
            }
            break;

        case ElectricMirrorState::EM_TOP_LEFT:
            if (em_but_right_value) {
                em_mv_right->publish(msg);
                current_em_state = ElectricMirrorState::EM_TOP;
            } else if (em_but_down_value) {
                em_mv_down->publish(msg);
                current_em_state = ElectricMirrorState::EM_LEFT;
            }
            break;

        case ElectricMirrorState::EM_LEFT:
            if (em_but_up_value) {
                em_mv_up->publish(msg);
            } else if (em_but_down_value) {
                em_mv_down->publish(msg);
            } else if (em_pos_top_value) {
                current_em_state = ElectricMirrorState::EM_TOP_LEFT;
            } else if (em_pos_bottom_value) {
                current_em_state = ElectricMirrorState::EM_BOTTOM_LEFT;
            } else if (em_but_right_value) {
                em_mv_right->publish(msg);
                current_em_state = ElectricMirrorState::EM_PENDING;
            }
            break;

        case ElectricMirrorState::EM_BOTTOM_LEFT:
            if (em_but_up_value) {
                em_mv_up->publish(msg);
                current_em_state = ElectricMirrorState::EM_LEFT;
            } else if (em_but_right_value) {
                em_mv_right->publish(msg);
                current_em_state = ElectricMirrorState::EM_BOTTOM;
            }
            break;

        case ElectricMirrorState::EM_BOTTOM:
            if (em_but_right_value) {
                em_mv_right->publish(msg);
            } else if (em_but_left_value) {
                em_mv_left->publish(msg);
            } else if (em_pos_left_value) {
                current_em_state = ElectricMirrorState::EM_BOTTOM_LEFT;
            } else if (em_pos_right_value) {
                current_em_state = ElectricMirrorState::EM_BOTTOM_RIGHT;
            } else if (em_but_up_value) {
                em_mv_up->publish(msg);
                current_em_state = ElectricMirrorState::EM_PENDING;
            }
            break;
            
        case ElectricMirrorState::EM_BOTTOM_RIGHT:
            if (em_but_left_value) {
                em_mv_left->publish(msg);
                current_em_state = ElectricMirrorState::EM_BOTTOM;
            } else if (em_but_up_value) {
                em_mv_up->publish(msg);
                current_em_state = ElectricMirrorState::EM_RIGHT;
            }
            break;

        case ElectricMirrorState::EM_RIGHT:
            if (em_but_up_value) {
                em_mv_up->publish(msg);
            } else if (em_but_down_value) {
                em_mv_down->publish(msg);
            } else if (em_pos_top_value) {
                current_em_state = ElectricMirrorState::EM_TOP_RIGHT;
            } else if (em_pos_bottom_value) {
                current_em_state = ElectricMirrorState::EM_BOTTOM_RIGHT;
            } else if (em_but_left_value) {
                em_mv_left->publish(msg);
                current_em_state = ElectricMirrorState::EM_PENDING;
            }
            break;

        case ElectricMirrorState::EM_TOP_RIGHT:
            if (em_but_down_value) {
                em_mv_down->publish(msg);
                current_em_state = ElectricMirrorState::EM_RIGHT;
            } else if (em_but_left_value) {
                em_mv_left->publish(msg);
                current_em_state = ElectricMirrorState::EM_TOP;
            }
            break;

        case ElectricMirrorState::EM_PENDING:
            if (em_but_up_value) {
                em_mv_up->publish(msg);
            } else if (em_but_down_value) {
                em_mv_down->publish(msg);
            } else if (em_but_right_value) {
                em_mv_right->publish(msg);
            } else if (em_but_left_value) {
                em_mv_left->publish(msg);
            } else if (em_pos_left_value) {
                current_em_state = ElectricMirrorState::EM_LEFT;
            } else if (em_pos_right_value) {
                current_em_state = ElectricMirrorState::EM_RIGHT;
            } else if (em_pos_top_value) {
                current_em_state = ElectricMirrorState::EM_TOP;
            } else if (em_pos_bottom_value) {
                current_em_state = ElectricMirrorState::EM_BOTTOM;
            } 
            break;
        
    }
}
// DELTA:change:7:LED_EM:END

// DELTA:add:4:Heatable
