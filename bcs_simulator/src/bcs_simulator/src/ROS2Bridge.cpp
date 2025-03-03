#include "ROS2Bridge.hpp"

ROS2Bridge::ROS2Bridge(QObject* parent) : QObject(parent), windowPosition(100), mirrorYPosition(50), mirrorXPosition(50), isMovingUp(false), isMovingDown(false) {
    
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("BCS_Simulator");

    autoMoveTimer = new QTimer(this);
    autoMoveTimer->setInterval(100); // Update every 100ms

    heatingTimer = new QTimer(this);
    heatingTimer->setInterval(30000);
    heatingTimer->setSingleShot(true);
    connect(heatingTimer, &QTimer::timeout, this, &ROS2Bridge::handleHeatingTimeout);
    
    remoteTimer = new QTimer(this);
    remoteTimer->setInterval(10000);
    remoteTimer->setSingleShot(true);
    connect(remoteTimer, &QTimer::timeout, this, &ROS2Bridge::handleRemoteTimeout);
    
    alarmTimer = new QTimer(this);
    alarmTimer->setInterval(5000);
    alarmTimer->setSingleShot(true);
    connect(alarmTimer, &QTimer::timeout, this, &ROS2Bridge::handleAlarmTimeout);
        
    // Setup connections
    setupConnections();
    
    // Initialize publishers and subscribers
    initializePublishers();
    initializeSubscribers();
    
    // Start ROS thread
    ros_thread = std::thread([this]() {
        rclcpp::spin(node);
    });
}

void ROS2Bridge::setupConnections() {
    
    connect(autoMoveTimer, &QTimer::timeout, this, &ROS2Bridge::updateAutoMove);
    connect(this, &ROS2Bridge::startAutoMoveRequested, this, &ROS2Bridge::startAutoMoveTimer);
    connect(this, &ROS2Bridge::stopAutoMoveRequested, this, &ROS2Bridge::stopAutoMoveTimer);
}


void ROS2Bridge::initializePublishers() {
    // HMI
    windowUpPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_but_mv_up", 10);
    windowDownPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_but_mv_dn", 10);
    mirrorUpPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_mv_up", 10);
    mirrorDownPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_mv_dn", 10);
    mirrorLeftPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_mv_left", 10);
    mirrorRightPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_mv_right", 10);
    activateAlarmPublisher = node->create_publisher<std_msgs::msg::Bool>("activate_as", 10);
    deactivateAlarmPublisher = node->create_publisher<std_msgs::msg::Bool>("deactivate_as", 10);
    confirmAlarmPublisher = node->create_publisher<std_msgs::msg::Bool>("confirm_alarm", 10);
    releaseWindowUpPublisher = node->create_publisher<std_msgs::msg::Bool>("release_pw_but_up", 10);
    releaseWindowDownPublisher = node->create_publisher<std_msgs::msg::Bool>("release_pw_but_dn", 10);

    // Power Window
    windowPositionUpPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_pos_up", 10);
    windowPositionDownPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_pos_dn", 10);

    // Finger Protection
    fingerDetectedPublisher = node->create_publisher<std_msgs::msg::Bool>("finger_detected", 10);
    
    // Exterior Mirror
    mirrorPosUpPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_top", 10);
    mirrorPosDownPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_bottom", 10);
    mirrorPosRightPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_right", 10);
    mirrorPosLeftPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_left", 10);
    mirrorColdPublisher = node->create_publisher<std_msgs::msg::Bool>("em_too_cold", 10);
    heaterTimerPublisher = node->create_publisher<std_msgs::msg::Bool>("time_heating_elapsed", 10);

    // Remote Control key
    remoteWindowUpPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_rm_up", 10);
    remoteWindowDownPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_rm_dn", 10);
    remoteLockPublisher = node->create_publisher<std_msgs::msg::Bool>("rck_but_lock", 10);
    remoteUnlockPublisher = node->create_publisher<std_msgs::msg::Bool>("rck_but_unlock", 10);
    carDoorPublisher = node->create_publisher<std_msgs::msg::Bool>("door_open", 10);
    remoteTimerPublisher = node->create_publisher<std_msgs::msg::Bool>("time_rck_sf_elapsed", 10);

    // Central Locking System
    centralLockPublisher = node->create_publisher<std_msgs::msg::Bool>("key_pos_lock", 10);
    centralUnlockPublisher = node->create_publisher<std_msgs::msg::Bool>("key_pos_unlock", 10);
    carDrivingPublisher = node->create_publisher<std_msgs::msg::Bool>("car_drives", 10);

    // Alarm System
    alarmASTriggerPublisher = node->create_publisher<std_msgs::msg::Bool>("as_alarm_detected", 10);
    alarmIMTriggerPublisher = node->create_publisher<std_msgs::msg::Bool>("im_alarm_detected", 10);
    alarmTimerPublisher = node->create_publisher<std_msgs::msg::Bool>("time_alarm_elapsed", 10);


}

void ROS2Bridge::initializeSubscribers() {

    // Manual Window Movement subscribers
    windowMoveUpSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_mv_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->handleWindowMoveUp(msg);}
    );
    windowMoveDownSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_mv_dn", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->handleWindowMoveDown(msg);}
    );

    // Automatic Window Movement subscribers
    windowAutoMoveUpSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_auto_mv_up", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->handleWindowAutoMoveUp(msg);}
    );
    windowAutoMoveDownSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_auto_mv_dn", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->handleWindowAutoMoveDown(msg);}
    );
    windowAutoMoveStopSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_auto_mv_stop", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->handleWindowAutoMoveStop(msg);}
    );

    // Power Window LED subscribers
    windowUpLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_up_on", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->windowUpLedOnMessage(msg);}
    );
    windowDownLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_dn_on", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->windowDownLedOnMessage(msg);}
    );
    windowUpLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_up_off", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->windowUpLedOffMessage(msg);}
    );
    windowDownLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_dn_off", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->windowDownLedOffMessage(msg);}
    );
    centralWindowUpLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_pw_cls_up_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->centralWindowUpLedOnMessage(msg);}
    );
    centralWindowUpLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_pw_cls_up_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->centralWindowUpLedOffMessage(msg);}
    );

    // Finger Protection subscribers
    fingerProtectionLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_fp_on", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->fingerProtectionLedOnMessage(msg);}
    );
    fingerProtectionLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_fp_off", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->fingerProtectionLedOffMessage(msg);}
    );

    // Mirror Movement subscribers
    mirrorUpSubscriber = node->create_subscription<std_msgs::msg::Bool>( "em_mv_up", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) { this->handleMirrorUp(msg);}
    );
    mirrorDownSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_mv_down", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->handleMirrorDown(msg);}
    );
    mirrorLeftSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_mv_left", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->handleMirrorLeft(msg);}
    );
    mirrorRightSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_mv_right", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->handleMirrorRight(msg);}
    );
    // Mirror Heating
    mirrorHeatingOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("heating_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorHeatingOn(msg);}
    );
    mirrorHeatingOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("heating_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorHeatingOff(msg);}
    );
    
    // Mirror LED subscriber
    mirrorUpLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_em_top_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) { this->mirrorUpLedOnMessage(msg);}
    );
    mirrorUpLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_em_top_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) { this->mirrorUpLedOffMessage(msg);}
    );
    mirrorDownLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_bottom_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorDownLedOnMessage(msg);}
    );
    mirrorDownLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_bottom_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorDownLedOffMessage(msg);}
    );
    mirrorLeftLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_left_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorLeftLedOnMessage(msg);}
    );
    mirrorLeftLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_left_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorLeftLedOffMessage(msg);}
    );
    mirrorRightLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_right_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorRightLedOnMessage(msg);}
    );
    mirrorRightLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_right_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorRightLedOffMessage(msg);}
    );
    mirrorHeaterLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_heating_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorHeaterLedOnMessage(msg);}
    );
    mirrorHeaterLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_heating_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->mirrorHeaterLedOffMessage(msg);}
    );


    // Car Lock subscribers
    carLockedSubscriber = node->create_subscription<std_msgs::msg::Bool>("car_lock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->carLockedMessage(msg);}
    );
    carUnlockedSubscriber = node->create_subscription<std_msgs::msg::Bool>("car_unlock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->carUnlockedMessage(msg);}
    );

    // CLS LED 
    clsLockSubscriber = node->create_subscription<std_msgs::msg::Bool>("cls_lock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->clsLockMessage(msg);}
    );
    clsUnlockSubscriber = node->create_subscription<std_msgs::msg::Bool>("cls_unlock", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->clsUnlockMessage(msg);}
    );
    centralLockLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_cls_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->centralLockLedOnMessage(msg);}
    );
    centralLockLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_cls_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->centralLockLedOffMessage(msg);}
    );

    // AS 
    alarmSignalOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "as_alarm_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmSignalOnMessage(msg);}
    );
    alarmSignalOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "as_alarm_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmSignalOffMessage(msg);}
    );
    alarmActivationOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "as_active_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmActivationOnMessage(msg);}
    );
    alarmActivationOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "as_active_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmActivationOffMessage(msg);}
    );
    alarmInteriorOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "as_im_alarm_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmInteriorOnMessage(msg);}
    );
    alarmInteriorOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "as_im_alarm_off", 10, 
    [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmInteriorOffMessage(msg);}
    );

    // AS LED
    alarmDetectedLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_as_alarm_detected_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmDetectedLedOnMessage(msg);}
    );
    alarmDetectedLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_as_alarm_detected_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmDetectedLedOffMessage(msg);}
    );
    alarmSignalLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_as_alarm_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmSignalLedOnMessage(msg);}
    );
    alarmSignalLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_as_alarm_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmSignalLedOffMessage(msg);}
    );
    alarmActivationLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_as_active_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmActivationLedOnMessage(msg);}
    );
    alarmActivationLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_as_active_off", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmActivationLedOffMessage(msg);}
    );
    alarmInteriorLedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_as_im_alarm_on", 10, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmInteriorLedOnMessage(msg);}
    );
    alarmInteriorLedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>( "led_as_im_alarm_off", 10, 
    [this](const std_msgs::msg::Bool::SharedPtr msg) {this->alarmInteriorLedOffMessage(msg);}
    );
}

// Method to publish messages when buttons are pressed or output is generated from simulator
Q_INVOKABLE void ROS2Bridge::publishWindowUp() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    windowUpPublisher->publish(message);
    addMessage(
        "Window Up", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishWindowDown() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    windowDownPublisher->publish(message);
    addMessage(
        "Window Down", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishWindowPositionUp(bool position) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = position;
    windowPositionUpPublisher->publish(message);
    if (position == 1) {
        emit windowPositionUpMaxSignal();
        addMessage(
            "Window Position Up Max", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
Q_INVOKABLE void ROS2Bridge::publishWindowPositionDown(bool position) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = position;
    windowPositionDownPublisher->publish(message);
    if (position == 1) {
        emit windowPositionDownMaxSignal();
        addMessage(
            "Window Position Down Max", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
Q_INVOKABLE void ROS2Bridge::fingerDetected(bool detection) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = detection;
    fingerDetectedPublisher->publish(message);
    if (detection == 1) {
        addMessage(
            "Finger Detected", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    } else {
        addMessage(
            "Finger removed", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
Q_INVOKABLE void ROS2Bridge::publishEMPosTop(bool top) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = top;
    if (top == 1) {
        mirrorPosUpPublisher->publish(message);
        emit mirrorPosUpSignal();
        addMessage(
            "External Mirror Top position Max", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    } else if (top == 0) {
        mirrorPosUpPublisher->publish(message);
    }
}
Q_INVOKABLE void ROS2Bridge::publishEMPosBottom(bool bottom) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = bottom;
    if (bottom == 1) {
        mirrorPosDownPublisher->publish(message);
        emit mirrorPosDownSignal();
        addMessage(
            "External Mirror Bottom position Max", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    } else if (bottom == 0) {
        mirrorPosDownPublisher->publish(message);
    }
}
Q_INVOKABLE void ROS2Bridge::publishEMPosLeft(bool left) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = left;
    if (left == 1) {
        mirrorPosLeftPublisher->publish(message);
        emit mirrorPosLeftSignal();
        addMessage(
            "External Mirror Left position Max", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    } else if (left == 0) {
        mirrorPosLeftPublisher->publish(message);
    }
}
Q_INVOKABLE void ROS2Bridge::publishEMPosRight(bool right) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = right;
    if (right == 1) {
        mirrorPosRightPublisher->publish(message);
        emit mirrorPosRightSignal();
        addMessage(
            "External Mirror Right position Max", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    } else if (right == 0) {
        mirrorPosRightPublisher->publish(message);
    }
}
Q_INVOKABLE void ROS2Bridge::publishEMCold(bool cold) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = cold;
    mirrorColdPublisher->publish(message);
    if (cold == 1){
        addMessage(
            "Mirror Cold", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
Q_INVOKABLE void ROS2Bridge::publishMirrorUp() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    mirrorUpPublisher->publish(message);
    addMessage(
        "Mirror Up", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishMirrorDown() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    mirrorDownPublisher->publish(message);
    addMessage(
        "Mirror Down", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishMirrorLeft() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    mirrorLeftPublisher->publish(message);
    addMessage(
        "Mirror Left", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishMirrorRight() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    mirrorRightPublisher->publish(message);
    addMessage(
        "Mirror Right", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishAlarmActivate() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    activateAlarmPublisher->publish(message);
    addMessage(
        "Alarm Activate", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishAlarmDeactivate() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    deactivateAlarmPublisher->publish(message);
    QMetaObject::invokeMethod(alarmTimer, "stop", Qt::QueuedConnection);
    addMessage(
        "Alarm Deactivate", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishAlarmConfirm() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    confirmAlarmPublisher->publish(message);
    addMessage(
        "Alarm Confirmed", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishAlarmTriggered() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    alarmASTriggerPublisher->publish(message);
    QMetaObject::invokeMethod(alarmTimer, "start", Qt::QueuedConnection);
    addMessage(
        "Alarm Triggered", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishAlarmIMTriggered() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    alarmIMTriggerPublisher->publish(message);
    QMetaObject::invokeMethod(alarmTimer, "start", Qt::QueuedConnection);
    addMessage(
        "Interior Monitoring Alarm Triggered", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishReleaseWindowUp() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    releaseWindowUpPublisher->publish(message);
    addMessage(
        "Release Window Up", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishReleaseWindowDown() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    releaseWindowDownPublisher->publish(message);
    addMessage(
        "Release Window Down", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishRemoteWindowUp() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    remoteWindowUpPublisher->publish(message);
    addMessage(
        "Remote Window Up", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishRemoteWindowDown() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    remoteWindowDownPublisher->publish(message);
    addMessage(
        "Remote Window Down", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishRemoteLock() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    remoteLockPublisher->publish(message);
    addMessage(
        "Remote Lock", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishRemoteUnlock() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    remoteUnlockPublisher->publish(message);
    QMetaObject::invokeMethod(remoteTimer, "start", Qt::QueuedConnection);
    QMetaObject::invokeMethod(alarmTimer, "stop", Qt::QueuedConnection);
    addMessage(
        "Remote Unlock", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::carDoor(bool door) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = door;
    carDoorPublisher->publish(message);
    if (door == 1) {
        QMetaObject::invokeMethod(heatingTimer, "stop", Qt::QueuedConnection);
        addMessage(
            "Car Door Open", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    } else {
        addMessage(
            "Car Door Closed", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
Q_INVOKABLE void ROS2Bridge::carDriving(bool driving) {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = driving;
    carDrivingPublisher->publish(message);
    if (driving == 1) {
        addMessage(
            "Car in Driving state", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    } else {
        addMessage(
            "Car stopped", 
            "outgoing", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
Q_INVOKABLE void ROS2Bridge::publishCentralLock() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    centralLockPublisher->publish(message);
    addMessage(
        "Central Lock", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}
Q_INVOKABLE void ROS2Bridge::publishCentralUnlock() {
    if (m_testMode) return;
    auto message = std_msgs::msg::Bool();
    message.data = true;
    centralUnlockPublisher->publish(message);
    addMessage(
        "Central Unlock", 
        "outgoing", 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
    );
}

// Method to subscribe to the messages
void ROS2Bridge::windowUpLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit windowUpLedOnSignal();
        addMessage(
            QString("Window up ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        ); 
    } 
}
void ROS2Bridge::windowDownLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit windowDownLedOnSignal();
        addMessage(
            QString("Window down ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }   
}
void ROS2Bridge::windowUpLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit windowUpLedOffSignal();
        addMessage(
            QString("Window up OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        ); 
    } 
}
void ROS2Bridge::windowDownLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit windowDownLedOffSignal();
        addMessage(
            QString("Window down OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }   
}
void ROS2Bridge::fingerProtectionLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit fingerProtectionLedOnSignal();
        addMessage(
            QString("Finger protection interruption received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::fingerProtectionLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit fingerProtectionLedOffSignal();
        addMessage(
            QString("Finger protection interruption released"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }
}
void ROS2Bridge::centralWindowUpLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit centralWindowUpLedOnSignal();
        addMessage(
            QString("cls window up ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::centralWindowUpLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit centralWindowUpLedOffSignal();
        addMessage(
            QString("cls window up OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }
}
void ROS2Bridge::mirrorHeatingOn(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorHeatingOnSignal();
        QMetaObject::invokeMethod(heatingTimer, "start", Qt::QueuedConnection);
        addMessage(
            QString("mirror heating ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }
}
void ROS2Bridge::mirrorHeatingOff(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorHeatingOffSignal();
        addMessage(
            QString("mirror heating OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }
}
void ROS2Bridge::mirrorUpLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorUpLedOnSignal();
        addMessage(
            QString("Mirror at Top position LED received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::mirrorUpLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorUpLedOffSignal(); 
    } 
}
void ROS2Bridge::mirrorDownLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorDownLedOnSignal();
        addMessage(
            QString("Mirror at Bottom position LED received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::mirrorDownLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorDownLedOffSignal(); 
    } 
}
void ROS2Bridge::mirrorLeftLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorLeftLedOnSignal();
        addMessage(
            QString("Mirror at Left position LED received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::mirrorLeftLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorLeftLedOffSignal(); 
    } 
}
void ROS2Bridge::mirrorRightLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorRightLedOnSignal();
        addMessage(
            QString("Mirror at Right position LED received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::mirrorRightLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorRightLedOffSignal(); 
    } 
}
void ROS2Bridge::mirrorHeaterLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorHeaterLedOnSignal();
        addMessage(
            QString("Mirror Heater ON LED received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::mirrorHeaterLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit mirrorHeaterLedOffSignal(); 
    } 
}
void ROS2Bridge::alarmDetectedLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmDetectedLedOnSignal();
        addMessage(
            QString("LED Alarm detected ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::alarmDetectedLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmDetectedLedOffSignal();
        addMessage(
            QString("LED Alarm detected OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::alarmSignalOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmSignalOnSignal();
        addMessage(
            QString("Alarm signal ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::alarmSignalOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmSignalOffSignal();
        addMessage(
            QString("Alarm signal OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::alarmInteriorOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmInteriorOnSignal();
        addMessage(
            QString("Alarm interior signal ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::alarmInteriorOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmInteriorOffSignal();
        addMessage(
            QString("Alarm interior signal OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::alarmActivationOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmActivationOnSignal();
        addMessage(
            QString("Alarm activation ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::alarmActivationOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmActivationOffSignal();
        addMessage(
            QString("Alarm activation OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::alarmSignalLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmSignalLedOnSignal();
        addMessage(
            QString("LED Alarm signal ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::alarmSignalLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmSignalLedOffSignal();
        addMessage(
            QString("LED Alarm signal OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::alarmInteriorLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmInteriorLedOnSignal();
        addMessage(
            QString("LED Alarm interior signal ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::alarmInteriorLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmInteriorLedOffSignal();
        addMessage(
            QString("LED Alarm interior signal OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::alarmActivationLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmActivationLedOnSignal();
        addMessage(
            QString("LED Alarm activation ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    } 
}
void ROS2Bridge::alarmActivationLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit alarmActivationLedOffSignal();
        addMessage(
            QString("LED Alarm activation OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::carLockedMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit carLockedSignal();
        addMessage(
            QString("Car locked command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }
}
void ROS2Bridge::carUnlockedMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit carUnlockedSignal();
        addMessage(
            QString("Car unlocked command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }
}
void ROS2Bridge::clsLockMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit clsLockSignal();
        addMessage(
            QString("Central lock command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }
}
void ROS2Bridge::clsUnlockMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit clsUnlockSignal();
        addMessage(
            QString("Central unlock command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );   
    }
}
void ROS2Bridge::centralLockLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit centralLockLedOnSignal();
        addMessage(
            QString("Central lock  ON command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        ); 
    }
}  
void ROS2Bridge::centralLockLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        emit centralLockLedOffSignal();
        addMessage(
            QString("Central lock OFF command received"), 
            "incoming", 
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::handleWindowMoveUp(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        int newPosition = std::min(windowPosition + 10, 100);
        setSliderPosition(newPosition);
        emit windowMoveUpSignal();
    }
}
void ROS2Bridge::handleWindowMoveDown(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        int newPosition = std::max(windowPosition - 10, 0);
        setSliderPosition(newPosition);
        emit windowMoveDownSignal();
    }
}
void ROS2Bridge::handleWindowAutoMoveUp(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        isMovingUp = true;
        isMovingDown = false;
        emit windowAutoMoveUpSignal();
        emit startAutoMoveRequested();
        addMessage(
            QString("Auto window movement up started"),
            "incoming",
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::handleWindowAutoMoveDown(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        isMovingDown = true;
        isMovingUp = false;
        emit windowAutoMoveDownSignal();
        emit startAutoMoveRequested();
        addMessage(
            QString("Auto window movement down started"),
            "incoming",
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::handleWindowAutoMoveStop(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        isMovingUp = false;
        isMovingDown = false;
        emit windowAutoMoveStopSignal();
        emit stopAutoMoveRequested();
        addMessage(
            QString("Window movement stopped at position: %1").arg(windowPosition),
            "incoming",
            QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")
        );
    }
}
void ROS2Bridge::handleMirrorUp(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        int newMirrorUpPosition = std::min(mirrorYPosition + 10, 100);
        emit mirrorMoveUpSignal();
        setMirrorVerticalPosition(newMirrorUpPosition);
    }
}
void ROS2Bridge::handleMirrorDown(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        int newMirrorDownPosition = std::max(mirrorYPosition - 10, 0);
        emit mirrorMoveDownSignal();
        setMirrorVerticalPosition(newMirrorDownPosition);
    }
}
void ROS2Bridge::handleMirrorLeft(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        int newMirrorLeftPosition = std::min(mirrorXPosition - 10, 100);
        emit mirrorMoveLeftSignal();
        setMirrorHorizontalPosition(newMirrorLeftPosition);
    }
}
void ROS2Bridge::handleMirrorRight(const std_msgs::msg::Bool::SharedPtr msg) {
    if (m_testMode) return;
    if (msg->data) {
        int newMirrorRightPosition = std::max(mirrorXPosition + 10, 0);
        emit mirrorMoveRightSignal();
        setMirrorHorizontalPosition(newMirrorRightPosition);
    }
}


int ROS2Bridge::sliderPosition() const {
    return windowPosition;
}
int ROS2Bridge::mirrorHorizontalPosition() const {
    return mirrorXPosition;
}
int ROS2Bridge::mirrorVerticalPosition() const {
    return mirrorYPosition;
}
void ROS2Bridge::setSliderPosition(int position) {
    if (windowPosition != position) {
        windowPosition = position;
        emit sliderPositionChanged(position);
    }
}
void ROS2Bridge::setMirrorVerticalPosition(int position) {
    if (mirrorYPosition != position) {
        mirrorYPosition = position;
        emit mirrorVerticalPositionChanged(position);
    }
}
void ROS2Bridge::setMirrorHorizontalPosition(int position) {
    if (mirrorXPosition != position) {
        mirrorXPosition = position;
        emit mirrorHorizontalPositionChanged(position);
    }
}
void ROS2Bridge::startAutoMoveTimer() {
    QMetaObject::invokeMethod(autoMoveTimer, [this]() {
        if (!autoMoveTimer->isActive()) {
            autoMoveTimer->start();
        }
    }, Qt::QueuedConnection);
}
void ROS2Bridge::stopAutoMoveTimer() {
    QMetaObject::invokeMethod(autoMoveTimer, [this]() {
        if (autoMoveTimer->isActive()) {
            autoMoveTimer->stop();
        }
    }, Qt::QueuedConnection);
}
void ROS2Bridge::updateAutoMove() {

    if (!isMovingUp && !isMovingDown) {
        emit stopAutoMoveRequested();
        return;
    }

    if (isMovingUp) {
        RCLCPP_DEBUG(node->get_logger(), "Moving up, current position: %d", windowPosition);
        int newPosition = std::min(windowPosition + 5, 100); // Smaller increment for smoother movement
        if (newPosition != windowPosition) {
            setSliderPosition(newPosition);
        } else {
            // Stop at the top
            isMovingUp = false;
            emit stopAutoMoveRequested();
        }
    } else if (isMovingDown) {
        int newPosition = std::max(windowPosition - 5, 0); // Smaller increment for smoother movement
        if (newPosition != windowPosition) {
            setSliderPosition(newPosition);
        } else {
            // Stop at the bottom
            isMovingDown = false;
            emit stopAutoMoveRequested();
        }
    }
}
// Add timer handlers
void ROS2Bridge::handleHeatingTimeout() {
    if (m_testMode) return;
    try {
        auto message = std_msgs::msg::Bool();
        message.data = true;
        heaterTimerPublisher->publish(message);
        addMessage("Heating timer elapsed", "info");
    } catch (const std::exception& e) {
        addMessage("Failed to publish heating timer message: " + QString(e.what()), "error");
    }
}

void ROS2Bridge::handleRemoteTimeout() {
    if (m_testMode) return;
    try {
        auto message = std_msgs::msg::Bool();
        message.data = true;
        remoteTimerPublisher->publish(message);
        addMessage("Remote timer elapsed", "info");
    } catch (const std::exception& e) {
        addMessage("Failed to publish remote timer message: " + QString(e.what()), "error");
    }
}

void ROS2Bridge::handleAlarmTimeout() {
    if (m_testMode) return;
    try {
        auto message = std_msgs::msg::Bool();
        message.data = true;
        alarmTimerPublisher->publish(message);
        addMessage("Alarm timer elapsed", "info");
    } catch (const std::exception& e) {
        addMessage("Failed to handle alarm timer: " + QString(e.what()), "error");
    }
}
void ROS2Bridge::messageCallback(const std_msgs::msg::String::SharedPtr msg) {
    // Add the received message to the model with more details
    addMessage(
        QString::fromStdString(msg->data), 
        "incoming"
    );
}
void ROS2Bridge::addMessage(const QString& message, const QString& type, const QString& timestamp) {
    QVariantMap messageData;
    messageData["message"] = message;
    messageData["type"] = type;
    messageData["number"] = m_messageModel.rowCount() + 1;
    messageData["timestamp"] = timestamp.isEmpty() ? 
        QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss") : timestamp;
    m_messageModel.appendMessage(messageData);
}
QAbstractListModel* ROS2Bridge::messageModel() {
    return &m_messageModel;
}

ROS2Bridge::~ROS2Bridge() {
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    QMetaObject::invokeMethod(autoMoveTimer, [this]() {
        if (autoMoveTimer->isActive()) {
            autoMoveTimer->stop();
        }
        delete autoMoveTimer;
    }, Qt::BlockingQueuedConnection); 

    QMetaObject::invokeMethod(heatingTimer, [this]() {
        if (heatingTimer->isActive()) {
            heatingTimer->stop();
        }
        delete heatingTimer;
    }, Qt::BlockingQueuedConnection);

    QMetaObject::invokeMethod(remoteTimer, [this]() {
        if (remoteTimer->isActive()) {
            remoteTimer->stop();
        }
        delete remoteTimer;
    }, Qt::BlockingQueuedConnection);

    QMetaObject::invokeMethod(alarmTimer, [this]() {
        if (alarmTimer->isActive()) {
            alarmTimer->stop();
        }
        delete alarmTimer;
    }, Qt::BlockingQueuedConnection);
}