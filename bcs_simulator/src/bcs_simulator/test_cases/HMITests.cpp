#include "HMITests.hpp"
#include <iostream>
#include <chrono>
#include <thread>

HMITests::HMITests (std::shared_ptr<rclcpp::Node> node_) : node(node_) {

    if (!node) {
        throw std::runtime_error("Failed to create test node");
    }
    
    try {
        // HMI
        windowUpHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_but_mv_up", 10);
        windowDownHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_but_mv_dn", 10);
        mirrorUpHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_mv_up", 10);
        mirrorDownHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_mv_dn", 10);
        mirrorLeftHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_mv_left", 10);
        mirrorRightHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_mv_right", 10);
        windowUpHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_but_up", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowUpHMIReceived = msg->data;}
        );
        windowDownHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_but_dn", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowDownHMIReceived = msg->data;}
        );
        mirrorUpHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("em_but_up", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorUpHMIReceived = msg->data;}
        );
        mirrorDownHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("em_but_down", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorDownHMIReceived = msg->data;}
        );
        mirrorLeftHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("em_but_left", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorLeftHMIReceived = msg->data;}
        );
        mirrorRightHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("em_but_right", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorRightHMIReceived = msg->data;}
        );

        alarmActivatedHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("activate_as", 10);
        alarmDeactivatedHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("deactivate_as", 10);
        alarmActivatedHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("as_activated", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmActivatedHMIReceived = msg->data;}
        );
        alarmDeactivatedHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("as_deactivated", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmDeactivatedHMIReceived = msg->data;}
        );

        confirmAlarmHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("confirm_alarm", 10);
        confirmAlarmHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("as_alarm_was_confirmed", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {confirmAlarmHMIReceived = msg->data;}
        );

        releasePWUpHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("release_pw_but_up", 10);
        releasePWDownHMIPublisher = node->create_publisher<std_msgs::msg::Bool>("release_pw_but_dn", 10);
        releasePWHMISubscriber = node->create_subscription<std_msgs::msg::Bool>("release_pw_but", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {releasePWHMIReceived = msg->data;}
        );


        // LED ManPW
        windowUpLEDManPWPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_mv_up", 10);
        windowDownLEDManPWPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_mv_dn", 10);
        releaseLEDManPWPublisher = node->create_publisher<std_msgs::msg::Bool>("release_pw_but", 10);
        windowUpLEDOnManPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_up_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowUpLEDOnManPWReceived = msg->data;}
        );
        windowUpLEDOffManPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_up_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowUpLEDOffManPWReceived = msg->data;}
        );
        windowDownLEDOnManPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_dn_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowDownLEDOnManPWReceived = msg->data;}
        );
        windowDownLEDOffManPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_dn_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowDownLEDOffManPWReceived = msg->data;}
        );

        // LED AutoPW
        windowUpLEDAutoPWPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_auto_mv_up", 10);
        windowDownLEDAutoPWPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_auto_mv_dn", 10);
        windowStopLEDAutoPWPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_auto_mv_stop", 10);
        windowUpLEDOnAutoPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_up_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowUpLEDOnAutoPWReceived = msg->data;}
        );
        windowUpLEDOffAutoPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_up_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowUpLEDOffAutoPWReceived = msg->data;}
        );
        windowDownLEDOnAutoPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_dn_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowDownLEDOnAutoPWReceived = msg->data;}
        );
        windowDownLEDOffAutoPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_dn_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowDownLEDOffAutoPWReceived = msg->data;}
        );

        // LED AutoPW + CLS
        centralLockLEDAutoPWPublisher = node->create_publisher<std_msgs::msg::Bool>("cls_lock", 10);
        centralUnlockLEDAutoPWPublisher = node->create_publisher<std_msgs::msg::Bool>("cls_unlock", 10);
        centralLockLEDOnAutoPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_cls_up_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {centralLockLEDOnAutoPWReceived = msg->data;}
        );
        centralUnlockLEDOffAutoPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_pw_cls_up_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {centralUnlockLEDOffAutoPWReceived = msg->data;}
        );

        //LED EM Top
        mirrorVertTopPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_vert_top", 10);
        mirrorVertTopPendPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_vert_pend", 10);
        mirrorUpLEDOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_top_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorUpLEDOnReceived = msg->data;}
        );
        mirrorUpLEDOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_top_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorUpLEDOffReceived = msg->data;}
        );

        //LED EM Left
        mirrorHorLeftPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_hor_left", 10);
        mirrorHorLeftPendPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_hor_pend", 10);
        mirrorLeftLEDOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_left_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorLeftLEDOnReceived = msg->data;}
        );
        mirrorLeftLEDOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_left_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorLeftLEDOffReceived = msg->data;}
        );

        //LED EM Bottom
        mirrorVertBottomPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_vert_bottom", 10);
        mirrorVertBottomPendPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_vert_pend", 10);
        mirrorDownLEDOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_bottom_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorDownLEDOnReceived = msg->data;}
        );
        mirrorDownLEDOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_bottom_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorDownLEDOffReceived = msg->data;}
        );

        //LED EM Right
        mirrorHorRightPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_hor_right", 10);
        mirrorHorRightPendPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_hor_pend", 10);
        mirrorRightLEDOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_right_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorRightLEDOnReceived = msg->data;}
        );
        mirrorRightLEDOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_right_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorRightLEDOffReceived = msg->data;}
        );

        //LED EM Heating
        heatingOnLEDPublisher = node->create_publisher<std_msgs::msg::Bool>("heating_on", 10);
        heatingOffLEDPublisher = node->create_publisher<std_msgs::msg::Bool>("heating_off", 10);
        heatingOnLEDSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_heating_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {heatingOnLEDReceived = msg->data;}
        );
        heatingOffLEDSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_em_heating_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {heatingOffLEDReceived = msg->data;}
        );

        //LED FP
        fpOnPublisher = node->create_publisher<std_msgs::msg::Bool>("fp_on", 10);
        fpOffPublisher = node->create_publisher<std_msgs::msg::Bool>("fp_off", 10);
        fpOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_fp_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {fpOnReceived = msg->data;}
        );
        fpOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_fp_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {fpOffReceived = msg->data;}
        );

        // LED AS Active
        alarmActiveOnPublisher = node->create_publisher<std_msgs::msg::Bool>("as_active_on", 10);
        alarmActiveOffPublisher = node->create_publisher<std_msgs::msg::Bool>("as_active_off", 10);
        alarmActiveOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_as_active_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmActiveOnReceived = msg->data;}
        );
        alarmActiveOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_as_active_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmActiveOffReceived = msg->data;}
        );

        // LED AS Alarm
        alarmOnPublisher = node->create_publisher<std_msgs::msg::Bool>("as_alarm_on", 10);
        alarmOffPublisher = node->create_publisher<std_msgs::msg::Bool>("as_alarm_off", 10);
        alarmOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_as_alarm_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmOnReceived = msg->data;}
        );
        alarmOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_as_alarm_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmOffReceived = msg->data;}
        );

        // LED AS Detected
        alarmDetectedOnPublisher = node->create_publisher<std_msgs::msg::Bool>("as_alarm_was_detected", 10);
        alarmDetectedOffPublisher = node->create_publisher<std_msgs::msg::Bool>("as_alarm_was_confirmed", 10);
        alarmDetectedOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_as_alarm_detected_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmDetectedOnReceived = msg->data;}
        );
        alarmDetectedOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_as_alarm_detected_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmDetectedOffReceived = msg->data;}
        );

        // LED AS IM
        alarmIMOnPublisher = node->create_publisher<std_msgs::msg::Bool>("as_im_alarm_on", 10);
        alarmIMOffPublisher = node->create_publisher<std_msgs::msg::Bool>("as_im_alarm_off", 10);
        alarmIMOnSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_as_im_alarm_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmIMOnReceived = msg->data;}
        );
        alarmIMOffSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_as_im_alarm_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmIMOffReceived = msg->data;}
        );

        // LED CLS
        clsLockPublisher = node->create_publisher<std_msgs::msg::Bool>("cls_lock", 10);
        clsUnlockPublisher = node->create_publisher<std_msgs::msg::Bool>("cls_unlock", 10);
        clsLockSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_cls_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {clsLockReceived = msg->data;}
        );
        clsUnlockSubscriber = node->create_subscription<std_msgs::msg::Bool>("led_cls_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {clsUnlockReceived = msg->data;}
        );
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to initialize publishers/subscribers: " + std::string(e.what()));
    }
    
}

void HMITests::wait(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void HMITests::logTestResult(const std::string& testId, bool passed) {
    std::cout << "Test " << testId << ": " << (passed ? "PASS" : "FAIL") << std::endl;
}

void HMITests::resetSignals() {
    windowUpHMIReceived = false;
    windowDownHMIReceived = false;
    mirrorUpHMIReceived = false;
    mirrorDownHMIReceived = false;
    mirrorLeftHMIReceived = false;
    mirrorRightHMIReceived = false;

    alarmActivatedHMIReceived = false;
    alarmDeactivatedHMIReceived = false;
    confirmAlarmHMIReceived = false;
    releasePWHMIReceived = false;

    windowUpLEDOnAutoPWReceived = false;
    windowUpLEDOffAutoPWReceived = false;
    windowDownLEDOnAutoPWReceived = false;
    windowDownLEDOffAutoPWReceived = false;

    mirrorUpLEDOnReceived = false;
    mirrorUpLEDOffReceived = false;

    mirrorLeftLEDOnReceived = false;
    mirrorLeftLEDOffReceived = false;

    mirrorDownLEDOnReceived = false;
    mirrorDownLEDOffReceived = false;

    mirrorRightLEDOnReceived = false;
    mirrorRightLEDOffReceived = false;

    heatingOnLEDReceived = false;
    heatingOffLEDReceived = false;

    fpOnReceived = false;
    fpOffReceived = false;

    alarmActiveOnReceived = false;
    alarmActiveOffReceived = false;

    alarmOnReceived = false;
    alarmOffReceived = false;

    alarmDetectedOnReceived = false;
    alarmDetectedOffReceived = false;

    alarmIMOnReceived = false;
    alarmIMOffReceived = false;

    clsLockReceived = false;
    clsUnlockReceived = false;
}

bool HMITests::waitForSignal(bool& signal) {
    int elapsed = 0;
    while (!signal && elapsed < TIMEOUT_MS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // rclcpp::spin_some(node); 
        wait(100);
        elapsed += 100;
    }
    return signal;
}

HMITests::~HMITests() {
    try {
        // First reset all publishers
        if (windowUpHMIPublisher) windowUpHMIPublisher.reset();
        if (windowDownHMIPublisher) windowDownHMIPublisher.reset();
        if (mirrorUpHMIPublisher) mirrorUpHMIPublisher.reset();
        if (mirrorDownHMIPublisher) mirrorDownHMIPublisher.reset();
        if (mirrorLeftHMIPublisher) mirrorLeftHMIPublisher.reset();
        if (mirrorRightHMIPublisher) mirrorRightHMIPublisher.reset();
        if (alarmActivatedHMIPublisher) alarmActivatedHMIPublisher.reset();
        if (alarmDeactivatedHMIPublisher) alarmDeactivatedHMIPublisher.reset();
        if (confirmAlarmHMIPublisher) confirmAlarmHMIPublisher.reset();
        if (releasePWUpHMIPublisher) releasePWUpHMIPublisher.reset();
        if (releasePWDownHMIPublisher) releasePWDownHMIPublisher.reset();
        if (windowUpLEDManPWPublisher) windowUpLEDManPWPublisher.reset();
        if (windowDownLEDManPWPublisher) windowDownLEDManPWPublisher.reset();
        if (releaseLEDManPWPublisher) releaseLEDManPWPublisher.reset();
        if (windowUpLEDAutoPWPublisher) windowUpLEDAutoPWPublisher.reset();
        if (windowDownLEDAutoPWPublisher) windowDownLEDAutoPWPublisher.reset();
        if (windowStopLEDAutoPWPublisher) windowStopLEDAutoPWPublisher.reset();
        if (centralLockLEDAutoPWPublisher) centralLockLEDAutoPWPublisher.reset();
        if (centralUnlockLEDAutoPWPublisher) centralUnlockLEDAutoPWPublisher.reset();
        if (mirrorVertTopPublisher) mirrorVertTopPublisher.reset();
        if (mirrorVertTopPendPublisher) mirrorVertTopPendPublisher.reset();
        if (mirrorHorLeftPublisher) mirrorHorLeftPublisher.reset();
        if (mirrorHorLeftPendPublisher) mirrorHorLeftPendPublisher.reset();
        if (mirrorVertBottomPublisher) mirrorVertBottomPublisher.reset();
        if (mirrorVertBottomPendPublisher) mirrorVertBottomPendPublisher.reset();
        if (mirrorHorRightPublisher) mirrorHorRightPublisher.reset();
        if (mirrorHorRightPendPublisher) mirrorHorRightPendPublisher.reset();
        if (heatingOnLEDPublisher) heatingOnLEDPublisher.reset();
        if (heatingOffLEDPublisher) heatingOffLEDPublisher.reset();
        if (fpOnPublisher) fpOnPublisher.reset();
        if (fpOffPublisher) fpOffPublisher.reset();
        if (alarmActiveOnPublisher) alarmActiveOnPublisher.reset();
        if (alarmActiveOffPublisher) alarmActiveOffPublisher.reset();
        if (alarmOnPublisher) alarmOnPublisher.reset();
        if (alarmOffPublisher) alarmOffPublisher.reset();
        if (alarmDetectedOnPublisher) alarmDetectedOnPublisher.reset();
        if (alarmDetectedOffPublisher) alarmDetectedOffPublisher.reset();
        if (alarmIMOnPublisher) alarmIMOnPublisher.reset();
        if (alarmIMOffPublisher) alarmIMOffPublisher.reset();
        if (clsLockPublisher) clsLockPublisher.reset();
        if (clsUnlockPublisher) clsUnlockPublisher.reset();

        // Then reset all subscribers
        if (windowUpHMISubscriber) windowUpHMISubscriber.reset();
        if (windowDownHMISubscriber) windowDownHMISubscriber.reset();
        if (mirrorUpHMISubscriber) mirrorUpHMISubscriber.reset();
        if (mirrorDownHMISubscriber) mirrorDownHMISubscriber.reset();
        if (mirrorLeftHMISubscriber) mirrorLeftHMISubscriber.reset();
        if (mirrorRightHMISubscriber) mirrorRightHMISubscriber.reset();
        if (alarmActivatedHMISubscriber) alarmActivatedHMISubscriber.reset();
        if (alarmDeactivatedHMISubscriber) alarmDeactivatedHMISubscriber.reset();
        if (confirmAlarmHMISubscriber) confirmAlarmHMISubscriber.reset();
        if (releasePWHMISubscriber) releasePWHMISubscriber.reset();
        if (windowUpLEDOnManPWSubscriber) windowUpLEDOnManPWSubscriber.reset();
        if (windowUpLEDOffManPWSubscriber) windowUpLEDOffManPWSubscriber.reset();
        if (windowDownLEDOnManPWSubscriber) windowDownLEDOnManPWSubscriber.reset();
        if (windowDownLEDOffManPWSubscriber) windowDownLEDOffManPWSubscriber.reset();
        if (windowUpLEDOnAutoPWSubscriber) windowUpLEDOnAutoPWSubscriber.reset();
        if (centralLockLEDOnAutoPWSubscriber) centralLockLEDOnAutoPWSubscriber.reset();
        if (centralUnlockLEDOffAutoPWSubscriber) centralUnlockLEDOffAutoPWSubscriber.reset();
        if (mirrorUpLEDOnSubscriber) mirrorUpLEDOnSubscriber.reset();
        if (mirrorUpLEDOffSubscriber) mirrorUpLEDOffSubscriber.reset();
        if (mirrorLeftLEDOnSubscriber) mirrorLeftLEDOnSubscriber.reset();
        if (mirrorLeftLEDOffSubscriber) mirrorLeftLEDOffSubscriber.reset();
        if (mirrorDownLEDOnSubscriber) mirrorDownLEDOnSubscriber.reset();
        if (mirrorDownLEDOffSubscriber) mirrorDownLEDOffSubscriber.reset();
        if (mirrorRightLEDOnSubscriber) mirrorRightLEDOnSubscriber.reset();
        if (mirrorRightLEDOffSubscriber) mirrorRightLEDOffSubscriber.reset();
        if (heatingOnLEDSubscriber) heatingOnLEDSubscriber.reset();
        if (heatingOffLEDSubscriber) heatingOffLEDSubscriber.reset();
        if (fpOnSubscriber) fpOnSubscriber.reset();
        if (fpOffSubscriber) fpOffSubscriber.reset();
        if (alarmActiveOnSubscriber) alarmActiveOnSubscriber.reset();
        if (alarmActiveOffSubscriber) alarmActiveOffSubscriber.reset();
        if (alarmOnSubscriber) alarmOnSubscriber.reset();
        if (alarmOffSubscriber) alarmOffSubscriber.reset();
        if (alarmDetectedOnSubscriber) alarmDetectedOnSubscriber.reset();
        if (alarmIMOnSubscriber) alarmIMOnSubscriber.reset();
        if (alarmIMOffSubscriber) alarmIMOffSubscriber.reset();
        if (clsLockSubscriber) clsLockSubscriber.reset();
        if (clsUnlockSubscriber) clsUnlockSubscriber.reset();

        // Finally reset the node
        if (node) {
            node.reset();
        }


    } catch (const std::exception& e) {
        std::cerr << "Error in HMITests destructor: " << e.what() << std::endl;
    } catch (...) {
        std::cout << "Unknown error in HMITests destructor" << std::endl;
    }
}

void HMITests::runHMITest() {

    std::cout << "\n=== HMI Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        windowDownHMIPublisher->publish(msg);
        logTestResult("t1", waitForSignal(windowDownHMIReceived));
        wait(1000);
        

        // t3
        resetSignals();
        windowUpHMIPublisher->publish(msg);
        logTestResult("t3", waitForSignal(windowUpHMIReceived));
        wait(1000);
        

        // t5
        resetSignals();
        mirrorUpHMIPublisher->publish(msg);
        logTestResult("t5", waitForSignal(mirrorUpHMIReceived));
        wait(1000);
        

        // t7
        resetSignals();
        mirrorDownHMIPublisher->publish(msg);
        logTestResult("t7", waitForSignal(mirrorDownHMIReceived));
        wait(1000);
        

        // t9
        resetSignals();
        mirrorRightHMIPublisher->publish(msg);
        logTestResult("t9", waitForSignal(mirrorRightHMIReceived));
        wait(1000);
        

        // t11
        resetSignals();
        mirrorLeftHMIPublisher->publish(msg);
        logTestResult("t11", waitForSignal(mirrorLeftHMIReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== HMI Test Completed ===" << std::endl;
}

void HMITests::runHMIWithASTest() {

    std::cout << "\n=== HMI with AS Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t13
        resetSignals();
        alarmActivatedHMIPublisher->publish(msg);
        logTestResult("t13", waitForSignal(alarmActivatedHMIReceived));
        wait(1000);
        

        // t15
        resetSignals();
        alarmDeactivatedHMIPublisher->publish(msg);
        logTestResult("t15", waitForSignal(alarmDeactivatedHMIReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== HMI with AS Test Completed ===" << std::endl;
}

void HMITests::runHMIWithASLEDTest() {

    std::cout << "\n=== HMI with AS LED Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t17
        resetSignals();
        confirmAlarmHMIPublisher->publish(msg);
        logTestResult("t17", waitForSignal(confirmAlarmHMIReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== HMI with AS LED Test Completed ===" << std::endl;
}

void HMITests::runHMIWithManPWLEDTest() {

    std::cout << "\n=== HMI with ManPW LED Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t19
        resetSignals();
        releasePWDownHMIPublisher->publish(msg);
        logTestResult("t19", waitForSignal(releasePWHMIReceived));
        wait(1000);
        

        // t20
        resetSignals();
        releasePWUpHMIPublisher->publish(msg);
        logTestResult("t20", waitForSignal(releasePWHMIReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== HMI with ManPW LED Test Completed ===" << std::endl;
}

void HMITests::runLEDManPWTest() {

    std::cout << "\n=== LED ManPW Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        windowDownLEDManPWPublisher->publish(msg);
        logTestResult("t1", waitForSignal(windowDownLEDOnManPWReceived));
        wait(1000);
        

        // t3
        resetSignals();
        releaseLEDManPWPublisher->publish(msg);
        logTestResult("t3", waitForSignal(windowDownLEDOffManPWReceived));
        wait(1000);
        

        // t5
        resetSignals();
        windowUpLEDManPWPublisher->publish(msg);
        logTestResult("t5", waitForSignal(windowUpLEDOnManPWReceived));
        wait(1000);
        

        // t7
        resetSignals();
        releaseLEDManPWPublisher->publish(msg);
        logTestResult("t7", waitForSignal(windowUpLEDOffManPWReceived));
        wait(1000);
        
    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED ManPW Test Completed ===" << std::endl;
}

void HMITests::runLEDAutoPWTest() {

    std::cout << "\n=== LED AutoPW Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        windowStopLEDAutoPWPublisher->publish(msg);
        windowStopLEDAutoPWPublisher->publish(msg);
        windowStopLEDAutoPWPublisher->publish(msg);
        wait(500);
        windowDownLEDAutoPWPublisher->publish(msg);
        logTestResult("t1", waitForSignal(windowDownLEDOnAutoPWReceived));
        wait(1000);
        

        // t3
        resetSignals();
        windowStopLEDAutoPWPublisher->publish(msg);
        logTestResult("t3", waitForSignal(windowDownLEDOffAutoPWReceived));
        wait(1000);
        

        // t5
        resetSignals();
        windowUpLEDAutoPWPublisher->publish(msg);
        logTestResult("t5", waitForSignal(windowUpLEDOnAutoPWReceived));
        wait(1000);
        

        // t7
        resetSignals();
        windowStopLEDAutoPWPublisher->publish(msg);
        logTestResult("t7", waitForSignal(windowUpLEDOffAutoPWReceived));
        wait(1000);
        


    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED AutoPW Test Completed ===" << std::endl;
}

void HMITests::runLEDAutoPWWithCLSTest() {

    std::cout << "\n=== LED AutoPW with CLS Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t9
        resetSignals();
        windowStopLEDAutoPWPublisher->publish(msg);
        windowStopLEDAutoPWPublisher->publish(msg);
        windowStopLEDAutoPWPublisher->publish(msg);
        wait(500);
        centralLockLEDAutoPWPublisher->publish(msg);
        windowUpLEDAutoPWPublisher->publish(msg);
        logTestResult("t9", waitForSignal(centralLockLEDOnAutoPWReceived));
        wait(1000);
        

        // t14
        resetSignals();
        centralUnlockLEDAutoPWPublisher->publish(msg);
        windowStopLEDAutoPWPublisher->publish(msg);
        logTestResult("t14", waitForSignal(centralUnlockLEDOffAutoPWReceived));
        wait(1000);
        

        // t16
        resetSignals();
        windowStopLEDAutoPWPublisher->publish(msg);
        centralLockLEDAutoPWPublisher->publish(msg);
        windowUpLEDAutoPWPublisher->publish(msg);
        wait(500);
        windowStopLEDAutoPWPublisher->publish(msg);
        logTestResult("t16", waitForSignal(centralUnlockLEDOffAutoPWReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED AutoPW with CLS Test Completed ===" << std::endl;
}


void HMITests::runLEDEMTTest() {

    std::cout << "\n=== LED EM Top Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        mirrorVertTopPublisher->publish(msg);
        logTestResult("t1", waitForSignal(mirrorUpLEDOnReceived));
        wait(1000);
        

        // t3
        resetSignals();
        mirrorVertTopPendPublisher->publish(msg);
        logTestResult("t3", waitForSignal(mirrorUpLEDOffReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED EM Top Test Completed ===" << std::endl;
}

void HMITests::runLEDEMLTest() {

    std::cout << "\n=== LED EM Left Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        mirrorHorLeftPublisher->publish(msg);
        logTestResult("t1", waitForSignal(mirrorLeftLEDOnReceived));
        wait(1000);
        

        // t3
        resetSignals();
        mirrorHorLeftPendPublisher->publish(msg);
        logTestResult("t3", waitForSignal(mirrorLeftLEDOffReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED EM Left Test Completed ===" << std::endl;
}

void HMITests::runLEDEMBTest() {

    std::cout << "=== LED EM Bottom Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        mirrorVertBottomPublisher->publish(msg);
        logTestResult("t1", waitForSignal(mirrorDownLEDOnReceived));
        wait(1000);
        

        // t3
        resetSignals();
        mirrorVertBottomPendPublisher->publish(msg);
        logTestResult("t3", waitForSignal(mirrorDownLEDOffReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED EM Bottom Test Completed ===" << std::endl;
}

void HMITests::runLEDEMRTest() {

    std::cout << "\n=== LED EM Right Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        mirrorHorRightPublisher->publish(msg);
        logTestResult("t1", waitForSignal(mirrorRightLEDOnReceived));
        wait(1000);
        

        // t3
        resetSignals();
        mirrorHorRightPendPublisher->publish(msg);
        logTestResult("t3", waitForSignal(mirrorRightLEDOffReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED EM Right Test Completed ===" << std::endl;
}

void HMITests::runLEDEMHTest() {

    std::cout << "\n=== LED EM Heating Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        heatingOnLEDPublisher->publish(msg);
        logTestResult("t1", waitForSignal(heatingOnLEDReceived));
        wait(1000);
        

        // t3
        resetSignals();
        heatingOffLEDPublisher->publish(msg);
        logTestResult("t3", waitForSignal(heatingOffLEDReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED EM Heating Test Completed ===" << std::endl;
}

void HMITests::runLEDFPTest() {

    std::cout << "\n=== LED FP Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        fpOffPublisher->publish(msg);
        fpOffPublisher->publish(msg);
        wait(500);
        fpOnPublisher->publish(msg);
        logTestResult("t1", waitForSignal(fpOnReceived));
        wait(1000);
        

        // t3
        resetSignals();
        wait(500);
        fpOffPublisher->publish(msg);
        logTestResult("t3", waitForSignal(fpOffReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED FP Test Completed ===" << std::endl;
}

void HMITests::runLEDASACTest() {

    std::cout << "\n=== LED Alarm Active Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        alarmActiveOnPublisher->publish(msg);
        logTestResult("t1", waitForSignal(alarmActiveOnReceived));
        wait(1000);
        

        // t3
        resetSignals();
        alarmActiveOffPublisher->publish(msg);
        logTestResult("t3", waitForSignal(alarmActiveOffReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED Alarm Active Test Completed ===" << std::endl;
}

void HMITests::runLEDASALTest() {

    std::cout << "\n=== LED Alarm Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        alarmOnPublisher->publish(msg);
        logTestResult("t1", waitForSignal(alarmOnReceived));
        wait(1000);
        

        // t3
        resetSignals();
        alarmOffPublisher->publish(msg);
        logTestResult("t3", waitForSignal(alarmOffReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED Alarm Test Completed ===" << std::endl;
}

void HMITests::runLEDASADTest() {

    std::cout << "\n=== LED Alarm Detected Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        alarmDetectedOnPublisher->publish(msg);
        logTestResult("t1", waitForSignal(alarmDetectedOnReceived));
        wait(1000);
        

        // t3
        resetSignals();
        alarmDetectedOffPublisher->publish(msg);
        logTestResult("t3", waitForSignal(alarmDetectedOffReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED Alarm Detected Test Completed ===" << std::endl;
}

void HMITests::runLEDASIMTest() {

    std::cout << "\n=== LED Alarm IM Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        alarmIMOnPublisher->publish(msg);
        logTestResult("t1", waitForSignal(alarmIMOnReceived));
        wait(1000);
        

        // t3
        resetSignals();
        alarmIMOffPublisher->publish(msg);
        logTestResult("t3", waitForSignal(alarmIMOffReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED Alarm IM Test Completed ===" << std::endl;
}

void HMITests::runLEDCLSTest() {

    std::cout << "\n=== LED CLS Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        clsUnlockPublisher->publish(msg);
        clsUnlockPublisher->publish(msg);
        wait(500);
        clsLockPublisher->publish(msg);
        logTestResult("t1", waitForSignal(clsLockReceived));
        wait(1000);
        

        // t3
        resetSignals();
        wait(500);
        clsUnlockPublisher->publish(msg);
        logTestResult("t3", waitForSignal(clsUnlockReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== LED CLS Test Completed ===" << std::endl;
}
