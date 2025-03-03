#include "PowerWindowTests.hpp"
#include <iostream>
#include <chrono>
#include <thread>

PowerWindowTests::PowerWindowTests(std::shared_ptr<rclcpp::Node> node_) : node(node_) {

    if (!node) {
        throw std::runtime_error("Failed to create test node");
    }
    
    try {
        windowPositionUpPWPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_pos_up", 10);
        windowPositionDownPWPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_pos_dn", 10);
        windowUpPWPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_but_up", 10);
        windowDownPWPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_but_dn", 10);
        
        // CLS
        clsLockPWPublisher = node->create_publisher<std_msgs::msg::Bool>("cls_lock", 10);
        clsUnlockPWPublisher = node->create_publisher<std_msgs::msg::Bool>("cls_unlock", 10);

        // Manual PW
        windowUpPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_mv_up", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowUpPWReceived = msg->data;}
        );
        windowDownPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_mv_dn", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowDownPWReceived = msg->data;}
        );
        
        // Auto PW
        windowUpAutoPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_auto_mv_up", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowUpAutoPWReceived = msg->data;}
        );
        windowDownAutoPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_auto_mv_dn", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowDownAutoPWReceived = msg->data;}
        );
        windowStopAutoPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_auto_mv_stop", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowStopAutoPWReceived = msg->data;}
        );

        // FP
        fpOnPWPublisher = node->create_publisher<std_msgs::msg::Bool>("fp_on", 10);
        fpOffPWPublisher = node->create_publisher<std_msgs::msg::Bool>("fp_off", 10);
        fingerDetectedPWPublisher = node->create_publisher<std_msgs::msg::Bool>("finger_detected", 10);
        fpOnPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("fp_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {fpOnValue = msg->data;}
        );
        fpOffPWSubscriber = node->create_subscription<std_msgs::msg::Bool>("fp_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {fpOffValue = msg->data;}
        );
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to initialize publishers/subscribers: " + std::string(e.what()));
    }
}

void PowerWindowTests::wait(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void PowerWindowTests::logTestResult(const std::string& testId, bool passed) {
    std::cout << "Test " << testId << ": " << (passed ? "PASS" : "FAIL") << std::endl;
}

void PowerWindowTests::resetSignals() {
    windowUpPWReceived = false;
    windowDownPWReceived = false;
    windowUpAutoPWReceived = false;
    windowDownAutoPWReceived = false;
    windowStopAutoPWReceived = false;
    fpOnValue = false;
    fpOffValue = false;
}

bool PowerWindowTests::waitForSignal(bool& signal) {
    int elapsed = 0;
    while (!signal && elapsed < TIMEOUT_MS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // rclcpp::spin_some(node);  
        wait(100);
        elapsed += 100;
    }
    return signal;
}

PowerWindowTests::~PowerWindowTests() {
    try {
        // First reset all publishers
        if (windowPositionUpPWPublisher) windowPositionUpPWPublisher.reset();
        if (windowPositionDownPWPublisher) windowPositionDownPWPublisher.reset();
        if (windowUpPWPublisher) windowUpPWPublisher.reset();
        if (windowDownPWPublisher) windowDownPWPublisher.reset();
        if (clsLockPWPublisher) clsLockPWPublisher.reset();
        if (clsUnlockPWPublisher) clsUnlockPWPublisher.reset();
        if (fpOnPWPublisher) fpOnPWPublisher.reset();
        if (fpOffPWPublisher) fpOffPWPublisher.reset();
        if (fingerDetectedPWPublisher) fingerDetectedPWPublisher.reset();

        // Then reset all subscribers
        if (windowUpPWSubscriber) windowUpPWSubscriber.reset();
        if (windowDownPWSubscriber) windowDownPWSubscriber.reset();
        if (windowUpAutoPWSubscriber) windowUpAutoPWSubscriber.reset();
        if (windowDownAutoPWSubscriber) windowDownAutoPWSubscriber.reset();
        if (windowStopAutoPWSubscriber) windowStopAutoPWSubscriber.reset();
        if (fpOnPWSubscriber) fpOnPWSubscriber.reset();
        if (fpOffPWSubscriber) fpOffPWSubscriber.reset();

        // Finally reset the node
        if (node) {
            node.reset();
        }


    } catch (const std::exception& e) {
        std::cout << "Error in PowerWindowTests destructor:" << e.what() << std::endl;
    } catch (...) {
        std::cout << "Unknown error in PowerWindowTests destructor" << std::endl;
    }
}

void PowerWindowTests::runManualPWTest() {

    std::cout << "\n=== Manual Power Window Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        msg.data = true;
        windowPositionUpPWPublisher->publish(msg);
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t1", waitForSignal(windowDownPWReceived));
        wait(1000);

        // t3
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t3", waitForSignal(windowDownPWReceived));
        wait(1000);
        
        // t10
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        wait(500);
        windowUpPWPublisher->publish(msg);
        logTestResult("t10", waitForSignal(windowUpPWReceived));
        wait(1000);
        
        // t7
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        wait(500);
        windowUpPWPublisher->publish(msg);
        logTestResult("t7", waitForSignal(windowUpPWReceived));
        wait(1000);

        // t8
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t8", waitForSignal(windowDownPWReceived));
        wait(1000);
        
        // t4
        resetSignals();
        msg.data = true;
        windowPositionDownPWPublisher->publish(msg);
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t4", !waitForSignal(windowDownPWReceived));
        wait(1000);
        
        // t5
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        wait(500);
        windowUpPWPublisher->publish(msg);
        logTestResult("t5", waitForSignal(windowUpPWReceived));
        wait(1000);
        
        // t11
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        fpOnPWPublisher->publish(msg);
        wait(500);
        windowUpPWPublisher->publish(msg);
        logTestResult("t11", !waitForSignal(windowUpPWReceived));
        wait(1000);
        
        // t12
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        wait(500);
        fpOffPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        logTestResult("t12", waitForSignal(windowDownPWReceived));
        wait(1000);

        // t10
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        wait(500);
        windowUpPWPublisher->publish(msg);
        logTestResult("t10", waitForSignal(windowUpPWReceived));
        wait(1000);

        // t9
        resetSignals();
        msg.data = false;
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        windowPositionUpPWPublisher->publish(msg);
        wait(500);
        windowUpPWPublisher->publish(msg);
        logTestResult("t9", !waitForSignal(windowUpPWReceived));
        wait(1000);
        
    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== Manual Power Window Test Completed ===" << std::endl;
}


void PowerWindowTests::runManualPWWithCLSTest() {
    
    std::cout << "\n=== Manual Power Window with CLS Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;

     try {
        // t15
        resetSignals();
        msg.data = false;
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        windowPositionUpPWPublisher->publish(msg);
        clsLockPWPublisher->publish(msg);
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t15", !waitForSignal(windowDownPWReceived));
        wait(1000);
        

        // t14
        resetSignals();
        windowPositionUpPWPublisher->publish(msg);
        clsUnlockPWPublisher->publish(msg);
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t14", waitForSignal(windowDownPWReceived));
        wait(1000);
        

        // t24
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        msg.data = true;
        windowPositionDownPWPublisher->publish(msg);
        clsLockPWPublisher->publish(msg);
        wait(500);
        windowUpPWPublisher->publish(msg);
        logTestResult("t24", waitForSignal(windowUpPWReceived));
        wait(1000);
        

        // t23
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        windowPositionDownPWPublisher->publish(msg);
        msg.data = true;
        clsLockPWPublisher->publish(msg);
        wait(500);
        windowUpPWPublisher->publish(msg);
        logTestResult("t23", waitForSignal(windowUpPWReceived));
        wait(1000);
        


        // t22
        resetSignals();
        windowPositionUpPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        clsLockPWPublisher->publish(msg);
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t22", !waitForSignal(windowDownPWReceived));
        wait(1000);
        

        // t18
        resetSignals();
        clsLockPWPublisher->publish(msg);
        fpOnPWPublisher->publish(msg);
        wait(500);
        fpOffPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        logTestResult("t18", !waitForSignal(windowDownPWReceived));
        wait(1000);
        

        // t19
        resetSignals();
        clsUnlockPWPublisher->publish(msg);
        fpOnPWPublisher->publish(msg);
        wait(500);
        fpOffPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        logTestResult("t19", waitForSignal(windowDownPWReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== Manual Power Window with CLS Test Completed ===" << std::endl;
}

void PowerWindowTests::runAutomaticPWTest() {

    std::cout << "\n=== Automatic Power Window Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        windowDownPWPublisher->publish(msg);
        logTestResult("t1", waitForSignal(windowDownAutoPWReceived));
        wait(1000);
        

        // t9
        resetSignals();
        windowUpPWPublisher->publish(msg);
        logTestResult("t9", waitForSignal(windowStopAutoPWReceived));
        wait(1000);
        

        // t11
        resetSignals();
        windowDownPWPublisher->publish(msg);
        logTestResult("t11", waitForSignal(windowDownAutoPWReceived));
        wait(1000);
        

        // t3
        resetSignals();
        windowPositionDownPWPublisher->publish(msg);
        logTestResult("t3", waitForSignal(windowStopAutoPWReceived));
        wait(1000);
        

        // t5
        resetSignals();
        windowUpPWPublisher->publish(msg);
        logTestResult("t5", waitForSignal(windowUpAutoPWReceived));
        wait(1000);
        

        // t15
        resetSignals();
        windowDownPWPublisher->publish(msg);
        logTestResult("t15", waitForSignal(windowStopAutoPWReceived));
        wait(1000);
        

        // t13
        resetSignals();
        windowUpPWPublisher->publish(msg);
        logTestResult("t13", waitForSignal(windowUpAutoPWReceived));
        wait(1000);
        

        // t17
        resetSignals();
        fpOnPWPublisher->publish(msg);
        logTestResult("t17", waitForSignal(windowStopAutoPWReceived));
        wait(1000);
        

        // t19
        resetSignals();
        fpOffPWPublisher->publish(msg);
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t19", waitForSignal(windowDownAutoPWReceived));
        wait(1000);
        

        // t9
        resetSignals();
        windowUpPWPublisher->publish(msg);
        logTestResult("t9", waitForSignal(windowStopAutoPWReceived));
        wait(1000);
        

        // t13
        resetSignals();
        windowUpPWPublisher->publish(msg);
        logTestResult("t13", waitForSignal(windowUpAutoPWReceived));
        wait(1000);
        

        // t7
        resetSignals();
        windowPositionUpPWPublisher->publish(msg);
        logTestResult("t7", waitForSignal(windowStopAutoPWReceived));
        wait(1000);
        
        
    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== Automatic Power Window Test Completed ===" << std::endl;
}

void PowerWindowTests::runAutomaticPWWithCLSTest() {

    std::cout << "\n=== Automatic Power Window with CLS Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t20
        resetSignals();
        windowPositionUpPWPublisher->publish(msg);
        clsLockPWPublisher->publish(msg);
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t20", !waitForSignal(windowDownAutoPWReceived));
        wait(1000);
        

        // t21
        resetSignals();
        windowPositionUpPWPublisher->publish(msg);
        clsUnlockPWPublisher->publish(msg);
        wait(500);
        windowDownPWPublisher->publish(msg);
        logTestResult("t21", waitForSignal(windowDownAutoPWReceived));
        wait(1000);
        

        // t30
        resetSignals();
        msg.data = false;
        windowPositionUpPWPublisher->publish(msg);
        msg.data = true;
        windowPositionDownPWPublisher->publish(msg);
        wait(500);
        clsLockPWPublisher->publish(msg);
        logTestResult("t30", waitForSignal(windowUpAutoPWReceived));
        wait(1000);
        

        // t23
        resetSignals();
        msg.data = true;
        windowPositionUpPWPublisher->publish(msg);
        wait(500);
        windowUpPWPublisher->publish(msg);
        logTestResult("t23", waitForSignal(windowStopAutoPWReceived));
        wait(1000);
        

        // t32
        resetSignals();
        clsUnlockPWPublisher->publish(msg);
        wait(500);
        windowDownPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        wait(500);
        msg.data = true;
        clsLockPWPublisher->publish(msg);
        logTestResult("t32-1", waitForSignal(windowStopAutoPWReceived));
        logTestResult("t32-2", waitForSignal(windowUpAutoPWReceived));
        wait(1000);
        

        // t34
        resetSignals();
        clsUnlockPWPublisher->publish(msg);
        wait(500);
        windowPositionUpPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        wait(2000);
        msg.data = true;
        windowUpPWPublisher->publish(msg);
        wait(500);
        clsLockPWPublisher->publish(msg);
        logTestResult("t34", waitForSignal(windowUpAutoPWReceived));
        wait(1000);
        

        // t26
        resetSignals();
        fpOnPWPublisher->publish(msg);
        logTestResult("t26", waitForSignal(windowStopAutoPWReceived));
        wait(1000);

        // t29
        resetSignals();
        clsUnlockPWPublisher->publish(msg);
        wait(500);
        fpOffPWPublisher->publish(msg);
        windowDownPWPublisher->publish(msg);
        logTestResult("t29", waitForSignal(windowDownAutoPWReceived));
        wait(1000);
        

        // t28
        resetSignals();
        clsLockPWPublisher->publish(msg);
        wait(500);
        fpOnPWPublisher->publish(msg);
        logTestResult("t28", !waitForSignal(windowDownAutoPWReceived));
        clsUnlockPWPublisher->publish(msg);
        fpOffPWPublisher->publish(msg);
        windowPositionUpPWPublisher->publish(msg);
        wait(1000);
        
        
    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }
    std::cout << "=== Automatic Power Window with CLS Test Completed ===" << std::endl;
}

void PowerWindowTests::runFPTest() {

    std::cout << "\n=== Finger Protection Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        fingerDetectedPWPublisher->publish(msg);
        logTestResult("t1", waitForSignal(fpOnValue));
        wait(1000);
        
        // t3
        resetSignals();
        windowDownPWPublisher->publish(msg);
        logTestResult("t3", waitForSignal(fpOffValue));
        wait(1000);
        
    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== Finger Protection Test Completed ===" << std::endl;
}
