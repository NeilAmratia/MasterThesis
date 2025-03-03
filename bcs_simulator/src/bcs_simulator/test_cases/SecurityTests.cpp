#include "SecurityTests.hpp"
#include <iostream>
#include <chrono>
#include <thread>

SecurityTests::SecurityTests (std::shared_ptr<rclcpp::Node> node_) : node(node_) {
    
    if (!node) {
        throw std::runtime_error("Failed to create test node");
    }
    
    try {
        // RCK
        remoteLockRCKPublisher = node->create_publisher<std_msgs::msg::Bool>("rck_but_lock", 10);
        remoteUnlockRCKPublisher = node->create_publisher<std_msgs::msg::Bool>("rck_but_unlock", 10);
        remoteLockRCKSubscriber = node->create_subscription<std_msgs::msg::Bool>("rck_lock", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {remoteLockRCKReceived = msg->data;}
        );
        remoteUnlockRCKSubscriber = node->create_subscription<std_msgs::msg::Bool>("rck_unlock", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {remoteUnlockRCKReceived = msg->data;}
        );

        // RCK + SF
        doorOpenRCKPublisher = node->create_publisher<std_msgs::msg::Bool>("door_open", 10);
        timerRCKPublisher = node->create_publisher<std_msgs::msg::Bool>("time_rck_sf_elapsed", 10);

        // RCK + Control Automatic PW
        windowUpRCKPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_rm_up", 10);
        windowDownRCKPublisher = node->create_publisher<std_msgs::msg::Bool>("pw_rm_dn", 10);
        windowUpRCKSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_but_up", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowUpRCKReceived = msg->data;}
        );
        windowDownRCKSubscriber = node->create_subscription<std_msgs::msg::Bool>("pw_but_dn", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {windowDownRCKReceived = msg->data;}
        );

        // CLS
        centralLockCLSPublisher = node->create_publisher<std_msgs::msg::Bool>("key_pos_lock", 10);
        centralUnlockCLSPublisher = node->create_publisher<std_msgs::msg::Bool>("key_pos_unlock", 10);
        centralLockCLSSubscriber = node->create_subscription<std_msgs::msg::Bool>("cls_lock", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {centralLockCLSReceived = msg->data;}
        );
        centralUnlockCLSSubscriber = node->create_subscription<std_msgs::msg::Bool>("cls_unlock", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {centralUnlockCLSReceived = msg->data;}
        );

        // CLS + RCK
        remoteLockCLSPublisher = node->create_publisher<std_msgs::msg::Bool>("rck_lock", 10);
        remoteUnlockCLSPublisher = node->create_publisher<std_msgs::msg::Bool>("rck_unlock", 10);

        // CLS + Auto Locking
        carDrivesCLSPublisher = node->create_publisher<std_msgs::msg::Bool>("car_drives", 10);
        doorOpenCLSPublisher = node->create_publisher<std_msgs::msg::Bool>("door_open", 10);
        carLockCLSSubscriber = node->create_subscription<std_msgs::msg::Bool>("car_lock", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {carLockCLSReceived = msg->data;}
        );
        carUnlockCLSSubscriber = node->create_subscription<std_msgs::msg::Bool>("car_unlock", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {carUnlockCLSReceived = msg->data;}
        );

        // AS
        alarmSystemLockASPublisher = node->create_publisher<std_msgs::msg::Bool>("key_pos_lock", 10);
        alarmSystemUnlockASPublisher = node->create_publisher<std_msgs::msg::Bool>("key_pos_unlock", 10);
        alarmActivatedASPublisher = node->create_publisher<std_msgs::msg::Bool>("as_activated", 10);
        alarmDeactivatedASPublisher = node->create_publisher<std_msgs::msg::Bool>("as_deactivated", 10);
        alarmTimerASPublisher = node->create_publisher<std_msgs::msg::Bool>("time_alarm_elapsed", 10);
        alarmDetectedASPublisher = node->create_publisher<std_msgs::msg::Bool>("as_alarm_detected", 10);
        alarmActiveOnASSubscriber = node->create_subscription<std_msgs::msg::Bool>("as_active_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmActiveOnASReceived = msg->data;}
        );
        alarmActiveOffASSubscriber = node->create_subscription<std_msgs::msg::Bool>("as_active_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmActiveOffASReceived = msg->data;}
        );
        alarmAlarmOnASSubscriber = node->create_subscription<std_msgs::msg::Bool>("as_alarm_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmAlarmOnASReceived = msg->data;}
        );
        alarmAlarmOffASSubscriber = node->create_subscription<std_msgs::msg::Bool>("as_alarm_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmAlarmOffASReceived = msg->data;}
        );
        alarmAlarmDetectedASSubscriber = node->create_subscription<std_msgs::msg::Bool>("as_alarm_was_detected", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmAlarmDetectedASReceived = msg->data;}
        );
        
        // AS + Control AS
        remoteLockASPublisher = node->create_publisher<std_msgs::msg::Bool>("rck_lock", 10);
        remoteUnlockASPublisher = node->create_publisher<std_msgs::msg::Bool>("rck_unlock", 10);

        // AS + IM
        alarmIMASPublisher = node->create_publisher<std_msgs::msg::Bool>("im_alarm_detected", 10);
        alarmIMOnASSubscriber = node->create_subscription<std_msgs::msg::Bool>("as_im_alarm_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmIMOnASReceived = msg->data;}
        );
        alarmIMOffASSubscriber = node->create_subscription<std_msgs::msg::Bool>("as_im_alarm_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {alarmIMOffASReceived = msg->data;}
        );
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to initialize publishers/subscribers: " + std::string(e.what()));
    }
}

void SecurityTests::wait(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void SecurityTests::logTestResult(const std::string& testId, bool passed) {
    std::cout << "Test " << testId << ": " << (passed ? "PASS" : "FAIL") << std::endl;
}

void SecurityTests::resetSignals() {
    remoteLockRCKReceived = false;
    remoteUnlockRCKReceived = false;

    windowUpRCKReceived = false;
    windowDownRCKReceived = false;

    centralLockCLSReceived = false;
    centralUnlockCLSReceived = false;

    carLockCLSReceived = false;
    carUnlockCLSReceived = false;

    alarmActiveOnASReceived = false;
    alarmActiveOffASReceived = false;
    alarmAlarmOnASReceived = false;
    alarmAlarmOffASReceived = false;
    alarmAlarmDetectedASReceived = false;

    alarmIMOnASReceived = false;
    alarmIMOffASReceived = false;
}

SecurityTests::~SecurityTests() {
    try {
        // First reset all publishers
        if (remoteLockRCKPublisher) remoteLockRCKPublisher.reset();
        if (remoteUnlockRCKPublisher) remoteUnlockRCKPublisher.reset();
        if (doorOpenRCKPublisher) doorOpenRCKPublisher.reset();
        if (timerRCKPublisher) timerRCKPublisher.reset();
        if (windowUpRCKPublisher) windowUpRCKPublisher.reset();
        if (windowDownRCKPublisher) windowDownRCKPublisher.reset();
        if (centralLockCLSPublisher) centralLockCLSPublisher.reset();
        if (centralUnlockCLSPublisher) centralUnlockCLSPublisher.reset();
        if (remoteLockCLSPublisher) remoteLockCLSPublisher.reset();
        if (remoteUnlockCLSPublisher) remoteUnlockCLSPublisher.reset();
        if (carDrivesCLSPublisher) carDrivesCLSPublisher.reset();
        if (doorOpenCLSPublisher) doorOpenCLSPublisher.reset();
        if (alarmSystemLockASPublisher) alarmSystemLockASPublisher.reset();
        if (alarmSystemUnlockASPublisher) alarmSystemUnlockASPublisher.reset();
        if (alarmActivatedASPublisher) alarmActivatedASPublisher.reset();
        if (alarmDeactivatedASPublisher) alarmDeactivatedASPublisher.reset();
        if (alarmTimerASPublisher) alarmTimerASPublisher.reset();
        if (alarmDetectedASPublisher) alarmDetectedASPublisher.reset();
        if (remoteLockASPublisher) remoteLockASPublisher.reset();
        if (remoteUnlockASPublisher) remoteUnlockASPublisher.reset();
        if (alarmIMASPublisher) alarmIMASPublisher.reset();

        // Then reset all subscribers
        if (remoteLockRCKSubscriber) remoteLockRCKSubscriber.reset();
        if (remoteUnlockRCKSubscriber) remoteUnlockRCKSubscriber.reset();
        if (windowUpRCKSubscriber) windowUpRCKSubscriber.reset();
        if (windowDownRCKSubscriber) windowDownRCKSubscriber.reset();
        if (centralLockCLSSubscriber) centralLockCLSSubscriber.reset();
        if (centralUnlockCLSSubscriber) centralUnlockCLSSubscriber.reset();
        if (carLockCLSSubscriber) carLockCLSSubscriber.reset();
        if (carUnlockCLSSubscriber) carUnlockCLSSubscriber.reset();
        if (alarmActiveOnASSubscriber) alarmActiveOnASSubscriber.reset();
        if (alarmActiveOffASSubscriber) alarmActiveOffASSubscriber.reset();
        if (alarmAlarmOnASSubscriber) alarmAlarmOnASSubscriber.reset();
        if (alarmAlarmOffASSubscriber) alarmAlarmOffASSubscriber.reset();
        if (alarmAlarmDetectedASSubscriber) alarmAlarmDetectedASSubscriber.reset();
        if (alarmIMOnASSubscriber) alarmIMOnASSubscriber.reset();
        if (alarmIMOffASSubscriber) alarmIMOffASSubscriber.reset();

        // Finally reset the node
        if (node) {
            node.reset();
        }


    } catch (const std::exception& e) {
        std::cerr << "Error in SecurityTests destructor: " << e.what() << std::endl;
    } catch (...) {
        std::cout << "Unknown error in SecurityTests destructor" << std::endl;
    }
}

bool SecurityTests::waitForSignal(bool& signal) {
    int elapsed = 0;
    while (!signal && elapsed < TIMEOUT_MS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // rclcpp::spin_some(node); 
        wait(100);
        elapsed += 100;
    }
    return signal;
}

void SecurityTests::runRCKTest() {

    std::cout << "\n=== RCK Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        remoteUnlockRCKPublisher->publish(msg);
        remoteUnlockRCKPublisher->publish(msg);
        remoteUnlockRCKPublisher->publish(msg);
        wait(500);
        remoteLockRCKPublisher->publish(msg);
        logTestResult("t1", waitForSignal(remoteLockRCKReceived));
        wait(1000);
        

        // t3
        resetSignals();
        remoteUnlockRCKPublisher->publish(msg);
        logTestResult("t3", waitForSignal(remoteUnlockRCKReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== RCK Test Completed ===" << std::endl;
}

void SecurityTests::runRCKWithSFTest() {

    std::cout << "\n=== RCK with SF Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        remoteLockRCKPublisher->publish(msg);
        logTestResult("t1", waitForSignal(remoteLockRCKReceived));
        wait(1000);
        

        // t11
        resetSignals();
        remoteUnlockRCKPublisher->publish(msg);
        logTestResult("t11", waitForSignal(remoteUnlockRCKReceived));
        wait(1000);
        

        // t13
        resetSignals();
        doorOpenRCKPublisher->publish(msg);
        remoteLockRCKPublisher->publish(msg);
        logTestResult("t13", waitForSignal(remoteLockRCKReceived));
        wait(1000);
        

        // t10
        resetSignals();
        remoteUnlockRCKPublisher->publish(msg);
        timerRCKPublisher->publish(msg);
        logTestResult("t10", waitForSignal(remoteLockRCKReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== RCK with SF Test Completed ===" << std::endl;
}

void SecurityTests::runRCKWithControlAutoPWTest() {

    std::cout << "\n=== RCK with Control Auto PW Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t5
        resetSignals();
        windowUpRCKPublisher->publish(msg);
        logTestResult("t5", waitForSignal(windowUpRCKReceived));
        wait(1000);
        

        // t8
        resetSignals();
        windowDownRCKPublisher->publish(msg);
        logTestResult("t8", waitForSignal(windowDownRCKReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== RCK with Control Auto PW Test Completed ===" << std::endl;
}

void SecurityTests::runRCKWithSFandControlAutoPWTest() {

    std::cout << "\n=== RCK with SF and Control Auto PW Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t16
        resetSignals();
        remoteLockRCKPublisher->publish(msg);
        remoteUnlockRCKPublisher->publish(msg);
        windowUpRCKPublisher->publish(msg);
        logTestResult("t16", waitForSignal(windowUpRCKReceived));
        wait(1000);
        

        // t18
        resetSignals();
        windowDownRCKPublisher->publish(msg);
        logTestResult("t8", waitForSignal(windowDownRCKReceived));
        wait(1000);
        

        // t20
        resetSignals();
        doorOpenRCKPublisher->publish(msg);
        windowUpRCKPublisher->publish(msg);
        logTestResult("t20", waitForSignal(windowUpRCKReceived));
        wait(1000);
        

        // t22
        resetSignals();
        windowDownRCKPublisher->publish(msg);
        logTestResult("t22", waitForSignal(windowDownRCKReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== RCK with  SF and Control Auto PW Test Completed ===" << std::endl;
}

void SecurityTests::runCLSTest() {

    std::cout << "\n=== CLS Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t1
        resetSignals();
        centralUnlockCLSPublisher->publish(msg);
        centralUnlockCLSPublisher->publish(msg);
        wait(500);
        centralLockCLSPublisher->publish(msg);
        logTestResult("t1", waitForSignal(centralLockCLSReceived));
        wait(1000);
        

        // t3
        resetSignals();
        wait(500);
        centralUnlockCLSPublisher->publish(msg);
        logTestResult("t3", waitForSignal(centralUnlockCLSReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== CLS Test Completed ===" << std::endl;
}

void SecurityTests::runCLSWithRCKTest() {

    std::cout << "\n=== CLS with RCK Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {

        // t5
        resetSignals();
        remoteLockASPublisher->publish(msg);
        logTestResult("t5", waitForSignal(centralLockCLSReceived));
        wait(1000);
        

        // t6
        resetSignals();
        remoteUnlockCLSPublisher->publish(msg);
        logTestResult("t6", waitForSignal(centralUnlockCLSReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== CLS with RCK Test Completed ===" << std::endl;
}

void SecurityTests::runCLSWithAutoLockingTest() {

    std::cout << "\n=== CLS with Auto Locking Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {

        // t7
        resetSignals();
        centralUnlockCLSPublisher->publish(msg);
        carDrivesCLSPublisher->publish(msg);
        logTestResult("t7", waitForSignal(carLockCLSReceived));
        wait(1000);
        

        // t9
        resetSignals();
        doorOpenCLSPublisher->publish(msg);
        logTestResult("t9", waitForSignal(carUnlockCLSReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== CLS with Auto Locking Test Completed ===" << std::endl;
}

void SecurityTests::runASTest() {

    std::cout << "\n=== AS Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t2
        resetSignals();
        alarmDeactivatedASPublisher->publish(msg);
        wait(500);
        alarmSystemLockASPublisher->publish(msg);
        logTestResult("t2", !waitForSignal(alarmActiveOnASReceived));
        wait(1000);
        

        // t3
        resetSignals();
        alarmActivatedASPublisher->publish(msg);
        wait(500);
        alarmSystemLockASPublisher->publish(msg);
        logTestResult("t3", waitForSignal(alarmActiveOnASReceived));
        wait(1000);
        

        // t5
        resetSignals();
        alarmSystemUnlockASPublisher->publish(msg);
        logTestResult("t5", waitForSignal(alarmActiveOffASReceived));
        wait(1000);
        

        // t7
        resetSignals();
        alarmSystemLockASPublisher->publish(msg);
        wait(500);
        alarmDetectedASPublisher->publish(msg);
        logTestResult("t7", waitForSignal(alarmAlarmOnASReceived));
        wait(1000);
        

        // t11
        resetSignals();
        alarmTimerASPublisher->publish(msg);
        logTestResult("t11", waitForSignal(alarmAlarmOffASReceived));
        wait(1000);
        

        // t9
        resetSignals();
        alarmDetectedASPublisher->publish(msg);
        wait(500);
        alarmSystemUnlockASPublisher->publish(msg);
        logTestResult("t9-1", waitForSignal(alarmAlarmOffASReceived));
        logTestResult("t9-2", waitForSignal(alarmActiveOffASReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== AS Test completed ===" << std::endl;
}

void SecurityTests::runASWithControlASTest() {

    std::cout << "\n=== AS with Control AS Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t14
        resetSignals();
        alarmSystemUnlockASPublisher->publish(msg);
        wait(500);
        remoteLockASPublisher->publish(msg);
        logTestResult("t14", waitForSignal(alarmActiveOnASReceived));
        wait(1000);
        

        // t15
        resetSignals();
        remoteUnlockASPublisher->publish(msg);
        logTestResult("t15", waitForSignal(alarmActiveOffASReceived));
        wait(1000);
        

        // t16
        resetSignals();
        remoteLockASPublisher->publish(msg);
        wait(500);
        alarmDetectedASPublisher->publish(msg);
        wait(500);
        remoteUnlockASPublisher->publish(msg);
        logTestResult("t16", waitForSignal(alarmAlarmOffASReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== AS with Control AS Test completed ===" << std::endl;
}

void SecurityTests::runASWithIMTest() {

    std::cout << "\n=== AS with IM Test ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t3
        resetSignals();
        alarmActivatedASPublisher->publish(msg);
        wait(500);
        alarmSystemLockASPublisher->publish(msg);
        logTestResult("t3", waitForSignal(alarmActiveOnASReceived));
        wait(1000);
        

        // t17
        resetSignals();
        wait(500);
        alarmIMASPublisher->publish(msg);
        logTestResult("t17", waitForSignal(alarmIMOnASReceived));
        wait(1000);
        

        // t19
        resetSignals();
        alarmTimerASPublisher->publish(msg);
        logTestResult("t19", waitForSignal(alarmIMOffASReceived));
        wait(1000);
        

        // t21
        resetSignals();
        alarmIMASPublisher->publish(msg);
        wait(500);
        alarmSystemUnlockASPublisher->publish(msg);
        logTestResult("t21", waitForSignal(alarmIMOffASReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== AS with IM Test complete ===" << std::endl;
}
