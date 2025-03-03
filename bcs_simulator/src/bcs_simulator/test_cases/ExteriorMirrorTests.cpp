#include "ExteriorMirrorTests.hpp"
#include <iostream>
#include <chrono>
#include <thread>

ExteriorMirrorTests::ExteriorMirrorTests (std::shared_ptr<rclcpp::Node> node_) : node(node_) {

    if (!node) {
        throw std::runtime_error("Failed to create test node");
    }
    
    try {
        mirrorUpEMPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_up", 10);
        mirrorDownEMPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_down", 10);
        mirrorLeftEMPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_left", 10);
        mirrorRightEMPublisher = node->create_publisher<std_msgs::msg::Bool>("em_but_right", 10);

        mirrorPositionUpEMPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_top", 10);
        mirrorPositionDownEMPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_bottom", 10);
        mirrorPositionLeftEMPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_left", 10);
        mirrorPositionRightEMPublisher = node->create_publisher<std_msgs::msg::Bool>("em_pos_right", 10);

        mirrorUpEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_mv_up", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorMoveUpSignalReceived = msg->data;}
        );
        mirrorDownEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_mv_down", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorMoveDownSignalReceived = msg->data;}
        );
        mirrorLeftEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_mv_left", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorMoveLeftSignalReceived = msg->data;}
        );
        mirrorRightEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_mv_right", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {mirrorMoveRightSignalReceived = msg->data;}
        );

        mirrorColdEMPublisher = node->create_publisher<std_msgs::msg::Bool>("em_too_cold", 10);
        timerEMPublisher = node->create_publisher<std_msgs::msg::Bool>("time_heating_elapsed", 10);
        heatingOnEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("heating_on", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {heatingOnSignalReceived = msg->data;}
        );
        heatingOffEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("heating_off", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {heatingOffSignalReceived = msg->data;}
        );

        vertPendEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_pos_vert_pend", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {vertPendReceived = msg->data;}
        );
        vertTopEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_pos_vert_top", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {vertTopReceived = msg->data;}
        );
        vertBottomEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_pos_vert_bottom", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {vertBottomReceived = msg->data;}
        );
        horPendEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_pos_hor_pend", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {horPendReceived = msg->data;}
        );
        horLeftEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_pos_hor_left", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {horLeftReceived = msg->data;}
        );
        horRightEMSubscriber = node->create_subscription<std_msgs::msg::Bool>("em_pos_hor_right", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {horRightReceived = msg->data;}
        );
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to initialize publishers/subscribers: " + std::string(e.what()));
    }
}

void ExteriorMirrorTests::wait(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void ExteriorMirrorTests::logTestResult(const std::string& testId, bool passed) {
    std::cout << "Test " << testId << ": " << (passed ? "PASS" : "FAIL") << std::endl;
}


void ExteriorMirrorTests::resetSignals() {
    mirrorMoveUpSignalReceived = false;
    mirrorMoveDownSignalReceived = false;
    mirrorMoveLeftSignalReceived = false;
    mirrorMoveRightSignalReceived = false;

    heatingOnSignalReceived = false;
    heatingOffSignalReceived = false;

    vertPendReceived = false;
    vertTopReceived = false;
    vertBottomReceived = false;
    horPendReceived = false;
    horLeftReceived = false;
    horRightReceived = false;
}

bool ExteriorMirrorTests::waitForSignal(bool& signal) {
    int elapsed = 0;
    while (!signal && elapsed < TIMEOUT_MS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // rclcpp::spin_some(node); 
        wait(100);
        elapsed += 100;
    }
    return signal;
}

ExteriorMirrorTests::~ExteriorMirrorTests() {
    try {
        // First reset all publishers
        if (mirrorUpEMPublisher) mirrorUpEMPublisher.reset();
        if (mirrorDownEMPublisher) mirrorDownEMPublisher.reset();
        if (mirrorLeftEMPublisher) mirrorLeftEMPublisher.reset();
        if (mirrorPositionUpEMPublisher) mirrorPositionUpEMPublisher.reset();
        if (mirrorPositionDownEMPublisher) mirrorPositionDownEMPublisher.reset();
        if (mirrorPositionLeftEMPublisher) mirrorPositionLeftEMPublisher.reset();
        if (mirrorPositionRightEMPublisher) mirrorPositionRightEMPublisher.reset();
        if (mirrorColdEMPublisher) mirrorColdEMPublisher.reset();
        if (timerEMPublisher) timerEMPublisher.reset();

        // Then reset all subscribers
        if (mirrorUpEMSubscriber) mirrorUpEMSubscriber.reset();
        if (mirrorDownEMSubscriber) mirrorDownEMSubscriber.reset();
        if (mirrorLeftEMSubscriber) mirrorLeftEMSubscriber.reset();
        if (mirrorRightEMSubscriber) mirrorRightEMSubscriber.reset();
        if (heatingOnEMSubscriber) heatingOnEMSubscriber.reset();
        if (heatingOffEMSubscriber) heatingOffEMSubscriber.reset();
        if (vertPendEMSubscriber) vertPendEMSubscriber.reset();
        if (vertTopEMSubscriber) vertTopEMSubscriber.reset();
        if (horPendEMSubscriber) horPendEMSubscriber.reset();
        if (vertBottomEMSubscriber) vertBottomEMSubscriber.reset();
        if (horLeftEMSubscriber) horLeftEMSubscriber.reset();
        if (horRightEMSubscriber) horRightEMSubscriber.reset();

        // Finally reset the node
        if (node) {
            node.reset();
        }


    } catch (const std::exception& e) {
        std::cout << "Error in xteriorMirrorTests destructor:"  << std::endl;
    } catch (...) {
        std::cout << "Unknown error in ExteriorMirrorTests destructor" << std::endl;
    }
}

void ExteriorMirrorTests::runExteriorMirrorTest() {

    std::cout << "\n=== Exterior Mirror Test Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // t39
        resetSignals();
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t39", waitForSignal(mirrorMoveUpSignalReceived));
        wait(1000);
        

        // t1
        resetSignals();
        mirrorPositionUpEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t31", !waitForSignal(mirrorMoveUpSignalReceived));
        wait(1000);
        

        // t13
        resetSignals();
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t13", waitForSignal(mirrorMoveLeftSignalReceived));
        wait(1000);
        

        // t4
        resetSignals();
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t4", !waitForSignal(mirrorMoveLeftSignalReceived));
        wait(1000);
        

        // t23
        resetSignals();
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t23", waitForSignal(mirrorMoveDownSignalReceived));
        wait(1000);
        

        // t7
        resetSignals();
        mirrorPositionUpEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t7", !waitForSignal(mirrorMoveUpSignalReceived));
        wait(1000);
        

        // t22
        resetSignals();
        mirrorDownEMPublisher->publish(msg);
        mirrorDownEMPublisher->publish(msg);
        mirrorDownEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t22", waitForSignal(mirrorMoveUpSignalReceived));
        wait(1000);
        

        // t45
        resetSignals();
        mirrorRightEMPublisher->publish(msg);
        logTestResult("t45", waitForSignal(mirrorMoveRightSignalReceived));
        wait(1000);
        

        // t6
        resetSignals();
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t6", !waitForSignal(mirrorMoveLeftSignalReceived));
        wait(1000);
        

        // t10
        resetSignals();
        mirrorPositionDownEMPublisher->publish(msg);
        wait(500);
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t18", !waitForSignal(mirrorMoveDownSignalReceived));
        wait(1000);
        

        // t30
        resetSignals();
        mirrorRightEMPublisher->publish(msg);
        logTestResult("t30", waitForSignal(mirrorMoveRightSignalReceived));
        wait(1000);
        

        // t8
        resetSignals();
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t8", !waitForSignal(mirrorMoveLeftSignalReceived));
        wait(1000);
        

        // t46
        resetSignals();
        mirrorRightEMPublisher->publish(msg);
        mirrorRightEMPublisher->publish(msg);
        mirrorRightEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t46", waitForSignal(mirrorMoveUpSignalReceived));
        wait(1000);
        

        // t11
        resetSignals();
        mirrorPositionDownEMPublisher->publish(msg);
        wait(500);
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t11", !waitForSignal(mirrorMoveDownSignalReceived));
        wait(1000);
        

        // t9
        resetSignals();
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorRightEMPublisher->publish(msg);
        logTestResult("t9", !waitForSignal(mirrorMoveRightSignalReceived));
        wait(1000);
        

        // t29
        resetSignals();
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t29", waitForSignal(mirrorMoveLeftSignalReceived));
        wait(1000);
        

        // t28
        resetSignals();
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t28", waitForSignal(mirrorMoveLeftSignalReceived));
        wait(1000);
        

        // t35
        resetSignals();
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t35", waitForSignal(mirrorMoveUpSignalReceived));
        wait(1000);
        

        // t12
        resetSignals();
        mirrorPositionDownEMPublisher->publish(msg);
        wait(500);
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t12", !waitForSignal(mirrorMoveDownSignalReceived));
        wait(1000);
        

        // t31
        resetSignals();
        mirrorUpEMPublisher->publish(msg);
        mirrorUpEMPublisher->publish(msg);
        mirrorUpEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t31", waitForSignal(mirrorMoveUpSignalReceived));
        wait(1000);
        

        // t47
        resetSignals();
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t47", waitForSignal(mirrorMoveLeftSignalReceived));
        wait(1000);
        

        // t5
        resetSignals();
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorRightEMPublisher->publish(msg);
        logTestResult("t5", !waitForSignal(mirrorMoveRightSignalReceived));
        wait(1000);
        

        // t33
        resetSignals();
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t33", waitForSignal(mirrorMoveDownSignalReceived));
        wait(1000);
        
        
        // t3
        resetSignals();
        mirrorPositionUpEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t3", !waitForSignal(mirrorMoveUpSignalReceived));
        wait(1000);
        

        // t36
        resetSignals();
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t36", waitForSignal(mirrorMoveDownSignalReceived));
        wait(1000);
        
        

        // t17
        resetSignals();
        mirrorPositionUpEMPublisher->publish(msg);
        wait(500);
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t17", waitForSignal(mirrorMoveLeftSignalReceived));
        wait(1000);
        
        
        // t2
        resetSignals();
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorRightEMPublisher->publish(msg);
        logTestResult("t2", !waitForSignal(mirrorMoveRightSignalReceived));
        wait(1000);
        

        // t48
        resetSignals();
        mirrorLeftEMPublisher->publish(msg);
        mirrorLeftEMPublisher->publish(msg);
        mirrorLeftEMPublisher->publish(msg);
        wait(500);
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t48", waitForSignal(mirrorMoveDownSignalReceived));
        wait(1000);
        

        // t15
        resetSignals();
        mirrorPositionUpEMPublisher->publish(msg);
        wait(500);
        mirrorRightEMPublisher->publish(msg);
        logTestResult("t15", waitForSignal(mirrorMoveRightSignalReceived));
        wait(1000);
        

    }
    catch (const std::exception& e) {
         std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== Exterior Mirror Test Completed ===" << std::endl;
}


void ExteriorMirrorTests::runExteriorMirrorWithHeatableTest() {

    std::cout << "\n=== Exterior Mirror Test With Heatable Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {

        // Top 
        resetSignals();
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorColdEMPublisher->publish(msg);
        logTestResult("t77", waitForSignal(heatingOnSignalReceived));
        wait(1000);
        

        resetSignals();
        msg.data = true;
        timerEMPublisher->publish(msg);
        logTestResult("t79", waitForSignal(heatingOffSignalReceived));
        wait(1000);

        // Top Left
        resetSignals();
        msg.data = true;
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorColdEMPublisher->publish(msg);
        logTestResult("t49", waitForSignal(heatingOnSignalReceived));
        wait(1000);
        

        resetSignals();
        wait(500);
        msg.data = true;
        timerEMPublisher->publish(msg);
        logTestResult("t51", waitForSignal(heatingOffSignalReceived));
        wait(1000);
        

        // Left
        resetSignals();
        msg.data = true;
        mirrorDownEMPublisher->publish(msg);
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorColdEMPublisher->publish(msg);
        logTestResult("t53", waitForSignal(heatingOnSignalReceived));
        wait(1000);
        

        resetSignals();
        wait(500);
        msg.data = true;
        timerEMPublisher->publish(msg);
        logTestResult("t55", waitForSignal(heatingOffSignalReceived));
        wait(1000);
        

        // Bottom Left
        resetSignals();
        msg.data = true;
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorColdEMPublisher->publish(msg);
        logTestResult("t57", waitForSignal(heatingOnSignalReceived));
        wait(1000);
        

        resetSignals();
        msg.data = true;
        timerEMPublisher->publish(msg);
        logTestResult("t59", waitForSignal(heatingOffSignalReceived));
        wait(1000);

        // Bottom 
        resetSignals();
        msg.data = true;
        mirrorRightEMPublisher->publish(msg);
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorColdEMPublisher->publish(msg);
        logTestResult("t63", waitForSignal(heatingOnSignalReceived));
        wait(1000);
        

        resetSignals();
        msg.data = true;
        timerEMPublisher->publish(msg);
        logTestResult("t61", waitForSignal(heatingOffSignalReceived));
        wait(1000);

        // Bottom Right
        resetSignals();
        msg.data = true;
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorColdEMPublisher->publish(msg);
        logTestResult("t67", waitForSignal(heatingOnSignalReceived));
        wait(1000);
        

        resetSignals();
        msg.data = true;
        timerEMPublisher->publish(msg);
        logTestResult("t65", waitForSignal(heatingOffSignalReceived));
        wait(1000);

        // Right
        resetSignals();
        msg.data = true;
        mirrorUpEMPublisher->publish(msg);
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorColdEMPublisher->publish(msg);
        logTestResult("t71", waitForSignal(heatingOnSignalReceived));
        wait(1000);
        

        resetSignals();
        msg.data = true;
        timerEMPublisher->publish(msg);
        logTestResult("t69", waitForSignal(heatingOffSignalReceived));
        wait(1000);

        // Top Right
        resetSignals();
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        msg.data = false;
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorColdEMPublisher->publish(msg);
        logTestResult("t75", waitForSignal(heatingOnSignalReceived));
        wait(1000);
        

        resetSignals();
        msg.data = true;
        timerEMPublisher->publish(msg);
        logTestResult("t73", waitForSignal(heatingOffSignalReceived));
        wait(1000);
        
        
        // Pending
        resetSignals();
        msg.data = false;
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorLeftEMPublisher->publish(msg);
        mirrorDownEMPublisher->publish(msg);
        wait(500);
        mirrorColdEMPublisher->publish(msg);
        logTestResult("t83", waitForSignal(heatingOnSignalReceived));
        wait(1000);
        

        resetSignals();
        msg.data = true;
        timerEMPublisher->publish(msg);
        logTestResult("t81", waitForSignal(heatingOffSignalReceived));
        wait(1000);


        
    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== Exterior Mirror With Heatable Test Completed ===" << std::endl;
}


void ExteriorMirrorTests::runExteriorMirrorWithLedTest() {

    std::cout << "\n=== Exterior Mirror Test With LED Started ===" << std::endl;
    std_msgs::msg::Bool msg;
    msg.data = true;
    
    try {
        // 1
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        logTestResult("t85", waitForSignal(vertTopReceived));
        wait(1000);
        

        // 2
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        wait(500);
        mirrorPositionLeftEMPublisher->publish(msg);
        logTestResult("t111", waitForSignal(horLeftReceived));
        wait(1000);
        

        // 3
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        mirrorRightEMPublisher->publish(msg);
        logTestResult("t89", waitForSignal(horPendReceived));
        wait(1000);
        

        // 4
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t91", waitForSignal(vertPendReceived));
        wait(1000);
        

        // 5
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        mirrorPositionUpEMPublisher->publish(msg);
        logTestResult("t93", waitForSignal(vertTopReceived));
        wait(1000);
        

        // 6
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorDownEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        mirrorPositionDownEMPublisher->publish(msg);
        logTestResult("t95", waitForSignal(vertBottomReceived));
        wait(1000);
        

        // 7
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t97", waitForSignal(vertPendReceived));
        wait(1000);
        

        // 8
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorRightEMPublisher->publish(msg);
        wait(500);
        mirrorPositionLeftEMPublisher->publish(msg);
        logTestResult("t99", waitForSignal(horLeftReceived));
        wait(1000);
        

        // 9
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        wait(500);
        mirrorRightEMPublisher->publish(msg);
        logTestResult("t101", waitForSignal(horPendReceived));
        wait(1000);
        

        // 10
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionDownEMPublisher->publish(msg);
        wait(500);
        mirrorPositionRightEMPublisher->publish(msg);
        logTestResult("t103", waitForSignal(horRightReceived));
        wait(1000);
        

        // 11
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t105", waitForSignal(horPendReceived));
        wait(1000);
        

        // 12
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t107", waitForSignal(vertPendReceived));
        wait(1000);
        

        // 13
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorPositionDownEMPublisher->publish(msg);
        logTestResult("t109", waitForSignal(vertBottomReceived));
        wait(1000);
        

        // 14
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionRightEMPublisher->publish(msg);
        mirrorUpEMPublisher->publish(msg);
        wait(500);
        mirrorPositionUpEMPublisher->publish(msg);
        logTestResult("t111", waitForSignal(vertTopReceived));
        wait(1000);
        

        // 15
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t113", waitForSignal(vertPendReceived));
        wait(1000);
        

        // 16
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorLeftEMPublisher->publish(msg);
        wait(500);
        mirrorPositionRightEMPublisher->publish(msg);
        logTestResult("t115", waitForSignal(horRightReceived));
        wait(1000);
        

        // 17
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t117", waitForSignal(horPendReceived));
        wait(1000);
        

        // 18
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionUpEMPublisher->publish(msg);
        wait(500);
        mirrorDownEMPublisher->publish(msg);
        logTestResult("t119", waitForSignal(vertPendReceived));
        wait(1000);
        

        // 19
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionLeftEMPublisher->publish(msg);
        logTestResult("t121", waitForSignal(horLeftReceived));
        wait(1000);
        

        // 20
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionLeftEMPublisher->publish(msg);
        wait(500);
        mirrorRightEMPublisher->publish(msg);
        logTestResult("t123", waitForSignal(horPendReceived));
        wait(1000);
        

        // 21
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionDownEMPublisher->publish(msg);
        logTestResult("t125", waitForSignal(vertBottomReceived));
        wait(1000);
        

        // 22
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionDownEMPublisher->publish(msg);
        wait(500);
        mirrorUpEMPublisher->publish(msg);
        logTestResult("t127", waitForSignal(vertPendReceived));
        wait(1000);
        

        // 23
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionRightEMPublisher->publish(msg);
        logTestResult("t129", waitForSignal(horRightReceived));
        wait(1000);
        

        // 24
        resetSignals();
        msg.data = false;
        mirrorPositionUpEMPublisher->publish(msg);
        mirrorPositionLeftEMPublisher->publish(msg);
        mirrorPositionDownEMPublisher->publish(msg);
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        msg.data = true;
        mirrorPositionRightEMPublisher->publish(msg);
        wait(500);
        mirrorLeftEMPublisher->publish(msg);
        logTestResult("t131", waitForSignal(horPendReceived));
        wait(1000);
        
    }
    catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
    }

    std::cout << "=== Exterior Mirror With LED Test Completed ===" << std::endl;
}

