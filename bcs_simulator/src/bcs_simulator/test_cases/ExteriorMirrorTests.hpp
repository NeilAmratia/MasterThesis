#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

class ExteriorMirrorTests {
public:
    explicit ExteriorMirrorTests(std::shared_ptr<rclcpp::Node> node_);
    ~ExteriorMirrorTests();

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorUpEMPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorDownEMPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorLeftEMPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorRightEMPublisher;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorPositionUpEMPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorPositionDownEMPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorPositionLeftEMPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorPositionRightEMPublisher;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorUpEMSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorDownEMSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorLeftEMSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorRightEMSubscriber;

    // Heatable
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorColdEMPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr timerEMPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heatingOnEMSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heatingOffEMSubscriber;

    // LED
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vertPendEMSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vertTopEMSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vertBottomEMSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr horPendEMSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr horLeftEMSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr horRightEMSubscriber;

    void runExteriorMirrorTest();
    void runExteriorMirrorWithHeatableTest();
    void runExteriorMirrorWithLedTest();

private:
    std::shared_ptr<rclcpp::Node> node;
    const int TIMEOUT_MS = 2000;

    bool mirrorMoveUpSignalReceived = false;
    bool mirrorMoveDownSignalReceived = false;
    bool mirrorMoveLeftSignalReceived = false;
    bool mirrorMoveRightSignalReceived = false;

    bool heatingOnSignalReceived = false;
    bool heatingOffSignalReceived = false;

    bool vertPendReceived = false;
    bool vertTopReceived = false;
    bool vertBottomReceived = false;
    bool horPendReceived = false;
    bool horLeftReceived = false;
    bool horRightReceived = false;

    void wait(int ms);
    void resetSignals();
    bool waitForSignal(bool& signal);
    void logTestResult(const std::string& testId, bool passed);

};