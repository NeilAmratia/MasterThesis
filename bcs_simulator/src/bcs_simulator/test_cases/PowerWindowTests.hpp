#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

class PowerWindowTests {
public:
    explicit PowerWindowTests(std::shared_ptr<rclcpp::Node> node_);
    ~PowerWindowTests();

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowPositionUpPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowPositionDownPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowUpPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowDownPWPublisher;
    
    // CLS
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clsLockPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clsUnlockPWPublisher;

    // Manual PW
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownPWSubscriber;
    
    // Auto PW
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpAutoPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownAutoPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowStopAutoPWSubscriber;

    // FP
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fpOnPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fpOffPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fingerDetectedPWPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fpOnPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fpOffPWSubscriber;

    void runManualPWTest();
    void runManualPWWithCLSTest();
    void runAutomaticPWTest();
    void runAutomaticPWWithCLSTest();
    void runFPTest();

private:
    std::shared_ptr<rclcpp::Node> node;
    static const int TIMEOUT_MS = 2000;

    bool windowUpPWReceived = false;
    bool windowDownPWReceived = false;
    bool windowUpAutoPWReceived = false;
    bool windowDownAutoPWReceived = false;
    bool windowStopAutoPWReceived = false;
    bool fpOnValue = false;
    bool fpOffValue = false;

    void resetSignals();
    bool waitForSignal(bool& signal);
    void wait(int ms);
    void logTestResult(const std::string& testId, bool passed);

};