#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

class SecurityTests {
public:
    explicit SecurityTests(std::shared_ptr<rclcpp::Node> node_);
    ~SecurityTests() ;

    // RCK
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteLockRCKPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteUnlockRCKPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr remoteLockRCKSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr remoteUnlockRCKSubscriber;

    // RCK + SF
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr doorOpenRCKPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr timerRCKPublisher;

    // RCK + Control Automatic PW
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowUpRCKPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowDownRCKPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpRCKSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownRCKSubscriber;

    // CLS
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr centralLockCLSPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr centralUnlockCLSPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr centralLockCLSSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr centralUnlockCLSSubscriber;

    // CLS + RCK
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteLockCLSPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteUnlockCLSPublisher;

    // CLS + Auto Locking
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr carDrivesCLSPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr doorOpenCLSPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr carLockCLSSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr carUnlockCLSSubscriber;

    // AS
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmSystemLockASPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmSystemUnlockASPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmActivatedASPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmDeactivatedASPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmTimerASPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmDetectedASPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmActiveOnASSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmActiveOffASSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmAlarmOnASSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmAlarmOffASSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmAlarmDetectedASSubscriber;

    // AS + Control AS
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteLockASPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteUnlockASPublisher;

    // AS + IM
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmIMASPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmIMOnASSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmIMOffASSubscriber;

    void runRCKTest();
    void runRCKWithSFTest();
    void runRCKWithControlAutoPWTest();
    void runRCKWithSFandControlAutoPWTest();
    void runCLSTest();
    void runCLSWithRCKTest();
    void runCLSWithAutoLockingTest();
    void runASTest();
    void runASWithControlASTest();
    void runASWithIMTest();

private:
    std::shared_ptr<rclcpp::Node> node;
    static const int TIMEOUT_MS = 2000;

    bool remoteLockRCKReceived = false;
    bool remoteUnlockRCKReceived = false;

    bool windowUpRCKReceived = false;
    bool windowDownRCKReceived = false;

    bool centralLockCLSReceived = false;
    bool centralUnlockCLSReceived = false;

    bool carLockCLSReceived = false;
    bool carUnlockCLSReceived = false;

    bool alarmActiveOnASReceived = false;
    bool alarmActiveOffASReceived = false;
    bool alarmAlarmOnASReceived = false;
    bool alarmAlarmOffASReceived = false;
    bool alarmAlarmDetectedASReceived = false;

    bool alarmIMOnASReceived = false;
    bool alarmIMOffASReceived = false;

    void wait(int ms);
    void resetSignals();
    bool waitForSignal(bool& signal);
    void logTestResult(const std::string& testId, bool passed);

};