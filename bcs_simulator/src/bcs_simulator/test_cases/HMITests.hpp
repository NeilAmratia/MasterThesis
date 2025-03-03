#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

class HMITests {
public:
    explicit HMITests(std::shared_ptr<rclcpp::Node> node_);
    ~HMITests();

    // HMI Standard
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowUpHMIPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowDownHMIPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorUpHMIPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorDownHMIPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorLeftHMIPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorRightHMIPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpHMISubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownHMISubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorUpHMISubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorDownHMISubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorLeftHMISubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorRightHMISubscriber;

    // HMI variants
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmActivatedHMIPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmDeactivatedHMIPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmActivatedHMISubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmDeactivatedHMISubscriber;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr confirmAlarmHMIPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr confirmAlarmHMISubscriber;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr releasePWUpHMIPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr releasePWDownHMIPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr releasePWHMISubscriber;

    // LED ManPW
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowUpLEDManPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowDownLEDManPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr releaseLEDManPWPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpLEDOnManPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpLEDOffManPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownLEDOnManPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownLEDOffManPWSubscriber;

    // LED AutoPW
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowUpLEDAutoPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowDownLEDAutoPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowStopLEDAutoPWPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpLEDOnAutoPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpLEDOffAutoPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownLEDOnAutoPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownLEDOffAutoPWSubscriber;

    // LED AutoPW + CLS
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr centralLockLEDAutoPWPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr centralUnlockLEDAutoPWPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr centralLockLEDOnAutoPWSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr centralUnlockLEDOffAutoPWSubscriber;

    //LED EM Top
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorVertTopPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorVertTopPendPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorUpLEDOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorUpLEDOffSubscriber;

    //LED EM Left
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorHorLeftPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorHorLeftPendPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorLeftLEDOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorLeftLEDOffSubscriber;

    //LED EM Bottom
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorVertBottomPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorVertBottomPendPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorDownLEDOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorDownLEDOffSubscriber;

    //LED EM Right
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorHorRightPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorHorRightPendPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorRightLEDOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorRightLEDOffSubscriber;

    //LED EM Heating
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heatingOnLEDPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heatingOffLEDPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heatingOnLEDSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heatingOffLEDSubscriber;

    //LED FP
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fpOnPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fpOffPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fpOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fpOffSubscriber;

    // LED AS Active
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmActiveOnPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmActiveOffPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmActiveOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmActiveOffSubscriber;

    // LED AS Alarm
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmOnPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmOffPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmOffSubscriber;

    // LED AS Detected
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmDetectedOnPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmDetectedOffPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmDetectedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmDetectedOffSubscriber;

    // LED AS IM
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmIMOnPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmIMOffPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmIMOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmIMOffSubscriber;

    // LED CLS
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clsLockPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clsUnlockPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clsLockSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clsUnlockSubscriber;

    void runHMITest();
    void runHMIWithASTest();
    void runHMIWithASLEDTest();
    void runHMIWithManPWLEDTest();
    void runLEDManPWTest();
    void runLEDAutoPWTest();
    void runLEDAutoPWWithCLSTest();
    void runLEDEMTTest();
    void runLEDEMLTest();
    void runLEDEMBTest();
    void runLEDEMRTest();
    void runLEDEMHTest();
    void runLEDFPTest();
    void runLEDASACTest();
    void runLEDASALTest();
    void runLEDASADTest();
    void runLEDASIMTest();
    void runLEDCLSTest();

private:
    std::shared_ptr<rclcpp::Node> node;
    const int TIMEOUT_MS = 2000;

    bool windowUpHMIReceived = false;
    bool windowDownHMIReceived = false;
    bool mirrorUpHMIReceived = false;
    bool mirrorDownHMIReceived = false;
    bool mirrorLeftHMIReceived = false;
    bool mirrorRightHMIReceived = false;

    bool alarmActivatedHMIReceived = false;
    bool alarmDeactivatedHMIReceived = false;
    bool confirmAlarmHMIReceived = false;
    bool releasePWHMIReceived = false;

    bool windowUpLEDOnManPWReceived = false;
    bool windowUpLEDOffManPWReceived = false;
    bool windowDownLEDOnManPWReceived = false;
    bool windowDownLEDOffManPWReceived = false;

    bool windowUpLEDOnAutoPWReceived = false;
    bool windowUpLEDOffAutoPWReceived = false;
    bool windowDownLEDOnAutoPWReceived = false;
    bool windowDownLEDOffAutoPWReceived = false;

    bool centralLockLEDOnAutoPWReceived = false;
    bool centralUnlockLEDOffAutoPWReceived = false;

    bool mirrorUpLEDOnReceived = false;
    bool mirrorUpLEDOffReceived = false;

    bool mirrorLeftLEDOnReceived = false;
    bool mirrorLeftLEDOffReceived = false;

    bool mirrorDownLEDOnReceived = false;
    bool mirrorDownLEDOffReceived = false;

    bool mirrorRightLEDOnReceived = false;
    bool mirrorRightLEDOffReceived = false;

    bool heatingOnLEDReceived = false;
    bool heatingOffLEDReceived = false;

    bool fpOnReceived = false;
    bool fpOffReceived = false;

    bool alarmActiveOnReceived = false;
    bool alarmActiveOffReceived = false;

    bool alarmOnReceived = false;
    bool alarmOffReceived = false;

    bool alarmDetectedOnReceived = false;
    bool alarmDetectedOffReceived = false;

    bool alarmIMOnReceived = false;
    bool alarmIMOffReceived = false;

    bool clsLockReceived = false;
    bool clsUnlockReceived = false;

    void wait(int ms);
    void resetSignals();
    bool waitForSignal(bool& signal);
    void logTestResult(const std::string& testId, bool passed);

};