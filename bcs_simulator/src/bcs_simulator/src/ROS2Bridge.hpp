#ifndef ROS2BRIDGE_HPP
#define ROS2BRIDGE_HPP
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <QObject>
#include <QQmlApplicationEngine>
#include <QGuiApplication>
#include <QAbstractListModel>
#include <QQmlContext>
#include <thread>
#include <QDateTime>
#include <QTimer> 
#include "MessageModel.hpp"

class ROS2Bridge : public QObject {
    Q_OBJECT
    Q_PROPERTY(QAbstractListModel* messageModel READ messageModel CONSTANT)
    Q_PROPERTY(int sliderPosition READ sliderPosition WRITE setSliderPosition NOTIFY sliderPositionChanged)
    Q_PROPERTY(int mirrorHorizontalPosition READ mirrorHorizontalPosition WRITE setMirrorHorizontalPosition NOTIFY mirrorHorizontalPositionChanged)
    Q_PROPERTY(int mirrorVerticalPosition READ mirrorVerticalPosition WRITE setMirrorVerticalPosition NOTIFY mirrorVerticalPositionChanged)
    Q_PROPERTY(bool inputsLocked READ inputsLocked NOTIFY inputsLockedChanged)
public: 
    explicit ROS2Bridge(QObject* parent = nullptr);
    ~ROS2Bridge();
public:
    Q_INVOKABLE void publishWindowUp();
    Q_INVOKABLE void publishWindowDown();
    Q_INVOKABLE void publishWindowPositionUp(bool position);
    Q_INVOKABLE void publishWindowPositionDown(bool position);
    Q_INVOKABLE void fingerDetected(bool detection);
    Q_INVOKABLE void publishEMPosTop(bool top);
    Q_INVOKABLE void publishEMPosBottom(bool bottom);
    Q_INVOKABLE void publishEMPosLeft(bool left);
    Q_INVOKABLE void publishEMPosRight(bool right);
    Q_INVOKABLE void publishEMCold(bool cold);
    Q_INVOKABLE void publishMirrorUp();
    Q_INVOKABLE void publishMirrorDown();
    Q_INVOKABLE void publishMirrorLeft();
    Q_INVOKABLE void publishMirrorRight();
    Q_INVOKABLE void publishAlarmActivate();
    Q_INVOKABLE void publishAlarmDeactivate();
    Q_INVOKABLE void publishAlarmConfirm();
    Q_INVOKABLE void publishAlarmTriggered();
    Q_INVOKABLE void publishAlarmIMTriggered();
    Q_INVOKABLE void publishReleaseWindowUp();
    Q_INVOKABLE void publishReleaseWindowDown();
    Q_INVOKABLE void publishRemoteWindowUp();
    Q_INVOKABLE void publishRemoteWindowDown();
    Q_INVOKABLE void publishRemoteLock();
    Q_INVOKABLE void publishRemoteUnlock();
    Q_INVOKABLE void carDoor(bool door);
    Q_INVOKABLE void carDriving(bool driving);
    Q_INVOKABLE void publishCentralLock();
    Q_INVOKABLE void publishCentralUnlock();
    void windowUpLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void windowDownLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void windowUpLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);  
    void windowDownLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void fingerProtectionLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void fingerProtectionLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void centralWindowUpLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void centralWindowUpLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorHeatingOn(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorHeatingOff(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorUpLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorUpLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorDownLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorDownLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorLeftLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorLeftLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorRightLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorRightLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorHeaterLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void mirrorHeaterLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmDetectedLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmDetectedLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmSignalOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmSignalOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmInteriorOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmInteriorOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmActivationOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmActivationOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmSignalLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmSignalLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmInteriorLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmInteriorLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmActivationLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void alarmActivationLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void carLockedMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void carUnlockedMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void clsLockMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void clsUnlockMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void centralLockLedOnMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void centralLockLedOffMessage(const std_msgs::msg::Bool::SharedPtr msg);
    void handleWindowMoveUp(const std_msgs::msg::Bool::SharedPtr msg);
    void handleWindowMoveDown(const std_msgs::msg::Bool::SharedPtr msg);
    void handleWindowAutoMoveUp(const std_msgs::msg::Bool::SharedPtr msg);
    void handleWindowAutoMoveDown(const std_msgs::msg::Bool::SharedPtr msg);
    void handleWindowAutoMoveStop(const std_msgs::msg::Bool::SharedPtr msg);
    void handleMirrorUp(const std_msgs::msg::Bool::SharedPtr msg);
    void handleMirrorDown(const std_msgs::msg::Bool::SharedPtr msg);
    void handleMirrorLeft(const std_msgs::msg::Bool::SharedPtr msg);
    void handleMirrorRight(const std_msgs::msg::Bool::SharedPtr msg);
    QAbstractListModel* messageModel();

    void setTestMode(bool enabled) { m_testMode = enabled; }
    bool isInTestMode() const { return m_testMode; }
    bool inputsLocked() const { return m_inputsLocked; }

public slots:
    int sliderPosition() const ;
    int mirrorHorizontalPosition() const ;
    int mirrorVerticalPosition() const ;
    void setSliderPosition(int position) ;
    void setMirrorVerticalPosition(int position) ;
    void setMirrorHorizontalPosition(int position) ;
    void startAutoMoveTimer() ;
    void stopAutoMoveTimer() ;
    void updateAutoMove() ;
    void handleHeatingTimeout();
    void handleRemoteTimeout();
    void handleAlarmTimeout();
    void messageCallback(const std_msgs::msg::String::SharedPtr msg) ;
    void addMessage(const QString& message, const QString& type, const QString& timestamp = "") ;

signals:
    void messageReceived(const QString& message);
    void windowMoveUpSignal();
    void windowMoveDownSignal();
    void windowPositionUpMaxSignal();
    void windowPositionDownMaxSignal();
    void windowAutoMoveUpSignal();
    void windowAutoMoveDownSignal();
    void windowAutoMoveStopSignal();

    void mirrorMoveUpSignal();
    void mirrorMoveDownSignal();
    void mirrorMoveLeftSignal();
    void mirrorMoveRightSignal();
    void mirrorPosUpSignal();
    void mirrorPosDownSignal();
    void mirrorPosLeftSignal();
    void mirrorPosRightSignal();
    void mirrorHeatingOnSignal();
    void mirrorHeatingOffSignal();

    void windowUpLedOnSignal();
    void windowUpLedOffSignal();
    void windowDownLedOnSignal();
    void windowDownLedOffSignal();
    void fingerProtectionLedOnSignal();
    void fingerProtectionLedOffSignal();
    void centralWindowUpLedOnSignal();
    void centralWindowUpLedOffSignal();
    void mirrorUpLedOnSignal();
    void mirrorUpLedOffSignal();
    void mirrorDownLedOnSignal();
    void mirrorDownLedOffSignal();
    void mirrorLeftLedOnSignal();
    void mirrorLeftLedOffSignal();
    void mirrorRightLedOnSignal();
    void mirrorRightLedOffSignal();
    void mirrorHeaterLedOnSignal();
    void mirrorHeaterLedOffSignal();
    void alarmDetectedLedOnSignal();
    void alarmDetectedLedOffSignal();
    void alarmSignalOnSignal();
    void alarmSignalOffSignal();
    void alarmInteriorOnSignal();
    void alarmInteriorOffSignal();
    void alarmActivationOnSignal();
    void alarmActivationOffSignal();
    void alarmSignalLedOnSignal();
    void alarmSignalLedOffSignal();
    void alarmInteriorLedOnSignal();
    void alarmInteriorLedOffSignal();
    void alarmActivationLedOnSignal();
    void alarmActivationLedOffSignal();
    void carLockedSignal();
    void carUnlockedSignal();
    void clsLockSignal();
    void clsUnlockSignal();
    void centralLockLedOnSignal();
    void centralLockLedOffSignal();
    void startAutoMoveRequested();
    void stopAutoMoveRequested();
    void sliderPositionChanged(int position);
    void mirrorVerticalPositionChanged(int position);
    void mirrorHorizontalPositionChanged(int position);
    void inputsLockedChanged();

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowUpPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowDownPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowPositionUpPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr windowPositionDownPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fingerDetectedPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorUpPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorDownPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorLeftPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorRightPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorPosUpPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorPosDownPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorPosRightPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorPosLeftPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mirrorColdPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heaterTimerPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr activateAlarmPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deactivateAlarmPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr confirmAlarmPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmASTriggerPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmIMTriggerPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarmTimerPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr releaseWindowUpPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr releaseWindowDownPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteTimerPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteWindowUpPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteWindowDownPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteLockPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remoteUnlockPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr carDoorPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr centralLockPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr centralUnlockPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr carDrivingPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowUpLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowDownLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fingerProtectionLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fingerProtectionLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr centralWindowUpLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr centralWindowUpLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorUpSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorDownSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorLeftSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorRightSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorColdSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorHeatingOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorHeatingOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorUpLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorUpLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorDownLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorDownLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorLeftLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorLeftLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorRightLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorRightLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorHeaterLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mirrorHeaterLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmDetectedLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmDetectedLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmSignalOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmSignalOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmInteriorOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmInteriorOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmActivationOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmActivationOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmSignalLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmSignalLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmInteriorLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmInteriorLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmActivationLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alarmActivationLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr carLockedSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr carUnlockedSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clsLockSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clsUnlockSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr centralLockLedOnSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr centralLockLedOffSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowMoveUpSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowMoveDownSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowAutoMoveUpSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowAutoMoveDownSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr windowAutoMoveStopSubscriber;
    int windowPosition;
    int mirrorXPosition;
    int mirrorYPosition;
    QTimer* autoMoveTimer;
    QTimer* heatingTimer;
    QTimer* remoteTimer;
    QTimer* alarmTimer;
    std::atomic<bool> isMovingUp{false};
    std::atomic<bool> isMovingDown{false};
    std::thread ros_thread;
    MessageModel m_messageModel;
    bool m_inputsLocked = false;

    bool m_testMode = false;

    void initializePublishers();
    void initializeSubscribers();
    void setupConnections();
};

#endif