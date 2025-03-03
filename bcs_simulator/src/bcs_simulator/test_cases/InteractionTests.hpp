#pragma once
#include "../src/ROS2Bridge.hpp"
#include <QObject>
#include <QThread>
#include <QString>
#include "PowerWindowTests.hpp"
#include "ExteriorMirrorTests.hpp"
#include "HMITests.hpp"
#include "SecurityTests.hpp"
#include "../src/MessageModel.hpp"

class InteractionTests : public QObject {
    Q_OBJECT
public:
    explicit InteractionTests(QObject* parent = nullptr);
    virtual ~InteractionTests() = default;

    void runMSC1Test();
    void runMSC2Test();
    void runMSC3Test();
    void runMSC4Test();
    void runMSC5Test();
    void runMSC6Test();
    void runMSC7Test();
    void runMSC8Test();
    void runMSC9Test();
    void runMSC10Test();
    void runMSC11Test();
    void runMSC12Test();
    void runMSC13Test();
    void runMSC14Test();
    void runMSC15Test();
    void runMSC16Test();
    void runMSC17Test();
    void runMSC18Test();
    void runMSC19Test();
    void runMSC20Test();
    void runMSC21Test();
    void runMSC22Test();
    void runMSC23Test();
    void runMSC24Test();
    void runMSC25Test();
    void runMSC26Test();
    void runMSC27Test();
    void runMSC28Test();
    void runMSC29Test();
    void runMSC30Test();
    void runMSC31Test();
    void runMSC32Test();
    void runMSC33Test();
    void runMSC34Test();
    void runMSC35Test();
    void runMSC36Test();
    void runMSC37Test();
    void runMSC38Test();
    void runMSC39Test();
    void runMSC40Test();
    void runMSC41Test();
    void runMSC42Test();
    void runMSC43Test();
    void runMSC44Test();
    void runMSC45Test();
    void runMSC46Test();
    void runMSC47Test();
    void runMSC48Test();
    void runMSC49Test();
    void runMSC50Test();
    void runMSC51Test();
    void runMSC52Test();
    void runMSC53Test();
    void runMSC54Test();
    void runMSC55Test();
    void runMSC56Test();
    void runMSC57Test();
    void runMSC58Test();
    void runMSC59Test();
    void runMSC60Test();
    void runMSC61Test();
    void runMSC62Test();
    void runMSC63Test();
    void runMSC64Test();
    void runMSC65Test();
    

private:
    rclcpp::Node::SharedPtr node;
    PowerWindowTests* pwTests;
    ExteriorMirrorTests* emTests;
    HMITests* hmiTests;
    SecurityTests* securityTests;
    MessageModel* m_messageModel;
    void addMessage(const QString& message, const QString& type);
    const int TIMEOUT_MS = 2000;

protected:
    void wait(int ms) { QThread::msleep(ms); }
};