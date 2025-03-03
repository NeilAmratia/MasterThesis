#include "InteractionTests.hpp"

InteractionTests::InteractionTests(QObject* parent) 
    : QObject(nullptr)
    , m_messageModel(new MessageModel(this))
{
    node = std::make_shared<rclcpp::Node>("interaction_test_node");
    pwTests = new PowerWindowTests(this);
    emTests = new ExteriorMirrorTests(this);
    hmiTests = new HMITests(this);
    securityTests = new SecurityTests(this); 
}

void InteractionTests::addMessage(const QString& message, const QString& type) {
    QVariantMap messageData;
    messageData["message"] = message;
    messageData["type"] = type;
    messageData["number"] = m_messageModel->rowCount() + 1;
    messageData["timestamp"] = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    m_messageModel->appendMessage(messageData);
}

void InteractionTests::runMSC1Test() {
    addMessage("MSC1: FP Interaction Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue)) {
            addMessage("Step 1: Finger Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        
        hmiTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowDownHMIReceived)) {
            addMessage("Step 2: Window Down Button", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        
        pwTests->resetSignals();
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 3: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        
        pwTests->resetSignals();
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 4: Window Movement Complete", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC2: All Steps", "test_pass");
        }
        
    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
}

void InteractionTests::runMSC2Test() {
    addMessage("MSC2: Window Operation Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // First Window Up Sequence
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowUpHMIReceived) ||
            !pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
            addMessage("Step 1: First Window Up Complete", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Second Window Up Sequence
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowUpHMIReceived) ||
            !pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
            addMessage("Step 2: Second Window Up Complete", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Finger Protection Sequence
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue)) {
            addMessage("Step 3: FP Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Up with FP Active
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowUpHMIReceived)) {
            addMessage("Step 4: Window Up Button with FP", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down with FP
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowDownHMIReceived) ||
            !pwTests->waitForSignal(pwTests->fpOffValue) ||
            !pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 5: Window Down with FP Complete", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Window Up Sequences
        for (int i = 0; i < 2; i++) {
            hmiTests->resetSignals();
            pwTests->resetSignals();
            hmiTests->windowUpHMIPublisher->publish(msg);
            if (!hmiTests->waitForSignal(hmiTests->windowUpHMIReceived) ||
                !pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
                addMessage("Step " + QString::number(6+i) + ": Final Window Up Complete", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        if (testPassed) {
            addMessage("MSC2: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC2: Test Complete", "info");
}

void InteractionTests::runMSC3Test() {
    addMessage("MSC3: Manual Window Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // First Window Up Sequence (2x)
        for (int i = 0; i < 2; i++) {
            pwTests->resetSignals();
            pwTests->windowUpPWPublisher->publish(msg);
            if (!pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
                addMessage("Step " + QString::number(1+i) + ": Window Up Operation", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Finger Protection Sequence
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue)) {
            addMessage("Step 3: FP Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Operation with FP Active
        pwTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 4: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down Operation
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 5: Window Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Additional Window Down Operations
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 6: Additional Window Down", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Position Check Down
        pwTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        for (int i = 0; i < 2; i++) {
            pwTests->resetSignals();
            pwTests->windowUpPWPublisher->publish(msg);
            if (!pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
                addMessage("Step " + QString::number(8+i) + ": Final Window Up", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Position Check Up
        pwTests->resetSignals();
        pwTests->windowPositionUpPWPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC3: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC3: Test Complete", "info");
}

void InteractionTests::runMSC4Test() {
    addMessage("MSC4: Exterior Mirror Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Left Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorLeftHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftHMIReceived) ||
            !emTests->waitForSignal(emTests->mirrorMoveLeftSignalReceived)) {
            addMessage("Step 1: Mirror Left Movement", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionLeftEMPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Up Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorUpHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorUpHMIReceived) ||
            !emTests->waitForSignal(emTests->mirrorMoveUpSignalReceived)) {
            addMessage("Step 3: Mirror Up Movement", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionUpEMPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Right Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorRightHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorRightHMIReceived) ||
            !emTests->waitForSignal(emTests->mirrorMoveRightSignalReceived)) {
            addMessage("Step 5: Mirror Right Movement", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionRightEMPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorDownHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorDownHMIReceived) ||
            !emTests->waitForSignal(emTests->mirrorMoveDownSignalReceived)) {
            addMessage("Step 7: Mirror Down Movement", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionDownEMPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC4: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC4: Test Complete", "info");
}

void InteractionTests::runMSC5Test() {
    addMessage("MSC5: Power Window Operation Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // First Two Window Up Sequences
        for (int i = 0; i < 2; i++) {
            hmiTests->resetSignals();
            pwTests->resetSignals();
            hmiTests->windowUpHMIPublisher->publish(msg);
            if (!hmiTests->waitForSignal(hmiTests->windowUpHMIReceived) ||
                !pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
                addMessage("Step " + QString::number(1+i) + ": Window Up Sequence", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Window Down Sequence
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowDownHMIReceived) ||
            !pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 3: Window Down Sequence", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Two Window Up Sequences
        for (int i = 0; i < 2; i++) {
            hmiTests->resetSignals();
            pwTests->resetSignals();
            hmiTests->windowUpHMIPublisher->publish(msg);
            if (!hmiTests->waitForSignal(hmiTests->windowUpHMIReceived) ||
                !pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
                addMessage("Step " + QString::number(4+i) + ": Final Window Up", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        if (testPassed) {
            addMessage("MSC5: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC5: Test Complete", "info");
}

void InteractionTests::runMSC6Test() {
    addMessage("MSC6: Finger Protection Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Finger Detection Sequence
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue)) {
            addMessage("Step 1: FP Detection & Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down with FP Active
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowDownHMIReceived) ||
            !pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 2: Window Down with FP", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC6: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC6: Test Complete", "info");
}

void InteractionTests::runMSC7Test() {
    addMessage("MSC7: FP with LED Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Finger Detection and LED Activation
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOnReceived)) {
            addMessage("Step 1: FP and LED Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down with FP and LED Deactivation
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOffReceived)) {
            addMessage("Step 2: FP and LED Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC7: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC7: Test Complete", "info");
}

void InteractionTests::runMSC8Test() {
    addMessage("MSC8: CLS Lock Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Key Lock Position
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 1: CLS Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down Button Press While Locked
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Key Unlock Position
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 3: CLS Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Auto Down After Unlock
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 4: Auto Window Operation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC8: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC8: Test Complete", "info");
}

void InteractionTests::runMSC9Test() {
    addMessage("MSC9: Auto Window with FP Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Auto Window Up Movement (3x)
        for (int i = 0; i < 3; i++) {
            pwTests->resetSignals();
            pwTests->windowUpPWPublisher->publish(msg);
            if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
                addMessage("Step " + QString::number(1+i) + ": Auto Window Up", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // FP Detection and Stop
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 4: FP Detection and Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Operation with FP
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 5: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Auto Window Down Movement (3x)
        for (int i = 0; i < 3; i++) {
            pwTests->resetSignals();
            pwTests->windowDownPWPublisher->publish(msg);
            if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
                addMessage("Step " + QString::number(6+i) + ": Auto Window Down", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Position Check and Stop
        pwTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 9: Position Detection and Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC9: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC9: Test Complete", "info");
}

void InteractionTests::runMSC10Test() {
    addMessage("MSC10: CLS with AutoPW and FP Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Unlock Sequence
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: CLS Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Auto Window Down Movement (3x)
        for (int i = 0; i < 3; i++) {
            pwTests->resetSignals();
            pwTests->windowDownPWPublisher->publish(msg);
            if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
                addMessage("Step " + QString::number(2+i) + ": Auto Window Down", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Position Down Check
        pwTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 5: Down Position Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Lock Sequence and Auto Up
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 6: CLS Lock", "test_fail");
            testPassed = false;
        }
        for (int i = 0; i < 2; i++) {
            if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
                addMessage("Step " + QString::number(7+i) + ": Auto Window Up", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // FP Detection and Stop
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 9: FP Detection and Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // FP Deactivation
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 10: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Unlock and Down Movement
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 11: Final Unlock", "test_fail");
            testPassed = false;
        }
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 12: Final Window Down", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC10: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC10: Test Complete", "info");
}

void InteractionTests::runMSC11Test() {
    addMessage("MSC11: CLS Lock/Unlock with AutoPW Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Lock Sequence
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 1: CLS Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down Attempt While Locked
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 2: Window Operation While Locked", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Unlock Sequence
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 3: CLS Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down After Unlock
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowDownHMIReceived) ||
            !pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 4: Window Operation After Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC11: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC11: Test Complete", "info");
}

void InteractionTests::runMSC12Test() {
    addMessage("MSC12: FP with LED Indicator Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Finger Detection and LED Activation
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOnReceived)) {
            addMessage("Step 1: FP and LED Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down with FP and LED Deactivation
        pwTests->resetSignals();
        hmiTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOffReceived)) {
            addMessage("Step 2: FP and LED Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC12: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC12: Test Complete", "info");
}

void InteractionTests::runMSC13Test() {
    addMessage("MSC13: Complex Auto Window Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Unlock and Window Down
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: Initial Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down to Position
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        for (int i = 0; i < 3; i++) {
            if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
                addMessage("Step 2: Auto Down Movement " + QString::number(i+1), "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Down Position Check
        pwTests->windowPositionDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 3: Down Position Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Lock and Auto Up
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralLockCLSPublisher->publish(msg);
        for (int i = 0; i < 2; i++) {
            if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
                addMessage("Step 4: Auto Up Movement " + QString::number(i+1), "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // FP Detection and Stop
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 5: FP Detection and Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down with FP
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 6: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Unlock and Down
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 7: Final Unlock", "test_fail");
            testPassed = false;
        }
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 8: Final Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC13: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC13: Test Complete", "info");
}

void InteractionTests::runMSC14Test() {
    addMessage("MSC14: Auto Window with FP Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Window Up Movement (3x)
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        for (int i = 0; i < 3; i++) {
            if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
                addMessage("Step " + QString::number(1+i) + ": Auto Up Movement", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // FP Detection and Stop
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 4: FP Detection and Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Operation with FP Active
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 5: FP Deactivation", "test_fail");
            testPassed = false;
        }
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 6: Auto Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Window Up to Position
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 7 Final Up Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();


        if (testPassed) {
            addMessage("MSC14: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC14: Test Complete", "info");
}

void InteractionTests::runMSC15Test() {
    addMessage("MSC15: Exterior Mirror with Heating Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Left Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorLeftHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftHMIReceived) ||
            !emTests->waitForSignal(emTests->mirrorMoveLeftSignalReceived)) {
            addMessage("Step 1: Mirror Left Movement", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionLeftEMPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Up Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorUpHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorUpHMIReceived) ||
            !emTests->waitForSignal(emTests->mirrorMoveUpSignalReceived)) {
            addMessage("Step 3: Mirror Up Movement", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionUpEMPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Heating Activation
        emTests->resetSignals();
        emTests->mirrorColdEMPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->heatingOnSignalReceived)) {
            addMessage("Step 5: Heating Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Right Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorRightHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorRightHMIReceived) ||
            !emTests->waitForSignal(emTests->mirrorMoveRightSignalReceived)) {
            addMessage("Step 6: Mirror Right Movement", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionRightEMPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorDownHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorDownHMIReceived) ||
            !emTests->waitForSignal(emTests->mirrorMoveDownSignalReceived)) {
            addMessage("Step 8: Mirror Down Movement", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionDownEMPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Heating Deactivation
        emTests->resetSignals();
        emTests->timerEMPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->heatingOffSignalReceived)) {
            addMessage("Step 10: Heating Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC15: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC15: Test Complete", "info");
}

void InteractionTests::runMSC16Test() {
    addMessage("MSC16: RCK with AutoPW and FP Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK Unlock Sequence
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: RCK Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Auto Window Down Movement (3x)
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        for (int i = 0; i < 3; i++) {
            if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
                addMessage("Step 2: Auto Down Movement " + QString::number(i+1), "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Position Down Check
        pwTests->windowPositionDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 3: Down Position Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // RCK Lock and Auto Up
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 4: RCK Lock", "test_fail");
            testPassed = false;
        }
        for (int i = 0; i < 2; i++) {
            if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
                addMessage("Step 5: Auto Up Movement " + QString::number(i+1), "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // FP Detection Sequence
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 6: FP Detection and Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // FP Deactivation
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 7: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Unlock and Down
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 8: Final Unlock", "test_fail");
            testPassed = false;
        }
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 9: Final Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC16: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC16: Test Complete", "info");
}

void InteractionTests::runMSC17Test() {
    addMessage("MSC17: RCK Lock/Unlock Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Lock Sequence
        securityTests->resetSignals();
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 1: RCK Lock Sequence", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Unlock Sequence
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 2: RCK Unlock Sequence", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC17: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC17: Test Complete", "info");
}

void InteractionTests::runMSC18Test() {
    addMessage("MSC18: Complete System Integration Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial RCK Unlock Sequence
        securityTests->resetSignals();
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: Initial Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down Movement (3x)
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        for (int i = 0; i < 3; i++) {
            if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
                addMessage("Step 2: Auto Down Movement " + QString::number(i+1), "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Position Check and Stop
        pwTests->windowPositionDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 3: Down Position Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // RCK Lock and Window Up
        securityTests->resetSignals();
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 4: RCK Lock", "test_fail");
            testPassed = false;
        }
        for (int i = 0; i < 2; i++) {
            if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
                addMessage("Step 5: Auto Up Movement " + QString::number(i+1), "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // FP Detection and Stop
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 6: FP Detection and Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down with FP Deactivation
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 7: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Unlock and Down Movement
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 8: Final Unlock", "test_fail");
            testPassed = false;
        }
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 9: Final Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC18: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC18: Test Complete", "info");
}

void InteractionTests::runMSC19Test() {
    addMessage("MSC19: Alarm System Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // AS Activation Sequence
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmActivatedHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmActivatedHMIReceived)) {
            addMessage("Step 1: AS Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Alarm Detection
        securityTests->resetSignals();
        securityTests->alarmDetectedASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived)) {
            addMessage("Step 3: Alarm Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // AS Deactivation
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmDeactivatedHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmDeactivatedHMIReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmAlarmOffASReceived)) {
            addMessage("Step 4: AS Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC19: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC19: Test Complete", "info");
}

void InteractionTests::runMSC20Test() {
    addMessage("MSC20: Immobilizer Alarm Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // AS Activation Sequence
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmActivatedHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmActivatedHMIReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 1: AS Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Immobilizer Alarm Detection
        securityTests->resetSignals();
        securityTests->alarmIMASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmIMOnASReceived)) {
            addMessage("Step 3: Immobilizer Alarm Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // AS Deactivation
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmDeactivatedHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmDeactivatedHMIReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmIMOffASReceived)) {
            addMessage("Step 4: AS Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC20: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC20: Test Complete", "info");
}

void InteractionTests::runMSC21Test() {
    addMessage("MSC21: AS Key Unlock Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // AS Activation Sequence
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmActivatedHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmActivatedHMIReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 1: AS Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();


        // Alarm Detection
        securityTests->resetSignals();
        securityTests->alarmDetectedASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived)) {
            addMessage("Step 3: Alarm Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Key Unlock Deactivation
        securityTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOffASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 4: Key Unlock Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC21: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC21: Test Complete", "info");
}

void InteractionTests::runMSC22Test() {
    addMessage("MSC22: AS Timer Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // AS Activation and Key Lock
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmActivatedHMIPublisher->publish(msg);
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmActivatedHMIReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 1: AS Activation and Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Alarm Detection
        securityTests->resetSignals();
        securityTests->alarmDetectedASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived)) {
            addMessage("Step 3: Alarm Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Timer Elapsed and Alarm Status
        securityTests->resetSignals();
        securityTests->alarmTimerASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmDetectedASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmAlarmOffASReceived)) {
            addMessage("Step 4: Timer Elapsed Processing", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Key Unlock and Deactivation
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 5: Key Unlock", "test_fail");
            testPassed = false;
        }
        hmiTests->alarmDeactivatedHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmDeactivatedHMIReceived)) {
            addMessage("Step 6: AS Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC22: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC22: Test Complete", "info");
}

void InteractionTests::runMSC23Test() {
    addMessage("MSC23: FP with LED and AutoPW Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Finger Detection and LED/AutoPW Activation
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOnReceived) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 1: FP and LED Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down Operation and FP Deactivation
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived) ||
            !pwTests->waitForSignal(pwTests->fpOffValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOffReceived)) {
            addMessage("Step 2: Window Operation and FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC23: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC23: Test Complete", "info");
}

void InteractionTests::runMSC24Test() {
    addMessage("MSC24: Auto Window Movement Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Window Up Movement (3x)
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        for (int i = 0; i < 3; i++) {
            if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
                addMessage("Step " + QString::number(1+i) + ": Initial Up Movement", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Window Stop Sequence
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 4: Window Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Window Up Movement (3x)
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        for (int i = 0; i < 3; i++) {
            if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
                addMessage("Step " + QString::number(5+i) + ": Final Up Movement", "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        if (testPassed) {
            addMessage("MSC24: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC24: Test Complete", "info");
}

void InteractionTests::runMSC25Test() {
    addMessage("MSC25: RCK with AS Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK Unlock and AS Deactivation
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 1: RCK Unlock and AS Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // RCK Lock and AS Activation
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 2: RCK Lock and AS Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC25: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC25: Test Complete", "info");
}

void InteractionTests::runMSC26Test() {
    addMessage("MSC26: RCK with Immobilizer Alarm Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK Lock and AS Activation
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 1: RCK Lock and AS Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Immobilizer Alarm Detection
        securityTests->resetSignals();
        securityTests->alarmIMASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmIMOnASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived)) {
            addMessage("Step 3: Immobilizer Alarm Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // RCK Unlock and System Deactivation
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->alarmIMOffASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmAlarmOffASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 4: System Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC26: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC26: Test Complete", "info");
}

void InteractionTests::runMSC27Test() {
    addMessage("MSC27: RCK and Manual AS Control Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK Unlock and Initial AS Off
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 1: RCK Unlock and AS Off", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Manual AS Deactivation
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmDeactivatedHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmDeactivatedHMIReceived)) {
            addMessage("Step 2: Manual AS Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Manual AS Activation
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmActivatedHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmActivatedHMIReceived)) {
            addMessage("Step 3: Manual AS Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // RCK Lock and AS On
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 4: RCK Lock and AS On", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC27: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC27: Test Complete", "info");
}

void InteractionTests::runMSC28Test() {
    addMessage("MSC28: Remote Window Control Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Unlock and Remote Down
        pwTests->resetSignals();
        securityTests->resetSignals();
        pwTests->clsUnlockPWPublisher->publish(msg);
        securityTests->windowDownRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 1: Initial Remote Down", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Remote Up and Stop
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->windowUpRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 2: Remote Up Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Remote Down and Local Stop
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->windowDownRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 3: Second Remote Down", "test_fail");
            testPassed = false;
        }
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 4: Local Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Lock and FP Intervention
        securityTests->resetSignals();
        pwTests->resetSignals();
        pwTests->clsLockPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
            addMessage("Step 5: Lock Auto Up", "test_fail");
            testPassed = false;
        }
        pwTests->fpOnPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 6: FP Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Remote Operations
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->windowUpRCKPublisher->publish(msg);
        securityTests->windowDownRCKPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC28: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC28: Test Complete", "info");
}

void InteractionTests::runMSC29Test() {
    addMessage("MSC29: Complex Remote Window Control Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Unlock and Remote Down
        pwTests->resetSignals();
        securityTests->resetSignals();
        pwTests->clsUnlockPWPublisher->publish(msg);
        securityTests->windowDownRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 1: Initial Remote Down", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Remote Up Stop
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->windowUpRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 2: Remote Up Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Second Remote Down and Local Stop
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->windowDownRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 3: Second Remote Down", "test_fail");
            testPassed = false;
        }
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 4: Local Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Lock and FP Intervention
        securityTests->resetSignals();
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->clsLockPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
            addMessage("Step 5: Lock Auto Up", "test_fail");
            testPassed = false;
        }
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 6: FP Detection and Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Remote Operations with FP
        securityTests->resetSignals();
        pwTests->resetSignals();
        pwTests->resetSignals();
        securityTests->windowUpRCKPublisher->publish(msg);
        securityTests->windowDownRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 7: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Remote Up
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->windowUpRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
            addMessage("Step 8: Final Remote Up", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC29: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC29: Test Complete", "info");
}

void InteractionTests::runMSC30Test() {
    addMessage("MSC30: Complex Remote Control Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK and CLS Unlock Sequence
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: RCK and CLS Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Remote Down Operation
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->windowDownRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived)) {
            addMessage("Step 2: Remote Down", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Local Stop and Up
        pwTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 3: Local Stop", "test_fail");
            testPassed = false;
        }
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
            addMessage("Step 4: Local Up", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // RCK Lock Sequence
        securityTests->resetSignals();
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralLockCLSReceived) ||
            !pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
            addMessage("Step 5: RCK Lock and Auto Up", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // FP Detection and Stop
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived)) {
            addMessage("Step 6: FP Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Remote Operations
        securityTests->resetSignals();
        pwTests->resetSignals();
        pwTests->resetSignals();
        securityTests->windowDownRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 7: FP Deactivation", "test_fail");
            testPassed = false;
        }
        securityTests->windowUpRCKPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived)) {
            addMessage("Step 8: Final Up Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC30: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC30: Test Complete", "info");
}

void InteractionTests::runMSC31Test() {
    addMessage("MSC31: RCK Safety Timer Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK and CLS Unlock Sequence
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: RCK and CLS Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Safety Timer and Auto Lock
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->timerRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 3: Safety Timer Auto Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC31: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC31: Test Complete", "info");
}

void InteractionTests::runMSC32Test() {
    addMessage("MSC32: RCK Safety Timer with AS Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK Unlock and AS Deactivation
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 1: RCK Unlock and AS Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Safety Timer and Auto Lock
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->timerRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 3: Safety Timer Auto Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC32: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC32: Test Complete", "info");
}

void InteractionTests::runMSC33Test() {
    addMessage("MSC33: RCK Safety Timer with CLS and AS Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK Unlock with CLS and AS
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralUnlockCLSReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 1: System Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Safety Timer and Auto Lock
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->timerRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralLockCLSReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 3: Safety Timer Auto Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC33: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC33: Test Complete", "info");
}

void InteractionTests::runMSC34Test() {
    addMessage("MSC34: RCK Safety Timer with Door Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial RCK Unlock
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: Initial Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Door Open Event
        securityTests->resetSignals();
        securityTests->doorOpenRCKPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Manual RCK Lock
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 5: Manual Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Manual RCK Unlock
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 6: Manual Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Safety Timer Auto Lock
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->timerRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 8: Safety Timer Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC34: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC34: Test Complete", "info");
}

void InteractionTests::runMSC35Test() {
    addMessage("MSC35: Complex Exterior Mirror Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Left Movement and LED Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorLeftHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveLeftSignalReceived)) {
            addMessage("Step 1: Mirror Left Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorRightLEDOffReceived)) {
            addMessage("Step 2: LED Pending State", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionLeftEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftLEDOnReceived)) {
            addMessage("Step 3: Left Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Up Movement and LED Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorUpHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveUpSignalReceived)) {
            addMessage("Step 4: Mirror Up Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorDownLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorUpLEDOffReceived)) {
            addMessage("Step 5: Vertical LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionUpEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorUpLEDOnReceived)) {
            addMessage("Step 6: Top Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Heating Activation
        emTests->resetSignals();
        hmiTests->resetSignals();
        emTests->mirrorColdEMPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->heatingOnSignalReceived) ||
            !hmiTests->waitForSignal(hmiTests->heatingOnLEDReceived)) {
            addMessage("Step 7: Heating Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Right Movement and LED Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorRightHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveRightSignalReceived)) {
            addMessage("Step 8: Mirror Right Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorRightLEDOffReceived)) {
            addMessage("Step 9: Horizontal LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionRightEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorRightLEDOnReceived)) {
            addMessage("Step 10: Right Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement and LED Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorDownHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveDownSignalReceived)) {
            addMessage("Step 11: Mirror Down Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorUpLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorDownLEDOffReceived)) {
            addMessage("Step 12: Vertical LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionDownEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorDownLEDOnReceived)) {
            addMessage("Step 13: Bottom Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Heating Deactivation
        emTests->resetSignals();
        hmiTests->resetSignals();
        emTests->timerEMPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->heatingOffSignalReceived) ||
            !hmiTests->waitForSignal(hmiTests->heatingOffLEDReceived)) {
            addMessage("Step 14: Heating Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC35: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC35: Test Complete", "info");
}

void InteractionTests::runMSC36Test() {
    addMessage("MSC36: Manual Window Control Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Up Movement with LED
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 1: Initial Up Movement and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Second Up Movement and FP Detection
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue)) {
            addMessage("Step 2: FP Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Button Release Sequence
        hmiTests->resetSignals();
        hmiTests->releasePWUpHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowUpLEDOffManPWReceived)) {
            addMessage("Step 3: Button Release LED Off", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement with FP Deactivation
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !pwTests->waitForSignal(pwTests->windowDownPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 4: Down Movement and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Button Release
        hmiTests->resetSignals();
        hmiTests->releaseLEDManPWPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowDownLEDOffManPWReceived)) {
            addMessage("Step 5: Down Button Release", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Up Movement
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 6: Final Up Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC36: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC36: Test Complete", "info");
}

void InteractionTests::runMSC37Test() {
    addMessage("MSC37: AS LED Indicator Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // AS Activation and Lock
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmActiveOnPublisher->publish(msg);
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmActiveOnASReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmActivatedHMIReceived)) {
            addMessage("Step 1: AS Activation and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Alarm Detection
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->alarmDetectedASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmOnReceived)) {
            addMessage("Step 3: Alarm Detection and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Timer and Alarm Status
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->alarmTimerASPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmDetectedOnReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmOffReceived)) {
            addMessage("Step 4: Timer and LED Updates", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // AS Deactivation
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmActiveOffASReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmActiveOffReceived)) {
            addMessage("Step 5: AS Deactivation and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Alarm Confirmation
        hmiTests->resetSignals();
        hmiTests->confirmAlarmHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->alarmDetectedOffReceived)) {
            addMessage("Step 6: Alarm Confirmation and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC37: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC37: Test Complete", "info");
}

void InteractionTests::runMSC38Test() {
    addMessage("MSC38: CLS with LED Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Lock Sequence with LED
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived) ||
            !hmiTests->waitForSignal(hmiTests->clsLockReceived)) {
            addMessage("Step 1: Lock and LED On", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Unlock Sequence with LED
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived) ||
            !hmiTests->waitForSignal(hmiTests->clsUnlockReceived)) {
            addMessage("Step 2: Unlock and LED Off", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC38: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC38: Test Complete", "info");
}

void InteractionTests::runMSC39Test() {
    addMessage("MSC39: CLS with Manual PW Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Lock Sequence with LED
        securityTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived) ||
            !hmiTests->waitForSignal(hmiTests->clsLockReceived)) {
            addMessage("Step 1: Lock and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Block Test
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 2: Window Block", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Unlock Sequence with LED
        securityTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived) ||
            !hmiTests->waitForSignal(hmiTests->clsUnlockReceived)) {
            addMessage("Step 3: Unlock and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Movement Test
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 4: Window Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC39: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC39: Test Complete", "info");
}

void InteractionTests::runMSC40Test() {
    addMessage("MSC40: CLS with Manual PW Basic Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Lock Sequence
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 1: CLS Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Block Test
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 2: Window Block", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Unlock Sequence
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 3: CLS Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Movement Test
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 4: Window Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC40: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC40: Test Complete", "info");
}

void InteractionTests::runMSC41Test() {
    addMessage("MSC41: Complex Manual PW Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Unlock and Window Down
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: Initial Unlock", "test_fail");
            testPassed = false;
        }
        for (int i = 0; i < 2; i++) {
            pwTests->windowDownPWPublisher->publish(msg);
            if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
                addMessage("Step 2: Window Down " + QString::number(i+1), "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Position Check and Lock
        pwTests->resetSignals();
        securityTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 3: Lock Sequence", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Up and FP Detection
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
            addMessage("Step 4: Window Up", "test_fail");
            testPassed = false;
        }
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue)) {
            addMessage("Step 5: FP Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // FP Deactivation
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 6: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Unlock and Movement
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 7: Final Unlock", "test_fail");
            testPassed = false;
        }
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 8: Final Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC41: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC41: Test Complete", "info");
}

void InteractionTests::runMSC42Test() {
    addMessage("MSC42: CLS with HMI Window Control Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Lock Sequence
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 1: CLS Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Block Test
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 2: Window Block", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Unlock Sequence
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 3: CLS Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Movement Test
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 4: Window Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC42: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC42: Test Complete", "info");
}

void InteractionTests::runMSC43Test() {
    addMessage("MSC43: Complex HMI Window Control Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Unlock and Window Down
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: Initial Unlock", "test_fail");
            testPassed = false;
        }
        for (int i = 0; i < 2; i++) {
            hmiTests->windowDownHMIPublisher->publish(msg);
            if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
                addMessage("Step 2: Window Down " + QString::number(i+1), "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Position Check and Lock
        pwTests->resetSignals();
        securityTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 3: Lock Sequence", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Up and FP Detection
        hmiTests->resetSignals();
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
            addMessage("Step 4: Window Up", "test_fail");
            testPassed = false;
        }
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue)) {
            addMessage("Step 5: FP Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // FP Deactivation
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 6: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Unlock and Movement
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 7: Final Unlock", "test_fail");
            testPassed = false;
        }
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 8: Final Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC43: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC43: Test Complete", "info");
}

void InteractionTests::runMSC44Test() {
    addMessage("MSC44: RCK with Manual PW Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK Unlock and Initial Window Down
        securityTests->resetSignals();
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: RCK Unlock", "test_fail");
            testPassed = false;
        }
        for (int i = 0; i < 2; i++) {
            pwTests->windowDownPWPublisher->publish(msg);
            if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
                addMessage("Step 2: Window Down " + QString::number(i+1), "test_fail");
                testPassed = false;
            }
            wait(1000);
            QCoreApplication::processEvents();
        }

        // Position Check and RCK Lock
        pwTests->resetSignals();
        securityTests->resetSignals();
        securityTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived)) {
            addMessage("Step 3: RCK Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Up and FP Detection
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpPWReceived)) {
            addMessage("Step 4: Window Up", "test_fail");
            testPassed = false;
        }
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue)) {
            addMessage("Step 5: FP Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // FP Deactivation
        pwTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue)) {
            addMessage("Step 6: FP Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Key Unlock and Final Movement
        securityTests->resetSignals();
        pwTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 7: Key Unlock", "test_fail");
            testPassed = false;
        }
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 8: Final Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC44: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC44: Test Complete", "info");
}

void InteractionTests::runMSC45Test() {
    addMessage("MSC45: RCK with CLS LED Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Lock Sequence with LED
        securityTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->remoteLockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteLockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralLockCLSReceived) ||
            !hmiTests->waitForSignal(hmiTests->clsLockReceived)) {
            addMessage("Step 1: Lock and LED On", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Unlock Sequence with LED
        securityTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->remoteUnlockRCKReceived) ||
            !securityTests->waitForSignal(securityTests->centralUnlockCLSReceived) ||
            !hmiTests->waitForSignal(hmiTests->clsUnlockReceived)) {
            addMessage("Step 2: Unlock and LED Off", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC45: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC45: Test Complete", "info");
}

void InteractionTests::runMSC46Test() {
    addMessage("MSC46: Manual PW with LED Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Up Movement with LED
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 1: Initial Up Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Second Up Movement and FP
        pwTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        pwTests->fpOnPWPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        // Button Release
        hmiTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        hmiTests->releasePWUpHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowUpLEDOffManPWReceived)) {
            addMessage("Step 3: Up Button Release", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement with LED
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        pwTests->fpOffPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 4: Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Button Release
        hmiTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        hmiTests->releasePWDownHMIPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->windowDownLEDOffManPWReceived)) {
            addMessage("Step 5: Down Button Release", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC46: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC46: Test Complete", "info");
}

void InteractionTests::runMSC47Test() {
    addMessage("MSC47: Complex Exterior Mirror Movement Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Left Movement and LED Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorLeftHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveLeftSignalReceived)) {
            addMessage("Step 1: Left Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorRightLEDOffReceived)) {
            addMessage("Step 2: Left LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionLeftEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftLEDOnReceived)) {
            addMessage("Step 3: Left Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Up Movement and LED Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorUpHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveUpSignalReceived)) {
            addMessage("Step 4: Up Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorDownLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorUpLEDOffReceived)) {
            addMessage("Step 5: Up LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionUpEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorUpLEDOnReceived)) {
            addMessage("Step 6: Up Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Right Movement and LED Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorRightHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveRightSignalReceived)) {
            addMessage("Step 7: Right Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorRightLEDOffReceived)) {
            addMessage("Step 8: Right LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionRightEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorRightLEDOnReceived)) {
            addMessage("Step 9: Right Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement and LED Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorDownHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveDownSignalReceived)) {
            addMessage("Step 10: Down Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorUpLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorDownLEDOffReceived)) {
            addMessage("Step 11: Down LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionDownEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorDownLEDOnReceived)) {
            addMessage("Step 12: Down Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC47: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC47: Test Complete", "info");
}

void InteractionTests::runMSC48Test() {
    addMessage("MSC48: FP with Window Movement Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // FP Detection and Activation
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue)) {
            addMessage("Step 1: FP Detection", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // FP Deactivation and Window Movement
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !pwTests->waitForSignal(pwTests->windowDownPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 2: Movement with LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC48: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC48: Test Complete", "info");
}

void InteractionTests::runMSC49Test() {
    addMessage("MSC49: FP with LED Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // FP Detection and LED Activation
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOnReceived)) {
            addMessage("Step 1: FP Detection and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Movement with LED
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 2: Window Movement and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // FP Deactivation and LED Off
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->fpOffPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOffReceived)) {
            addMessage("Step 3: FP Deactivation and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC49: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC49: Test Complete", "info");
}

void InteractionTests::runMSC50Test() {
    addMessage("MSC50: FP with HMI Control Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // FP Detection and LED Activation
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOnReceived)) {
            addMessage("Step 1: FP Detection and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // HMI Window Movement with LED
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 2: Window Movement and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // FP Deactivation and LED Off
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->fpOffPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOffReceived)) {
            addMessage("Step 3: FP Deactivation and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC50: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC50: Test Complete", "info");
}

void InteractionTests::runMSC51Test() {
    addMessage("MSC51: AS with Immobilizer Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // AS Activation and Lock
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmActiveOnPublisher->publish(msg);
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 1: AS Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Immobilizer Alarm Detection
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->alarmIMASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmOnReceived) ||
            !securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived)) {
            addMessage("Step 3: Immobilizer Alarm", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Unlock and Deactivation
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOffASReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmOffReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 4: System Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC51: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC51: Test Complete", "info");
}

void InteractionTests::runMSC52Test() {
    addMessage("MSC52: AS with LED Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // AS Activation with LED
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmActiveOnPublisher->publish(msg);
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmActiveOnASReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmActivatedHMIReceived)) {
            addMessage("Step 1: AS Activation and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Immobilizer Alarm
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->alarmIMASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmOnReceived) ||
            !securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived)) {
            addMessage("Step 3: Immobilizer Alarm", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // System Deactivation
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOffASReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmOffReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived) ||
            !hmiTests->waitForSignal(hmiTests->alarmActiveOffReceived)) {
            addMessage("Step 4: System Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC52: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC52: Test Complete", "info");
}

void InteractionTests::runMSC53Test() {
    addMessage("MSC53: FP Basic Sequence Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // FP Detection and LED
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOnReceived)) {
            addMessage("Step 1: FP Detection and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Button Press and FP Deactivation
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOffReceived) ||
            !pwTests->waitForSignal(pwTests->windowDownPWReceived)) {
            addMessage("Step 2: FP Deactivation and Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC53: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC53: Test Complete", "info");
}

void InteractionTests::runMSC54Test() {
    addMessage("MSC54: Auto Power Window Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Up Movement and LED
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 1: Initial Up Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Position Up and Stop
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowPositionUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOffManPWReceived)) {
            addMessage("Step 2: Up Position Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement and LED
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 3: Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Position Down and Stop
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOffManPWReceived)) {
            addMessage("Step 4: Down Position Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC54: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC54: Test Complete", "info");
}

void InteractionTests::runMSC55Test() {
    addMessage("MSC55: HMI Auto Power Window Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Up Movement
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 1: Initial Up Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Position Up and Stop
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowPositionUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOffManPWReceived)) {
            addMessage("Step 2: Up Position Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowDownAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 3: Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Position Down and Stop
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOffManPWReceived)) {
            addMessage("Step 4: Down Position Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Up Movement
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 5: Final Up Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC55: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC55: Test Complete", "info");
}

void InteractionTests::runMSC56Test() {
    addMessage("MSC56: Auto PW with FP Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Up Movement and FP
        hmiTests->resetSignals();
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 1: Initial Up", "test_fail");
            testPassed = false;
        }
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOffManPWReceived)) {
            addMessage("Step 2: FP Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement and FP Off
        hmiTests->resetSignals();
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !pwTests->waitForSignal(pwTests->windowDownAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 3: Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Position Down and Stop
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOffManPWReceived)) {
            addMessage("Step 4: Down Position", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Up Movement
        hmiTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->windowUpHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 5: Final Up", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC56: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC56: Test Complete", "info");
}

void InteractionTests::runMSC57Test() {
    addMessage("MSC57: Auto PW with Direct FP Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Initial Up Movement
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 1: Initial Up Movement", "test_fail");
            testPassed = false;
        }

        // FP Detection and Stop
        pwTests->resetSignals();
        pwTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !pwTests->waitForSignal(pwTests->windowStopAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOffManPWReceived)) {
            addMessage("Step 2: FP Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement with FP Off
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !pwTests->waitForSignal(pwTests->windowDownAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 3: Down Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Position Down and Stop
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowPositionDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOffManPWReceived)) {
            addMessage("Step 4: Down Position Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Final Up Movement
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowUpAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived)) {
            addMessage("Step 5: Final Up Movement", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC57: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC57: Test Complete", "info");
}

void InteractionTests::runMSC58Test() {
    addMessage("MSC58: FP with Auto PW Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // FP Detection and LED
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOnReceived)) {
            addMessage("Step 1: FP Detection and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Down with FP Deactivation
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowDownPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOffReceived) ||
            !pwTests->waitForSignal(pwTests->windowDownAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 2: Window Movement and LED States", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC58: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC58: Test Complete", "info");
}

void InteractionTests::runMSC59Test() {
    addMessage("MSC59: FP with HMI Auto PW Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // FP Detection and LED
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->fingerDetectedPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOnValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOnReceived)) {
            addMessage("Step 1: FP Detection and LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // HMI Window Down with FP Deactivation
        pwTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        hmiTests->windowDownHMIPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->fpOffValue) ||
            !hmiTests->waitForSignal(hmiTests->fpOffReceived) ||
            !pwTests->waitForSignal(pwTests->windowDownAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowDownLEDOnManPWReceived)) {
            addMessage("Step 2: Window Movement and LED States", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC59: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC59: Test Complete", "info");
}

void InteractionTests::runMSC60Test() {
    addMessage("MSC60: RCK with AS Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // RCK Unlock Sequence
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->remoteUnlockRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 1: RCK Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Safety Timer Lock
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->resetSignals();
        securityTests->timerRCKPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 3: Safety Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Immobilizer Alarm
        securityTests->resetSignals();
        securityTests->alarmIMASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived)) {
            addMessage("Step 5: Immobilizer Alarm", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC60: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC60: Test Complete", "info");
}

void InteractionTests::runMSC61Test() {
    addMessage("MSC61: CLS with Auto PW Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // CLS Unlock/Lock Sequence
        securityTests->resetSignals();
        pwTests->resetSignals();
        hmiTests->resetSignals();
        
        // Unlock
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralUnlockCLSReceived)) {
            addMessage("Step 1: CLS Unlock", "test_fail");
            testPassed = false;
        }
        
        // Lock and Window Up
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->centralLockCLSReceived) ||
            !pwTests->waitForSignal(pwTests->windowUpAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOnManPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->clsLockReceived)) {
            addMessage("Step 2: Lock and Window Up", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Window Position Up and Stop
        pwTests->resetSignals();
        hmiTests->resetSignals();
        pwTests->windowPositionUpPWPublisher->publish(msg);
        if (!pwTests->waitForSignal(pwTests->windowStopAutoPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->windowUpLEDOffManPWReceived) ||
            !hmiTests->waitForSignal(hmiTests->clsUnlockReceived)) {
            addMessage("Step 3: Window Stop", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC61: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC61: Test Complete", "info");
}

void InteractionTests::runMSC62Test() {
    addMessage("MSC62: Exterior Mirror Movement Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Left Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorLeftHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveLeftSignalReceived)) {
            addMessage("Step 1: Left Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorRightLEDOffReceived)) {
            addMessage("Step 2: Left LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionLeftEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftLEDOnReceived)) {
            addMessage("Step 3: Left Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Up Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorUpHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveUpSignalReceived)) {
            addMessage("Step 4: Up Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorDownLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorUpLEDOffReceived)) {
            addMessage("Step 5: Up LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionUpEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorUpLEDOnReceived)) {
            addMessage("Step 6: Top Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Right Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorRightHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveRightSignalReceived)) {
            addMessage("Step 7: Right Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorLeftLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorRightLEDOffReceived)) {
            addMessage("Step 8: Right LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionRightEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorRightLEDOnReceived)) {
            addMessage("Step 9: Right Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Down Movement Sequence
        hmiTests->resetSignals();
        emTests->resetSignals();
        hmiTests->mirrorDownHMIPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->mirrorMoveDownSignalReceived)) {
            addMessage("Step 10: Down Movement", "test_fail");
            testPassed = false;
        }
        if (!hmiTests->waitForSignal(hmiTests->mirrorUpLEDOffReceived) ||
            !hmiTests->waitForSignal(hmiTests->mirrorDownLEDOffReceived)) {
            addMessage("Step 11: Down LED Pending", "test_fail");
            testPassed = false;
        }
        emTests->mirrorPositionDownEMPublisher->publish(msg);
        if (!hmiTests->waitForSignal(hmiTests->mirrorDownLEDOnReceived)) {
            addMessage("Step 12: Bottom Position LED", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC62: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC62: Test Complete", "info");
}

void InteractionTests::runMSC63Test() {
    addMessage("MSC63: Exterior Mirror Heating Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Heating Activation
        emTests->resetSignals();
        hmiTests->resetSignals();
        emTests->mirrorColdEMPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->heatingOnSignalReceived) ||
            !hmiTests->waitForSignal(hmiTests->heatingOnLEDReceived)) {
            addMessage("Step 1: Heating Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Heating Deactivation
        emTests->resetSignals();
        hmiTests->resetSignals();
        emTests->timerEMPublisher->publish(msg);
        if (!emTests->waitForSignal(emTests->heatingOffSignalReceived) ||
            !hmiTests->waitForSignal(hmiTests->heatingOffLEDReceived)) {
            addMessage("Step 3: Heating Deactivation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC63: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC63: Test Complete", "info");
}

void InteractionTests::runMSC64Test() {
    addMessage("MSC64: Basic Alarm System Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // AS Activation and Lock
        hmiTests->resetSignals();
        securityTests->resetSignals();
        hmiTests->alarmActiveOnPublisher->publish(msg);
        securityTests->centralLockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmActiveOnASReceived)) {
            addMessage("Step 1: AS Activation", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Alarm Detection and Timer
        securityTests->resetSignals();
        securityTests->alarmDetectedASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmOnASReceived)) {
            addMessage("Step 3: Alarm Detection", "test_fail");
            testPassed = false;
        }
        securityTests->alarmTimerASPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmAlarmDetectedASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmAlarmOffASReceived) ||
            !securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 4: Alarm Timer", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // System Deactivation
        securityTests->resetSignals();
        hmiTests->resetSignals();
        securityTests->centralUnlockCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->alarmActiveOffASReceived)) {
            addMessage("Step 5: AS Unlock", "test_fail");
            testPassed = false;
        }
        hmiTests->alarmDeactivatedHMIPublisher->publish(msg);
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC64: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC64: Test Complete", "info");
}

void InteractionTests::runMSC65Test() {
    addMessage("MSC65: Car Drive Lock Test", "Started");
    std_msgs::msg::Bool msg;
    msg.data = true;
    bool testPassed = true;
    
    try {
        // Car Drive Lock
        securityTests->resetSignals();
        securityTests->carDrivesCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->carLockCLSReceived)) {
            addMessage("Step 1: Car Drive Lock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        // Door Open Unlock
        securityTests->resetSignals();
        securityTests->doorOpenCLSPublisher->publish(msg);
        if (!securityTests->waitForSignal(securityTests->carUnlockCLSReceived)) {
            addMessage("Step 2: Door Open Unlock", "test_fail");
            testPassed = false;
        }
        wait(1000);
        QCoreApplication::processEvents();

        if (testPassed) {
            addMessage("MSC65: All Steps", "test_pass");
        }

    } catch (const std::exception& e) {
        addMessage("Test failed: " + QString(e.what()), "error");
    }
    addMessage("MSC65: Test Complete", "info");
}
