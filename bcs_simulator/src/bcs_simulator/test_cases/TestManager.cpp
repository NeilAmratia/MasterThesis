#include <filesystem>
#include <iostream>
#include "TestManager.hpp"

TestManager::TestManager() {
    m_node = std::make_shared<rclcpp::Node>("test_manager");
    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_executor->add_node(m_node);
}

bool TestManager::loadConfiguration(const std::string& configPath) {
    if (!std::filesystem::exists(configPath)) {
        std::cerr << "Error: Configuration file not found at: " << configPath << std::endl;
        return false;
    }

    try {
        m_activeFeatures = m_configParser.parseActiveFeatures(configPath);
        
        std::cout << "Configuration loaded successfully" << std::endl;
        std::cout << "Active features:" << std::endl;
        
        for (const auto& feature : m_activeFeatures) {
            std::cout << "  • " << feature << std::endl;
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading configuration: " << e.what() << std::endl;
        return false;
    }
}

void TestManager::startConfiguredTests() {
    if (m_isTestRunning) {
        return;
    }

    std::cout << "Starting configured tests..." << std::endl;
    m_isTestRunning = true;

    try {
        if (!m_powerwindowtest) {
            m_powerwindowtest = std::make_unique<PowerWindowTests>(m_node);
        }

        if (!m_emTests) {
            m_emTests = std::make_unique<ExteriorMirrorTests>(m_node);
        }

        if (!m_hmiTests) {
            m_hmiTests = std::make_unique<HMITests>(m_node);
        }

        if (!m_securityTests) {
            m_securityTests = std::make_unique<SecurityTests>(m_node);
        }

        // if (!m_interactionTests) {
        //     m_interactionTests = std::make_unique<InteractionTests>(m_node);
        // }
        
        // Start spinning node in separate thread
        std::thread executor_thread([this]() {
            while (rclcpp::ok() && m_isTestRunning) {
                m_executor->spin_some();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });


        if (m_activeFeatures.find("Manual_PW") != m_activeFeatures.end() && m_powerwindowtest) {
            std::cout << "Running Manual Power Window Tests..." << std::endl;
            m_powerwindowtest->runManualPWTest();
            if (m_activeFeatures.find("CLS") != m_activeFeatures.end()) {
                m_powerwindowtest->runManualPWWithCLSTest();
            }
        }

        if (m_activeFeatures.find("Automatic_PW") != m_activeFeatures.end()) {
            m_powerwindowtest->runAutomaticPWTest();
            if (m_activeFeatures.find("CLS") != m_activeFeatures.end()) {
                m_powerwindowtest->runAutomaticPWWithCLSTest();
            }
        }

        if (m_activeFeatures.find("FP") != m_activeFeatures.end()) {
            m_powerwindowtest->runFPTest();
        }

        if (m_activeFeatures.find("EM") != m_activeFeatures.end()) {
            m_emTests->runExteriorMirrorTest();
            if (m_activeFeatures.find("Heatable") != m_activeFeatures.end()) {m_emTests->runExteriorMirrorWithHeatableTest();}
            if (m_activeFeatures.find("LED_EM") != m_activeFeatures.end()) {m_emTests->runExteriorMirrorWithLedTest();}
        }

        if (m_activeFeatures.find("HMI") != m_activeFeatures.end()) {
            m_hmiTests->runHMITest();
            if (m_activeFeatures.find("AS") != m_activeFeatures.end()) {m_hmiTests->runHMIWithASTest();}
            if (m_activeFeatures.find("LED_AS") != m_activeFeatures.end()) {m_hmiTests->runHMIWithASLEDTest();}
            if (m_activeFeatures.find("Manual_PW") != m_activeFeatures.end()) {
                if (m_activeFeatures.find("LED_PW") != m_activeFeatures.end()) {m_hmiTests->runHMIWithManPWLEDTest();}
            }
        }

        if (m_activeFeatures.find("LED_PW") != m_activeFeatures.end()) {
            if (m_activeFeatures.find("Manual_PW") != m_activeFeatures.end()) {m_hmiTests->runLEDManPWTest();}
            if (m_activeFeatures.find("Automatic_PW") != m_activeFeatures.end()) {
                m_hmiTests->runLEDAutoPWTest();
                if (m_activeFeatures.find("CLS") != m_activeFeatures.end()) {m_hmiTests->runLEDAutoPWWithCLSTest();}
            }
        }

        if (m_activeFeatures.find("LED_EM") != m_activeFeatures.end()) {
            m_hmiTests->runLEDEMTTest();
            m_hmiTests->runLEDEMLTest();
            m_hmiTests->runLEDEMBTest();
            m_hmiTests->runLEDEMRTest();
        }

        if (m_activeFeatures.find("LED_Heatable") != m_activeFeatures.end()) {
            m_hmiTests->runLEDEMHTest();
        }

        if (m_activeFeatures.find("LED_FP") != m_activeFeatures.end()) {
            m_hmiTests->runLEDFPTest();
        }

        if (m_activeFeatures.find("LED_AS") != m_activeFeatures.end()) {
            m_hmiTests->runLEDASACTest();
            m_hmiTests->runLEDASALTest();
            m_hmiTests->runLEDASADTest();
            if (m_activeFeatures.find("Interior_Monitoring") != m_activeFeatures.end()) {m_hmiTests->runLEDASIMTest();}
        }

        if (m_activeFeatures.find("LED_CLS") != m_activeFeatures.end()) {
            m_hmiTests->runLEDCLSTest();
        }

        if (m_activeFeatures.find("RCK") != m_activeFeatures.end()) {
            m_securityTests->runRCKTest();
            if (m_activeFeatures.find("Safety_Function") != m_activeFeatures.end()) {m_securityTests->runRCKWithSFTest();}
            if (m_activeFeatures.find("Control_Automatic_PW") != m_activeFeatures.end()) {m_securityTests->runRCKWithControlAutoPWTest();}
            if (m_activeFeatures.find("Safety_Function") != m_activeFeatures.end() && m_activeFeatures.find("Control_Automatic_PW") != m_activeFeatures.end()) {m_securityTests->runRCKWithSFandControlAutoPWTest();}
        }

        if (m_activeFeatures.find("CLS") != m_activeFeatures.end()) {
            m_securityTests->runCLSTest();
            if (m_activeFeatures.find("RCK") != m_activeFeatures.end()) {m_securityTests->runCLSWithRCKTest();}
            if (m_activeFeatures.find("Automatic_Locking") != m_activeFeatures.end()) {m_securityTests->runCLSWithAutoLockingTest();}
        }

        if (m_activeFeatures.find("AS") != m_activeFeatures.end()) {
            m_securityTests->runASTest();
            if (m_activeFeatures.find("Control_AS") != m_activeFeatures.end()) {m_securityTests->runASWithControlASTest();}
            if (m_activeFeatures.find("Interior_Monitoring") != m_activeFeatures.end()) {m_securityTests->runASWithIMTest();}
        }

        // Cleanup
        m_isTestRunning = false;
        executor_thread.join();

        std::cout << "All configured tests completed" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Test execution failed: " << e.what() << std::endl;
        m_isTestRunning = false;
    }

}

void TestManager::stopTests() {
    if (!m_isTestRunning) return;
    
    std::cout << "Tests stopped by user" << std::endl;
    m_isTestRunning = false;
}
