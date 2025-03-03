#ifndef TESTMANAGER_HPP
#define TESTMANAGER_HPP
#pragma once
#include <memory>
#include "ConfigParser.hpp"
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "PowerWindowTests.hpp"
#include "ExteriorMirrorTests.hpp"
#include "HMITests.hpp"
// #include "InteractionTests.hpp"
#include "SecurityTests.hpp"

class TestManager {
public:
    TestManager();
    ~TestManager() = default;

    bool loadConfiguration(const std::string& configPath);
    void startConfiguredTests();
    void stopTests();
    bool isRunning() const { return m_isTestRunning; }
    std::shared_ptr<rclcpp::Node> getNode() { return m_node; }

private:
    std::set<std::string> m_activeFeatures;
    std::shared_ptr<rclcpp::Node> m_node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    ConfigParser m_configParser;
    
    std::unique_ptr<PowerWindowTests> m_powerwindowtest;
    std::unique_ptr<ExteriorMirrorTests> m_emTests;
    std::unique_ptr<HMITests> m_hmiTests;
    std::unique_ptr<SecurityTests> m_securityTests;
    // std::unique_ptr<InteractionTests> m_interactionTests;
    
    std::atomic<bool> m_isTestRunning{false};
    
    void logMessage(const std::string& message, const std::string& type);
};

#endif