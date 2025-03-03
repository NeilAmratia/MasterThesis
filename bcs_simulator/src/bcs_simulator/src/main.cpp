#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QMetaType>
#include "ROS2Bridge.hpp"

int main(int argc, char *argv[]) {
    qRegisterMetaType<std_msgs::msg::Bool::SharedPtr>("std_msgs::msg::Bool::SharedPtr");
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    // Register the ROS2 bridge to QML
    qmlRegisterType<ROS2Bridge>("ROS2Bridge", 1, 0, "ROS2Bridge");
    qmlRegisterType<MessageModel>("MessageModels", 1, 0, "MessageModel");

    // Create ROS2Bridge with parent for proper memory management
    auto ros2Bridge = new ROS2Bridge(&app);

    // Set the context property so QML can access the message model
    engine.rootContext()->setContextProperty("ros2Bridge", ros2Bridge);

    // Load QML
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    return app.exec();
}