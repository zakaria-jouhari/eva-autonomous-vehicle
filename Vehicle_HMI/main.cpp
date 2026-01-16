// main.cpp
// Add ROS2 initialization

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <rclcpp/rclcpp.hpp>
#include "RouteService.h"

int main(int argc, char *argv[])
{
    // IMPORTANT: Initialize ROS2 BEFORE Qt
    rclcpp::init(argc, argv);
    
    QGuiApplication app(argc, argv);
    
    QQmlApplicationEngine engine;
    
    // Create RouteService instance (now with ROS2)
    RouteService routeService;
    
    // Expose to QML
    engine.rootContext()->setContextProperty("routeService", &routeService);
    
    // Load main QML
    const QUrl url(QStringLiteral("qrc:/Main.qml"));
    
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    
    engine.load(url);
    
    if (engine.rootObjects().isEmpty()) {
        rclcpp::shutdown();
        return -1;
    }
    
    // Run Qt event loop
    int result = app.exec();
    
    // Cleanup ROS2
    rclcpp::shutdown();
    
    return result;
}
