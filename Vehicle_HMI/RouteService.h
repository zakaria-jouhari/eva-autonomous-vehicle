// RouteService.h
// Version améliorée avec vitesse, distance restante, ETA

#ifndef ROUTESERVICE_H
#define ROUTESERVICE_H

#include <QObject>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <cmath>

class RouteService : public QObject
{
    Q_OBJECT
    
    // Propriétés existantes
    Q_PROPERTY(QVariantList globalPath READ globalPath NOTIFY globalPathChanged)
    Q_PROPERTY(QVariantList localPath READ localPath NOTIFY localPathChanged)
    Q_PROPERTY(QString routeStatus READ routeStatus NOTIFY routeStatusChanged)
    Q_PROPERTY(double routeDistance READ routeDistance NOTIFY routeDistanceChanged)
    Q_PROPERTY(double routeDuration READ routeDuration NOTIFY routeDurationChanged)
    Q_PROPERTY(int waypointCount READ waypointCount NOTIFY waypointCountChanged)
    Q_PROPERTY(bool isNavigating READ isNavigating NOTIFY isNavigatingChanged)
    
    // NOUVELLES PROPRIÉTÉS
    Q_PROPERTY(double currentSpeed READ currentSpeed NOTIFY currentSpeedChanged)
    Q_PROPERTY(double remainingDistance READ remainingDistance NOTIFY remainingDistanceChanged)
    Q_PROPERTY(double remainingTime READ remainingTime NOTIFY remainingTimeChanged)
    Q_PROPERTY(int progressPercent READ progressPercent NOTIFY progressPercentChanged)
    Q_PROPERTY(QString nextInstruction READ nextInstruction NOTIFY nextInstructionChanged)
    Q_PROPERTY(double distanceToNextTurn READ distanceToNextTurn NOTIFY distanceToNextTurnChanged)
    Q_PROPERTY(double currentX READ currentX NOTIFY currentPositionChanged)
    Q_PROPERTY(double currentY READ currentY NOTIFY currentPositionChanged)
    
public:
    explicit RouteService(QObject *parent = nullptr);
    ~RouteService();
    
    // Getters existants
    QVariantList globalPath() const { return m_globalPath; }
    QVariantList localPath() const { return m_localPath; }
    QString routeStatus() const { return m_routeStatus; }
    double routeDistance() const { return m_routeDistance; }
    double routeDuration() const { return m_routeDuration; }
    int waypointCount() const { return m_waypointCount; }
    bool isNavigating() const { return m_isNavigating; }
    
    // NOUVEAUX GETTERS
    double currentSpeed() const { return m_currentSpeed; }
    double remainingDistance() const { return m_remainingDistance; }
    double remainingTime() const { return m_remainingTime; }
    int progressPercent() const { return m_progressPercent; }
    QString nextInstruction() const { return m_nextInstruction; }
    double distanceToNextTurn() const { return m_distanceToNextTurn; }
    double currentX() const { return m_currentX; }
    double currentY() const { return m_currentY; }
    
public slots:
    void navigateTo(double x, double y);
    void cancelNavigation();
    void clearRoute();
    
signals:
    // Signaux existants
    void globalPathChanged();
    void localPathChanged();
    void routeStatusChanged();
    void routeDistanceChanged();
    void routeDurationChanged();
    void waypointCountChanged();
    void isNavigatingChanged();
    void routeReady();
    void navigationStarted();
    void navigationCompleted();
    
    // NOUVEAUX SIGNAUX
    void currentSpeedChanged();
    void remainingDistanceChanged();
    void remainingTimeChanged();
    void progressPercentChanged();
    void nextInstructionChanged();
    void distanceToNextTurnChanged();
    void currentPositionChanged();
    
private:
    void initializeROS2();
    void spinROS2();
    
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void statusCallback(const std_msgs::msg::String::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    void updateNavigationMetrics();
    double calculateDistance(double x1, double y1, double x2, double y2) const;
    int findClosestWaypointIndex() const;
    
    // ROS2 membres
    rclcpp::Node::SharedPtr m_rosNode;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goalPublisher;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_globalPathSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_localPathSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_statusSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odomSubscriber;
    
    QTimer *m_rosTimer;
    QTimer *m_updateTimer;
    
    // Données existantes
    QVariantList m_globalPath;
    QVariantList m_localPath;
    QString m_routeStatus;
    double m_routeDistance;
    double m_routeDuration;
    int m_waypointCount;
    bool m_isNavigating;
    
    // NOUVELLES DONNÉES
    double m_currentSpeed;
    double m_remainingDistance;
    double m_remainingTime;
    int m_progressPercent;
    QString m_nextInstruction;
    double m_distanceToNextTurn;
    double m_currentX;
    double m_currentY;
    double m_initialDistance;
};

#endif // ROUTESERVICE_H
