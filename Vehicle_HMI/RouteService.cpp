// RouteService.cpp
// Version sans warnings ROS2

#include "RouteService.h"
#include <QDebug>
#include <cmath>

RouteService::RouteService(QObject *parent)
    : QObject(parent)
    , m_routeDistance(0.0)
    , m_routeDuration(0.0)
    , m_waypointCount(0)
    , m_isNavigating(false)
    , m_routeStatus("Ready")
    , m_currentSpeed(0.0)
    , m_remainingDistance(0.0)
    , m_remainingTime(0.0)
    , m_progressPercent(0)
    , m_nextInstruction("No active navigation")
    , m_distanceToNextTurn(0.0)
    , m_currentX(0.0)
    , m_currentY(0.0)
    , m_initialDistance(0.0)
{
    qDebug() << "RouteService: Initializing...";
    
    initializeROS2();
    
    // Timer ROS2 (20 Hz)
    m_rosTimer = new QTimer(this);
    connect(m_rosTimer, &QTimer::timeout, this, &RouteService::spinROS2);
    m_rosTimer->start(50);
    
    // Timer de mise à jour des métriques (5 Hz)
    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &RouteService::updateNavigationMetrics);
    m_updateTimer->start(200);
    
    qDebug() << "RouteService: Initialized successfully";
}

RouteService::~RouteService()
{
    if (m_rosTimer) m_rosTimer->stop();
    if (m_updateTimer) m_updateTimer->stop();
    qDebug() << "RouteService: Destroyed";
}

void RouteService::initializeROS2()
{
    try {
        m_rosNode = rclcpp::Node::make_shared("vehicle_hmi_node");
        
        m_goalPublisher = m_rosNode->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
        
        // ✅ Utilisation de lambda au lieu de std::bind (évite les warnings)
        m_globalPathSubscriber = m_rosNode->create_subscription<nav_msgs::msg::Path>(
            "/planning/global_path", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                this->globalPathCallback(msg);
            });
        
        m_localPathSubscriber = m_rosNode->create_subscription<nav_msgs::msg::Path>(
            "/planning/local_path", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                this->localPathCallback(msg);
            });
        
        m_statusSubscriber = m_rosNode->create_subscription<std_msgs::msg::String>(
            "/planning/status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->statusCallback(msg);
            });
        
        m_odomSubscriber = m_rosNode->create_subscription<nav_msgs::msg::Odometry>(
            "/vehicle/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odomCallback(msg);
            });
        
        qDebug() << "ROS2: All subscribers/publishers created";
        
    } catch (const std::exception& e) {
        qCritical() << "ROS2: Failed to initialize:" << e.what();
        m_routeStatus = "ROS2 initialization failed";
        emit routeStatusChanged();
    }
}

void RouteService::spinROS2()
{
    if (m_rosNode) {
        rclcpp::spin_some(m_rosNode);
    }
}

void RouteService::navigateTo(double x, double y)
{
    if (!m_goalPublisher) {
        qWarning() << "ROS2: Goal publisher not initialized";
        return;
    }
    
    qDebug() << "RouteService: Navigating to (" << x << "," << y << ")";
    
    geometry_msgs::msg::PoseStamped goalMsg;
    goalMsg.header.stamp = m_rosNode->now();
    goalMsg.header.frame_id = "odom";
    goalMsg.pose.position.x = x;
    goalMsg.pose.position.y = y;
    goalMsg.pose.position.z = 0.0;
    goalMsg.pose.orientation.w = 1.0;
    
    m_goalPublisher->publish(goalMsg);
    
    m_isNavigating = true;
    m_routeStatus = "Planning route...";
    
    emit isNavigatingChanged();
    emit routeStatusChanged();
    emit navigationStarted();
}

void RouteService::cancelNavigation()
{
    qDebug() << "RouteService: Navigation cancelled";
    
    m_isNavigating = false;
    m_routeStatus = "Navigation cancelled";
    
    emit isNavigatingChanged();
    emit routeStatusChanged();
}

void RouteService::clearRoute()
{
    qDebug() << "RouteService: Clearing route";
    
    m_globalPath.clear();
    m_localPath.clear();
    m_routeDistance = 0.0;
    m_routeDuration = 0.0;
    m_waypointCount = 0;
    m_isNavigating = false;
    m_routeStatus = "Ready";
    m_remainingDistance = 0.0;
    m_remainingTime = 0.0;
    m_progressPercent = 0;
    m_nextInstruction = "No active navigation";
    m_distanceToNextTurn = 0.0;
    m_initialDistance = 0.0;
    
    emit globalPathChanged();
    emit localPathChanged();
    emit routeDistanceChanged();
    emit routeDurationChanged();
    emit waypointCountChanged();
    emit isNavigatingChanged();
    emit routeStatusChanged();
    emit remainingDistanceChanged();
    emit remainingTimeChanged();
    emit progressPercentChanged();
    emit nextInstructionChanged();
    emit distanceToNextTurnChanged();
}

void RouteService::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    m_currentX = msg->pose.pose.position.x;
    m_currentY = msg->pose.pose.position.y;
    
    // Vitesse (m/s vers km/h)
    double speed_ms = std::sqrt(
        msg->twist.twist.linear.x * msg->twist.twist.linear.x +
        msg->twist.twist.linear.y * msg->twist.twist.linear.y
    );
    m_currentSpeed = speed_ms * 3.6; // Conversion en km/h
    
    emit currentSpeedChanged();
    emit currentPositionChanged();
}

void RouteService::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // ✅ FILTRE: Éviter de retraiter le même path
    static size_t last_path_size = 0;
    static double last_end_x = 0.0, last_end_y = 0.0;
    
    if (msg->poses.size() == last_path_size && !msg->poses.empty()) {
        auto last_pose = msg->poses.back();
        if (std::abs(last_pose.pose.position.x - last_end_x) < 0.01 &&
            std::abs(last_pose.pose.position.y - last_end_y) < 0.01) {
            // Même path, ignorer
            return;
        }
    }
    
    // Nouveau path détecté
    last_path_size = msg->poses.size();
    if (!msg->poses.empty()) {
        last_end_x = msg->poses.back().pose.position.x;
        last_end_y = msg->poses.back().pose.position.y;
    }
    
    qDebug() << "ROS2: Processing NEW global path with" << msg->poses.size() << "waypoints";
    
    m_globalPath.clear();
    
    for (const auto& pose : msg->poses) {
        QVariantMap point;
        point["x"] = pose.pose.position.x;
        point["y"] = pose.pose.position.y;
        m_globalPath.append(point);
    }
    
    m_waypointCount = m_globalPath.size();
    
    // Calculer distance totale
    double totalDistance = 0.0;
    for (int i = 0; i < m_globalPath.size() - 1; ++i) {
        auto p1 = m_globalPath[i].toMap();
        auto p2 = m_globalPath[i + 1].toMap();
        totalDistance += calculateDistance(
            p1["x"].toDouble(), p1["y"].toDouble(),
            p2["x"].toDouble(), p2["y"].toDouble()
        );
    }
    
    m_routeDistance = totalDistance / 1000.0;  // km
    m_initialDistance = m_routeDistance;
    m_remainingDistance = m_routeDistance;
    
    // Durée estimée (40 km/h en ville)
    m_routeDuration = (totalDistance / 1000.0) / 40.0 * 60.0;  // minutes
    m_remainingTime = m_routeDuration;
    
    m_nextInstruction = "Follow the route";
    
    emit globalPathChanged();
    emit waypointCountChanged();
    emit routeDistanceChanged();
    emit routeDurationChanged();
    emit remainingDistanceChanged();
    emit remainingTimeChanged();
    emit nextInstructionChanged();
    emit routeReady();
}

void RouteService::localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    m_localPath.clear();
    
    for (const auto& pose : msg->poses) {
        QVariantMap point;
        point["x"] = pose.pose.position.x;
        point["y"] = pose.pose.position.y;
        m_localPath.append(point);
    }
    
    emit localPathChanged();
}

void RouteService::statusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    QString status = QString::fromStdString(msg->data);
    m_routeStatus = status;
    
    if (status.contains("FAILED")) {
        m_isNavigating = false;
        emit isNavigatingChanged();
    }
    
    emit routeStatusChanged();
}

double RouteService::calculateDistance(double x1, double y1, double x2, double y2) const
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

int RouteService::findClosestWaypointIndex() const
{
    if (m_globalPath.isEmpty()) return -1;
    
    int closestIdx = 0;
    double minDist = std::numeric_limits<double>::max();
    
    for (int i = 0; i < m_globalPath.size(); ++i) {
        auto pt = m_globalPath[i].toMap();
        double dist = calculateDistance(
            m_currentX, m_currentY,
            pt["x"].toDouble(), pt["y"].toDouble()
        );
        
        if (dist < minDist) {
            minDist = dist;
            closestIdx = i;
        }
    }
    
    return closestIdx;
}

void RouteService::updateNavigationMetrics()
{
    if (!m_isNavigating || m_globalPath.isEmpty()) {
        return;
    }
    
    // Trouver le waypoint le plus proche
    int closestIdx = findClosestWaypointIndex();
    if (closestIdx < 0) return;
    
    // Calculer distance restante
    double remaining = 0.0;
    
    // Distance jusqu'au waypoint le plus proche
    auto closestPt = m_globalPath[closestIdx].toMap();
    remaining += calculateDistance(
        m_currentX, m_currentY,
        closestPt["x"].toDouble(), closestPt["y"].toDouble()
    );
    
    // Distance entre les waypoints restants
    for (int i = closestIdx; i < m_globalPath.size() - 1; ++i) {
        auto p1 = m_globalPath[i].toMap();
        auto p2 = m_globalPath[i + 1].toMap();
        remaining += calculateDistance(
            p1["x"].toDouble(), p1["y"].toDouble(),
            p2["x"].toDouble(), p2["y"].toDouble()
        );
    }
    
    m_remainingDistance = remaining / 1000.0;  // km
    
    // Temps restant (basé sur vitesse actuelle ou 40 km/h par défaut)
    double speed = (m_currentSpeed > 5.0) ? m_currentSpeed : 40.0;
    m_remainingTime = (remaining / 1000.0) / speed * 60.0;  // minutes
    
    // Progression
    if (m_initialDistance > 0) {
        double traveled = m_initialDistance - m_remainingDistance;
        m_progressPercent = static_cast<int>((traveled / m_initialDistance) * 100.0);
        m_progressPercent = std::max(0, std::min(100, m_progressPercent));
    }
    
    // Distance au prochain point
    if (closestIdx + 1 < m_globalPath.size()) {
        auto nextPt = m_globalPath[closestIdx + 1].toMap();
        m_distanceToNextTurn = calculateDistance(
            m_currentX, m_currentY,
            nextPt["x"].toDouble(), nextPt["y"].toDouble()
        );
    }
    
    // Vérifier si arrivé
    auto destination = m_globalPath.last().toMap();
    double distToGoal = calculateDistance(
        m_currentX, m_currentY,
        destination["x"].toDouble(), destination["y"].toDouble()
    );
    
    if (distToGoal < 5.0) {  // Moins de 5 mètres
        m_isNavigating = false;
        m_routeStatus = "Destination reached!";
        m_nextInstruction = "You have arrived";
        emit navigationCompleted();
        emit isNavigatingChanged();
        emit routeStatusChanged();
        emit nextInstructionChanged();
    }
    
    emit remainingDistanceChanged();
    emit remainingTimeChanged();
    emit progressPercentChanged();
    emit distanceToNextTurnChanged();
}
