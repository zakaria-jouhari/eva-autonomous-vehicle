cmake_minimum_required(VERSION 3.16)

project(Vehicle_HMI VERSION 0.1 LANGUAGES CXX)
set(CMAKE_PREFIX_PATH "/home/ziko/Qt/6.10.1/gcc_64")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ============================================
# Qt setup (avec Location pour la carte)
# ============================================
find_package(Qt6 6.2 REQUIRED COMPONENTS 
    Core 
    Quick
    Location      # Pour Map et MapPolyline
    Positioning   # Pour QtPositioning
)

# ============================================
# ROS2 setup
# ============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ============================================
# Source files
# ============================================
set(PROJECT_SOURCES
    main.cpp
    RouteService.h
    RouteService.cpp
    resources.qrc
)

# ============================================
# Executable
# ============================================
add_executable(appVehicle_HMI ${PROJECT_SOURCES})

# ============================================
# Include directories
# ============================================
target_include_directories(appVehicle_HMI PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

# ============================================
# Link libraries
# ============================================
target_link_libraries(appVehicle_HMI PRIVATE
    # Qt
    Qt6::Core
    Qt6::Quick
    Qt6::Location
    Qt6::Positioning
    
    # ROS2
    ${rclcpp_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

# ============================================
# Link directories
# ============================================
target_link_directories(appVehicle_HMI PRIVATE
    ${rclcpp_LIBRARY_DIRS}
    ${nav_msgs_LIBRARY_DIRS}
    ${geometry_msgs_LIBRARY_DIRS}
    ${std_msgs_LIBRARY_DIRS}
)

# ============================================
# Install
# ============================================
install(TARGETS appVehicle_HMI
    RUNTIME DESTINATION bin
)

# ============================================
# Messages de debug
# ============================================
message(STATUS "Qt Version: ${Qt6_VERSION}")
message(STATUS "ROS2 found: rclcpp")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
