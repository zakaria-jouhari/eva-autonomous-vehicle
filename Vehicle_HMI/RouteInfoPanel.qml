// RouteInfoPanel.qml
// Panneau d'informations de route (version Qt6 compatible)

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    
    property var routeService: null
    
    width: 360
    height: visible ? (routeService && routeService.isNavigating ? 180 : 120) : 0
    color: "#1E1E1E"
    radius: 12
    border.color: "#4285F4"
    border.width: 2
    
    visible: routeService && routeService.waypointCount > 0
    
    // Ombre simulée avec Rectangle
    Rectangle {
        anchors.fill: parent
        anchors.margins: -8
        z: -1
        radius: parent.radius + 4
        color: "transparent"
        border.color: "#40000000"
        border.width: 8
        opacity: 0.3
    }
    
    Behavior on height {
        NumberAnimation { duration: 200; easing.type: Easing.OutCubic }
    }
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12
        
        // En-tête
        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            
            // Icône de navigation
            Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#4285F4"
                
                Text {
                    anchors.centerIn: parent
                    text: "🗺"
                    font.pixelSize: 20
                }
            }
            
            // Titre et statut
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 4
                
                Text {
                    text: routeService && routeService.isNavigating ? "Navigation active" : "Route calculée"
                    color: "white"
                    font.pixelSize: 18
                    font.bold: true
                }
                
                Text {
                    text: routeService ? routeService.routeStatus : ""
                    color: "#888"
                    font.pixelSize: 12
                    elide: Text.ElideRight
                    Layout.fillWidth: true
                }
            }
            
            // Bouton fermer
            Button {
                text: "✕"
                width: 36
                height: 36
                
                background: Rectangle {
                    color: parent.hovered ? "#EA4335" : "#333"
                    radius: 18
                }
                
                contentItem: Text {
                    text: parent.text
                    color: "white"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pixelSize: 18
                }
                
                onClicked: {
                    if (routeService) {
                        routeService.clearRoute();
                    }
                }
            }
        }
        
        // Informations de la route
        GridLayout {
            Layout.fillWidth: true
            columns: 3
            columnSpacing: 16
            rowSpacing: 8
            
            // Distance
            Rectangle {
                Layout.fillWidth: true
                height: 60
                color: "#2A2A2A"
                radius: 8
                
                ColumnLayout {
                    anchors.centerIn: parent
                    spacing: 4
                    
                    Text {
                        text: routeService ? routeService.routeDistance.toFixed(2) : "0.00"
                        color: "#4285F4"
                        font.pixelSize: 24
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                    
                    Text {
                        text: "km"
                        color: "#888"
                        font.pixelSize: 12
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }
            
            // Durée
            Rectangle {
                Layout.fillWidth: true
                height: 60
                color: "#2A2A2A"
                radius: 8
                
                ColumnLayout {
                    anchors.centerIn: parent
                    spacing: 4
                    
                    Text {
                        text: routeService ? Math.round(routeService.routeDuration) : "0"
                        color: "#34A853"
                        font.pixelSize: 24
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                    
                    Text {
                        text: "min"
                        color: "#888"
                        font.pixelSize: 12
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }
            
            // Waypoints
            Rectangle {
                Layout.fillWidth: true
                height: 60
                color: "#2A2A2A"
                radius: 8
                
                ColumnLayout {
                    anchors.centerIn: parent
                    spacing: 4
                    
                    Text {
                        text: routeService ? routeService.waypointCount : "0"
                        color: "#FBBC04"
                        font.pixelSize: 24
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                    
                    Text {
                        text: "points"
                        color: "#888"
                        font.pixelSize: 12
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }
        }
        
        // Boutons d'action (seulement si navigation active)
        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            visible: routeService && routeService.isNavigating
            
            Button {
                Layout.fillWidth: true
                height: 40
                text: "Centrer"
                
                background: Rectangle {
                    color: parent.pressed ? "#3367D6" : (parent.hovered ? "#5A8DEE" : "#4285F4")
                    radius: 8
                }
                
                contentItem: Text {
                    text: parent.text
                    color: "white"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pixelSize: 14
                }
                
                onClicked: {
                    console.log("Recentering map on route");
                }
            }
            
            Button {
                Layout.fillWidth: true
                height: 40
                text: "Annuler"
                
                background: Rectangle {
                    color: parent.pressed ? "#C5221F" : (parent.hovered ? "#F44336" : "#EA4335")
                    radius: 8
                }
                
                contentItem: Text {
                    text: parent.text
                    color: "white"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pixelSize: 14
                }
                
                onClicked: {
                    if (routeService) {
                        routeService.cancelNavigation();
                    }
                }
            }
        }
    }
}
