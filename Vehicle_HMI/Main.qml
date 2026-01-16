// Main.qml
// OPTIMISÉ pour écran 7" tactile (1024x600)
// Design professionnel automobile

import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtLocation 6.2
import QtPositioning 6.2

Window {
    id: mainWindow
    width: 1024
    height: 600
    visible: true
    title: "EVA COCKPIT"
    color: "#0A0A0A"
    
    // Constantes de design pour cohérence
    readonly property int headerHeight: 50
    readonly property int margin: 8
    readonly property int panelRadius: 12
    readonly property color accentBlue: "#4285F4"
    readonly property color accentGreen: "#34A853"
    readonly property color accentYellow: "#FBBC04"
    readonly property color accentRed: "#EA4335"
    
    // ==========================================
    // BARRE SUPÉRIEURE
    // ==========================================
    Rectangle {
        id: topBar
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: headerHeight
        color: "#1A1A1A"
        z: 100
        
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 15
            anchors.rightMargin: 15
            spacing: 15
            
            // Logo et titre
            Text {
                text: "EVA_COCKPIT"
                color: accentBlue
                font.pixelSize: 20
                font.bold: true
            }
            
            Text {
                text: "|"
                color: "#555"
                font.pixelSize: 18
            }
            
            Text {
                text: Qt.formatDateTime(new Date(), "MMM dd  hh:mm AP")
                color: "white"
                font.pixelSize: 14
            }
            
            Item { Layout.fillWidth: true }
            
            // Status ROS2
            Rectangle {
                Layout.preferredWidth: 100
                Layout.preferredHeight: 30
                radius: 15
                color: routeService && routeService.waypointCount > 0 ? accentGreen : "#444"
                
                Text {
                    anchors.centerIn: parent
                    text: routeService && routeService.waypointCount > 0 ? "🟢 ROS2" : "⚪ ROS2"
                    color: "white"
                    font.pixelSize: 12
                    font.bold: true
                }
            }
            
            // Température
            Text {
                text: "🌡️ 17°C"
                color: "white"
                font.pixelSize: 14
            }
            
            // Connectivité
            Row {
                spacing: 8
                Text { text: "📶"; color: accentBlue; font.pixelSize: 16 }
                Text { text: "📡"; color: accentBlue; font.pixelSize: 16 }
            }
        }
        
        Rectangle {
            anchors.bottom: parent.bottom
            width: parent.width
            height: 1
            color: "#333"
        }
    }
    
    // ==========================================
    // CONTENU PRINCIPAL (Split View)
    // ==========================================
    RowLayout {
        anchors {
            top: topBar.bottom
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            margins: margin
        }
        spacing: margin
        
        // ==========================================
        // PANNEAU GAUCHE - DASHBOARD
        // ==========================================
        Rectangle {
            Layout.preferredWidth: 380
            Layout.fillHeight: true
            color: "#111"
            radius: panelRadius
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 10
                
                // État du véhicule (taille fixe cohérente)
                VehicleStatusPanel {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 180  // ✅ Taille fixe cohérente
                }
                
                // Speedomètre circulaire
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 180
                    color: "#1A1A1A"
                    radius: 12
                    
                    Rectangle {
                        anchors.centerIn: parent
                        width: 160
                        height: 160
                        radius: 80
                        color: "#2A2A2A"
                        border.color: accentBlue
                        border.width: 4
                        
                        ColumnLayout {
                            anchors.centerIn: parent
                            spacing: 5
                            
                            Text {
                                text: Math.round(routeService ? routeService.currentSpeed : 0)
                                color: "white"
                                font.pixelSize: 50
                                font.bold: true
                                Layout.alignment: Qt.AlignHCenter
                            }
                            
                            Text {
                                text: "km/h"
                                color: "#888"
                                font.pixelSize: 14
                                Layout.alignment: Qt.AlignHCenter
                            }
                            
                            Text {
                                text: "Current Speed"
                                color: accentBlue
                                font.pixelSize: 11
                                Layout.alignment: Qt.AlignHCenter
                            }
                        }
                    }
                }
                
                // Info Navigation (quand active)
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 140
                    color: "#1A1A1A"
                    radius: 12
                    visible: routeService && routeService.isNavigating
                    
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 8
                        
                        Text {
                            text: "🧭 Navigation Active"
                            color: accentBlue
                            font.pixelSize: 14
                            font.bold: true
                        }
                        
                        // Barre de progression
                        Rectangle {
                            Layout.fillWidth: true
                            height: 25
                            color: "#2A2A2A"
                            radius: 12
                            
                            Rectangle {
                                width: parent.width * (routeService ? routeService.progressPercent / 100 : 0)
                                height: parent.height
                                radius: 12
                                color: accentGreen
                                
                                Behavior on width {
                                    NumberAnimation { duration: 300 }
                                }
                            }
                            
                            Text {
                                anchors.centerIn: parent
                                text: (routeService ? routeService.progressPercent : 0) + "%"
                                color: "white"
                                font.pixelSize: 12
                                font.bold: true
                            }
                        }
                        
                        // Métriques
                        GridLayout {
                            Layout.fillWidth: true
                            columns: 3
                            rowSpacing: 5
                            columnSpacing: 8
                            
                            NavMetric {
                                value: routeService ? routeService.remainingDistance.toFixed(1) : "0.0"
                                unit: "km left"
                                metricColor: accentGreen
                            }
                            
                            NavMetric {
                                value: routeService ? Math.round(routeService.remainingTime).toString() : "0"
                                unit: "min"
                                metricColor: accentYellow
                            }
                            
                            NavMetric {
                                value: routeService ? routeService.waypointCount.toString() : "0"
                                unit: "points"
                                metricColor: accentRed
                            }
                        }
                        
                        // Instruction
                        Rectangle {
                            Layout.fillWidth: true
                            height: 35
                            color: "#2A2A2A"
                            radius: 8
                            
                            Text {
                                anchors.centerIn: parent
                                text: routeService ? routeService.nextInstruction : ""
                                color: "white"
                                font.pixelSize: 11
                                elide: Text.ElideRight
                                width: parent.width - 16
                                horizontalAlignment: Text.AlignHCenter
                            }
                        }
                    }
                }
                
                // Route Info (quand pas de navigation)
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 100
                    color: "#1A1A1A"
                    radius: 12
                    visible: routeService && routeService.waypointCount > 0 && !routeService.isNavigating
                    
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 8
                        
                        Text {
                            text: "📍 Route calculée"
                            color: accentBlue
                            font.pixelSize: 14
                            font.bold: true
                        }
                        
                        GridLayout {
                            Layout.fillWidth: true
                            columns: 3
                            rowSpacing: 5
                            columnSpacing: 10
                            
                            RouteMetric {
                                label: "Distance"
                                value: routeService ? routeService.routeDistance.toFixed(2) + " km" : "0 km"
                                metricColor: accentGreen
                            }
                            
                            RouteMetric {
                                label: "Durée"
                                value: routeService ? Math.round(routeService.routeDuration) + " min" : "0 min"
                                metricColor: accentYellow
                            }
                            
                            RouteMetric {
                                label: "Points"
                                value: routeService ? routeService.waypointCount + " pts" : "0 pts"
                                metricColor: accentRed
                            }
                        }
                    }
                }
                
                Item { Layout.fillHeight: true }
            }
        }
        
        // ==========================================
        // PANNEAU DROIT - CARTE
        // ==========================================
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#1A1A1A"
            radius: panelRadius
            
            Map {
                id: map
                anchors.fill: parent
                anchors.margins: 3
                clip: true
                
                plugin: Plugin {
                    name: "osm"
                }
                
                center: QtPositioning.coordinate(33.5731, -7.5898)
                zoomLevel: 14
                
                // Route globale
                MapPolyline {
                    line.width: 6
                    line.color: accentBlue
                    visible: routeService && routeService.globalPath && routeService.globalPath.length > 0
                    
                    path: {
                        if (!routeService || !routeService.globalPath) return [];
                        var coords = [];
                        var metersPerDegreeLat = 111320.0;
                        var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                        for (var i = 0; i < routeService.globalPath.length; i++) {
                            var pt = routeService.globalPath[i];
                            var lat = 33.5731 + (pt.y / metersPerDegreeLat);
                            var lon = -7.5898 + (pt.x / metersPerDegreeLon);
                            coords.push(QtPositioning.coordinate(lat, lon));
                        }
                        return coords;
                    }
                }
                
                // Marqueur A (départ)
                MapQuickItem {
                    visible: routeService && routeService.globalPath && routeService.globalPath.length > 0
                    coordinate: {
                        if (!routeService || !routeService.globalPath || routeService.globalPath.length === 0) {
                            return QtPositioning.coordinate(33.5731, -7.5898);
                        }
                        var pt = routeService.globalPath[0];
                        var metersPerDegreeLat = 111320.0;
                        var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                        var lat = 33.5731 + (pt.y / metersPerDegreeLat);
                        var lon = -7.5898 + (pt.x / metersPerDegreeLon);
                        return QtPositioning.coordinate(lat, lon);
                    }
                    anchorPoint: Qt.point(15, 30)
                    sourceItem: MarkerItem { label: "A"; markerColor: accentBlue }
                }
                
                // Marqueur B (arrivée)
                MapQuickItem {
                    visible: routeService && routeService.globalPath && routeService.globalPath.length > 0
                    coordinate: {
                        if (!routeService || !routeService.globalPath || routeService.globalPath.length === 0) {
                            return QtPositioning.coordinate(33.5731, -7.5898);
                        }
                        var pt = routeService.globalPath[routeService.globalPath.length - 1];
                        var metersPerDegreeLat = 111320.0;
                        var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                        var lat = 33.5731 + (pt.y / metersPerDegreeLat);
                        var lon = -7.5898 + (pt.x / metersPerDegreeLon);
                        return QtPositioning.coordinate(lat, lon);
                    }
                    anchorPoint: Qt.point(15, 30)
                    sourceItem: MarkerItem { label: "B"; markerColor: accentRed }
                }
                
                // Position véhicule (suit l'odométrie)
                MapQuickItem {
                    coordinate: {
                        if (!routeService) return QtPositioning.coordinate(33.5731, -7.5898);
                        var metersPerDegreeLat = 111320.0;
                        var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                        var lat = 33.5731 + (routeService.currentY / metersPerDegreeLat);
                        var lon = -7.5898 + (routeService.currentX / metersPerDegreeLon);
                        return QtPositioning.coordinate(lat, lon);
                    }
                    anchorPoint: Qt.point(20, 20)
                    sourceItem: VehicleMarker {}
                }
                
                // Click handler pour définir destination
                MouseArea {
                    anchors.fill: parent
                    onClicked: function(mouse) {
                        var coord = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                        var metersPerDegreeLat = 111320.0;
                        var metersPerDegreeLon = 111320.0 * Math.cos(33.5731 * Math.PI / 180.0);
                        var x = (coord.longitude - (-7.5898)) * metersPerDegreeLon;
                        var y = (coord.latitude - 33.5731) * metersPerDegreeLat;
                        routeService.navigateTo(x, y)
                        notification.show("🎯 Nouvelle destination définie")
                    }
                }
                
                // Contrôles carte (en bas à droite)
                Column {
                    anchors.right: parent.right
                    anchors.bottom: parent.bottom
                    anchors.margins: 10
                    spacing: 5
                    z: 100
                    
                    MapButton { text: "+"; onClicked: map.zoomLevel += 1 }
                    MapButton { text: "−"; onClicked: map.zoomLevel -= 1 }
                    MapButton { 
                        text: "⌖"
                        backgroundColor: accentGreen
                        onClicked: {
                            map.center = QtPositioning.coordinate(33.5731, -7.5898)
                            map.zoomLevel = 14
                        }
                    }
                }
            }
        }
    }
    
    // ==========================================
    // NOTIFICATION (overlay)
    // ==========================================
    Rectangle {
        id: notification
        anchors.top: topBar.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 15
        width: 350
        height: 50
        radius: 25
        color: "#2A2A2A"
        border.color: accentBlue
        border.width: 2
        visible: false
        z: 200
        
        property string message: ""
        
        Text {
            anchors.centerIn: parent
            text: notification.message
            color: "white"
            font.pixelSize: 14
            font.bold: true
        }
        
        Timer {
            id: notificationTimer
            interval: 2500
            onTriggered: notification.visible = false
        }
        
        function show(msg) {
            notification.message = msg
            notification.visible = true
            notificationTimer.restart()
        }
    }
    
    // Connexions ROS2
    Connections {
        target: routeService
        function onRouteReady() { notification.show("✓ Route calculée avec succès") }
        function onNavigationStarted() { notification.show("🧭 Navigation démarrée") }
        function onNavigationCompleted() { notification.show("🎉 Vous êtes arrivé!") }
    }
    
    // ==========================================
    // COMPOSANTS RÉUTILISABLES
    // ==========================================
    component NavMetric: Rectangle {
        property string value: "0"
        property string unit: ""
        property color metricColor: accentBlue
        
        Layout.fillWidth: true
        height: 45
        color: "#2A2A2A"
        radius: 8
        
        ColumnLayout {
            anchors.centerIn: parent
            spacing: 2
            
            Text {
                text: value
                color: metricColor
                font.pixelSize: 16
                font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }
            Text {
                text: unit
                color: "#888"
                font.pixelSize: 9
                Layout.alignment: Qt.AlignHCenter
            }
        }
    }
    
    component RouteMetric: ColumnLayout {
        property string label: ""
        property string value: ""
        property color metricColor: accentBlue
        
        Layout.fillWidth: true
        spacing: 2
        
        Text {
            text: value
            color: metricColor
            font.pixelSize: 14
            font.bold: true
            Layout.alignment: Qt.AlignHCenter
        }
        Text {
            text: label
            color: "#888"
            font.pixelSize: 9
            Layout.alignment: Qt.AlignHCenter
        }
    }
    
    component MarkerItem: Rectangle {
        property string label: "A"
        property color markerColor: accentBlue
        
        width: 30
        height: 30
        radius: 15
        color: markerColor
        border.color: "white"
        border.width: 3
        
        Text {
            anchors.centerIn: parent
            text: label
            color: "white"
            font.bold: true
            font.pixelSize: 16
        }
    }
    
    component VehicleMarker: Rectangle {
        width: 40
        height: 40
        radius: 20
        color: accentGreen
        border.color: "white"
        border.width: 4
        
        Rectangle {
            anchors.centerIn: parent
            width: 8
            height: 8
            radius: 4
            color: "white"
        }
        
        SequentialAnimation on scale {
            running: true
            loops: Animation.Infinite
            NumberAnimation { to: 1.15; duration: 1000 }
            NumberAnimation { to: 1.0; duration: 1000 }
        }
    }
    
    component MapButton: Rectangle {
        property string text: "+"
        property color backgroundColor: accentBlue
        signal clicked()
        
        width: 40
        height: 40
        radius: 20
        color: backgroundColor
        
        Text {
            anchors.centerIn: parent
            text: parent.text
            font.pixelSize: 24
            font.bold: true
            color: "white"
        }
        
        MouseArea {
            anchors.fill: parent
            onClicked: parent.clicked()
        }
    }
}
