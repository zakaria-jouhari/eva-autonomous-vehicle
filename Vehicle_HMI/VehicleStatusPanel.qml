// VehicleStatusPanel.qml
// Version CORRIGÉE - Sans warnings + Design cohérent

import QtQuick 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    
    // Propriétés
    property int batteryLevel: 84
    property bool charging: false
    property double range: 342
    property bool doorFL: false
    property bool doorFR: false
    property bool doorRL: false
    property bool doorRR: false
    property bool trunk: false
    property bool seatbelt: true
    property bool parkingBrake: true
    property int temperature: 17
    
    color: "#1E1E1E"
    radius: 12
    border.color: "#4285F4"
    border.width: 2
    
    // Ombre simple
    Rectangle {
        anchors.fill: parent
        anchors.margins: -4
        color: "#40000000"
        radius: 14
        z: -1
    }
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 10
        
        // Titre
        Text {
            text: "État du Véhicule"
            color: "white"
            font.pixelSize: 16
            font.bold: true
            Layout.fillWidth: true
        }
        
        // Ligne 1: Batterie et Autonomie
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 70
            spacing: 10
            
            // Batterie
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#2A2A2A"
                radius: 8
                
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 8
                    
                    // Icône batterie
                    Item {
                        Layout.preferredWidth: 35
                        Layout.preferredHeight: 20
                        
                        Rectangle {
                            anchors.fill: parent
                            radius: 3
                            color: "transparent"
                            border.color: batteryLevel > 20 ? "#34A853" : "#EA4335"
                            border.width: 2
                            
                            Rectangle {
                                anchors.right: parent.right
                                anchors.rightMargin: -3
                                anchors.verticalCenter: parent.verticalCenter
                                width: 3
                                height: parent.height * 0.5
                                color: batteryLevel > 20 ? "#34A853" : "#EA4335"
                                radius: 1
                            }
                            
                            Rectangle {
                                anchors.left: parent.left
                                anchors.leftMargin: 2
                                anchors.verticalCenter: parent.verticalCenter
                                width: (parent.width - 4) * (batteryLevel / 100)
                                height: parent.height - 4
                                radius: 2
                                color: batteryLevel > 20 ? "#34A853" : "#EA4335"
                            }
                        }
                    }
                    
                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 2
                        
                        Text {
                            text: batteryLevel + "%"
                            color: "white"
                            font.pixelSize: 20
                            font.bold: true
                            Layout.alignment: Qt.AlignHCenter
                        }
                        Text {
                            text: charging ? "⚡ Charge" : "Batterie"
                            color: charging ? "#FBBC04" : "#888"
                            font.pixelSize: 10
                            Layout.alignment: Qt.AlignHCenter
                        }
                    }
                }
            }
            
            // Autonomie
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#2A2A2A"
                radius: 8
                
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 4
                    
                    Text {
                        text: "🛣️"
                        font.pixelSize: 20
                        Layout.alignment: Qt.AlignHCenter
                    }
                    
                    Text {
                        text: range + " km"
                        color: "white"
                        font.pixelSize: 18
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                    
                    Text {
                        text: "Autonomie"
                        color: "#888"
                        font.pixelSize: 9
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }
        }
        
        // Ligne 2: Indicateurs (2x4 grid)
        GridLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 80
            columns: 4
            rows: 2
            rowSpacing: 5
            columnSpacing: 5
            
            StatusIndicator {
                icon: "🚪"
                label: "AV.G"
                active: doorFL
                alertColor: "#EA4335"
            }
            
            StatusIndicator {
                icon: "🚪"
                label: "AV.D"
                active: doorFR
                alertColor: "#EA4335"
            }
            
            StatusIndicator {
                icon: "🚪"
                label: "AR.G"
                active: doorRL
                alertColor: "#EA4335"
            }
            
            StatusIndicator {
                icon: "🚪"
                label: "AR.D"
                active: doorRR
                alertColor: "#EA4335"
            }
            
            StatusIndicator {
                icon: "📦"
                label: "Coffre"
                active: trunk
                alertColor: "#FBBC04"
            }
            
            StatusIndicator {
                icon: "🔒"
                label: "Ceinture"
                active: !seatbelt
                alertColor: "#EA4335"
            }
            
            StatusIndicator {
                icon: "🅿️"
                label: "Frein"
                active: parkingBrake
                alertColor: "#EA4335"
            }
            
            StatusIndicator {
                icon: "🌡️"
                label: temperature + "°C"
                active: false
                normalColor: "#4285F4"
            }
        }
    }
    
    // Composant StatusIndicator (CORRIGÉ)
    component StatusIndicator: Rectangle {
        property string icon: ""
        property string label: ""
        property bool active: false
        property color alertColor: "#EA4335"
        property color normalColor: "#34A853"
        
        Layout.fillWidth: true
        Layout.fillHeight: true
        color: "#2A2A2A"
        radius: 6
        
        ColumnLayout {
            anchors.fill: parent
            spacing: 2
            
            Text {
                text: icon
                font.pixelSize: 18
                Layout.alignment: Qt.AlignHCenter  // ✅ Bon: Layout.alignment au lieu de anchors
            }
            
            Text {
                text: label
                color: active ? alertColor : "#555"
                font.pixelSize: 9
                font.bold: active
                Layout.alignment: Qt.AlignHCenter  // ✅ Bon
                elide: Text.ElideRight
            }
        }
    }
}
