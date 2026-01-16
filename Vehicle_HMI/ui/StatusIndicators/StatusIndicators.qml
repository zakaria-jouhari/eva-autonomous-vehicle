import QtQuick 2.15
import QtQuick.Controls 2.15

Item {
    id: status_indicators_item

    // Property declarations
    property int currentPower: 0
    property int currentOdometerValue: 0
    property int currentPower_level: 0
    property string currentGear: "P"  // Default to Park
    property bool seatbeltActive: false  // Boolean for on/off
    property bool brakesystemActive: false
    property bool door_openActive: false
    property bool high_beamActive: false

    // Gear data model
    property var gearData: {
        "P": { name: "Park", color: "#FF6B6B" },      // Red
        "R": { name: "Reverse", color: "#4ECDC4" },   // Cyan
        "N": { name: "Neutral", color: "#FFE66D" },   // Yellow
        "D": { name: "Drive", color: "#95E1D3" }      // Green
    }
    // Warning indicators data model
    property var warningIndicators: {
        "S": {
            name: "SEATBELT",
            iconOn: "qrc:/ui/assets/seatbelt_on.svg",
            iconOff: "qrc:/ui/assets/seatbelt_off.svg",
            color: "#FF4444"
        },
        "B": {
            name: "BRAKE",
            iconOn: "qrc:/ui/assets/brake_on.svg",
            iconOff: "qrc:/ui/assets/brake_off.svg",
            color: "#FF4444"
        },
        "O": {
            name: "DOOR",
            iconOn: "qrc:/ui/assets/door_on.svg",
            iconOff: "qrc:/ui/assets/door_off.svg",
            color: "#FFA500"
        },
        "H": {
            name: "HIGH-BEAM",
            iconOn: "qrc:/ui/assets/highbeam_on.svg",
            iconOff: "qrc:/ui/assets/highbeam_off.svg",
            color: "#4444FF"
        }
    }

    // Focus must be enabled to receive keyboard input
    focus: true

    // Keyboard event handler
    Keys.onPressed: (event) => {
        var key = event.text.toUpperCase()
        if (gearData.hasOwnProperty(key)) {
            currentGear = key
            event.accepted = true
        }
        // Handle warning indicators
        if (warningIndicators.hasOwnProperty(key)) {
            switch(key) {
                case "S":
                    seatbeltActive = !seatbeltActive
                    break
                case "B":
                    brakesystemActive = !brakesystemActive
                    break
                case "O":
                    door_openActive = !door_openActive
                    break
                case "H":
                    high_beamActive = !high_beamActive
                    break
            }
            event.accepted = true
        }
    }

    // ---------------------------
    // Power Indicator
    // ---------------------------
    Item {
        id: power_indacator_item
        anchors {
            right: parent.right
            verticalCenter: parent.verticalCenter
        }
        width: parent.width
        height: parent.height

        // Battery indicator icon
        Image {
            id: battery_indicator_icon
            source: "qrc:/ui/assets/battery_status.png"
            fillMode: Image.PreserveAspectFit
            anchors.right: parent.right
            width: parent.parent.width * 0.1
            height: parent.parent.height * 0.1
            anchors.bottom: parent.bottom
            anchors.bottomMargin: parent.parent.height * 0.04
        }

        Label {
            id: powerValueLabel
            text: currentPower + " Kw/h"
            color: "green"
            font.pixelSize: parent.parent.height * 0.035
            anchors.right: parent.right
            anchors.rightMargin: parent.width * 0.08
            anchors.bottom: parent.bottom
            anchors.bottomMargin: parent.parent.height * 0.04
        }

        Label {
            id: powerLevelLabel
            text: currentPower_level + " %"
            color: "white"
            font.pixelSize: parent.parent.height * 0.05
            anchors.right: parent.right
            anchors.rightMargin: parent.width * 0.08
            anchors.bottom: parent.bottom
            anchors.bottomMargin: parent.parent.height * 0.08
            font.bold: true
        }
    }

    // ---------------------------
    // Odometer Indicator
    // ---------------------------
    Item {
        id: odometer
        anchors {
            left: parent.left
            verticalCenter: parent.verticalCenter
        }
        width: parent.width
        height: parent.height

        Image {
            id: odometer_icon
            source: "qrc:/ui/assets/odometer.png"
            fillMode: Image.PreserveAspectFit
            anchors.left: parent.left
            width: parent.parent.width * 0.1
            height: parent.parent.height * 0.1
            anchors.bottom: parent.bottom
            anchors.bottomMargin: parent.parent.height * 0.06
            anchors.leftMargin: parent.width * 0.02
        }

        Label {
            id: odometer_value
            text: currentOdometerValue + " km"
            font.bold: true
            color: "white"
            font.pixelSize: parent.parent.height * 0.05
            anchors.left: parent.left
            anchors.leftMargin: parent.width * 0.11
            anchors.bottom: parent.bottom
            anchors.bottomMargin: parent.parent.height * 0.08
        }
    }

    // ---------------------------
    // Gear Selection Display
    // ---------------------------
    Item {
        id: gearSelection
        anchors {
            horizontalCenter: parent.horizontalCenter
            bottom: parent.bottom
            bottomMargin: parent.height*0.05
        }
        width: parent.width * 0.2
        height: parent.height * 0.2

        // Background circle/rectangle for gear indicator
        Rectangle {
            id: gearBackground
            anchors.bottom: parent.bottom
            anchors.horizontalCenter: parent.horizontalCenter
            width: parent.height * 0.4
            height: parent.height * 0.4
            radius: width / 2  // Make it circular
            color: "transparent"
            border.color: gearData[currentGear].color
            border.width: 3
            // Animated color transition
            Behavior on border.color {
                ColorAnimation { duration: 200 }
            }
        }

        // Gear letter (large)
        Text {
            id: gearLetter
            text: currentGear
            font.pixelSize: parent.height * 0.2
            font.bold: true
            color: gearData[currentGear].color
            anchors.centerIn: gearBackground

            // Animated color transition
            Behavior on color {
                ColorAnimation { duration: 200 }
            }

            // Scale animation on gear change
            SequentialAnimation on scale {
                id: gearChangeAnimation
                running: false
                NumberAnimation { to: 1.3; duration: 150 }
                NumberAnimation { to: 1.0; duration: 150 }
            }
        }

        // // Gear name (below the letter)
        Text {
            id: gearName
            text: gearData[currentGear].name
            font.pixelSize: parent.height * 0.15
            font.bold: true
            color: gearData[currentGear].color
            anchors.top: gearBackground.bottom
            anchors.topMargin: parent.height * 0.05
            anchors.horizontalCenter: parent.horizontalCenter

            // Animated color transition
            Behavior on color {
                ColorAnimation { duration: 200 }
            }
        }

        // All gear options (small indicators)
        Row {
            id: gearOptions
            anchors {
                horizontalCenter: parent.horizontalCenter
                bottom: gearBackground.top
                bottomMargin: parent.height * 0.02
            }
            spacing: parent.width * 0.085

            Repeater {
                model: ["P", "R", "N", "D"]

                Rectangle {
                    width: gearSelection.height * 0.15
                    height: gearSelection.height * 0.15
                    radius: width / 2
                    color: currentGear === modelData ? gearData[modelData].color : "transparent"
                    border.color: gearData[modelData].color
                    border.width: 2
                    opacity: currentGear === modelData ? 1.0 : 0.4

                    Text {
                        text: modelData
                        anchors.centerIn: parent
                        font.pixelSize: parent.height * 0.5
                        font.bold: true
                        color: currentGear === modelData ? "black" : gearData[modelData].color
                    }

                    Behavior on opacity {
                        NumberAnimation { duration: 200 }
                    }

                    Behavior on color {
                        ColorAnimation { duration: 200 }
                    }
                }
            }
        }
    }

    // ---------------------------
    // Warning Lights
    // ---------------------------
    Item {
        id: warning_lights
        anchors {
            // right: parent.parent.right
            // top: parent.top
            // topMargin: parent.height * 0.05
            // rightMargin: parent.width * 0.05
            left: parent.left
            top: parent.top
            topMargin: parent.height * 0.05
            leftMargin: parent.width * 0.02
        }
        width: parent.width * 0.3
        height: parent.height * 0.1

        Row {
            spacing: parent.width * 0.05
            //anchors.verticalCenter: parent.verticalCenter  // Center the entire row

            // Seatbelt Indicator
            Image {
                id: seatbelt_indicator
                source: seatbeltActive ?
                    "qrc:/ui/assets/seatbelt_on.svg" :
                    "qrc:/ui/assets/seatbelt_off.svg"
                fillMode: Image.PreserveAspectFit
                width: warning_lights.width * 0.10
                height: warning_lights.height * 0.4
                opacity: seatbeltActive ? 1.0 : 0.3
                //anchors.verticalCenter: parent.verticalCenter  // Center this image

                Behavior on opacity {
                    NumberAnimation { duration: 200 }
                }

                // Pulse animation when active
                SequentialAnimation on scale {
                    running: seatbeltActive
                    loops: Animation.Infinite
                    NumberAnimation { to: 1.1; duration: 500 }
                    NumberAnimation { to: 1.0; duration: 500 }
                }
            }

            // Brake System Indicator
            Image {
                id: brake_indicator
                source: brakesystemActive ?
                    "qrc:/ui/assets/brake_on.svg" :
                    "qrc:/ui/assets/brake_off.svg"
                fillMode: Image.PreserveAspectFit
                width: warning_lights.width * 0.10
                height: warning_lights.height * 0.4
                opacity: brakesystemActive ? 1.0 : 0.3
                //anchors.verticalCenter: parent.verticalCenter
                //anchors.verticalCenterOffset: -20  // Move up by 10 pixels

                Behavior on opacity {
                    NumberAnimation { duration: 200 }
                }

                SequentialAnimation on scale {
                    running: brakesystemActive
                    loops: Animation.Infinite
                    NumberAnimation { to: 1.1; duration: 500 }
                    NumberAnimation { to: 1.0; duration: 500 }
                }
            }

            // Door Open Indicator
            Image {
                id: door_indicator
                source: door_openActive ?
                    "qrc:/ui/assets/door_on.svg" :
                    "qrc:/ui/assets/door_off.svg"
                fillMode: Image.PreserveAspectFit
                width: warning_lights.width * 0.10
                height: warning_lights.height * 0.4
                opacity: door_openActive ? 1.0 : 0.3
                //anchors.verticalCenter: parent.verticalCenter
                //anchors.verticalCenterOffset: -35  // Move up by 10 pixels
                Behavior on opacity {
                    NumberAnimation { duration: 200 }
                }

                SequentialAnimation on scale {
                    running: door_openActive
                    loops: Animation.Infinite
                    NumberAnimation { to: 1.1; duration: 500 }
                    NumberAnimation { to: 1.0; duration: 500 }
                }
            }

            // High Beam Indicator
            Image {
                id: highbeam_indicator
                source: high_beamActive ?
                    "qrc:/ui/assets/highbeam_on.svg" :
                    "qrc:/ui/assets/highbeam_off.svg"
                fillMode: Image.PreserveAspectFit
                width: warning_lights.width * 0.10
                height: warning_lights.height * 0.4
                opacity: high_beamActive ? 1.0 : 0.3
                //anchors.verticalCenter: parent.verticalCenter
                //anchors.verticalCenterOffset: -40  // Move up by 10 pixels
                Behavior on opacity {
                    NumberAnimation { duration: 200 }
                }

                SequentialAnimation on scale {
                    running: high_beamActive
                    loops: Animation.Infinite
                    NumberAnimation { to: 1.1; duration: 500 }
                    NumberAnimation { to: 1.0; duration: 500 }
                }
            }
        }
    }


    // Monitor gear changes to trigger animation
    onCurrentGearChanged: {
        gearChangeAnimation.running = true
    }


    // Timer to update the power value
    Timer {
        id: powerUpdateTimer
        interval: 5000  // Update every 5 seconds
        running: true
        repeat: true
        onTriggered: {
            currentPower = Math.floor(Math.random() * 101)
            currentPower_level = Math.floor(Math.random() * 100)
            currentOdometerValue = Math.floor(Math.random() * 350)
        }
    }
}
