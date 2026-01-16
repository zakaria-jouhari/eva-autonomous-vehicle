import QtQuick 2.15
import QtQuick.Controls 2.15

Rectangle {
    id: top_bar
    anchors {
        top: parent.top
        horizontalCenter: parent.horizontalCenter
    }
    //anchors.topMargin: parent.parent.height * 0.16
     anchors.topMargin: parent.parent.height * 0.05
    radius: parent.parent.height * 0.03
    color: "black"
    height: parent.height / 10
    width: parent.width * 0.4
    property int currentWeather: 0

    // Current date and time
    Row {
        id: dateTimeRow
        anchors.left: parent.left
        anchors.leftMargin: parent.width*0.05
        anchors.verticalCenter: parent.verticalCenter
        spacing: parent.width*0.03

        // Date (Short format: "Nov29")
        Label {
            id: dateLabel
            text: Qt.formatDateTime(new Date(), "MMMdd")
            color: "white"
            font.pixelSize: parent.parent.height * 0.22
        }

        // Time (12-hour format: "11:47AM")
        Label {
            id: timeLabel
            text: Qt.formatDateTime(new Date(), "hh:mmAP")
            color: "white"
            font.pixelSize: parent.parent.height * 0.22
        }
    }

    // User_id
    Row {
        id: userRow
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        spacing: parent.width*0.03

        // Username
        Label {
            id: userLabel
            text: "User"
            color: "white"
            font.pixelSize: parent.parent.height * 0.4
        }
    }

    // Weather and connectivity info
    Row {
        id: connectivityRow
        anchors.right: parent.right
        anchors.rightMargin: 15
        anchors.verticalCenter: parent.verticalCenter
        spacing: parent.width*0.03

        // Weather status - FIXED: Use Row for icon + temperature
        Row {
            id: weatherRow
            spacing: parent.width * 0.01  // Small spacing between icon and text
            anchors.verticalCenter: parent.verticalCenter

            Image {
                id: weatherIcon
                source: "qrc:/ui/assets/weather.png"
                width: parent.parent.parent.height * 0.4
                height: parent.parent.parent.height * 0.4
                fillMode: Image.PreserveAspectFit
                anchors.verticalCenter: parent.verticalCenter
            }

            Label {
                id: weatherValue
                text: currentWeather + "°C"
                color: "white"
                font.pixelSize: parent.parent.parent.height * 0.25
                anchors.verticalCenter: parent.verticalCenter
            }
        }

        // Bluetooth status
        Image {
            id: bluetoothIcon
            source: "qrc:/ui/assets/bluetooth.png"
            width: parent.parent.height * 0.35
            height: parent.parent.height * 0.35
            fillMode: Image.PreserveAspectFit
            anchors.verticalCenter: parent.verticalCenter
        }

        // WiFi status
        Image {
            id: wifiIcon
            source: "qrc:/ui/assets/wifi.png"
            width: parent.parent.height * 0.35
            height: parent.parent.height * 0.35
            fillMode: Image.PreserveAspectFit
            anchors.verticalCenter: parent.verticalCenter
        }
    }

    // Timer to update time every second
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: {
            timeLabel.text = Qt.formatDateTime(new Date(), "hh:mmAP")
            dateLabel.text = Qt.formatDateTime(new Date(), "MMMdd")
        }
    }

    // Timer to update the weather
    Timer {
        id: weatherTimer
        interval: 3000  // Update every 2 seconds (more realistic)
        running: true
        repeat: true
        onTriggered: {
            // Generate random temperature between 15 and 35°C
            currentWeather = 15 + Math.floor(Math.random() * 21)
        }
    }

    // Start with an initial random value
    Component.onCompleted: {
        currentWeather = 15 + Math.floor(Math.random() * 21)
    }
}
