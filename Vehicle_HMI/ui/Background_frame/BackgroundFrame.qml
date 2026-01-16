import QtQuick 2.15

Rectangle {
    id: bgFrame
    anchors.fill: parent
    color: "white"

    Image {
        id: background_frame
        anchors.fill: parent  // Fill the BackgroundFrame rectangle
        source: "qrc:/ui/assets/background_frame.png"
        fillMode: Image.Stretch

        // Optional: Add margins if needed
        // anchors.margins: 20
    }
}
