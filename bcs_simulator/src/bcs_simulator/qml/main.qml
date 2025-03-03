import QtQuick 2.12
import QtQuick.Controls 2.12
import QtGraphicalEffects 1.12
import QtQuick.Layouts 1.12
import "inputs"
import "outputs"

ApplicationWindow {
    id: mainWindow
    width: 1280
    height: 720
    minimumWidth: 1180
    minimumHeight: 663
    maximumWidth: 1280
    maximumHeight: 720
    visible: true
    title: qsTr("BCS Simulator")

    readonly property color colorGlow: "#1d6d64"
    readonly property color colorWarning: "#d5232f"
    readonly property color colorMain: "#6affcd"
    readonly property color colorBright: "#ffffff"
    readonly property color colorLightGrey: "#888"
    readonly property color colorDarkGrey: "#333"

    readonly property int fontSizeExtraSmall: Qt.application.font.pixelSize * 0.8
    readonly property int fontSizeMedium: Qt.application.font.pixelSize * 1.5
    readonly property int fontSizeLarge: Qt.application.font.pixelSize * 2
    readonly property int fontSizeExtraLarge: Qt.application.font.pixelSize * 5

    Rectangle {
        id: mainContainer
        anchors.fill: parent
        color: "black"

        // Input Panel
        Rectangle {
            id: inputPanel
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            width: parent.width / 3 
            color: "transparent"
            border.color: colorGlow 
            border.width: 3

            // HMI
            Rectangle {
                id: firstPanel
                anchors.left: parent.left
                anchors.top: parent.top
                anchors.right: parent.right
                anchors.margins: 30
                height: parent.height * 0.33
                color: "transparent"
                border.color: colorGlow
                border.width: 2


                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: parent.top
                    anchors.topMargin: 10
                    text: "HMI"
                    font.pixelSize: 16
                    font.bold: true
                    color: "white"
                }

                HMI {
                    id: hmi
                    anchors.fill: parent
                }
            }

            // Remote and central locking system
            Rectangle {
                id: secondPanel
                anchors.left: parent.left
                anchors.top: firstPanel.bottom
                anchors.right: parent.right
                height: parent.height * 0.25
                color: "transparent" 

                RemoteControl {
                    id: remoteControl
                    anchors.fill: parent
                }
            }

            // Alarm and car state
            Rectangle {
                id: thirdPanel
                anchors.left: parent.left
                anchors.top: secondPanel.bottom
                anchors.right: parent.right
                anchors.bottom: parent.bottom
                color: "transparent" 

                AlarmControl {
                    id: alarmControl
                    anchors.fill: parent
                }
            }
        }

        // Messages Log
        Rectangle {
            id: messagesLog
            anchors.left: inputPanel.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            width: parent.width / 3 
            color: "transparent" 

            ScrollView {
                id: scrollview
                width: parent.width
                anchors.fill: parent
                anchors.margins: 10

                ListView {
                    id: messageListView
                    anchors.fill: parent
                    model: ros2Bridge.messageModel
                    delegate: Text {
                        width: messagesLog.width
                        text: `${model.number}. [${model.timestamp}] ${model.type.toUpperCase()}: ${model.message}`
                        color: model.type === "outgoing" ? "#17a81a" : "white"
                        font.pixelSize: 14
                        wrapMode: Text.WordWrap
                    }

                    onCountChanged: {
                        positionViewAtEnd()
                    }

                    ScrollBar.vertical: ScrollBar {}
                }
            }
        }

        // Output Panel
        Rectangle {
            id: outputPanel
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            width: parent.width / 3
            color: "transparent" 
            border.color: colorGlow 
            border.width: 3

            // Left Section (WindowControl + MirrorControl)
            Rectangle {
                id: leftSection
                anchors.left: parent.left
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                width: parent.width * 0.67
                color: "transparent" 

                // WindowControl Section
                Rectangle {
                    id: windowControlSection
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.right: parent.right
                    anchors.margins: 10
                    height: parent.height * 0.45
                    color: "transparent" 
                    border.color: colorGlow 
                    border.width: 2

                    Text {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.top: parent.top
                        anchors.topMargin: 10
                        text: "WindowControl"
                        font.pixelSize: 16
                        font.bold: true
                        color: "white"
                    }

                    // Include the WindowControl QML file
                    WindowControl {
                        id: windowControl
                        anchors.fill: parent
                    }
                }

                // MirrorControl Section
                Rectangle {
                    id: mirrorControlSection
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.top: windowControlSection.bottom
                    anchors.bottom: parent.bottom
                    anchors.margins: 10
                    color: "transparent"
                    border.color: colorGlow 
                    border.width: 2

                    Text {
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.top: parent.top
                        anchors.topMargin: 10
                        text: "MirrorControl"
                        font.pixelSize: 16
                        font.bold: true
                        color: "white"
                    }

                    // Include the MirrorControl QML file
                    MirrorControl {
                        id: mirrorControl
                        anchors.fill: parent
                    }
                }
            }

            // Right Section (LedIndicators)
            Rectangle {
                id: ledIndicatorsSection
                anchors.right: parent.right
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                anchors.rightMargin: 10
                anchors.topMargin: 10
                anchors.bottomMargin: 10
                anchors.left: leftSection.right
                color: "transparent" 
                border.color: colorGlow 
                border.width: 2

                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: parent.top
                    anchors.topMargin: 10
                    text: "Indicators"
                    font.pixelSize: 16
                    font.bold: true
                    color: "white"
                }

                // Include the LedIndicators QML file
                LedIndicators {
                    id: ledIndicators
                    anchors.fill: parent
                    anchors.margins: 10
                }
            }
        }
    }
}
