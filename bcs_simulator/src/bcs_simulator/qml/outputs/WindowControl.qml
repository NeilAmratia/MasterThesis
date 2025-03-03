import QtQuick 2.12
import QtQuick.Controls 2.12

Item {
    id: windowControl
    width: parent.width
    height: parent.height

    // LED Colors
    readonly property color colorGlow: "#1d6d64"
    readonly property color colorWarning: "#d5232f"
    readonly property color colorMain: "#6affcd"
    readonly property color colorBright: "#ffffff"
    readonly property color colorLightGrey: "#888"
    readonly property color colorDarkGrey: "#333"

    // Slider for window position
    Rectangle {
        id: windowSliderRect
        width: parent.width * 0.7
        height: parent.height * 0.8
        anchors.left: parent.left
        anchors.verticalCenter: parent.verticalCenter
        color: "transparent"

        Slider {
            id: windowSlider
            orientation: Qt.Vertical
            from: 0
            to: 100
            stepSize: 1
            value: ros2Bridge.sliderPosition
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.leftMargin: 80
            anchors.topMargin: 50 
            height: 200 
            width: 10

            background: Rectangle {
                implicitWidth: 5
                implicitHeight: windowSlider.height
                x: (parent.width - width) / 2
                color: "white" 
                radius: 5

                Rectangle {
                    width: parent.width
                    height: (1 - windowSlider.visualPosition) * parent.height
                    color: "#17a81a"  
                    radius: 5
                    anchors.bottom: parent.bottom
                }
            }

            handle: Rectangle {
                x: (parent.width - width) / 2  
                y: windowSlider.visualPosition * (windowSlider.height - height)
                width: 20
                height: 20
                radius: 10
                color: "#17a81a"
                border.color: "black"
                border.width: 1
            }

            onValueChanged: {
                if (pressed) { 
                    ros2Bridge.sliderPosition = value
                }
                if (value === 100) {
                    ros2Bridge.publishWindowPositionUp(true);
                }
                if (value === 0) {
                    ros2Bridge.publishWindowPositionDown(true);
                }
                if (value !== 0 && value !== 100){
                    ros2Bridge.publishWindowPositionUp(false);
                    ros2Bridge.publishWindowPositionDown(false);
                }
            }
        }

        Text {
            id: windowPositionText
            text: qsTr("Window Position: ") + windowSlider.value + "%"
            font.pixelSize: 12
            color: "white"
            anchors.top: windowSlider.bottom
            anchors.topMargin: 15
            anchors.horizontalCenter: windowSlider.horizontalCenter
        }
    }

    // LED indicators (using Rectangles with rounded edges for a "LED" look)
    Rectangle {
        id: ledIndicatorsRect
        anchors.left: windowSliderRect.right
        anchors.leftMargin: 20
        height: parent.height
        color: "transparent"
        anchors.verticalCenter: parent.verticalCenter

        Column {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.verticalCenter: parent.verticalCenter
            spacing: 30

            // Window Up LED
            Rectangle {
                id: upLed
                width: 20
                height: 20
                radius: width / 2
                color: colorLightGrey
                border.color: colorDarkGrey
                border.width: 1

                Connections {
                    target: ros2Bridge  
                    function onWindowUpLedOnSignal() {  
                        upLed.color = "#17a81a"
                    }
                    function onWindowUpLedOffSignal() {  
                        upLed.color = "white"
                    }
                }

                Text {
                    text: qsTr("Window Up")
                    font.pixelSize: 12
                    color: "white"
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: upLed.bottom
                    anchors.topMargin: 7   
                }

            }


            // Window Down LED
            Rectangle {
                id: downLed
                width: 20
                height: 20
                radius: width / 2
                color: colorLightGrey
                border.color: colorDarkGrey
                border.width: 1

                Connections {
                    target: ros2Bridge  
                    function onWindowDownLedOnSignal() {  
                        downLed.color = "#17a81a"
                    }
                    function onWindowDownLedOffSignal() {  
                        downLed.color = "white"
                    }
                }

                Text {
                    text: qsTr("Window Down")
                    font.pixelSize: 12
                    color: "white"
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: downLed.bottom
                    anchors.topMargin: 7  
                }
            }



            // Finger Protection LED
            Rectangle {
                id: fingerProtectionLed
                width: 20
                height: 20
                radius: width / 2
                color: colorLightGrey
                border.color: colorDarkGrey
                border.width: 1
                Timer {
                    id: fingerProtectionLedResetTimer
                    interval: 1000  
                    running: false
                    repeat: false
                    onTriggered: {
                        fingerProtectionLed.color = colorLightGrey
                    }
                }

                Connections {
                    target: ros2Bridge  
                    function onFingerProtectionLedOnSignal() {  
                        fingerProtectionLed.color = "#17a81a"
                    }
                    function onFingerProtectionLedOffSignal() {  
                        fingerProtectionLed.color = "white"
                    }
                }

                Text {
                    text: qsTr("Finger Protection")
                    font.pixelSize: 12
                    color: "white"
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: fingerProtectionLed.bottom
                    anchors.topMargin: 7 
                }
            }

            // Central lock Window LED
            Rectangle {
                id: clsWindowUp
                width: 20
                height: 20
                radius: width / 2
                color: colorLightGrey
                border.color: colorDarkGrey
                border.width: 1

                Connections {
                    target: ros2Bridge  
                    function onCentralWindowUpLedOnSignal() {  
                        clsWindowUp.color = "#17a81a"
                    }
                    function onCentralWindowUpLedOffSignal() {  
                        clsWindowUp.color = "red"
                    }
                }

                Text {
                    text: qsTr("cls Window Up")
                    font.pixelSize: 12
                    color: "white"
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: clsWindowUp.bottom
                    anchors.topMargin: 7  
                }
            }
        }

    }
}
