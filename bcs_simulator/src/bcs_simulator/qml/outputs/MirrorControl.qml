import QtQuick 2.12
import QtQuick.Controls 2.12

Item {
    id: mirrorControl
    width: parent.width
    height: parent.height

    // LED Colors
    readonly property color colorGlow: "#17a81a"
    readonly property color colorWarning: "#d5232f"
    readonly property color colorMain: "#6affcd"
    readonly property color colorBright: "#ffffff"
    readonly property color colorLightGrey: "#888"
    readonly property color colorDarkGrey: "#333"

    property bool mirrorHeaterActive: false

    // Left Side (Mirror control sliders)
    Rectangle {
        id: mirrorSliderRect
        width: parent.width * 0.7
        height: parent.height * 0.8
        anchors.left: parent.left
        anchors.leftMargin: 15
        anchors.verticalCenter: parent.verticalCenter
        color: "transparent"

        // Horizontal Position Slider
        Rectangle {
            width: parent.width * 0.8
            height: parent.height * 0.5
            anchors.top: mirrorSliderRect.top
            color: "transparent"

            Slider {
                id: mirrorXSlider
                orientation: Qt.Horizontal
                from: 0
                to: 100
                stepSize: 1
                value: ros2Bridge.mirrorHorizontalPosition
                anchors.centerIn: parent
                width: parent.width
            
                background: Rectangle {
                    implicitWidth: mirrorXSlider.width
                    implicitHeight: 5
                    y: (parent.height - height) / 2
                    color: "white" 
                    radius: 5

                    Rectangle {
                        width: (mirrorXSlider.visualPosition) * parent.width
                        height: parent.height
                        color: "#17a81a"  
                        radius: 5
                        anchors.bottom: parent.bottom
                    }
                }

                handle: Rectangle {
                    x: mirrorXSlider.visualPosition * (mirrorXSlider.width - width)
                    y: (parent.height - height) / 2 
                    width: 20
                    height: 20
                    radius: 10
                    color: "#17a81a"
                    border.color: "black"
                    border.width: 1
                }

                onValueChanged: {
                    if (pressed) { 
                        ros2Bridge.mirrorHorizontalPosition = value
                    }
                    if (value === 100) {
                        ros2Bridge.publishEMPosRight(true);
                        ros2Bridge.publishEMPosLeft(false);
                    }
                    if (value === 0) {
                        ros2Bridge.publishEMPosRight(false);
                        ros2Bridge.publishEMPosLeft(true);
                    }
                    if (value === 0 && value === 100){
                        ros2Bridge.publishEMPosRight(false);
                        ros2Bridge.publishEMPosLeft(false);
                    }
                }
            }

            Text {
                text: qsTr("Mirror X Position: ") + mirrorXSlider.value + "%"
                font.pixelSize: 12
                color: "white"
                anchors.top: mirrorXSlider.bottom
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.topMargin: 10
            }
        }

        // Vertical Position Slider
        Rectangle {
            width: parent.width * 0.8
            height: parent.height * 0.5
            anchors.bottom: mirrorSliderRect.bottom
            color: "transparent"

            Slider {
                id: mirrorYSlider
                orientation: Qt.Vertical
                from: 0
                to: 100
                stepSize: 1
                value: ros2Bridge.mirrorVerticalPosition
                anchors.centerIn: parent
                height: parent.height * 0.9

                background: Rectangle {
                    implicitWidth: 5
                    implicitHeight: mirrorYSlider.height
                    x: (parent.width - width) / 2
                    color: "white" 
                    radius: 5

                    Rectangle {
                        width: parent.width
                        height: (1 - mirrorYSlider.visualPosition) * parent.height
                        color: "#17a81a"  
                        radius: 5
                        anchors.bottom: parent.bottom
                    }
                }

                handle: Rectangle {
                    x: (parent.width - width) / 2  
                    y: mirrorYSlider.visualPosition * (mirrorYSlider.height - height)
                    width: 20
                    height: 20
                    radius: 10
                    color: "#17a81a"
                    border.color: "black"
                    border.width: 1
                    anchors.horizontalCenter: mirrorYSlider.horizontalCenter
                }

                onValueChanged: {
                    if (pressed) { 
                        ros2Bridge.mirrorVerticalPosition = value
                    }
                    if (value === 100) {
                        ros2Bridge.publishEMPosTop(true);
                        ros2Bridge.publishEMPosBottom(false);
                    }
                    if (value === 0) {
                        ros2Bridge.publishEMPosTop(false);
                        ros2Bridge.publishEMPosBottom(true);
                    }
                    if (value === 0 && value === 100){
                        ros2Bridge.publishEMPosTop(false);
                        ros2Bridge.publishEMPosBottom(false);
                    }
                }
            }

            Text {
                text: qsTr("Mirror Y Position: ") + mirrorYSlider.value + "%"
                font.pixelSize: 12
                color: "white"
                anchors.top: mirrorYSlider.bottom
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.topMargin: 10
            }
        }

    }

    // Right Side (LED indicators)
    Rectangle {
        id: ledIndicatorsRect
        anchors.left: mirrorSliderRect.right
        anchors.leftMargin: 5
        height: parent.height
        color: "transparent"
        anchors.verticalCenter: parent.verticalCenter

        Image {
            id: mirrorHeaterImage
            anchors.top: parent.top
            anchors.margins: 35
            anchors.horizontalCenter: parent.horizontalCenter
            source: mirrorHeaterActive ? "qrc:/assets/mirror_heating_on.png" : "qrc:/assets/mirror_heating_off.png" 
            width: 24
            height: 24

            Connections {
                    target: ros2Bridge  
                    function onMirrorHeatingOnSignal() {  
                        mirrorHeaterActive = true
                    }
                    function onMirrorHeatingOffSignal() {  
                        mirrorHeaterActive = false
                    }
                }
        }

        Text {
            id: mirrorHeaterText
            text: "Heater"
            font.pixelSize: 12
            color: "white"
            anchors.top: mirrorHeaterImage.bottom
            anchors.topMargin: 7
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Column {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: mirrorHeaterText.bottom
            anchors.topMargin: 10
            spacing: 30

            // Mirror Up
            Rectangle {
                id: mirrorUp
                width: 20
                height: 20
                radius: width / 2
                color: colorLightGrey
                border.color: colorDarkGrey
                border.width: 1
                Timer {
                    id: mirrorUpResetTimer
                    interval: 1500  
                    running: false
                    repeat: false
                    onTriggered: {
                        mirrorUp.color = colorLightGrey
                    }
                }

                Connections {
                    target: ros2Bridge  
                    function onMirrorUpLedOnSignal() {  
                        mirrorUp.color = "#17a81a"
                    }
                    function onMirrorUpLedOffSignal() {  
                        mirrorUp.color = colorLightGrey
                    }
                }

                Text {
                    text: qsTr("Mirror Up")
                    font.pixelSize: 12
                    color: "white"
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: mirrorUp.bottom
                    anchors.topMargin: 7
                }
            }

            // Mirror Down
            Rectangle {
                id: mirrorDown
                width: 20
                height: 20
                radius: width / 2
                color: colorLightGrey
                border.color: colorDarkGrey
                border.width: 1
                Timer {
                    id: mirrorDownResetTimer
                    interval: 1500  
                    running: false
                    repeat: false
                    onTriggered: {
                        mirrorDown.color = colorLightGrey
                    }
                }

                Connections {
                    target: ros2Bridge  
                    function onMirrorDownLedOnSignal() {  
                        mirrorDown.color = "#17a81a"
                    }
                    function onMirrorDownLedOffSignal() {  
                        mirrorDown.color = colorLightGrey
                    }
                }

                Text {
                    text: qsTr("Mirror Down")
                    font.pixelSize: 12
                    color: "white"
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: mirrorDown.bottom
                    anchors.topMargin: 7
                }
            }

            // Mirror Left
            Rectangle {
                id: mirrorLeft
                width: 20
                height: 20
                radius: width / 2
                color: colorLightGrey
                border.color: colorDarkGrey
                border.width: 1
                Timer {
                    id: mirrorLeftResetTimer
                    interval: 1500  
                    running: false
                    repeat: false
                    onTriggered: {
                        mirrorLeftLed.color = colorLightGrey
                    }
                }

                Connections {
                    target: ros2Bridge  
                    function onMirrorLeftLedOnSignal() {  
                        mirrorLeft.color = "#17a81a"
                    }
                    function onMirrorLeftLedOffSignal() {  
                        mirrorLeft.color = colorLightGrey
                    }
                }

                Text {
                    text: qsTr("Mirror Left")
                    font.pixelSize: 12
                    color: "white"
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: mirrorLeft.bottom
                    anchors.topMargin: 7
                }
            }

            // Mirror Right
            Rectangle {
                id: mirrorRight
                width: 20
                height: 20
                radius: width / 2
                color: colorLightGrey
                border.color: colorDarkGrey
                border.width: 1
                Timer {
                    id: mirrorRightResetTimer
                    interval: 1500  
                    running: false
                    repeat: false
                    onTriggered: {
                        mirrorRight.color = colorLightGrey
                    }
                }

                Connections {
                    target: ros2Bridge  
                    function onMirrorRightLedOnSignal() {  
                        mirrorRight.color = "#17a81a"
                    }
                    function onMirrorRightLedOffSignal() {  
                        mirrorRight.color = colorLightGrey
                    }
                }

                Text {
                    text: qsTr("Mirror Right")
                    font.pixelSize: 12
                    color: "white"
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: mirrorRight.bottom
                    anchors.topMargin: 7
                }
            }

            // Heater
            Rectangle {
                id: mirrorHeater
                width: 20
                height: 20
                radius: width / 2
                color: colorLightGrey
                border.color: colorDarkGrey
                border.width: 1
                Timer {
                    id: mirrorHeaterResetTimer
                    interval: 1500  
                    running: false
                    repeat: false
                    onTriggered: {
                        mirrorHeater.color = colorLightGrey
                    }
                }

                Connections {
                    target: ros2Bridge  
                    function onMirrorHeaterLedOnSignal() {  
                        mirrorHeater.color = "#17a81a"
                    }
                    function onMirrorHeaterLedOffSignal() {  
                        mirrorHeater.color = colorLightGrey
                    }
                }

                Text {
                    text: qsTr("Heater")
                    font.pixelSize: 12
                    color: "white"
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: mirrorHeater.bottom
                    anchors.topMargin: 7
                }
            }
        }
    }
}
