import QtQuick 2.12
import QtQuick.Controls 2.12

Item {
    id: alarmControl
    width: parent.width * 0.85
    height: parent.height

    property bool alarmTriggerIsPressed: false
    property bool alarmIMTriggerIsPressed: false
    property bool carDrivingIsPressed: false
    property bool loadConfigurationIsPressed: false
    property bool runConfigurationIsOn: false
    property bool isTestRunning: false

    // Alarm Section
    Rectangle {
        id: alarmControls
        anchors.left: parent.left
        anchors.leftMargin: 30
        width: parent.width * 0.45
        anchors.topMargin: 10
        height: parent.height * 0.45
        color: "transparent"
        border.color: colorGlow   
        border.width: 2

        Text {
            id: alarmSystemText
            text: "Alarm System"
            font.pixelSize: 14
            font.bold: true
            color: "white"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            anchors.margins: 10
        }

        Rectangle {
            id: alarmTrigger
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: alarmSystemText.bottom
            anchors.topMargin: 9
            width: 120
            height: 25
            radius: 10 
            color: alarmTriggerIsPressed ? colorGlow : "transparent" 
            border.color: colorGlow
            border.width: 2

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    alarmTriggerIsPressed = true
                }
                onReleased: {
                    alarmTriggerIsPressed = false
                    ros2Bridge.publishAlarmTriggered()
                }
            }

            Text {
                anchors.centerIn: parent
                text: "Alarm Trigger"
                font.pixelSize: 8
                font.bold: true
                color: "white"
            }
        }

        Rectangle {
            id: alarmIMTrigger
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: alarmTrigger.bottom
            anchors.topMargin: 7
            width: 120
            height: 25
            radius: 10 
            color: alarmIMTriggerIsPressed ? colorGlow : "transparent" 
            border.color: colorGlow
            border.width: 2

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    alarmIMTriggerIsPressed = true
                }
                onReleased: {
                    alarmIMTriggerIsPressed = false
                    ros2Bridge.publishAlarmIMTriggered()
                }
            }

            Text {
                anchors.centerIn: parent
                text: "Alarm Interior"
                font.pixelSize: 8
                font.bold: true
                color: "white"
            }
        }
    }

    // Testing Section
    Rectangle {
        id: test
        anchors.left: parent.left
        anchors.leftMargin: 30
        width: parent.width * 0.45
        anchors.top: alarmControls.bottom
        anchors.topMargin: 10
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 30
        color: "transparent"
        border.color: colorGlow   
        border.width: 2

        Text {
            id: testingText
            text: "Automated Testing"
            font.pixelSize: 14
            font.bold: true
            color: "white"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            anchors.margins: 10
        }

        Rectangle {
            id: loadConfiguration
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: testingText.bottom
            anchors.topMargin: 7
            width: 120
            height: 25
            radius: 10 
            color: loadConfigurationIsPressed ? colorGlow : "transparent" 
            border.color: colorGlow
            border.width: 2

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    loadConfigurationIsPressed = true
                }
                onReleased: {
                    loadConfigurationIsPressed = false
                    testManager.loadConfiguration();
                }
            }

            Text {
                anchors.centerIn: parent
                text: "Load Configuration"
                font.pixelSize: 8
                font.bold: true
                color: "white"
            }
        }

        Rectangle {
            id: runConfiguration
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: loadConfiguration.bottom
            anchors.topMargin: 5
            width: 120
            height: 25
            radius: 10 
            color: runConfigurationIsOn ? colorGlow : "transparent" 
            border.color: colorGlow
            border.width: 2

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    runConfigurationIsOn = true
                }
                onReleased: {
                    if (!isTestRunning) {
                        isTestRunning = true;
                    } else {
                        isTestRunning = false;
                    }
                    runConfigurationIsOn = false
                }

            }

            Text {
                id : testText
                anchors.centerIn: parent
                text: isTestRunning ? "Stop Tests" : "Start Tests"
                font.pixelSize: 8
                font.bold: true
                color: "white"
            }
            

            Label {
                visible: ros2Bridge.inputsLocked
                text: "Manual inputs disabled during testing"
                color: "red"
                width: parent.width
            }
        }
    }

    // Car state selection
    Rectangle {
        id: carState
        anchors.left: alarmControls.right
        anchors.leftMargin: 10
        anchors.top: parent.top
        anchors.topMargin: 10
        anchors.right: parent.right
        anchors.rightMargin: 30
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 30
        color: "transparent"
        border.color: colorGlow   
        border.width: 2

        Text {
            id: carCurrentState
            text: "Current Car State"
            font.pixelSize: 14
            font.bold: true
            color: "white"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            anchors.margins: 10
        }

        Column {
            spacing: 10
            anchors.verticalCenter: carState.verticalCenter
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.rightMargin: 0
            anchors.bottomMargin: 10
            anchors.topMargin: 10
            anchors.leftMargin: 0


            SwitchDelegate {
                id: carDriving
                text: qsTr("Car Drives")
                checked: true
                width: parent.width
                height: 25
                anchors.margins: 0

                contentItem: Text {
                    font.pixelSize: 10
                    font.bold: true
                    text: carDriving.text
                    opacity: enabled ? 1.0 : 0.3
                    color: "white"
                    elide: Text.ElideRight
                    verticalAlignment: Text.AlignVCenter
                }

                indicator: Rectangle {
                    implicitWidth: 30
                    implicitHeight: 16
                    x: carDriving.width - width - 10
                    y: parent.height / 2 - height / 2
                    radius: 13
                    color: carDriving.checked ? "#17a81a" : "transparent"
                    border.color: carDriving.checked ? "#17a81a" : "#cccccc"

                    Rectangle {
                        x: carDriving.checked ? parent.width - width : 0
                        width: 16
                        height: 16
                        radius: 9
                        color: carDriving.down ? "#cccccc" : "#ffffff"
                        border.color: carDriving.checked ? (carDriving.down ? "#17a81a" : "#21be2b") : "#999999"
                    }
                }

                onCheckedChanged: {
                    if (carDriving.checked) {
                        ros2Bridge.carDriving(true)
                    } else if (!carDriving.checked) {
                        ros2Bridge.carDriving(false)
                    }
                }
            }

            SwitchDelegate {
                id: carDoor
                text: qsTr("Car Door")
                width: parent.width
                height: 25
                anchors.margins: 0

                contentItem: Text {
                    text: carDoor.text
                    font.pixelSize: 10
                    font.bold: true
                    opacity: enabled ? 1.0 : 0.3
                    color: "white"
                    elide: Text.ElideRight
                    verticalAlignment: Text.AlignVCenter
                }

                indicator: Rectangle {
                    implicitWidth: 30
                    implicitHeight: 16
                    x: carDoor.width - width  - 10
                    y: parent.height / 2 - height / 2
                    radius: 13
                    color: carDoor.checked ? "#17a81a" : "transparent"
                    border.color: carDoor.checked ? "#17a81a" : "#cccccc"

                    Rectangle {
                        x: carDoor.checked ? parent.width - width : 0
                        width: 16
                        height: 16
                        radius: 9
                        color: carDoor.down ? "#cccccc" : "#ffffff"
                        border.color: carDoor.checked ? (carDoor.down ? "#17a81a" : "#21be2b") : "#999999"
                    }
                }

                onCheckedChanged: {
                    if (carDoor.checked) {
                        ros2Bridge.carDoor(true)
                    } else if (!carDoor.checked) {
                        ros2Bridge.carDoor(false)
                    }
                }
            }

            SwitchDelegate {
                id: fingerProtection
                text: qsTr("Finger Detected")
                width: parent.width
                height: 25
                anchors.margins: 0

                contentItem: Text {
                    text: fingerProtection.text
                    font.pixelSize: 10
                    font.bold: true
                    opacity: enabled ? 1.0 : 0.3
                    color: "white"
                    elide: Text.ElideRight
                    verticalAlignment: Text.AlignVCenter
                }

                indicator: Rectangle {
                    implicitWidth: 30
                    implicitHeight: 16
                    x: fingerProtection.width - width - 10
                    y: parent.height / 2 - height / 2
                    radius: 13
                    color: fingerProtection.checked ? "#17a81a" : "transparent"
                    border.color: fingerProtection.checked ? "#17a81a" : "#cccccc"

                    Rectangle {
                        x: fingerProtection.checked ? parent.width - width : 0
                        width: 16
                        height: 16
                        radius: 9
                        color: fingerProtection.down ? "#cccccc" : "#ffffff"
                        border.color: fingerProtection.checked ? (fingerProtection.down ? "#17a81a" : "#21be2b") : "#999999"
                    }
                }

                Connections {
                    target: ros2Bridge
                    function onWindowMoveDownSignal() {
                        fingerProtection.checked = false
                    }
                    function onWindowAutoMoveDownSignal() {
                        fingerProtection.checked = false
                    }
                }

                onCheckedChanged: {
                    if (fingerProtection.checked) {
                        ros2Bridge.fingerDetected(true)
                    } else if (!fingerProtection.checked) {
                        ros2Bridge.fingerDetected(false)
                    }
                }
            }

            SwitchDelegate {
                id: mirrorCold
                text: qsTr("MIrror Cold")
                width: parent.width
                height: 25
                anchors.margins: 0

                contentItem: Text {
                    text: mirrorCold.text
                    font.pixelSize: 10
                    font.bold: true
                    opacity: enabled ? 1.0 : 0.3
                    color: "white"
                    elide: Text.ElideRight
                    verticalAlignment: Text.AlignVCenter
                }

                indicator: Rectangle {
                    implicitWidth: 30
                    implicitHeight: 16
                    x: mirrorCold.width - width - 10
                    y: parent.height / 2 - height / 2
                    radius: 13
                    color: mirrorCold.checked ? "#17a81a" : "transparent"
                    border.color: mirrorCold.checked ? "#17a81a" : "#cccccc"

                    Rectangle {
                        x: mirrorCold.checked ? parent.width - width : 0
                        width: 16
                        height: 16
                        radius: 9
                        color: mirrorCold.down ? "#cccccc" : "#ffffff"
                        border.color: mirrorCold.checked ? (mirrorCold.down ? "#17a81a" : "#21be2b") : "#999999"
                    }
                }
                Connections {
                    target: ros2Bridge
                    function onMirrorHeatingOnSignal() {
                        mirrorCold.checked = false
                    }
                }

                onCheckedChanged: {
                    if (mirrorCold.checked) {
                        ros2Bridge.publishEMCold(true)
                    } else if (!mirrorCold.checked) {
                        ros2Bridge.publishEMCold(false)
                    }
                }
            }
        }
    }
}

