import QtQuick 2.12
import QtQuick.Controls 2.12

Item {
    id: ledIndicators
    width: parent.width
    height: parent.height

    // LED Colors
    readonly property color colorGlow: "#1d6d64"
    readonly property color colorWarning: "#d5232f"
    readonly property color colorMain: "#6affcd"
    readonly property color colorBright: "#ffffff"
    readonly property color colorLightGrey: "#888"
    readonly property color colorDarkGrey: "#333"

    property bool carLockImage: false
    property bool clsLockImage: false
    property bool alarmImage: false
    property bool alarmActiveImage: false
    property bool alarmIMImage: false


    Rectangle {
        id: ledIndicatorsMain
        anchors.fill: parent
        color: "transparent"

        Rectangle {
            id: carLock1
            anchors.top : parent.top
            anchors.topMargin : 40
            anchors.horizontalCenter: parent.horizontalCenter

            Image {
                id: carLockImage
                anchors.top : parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                source: carLockImage ? "qrc:/assets/lock.png" : "qrc:/assets/unlock.png"
                width: 24
                height: 24

                Connections {
                    target: ros2Bridge  
                    function onCarLockedSignal() {  
                        carLockImage = true
                    }
                    function onCarUnlockedSignal() {  
                        carLockImage = false
                    }
                }
            }

            Text {
                text: "Car Lock"
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: carLockImage.bottom
                anchors.topMargin: 10
            }
        }

        Rectangle {
            id: clsLock1
            anchors.top : carLock1.bottom
            anchors.topMargin : 65
            anchors.horizontalCenter: parent.horizontalCenter

            Image {
                id: clsLockImage
                anchors.top : parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                source: clsLockImage ? "qrc:/assets/lock.png" : "qrc:/assets/unlock.png"
                width: 24
                height: 24

                Connections {
                    target: ros2Bridge  
                    function onClsLockSignal() {  
                        clsLockImage = true
                    }
                    function onClsUnlockSignal() {  
                        clsLockImage = false
                    }
                }
            }

            Text {
                text: "Central Lock"
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: clsLockImage.bottom
                anchors.topMargin: 10
            }
        }

        Rectangle {
            id: alarmAS1
            anchors.top : clsLock1.bottom
            anchors.topMargin : 65
            anchors.horizontalCenter: parent.horizontalCenter

            Image {
                id: alarmASImage
                anchors.top : parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                source: alarmImage ? "qrc:/assets/alarm_on.png" : "qrc:/assets/alarm_off.png"
                width: 24
                height: 24

                Connections {
                    target: ros2Bridge  
                    function onAlarmSignalOnSignal() {  
                        alarmImage = true
                    }
                    function onAlarmSignalOffSignal() {  
                        alarmImage = false
                    }
                }
            }

            Text {
                text: "Alarm AS"
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: alarmASImage.bottom
                anchors.topMargin: 10
            }
        }
        Rectangle {
            id: alarmIs1
            anchors.top : alarmAS1.bottom
            anchors.topMargin : 65
            anchors.horizontalCenter: parent.horizontalCenter
            
            Image {
                id: alarmIsActiveImage
                anchors.top : parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                source: alarmActiveImage ? "qrc:/assets/alarm_on.png" : "qrc:/assets/alarm_off.png"
                width: 24
                height: 24

                Connections {
                    target: ros2Bridge  
                    function onAlarmActivationOnSignal() {  
                        alarmActiveImage = true
                    }
                    function onAlarmActivationOffSignal() {  
                        alarmActiveImage = false
                    }
                }
            }

            Text {
                text: "Alarm Active"
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: alarmIsActiveImage.bottom
                anchors.topMargin: 10
            }
        }

        Rectangle {
            id: alarmOfIM1
            anchors.top : alarmIs1.bottom
            anchors.topMargin : 65
            anchors.horizontalCenter: parent.horizontalCenter

            Image {
                id: alarmOfIMImage
                anchors.top : parent.top
                anchors.horizontalCenter: parent.horizontalCenter
                source: alarmIMImage ? "qrc:/assets/alarm_on.png" : "qrc:/assets/alarm_off.png"
                width: 24
                height: 24

                Connections {
                    target: ros2Bridge  
                    function onAlarmInteriorOnSignal() {  
                        alarmIMImage = true
                    }
                    function onAlarmInteriorOffSignal() {  
                        alarmIMImage = false
                    }
                }
            }

            Text {
                text: "Alarm IM"
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: alarmOfIMImage.bottom
                anchors.topMargin: 10
            }
            
        }
        // Alarm detected
        Rectangle {
            id: led1
            width: 20
            height: 20
            anchors.top : alarmOfIM1.bottom
            anchors.topMargin : 65
            anchors.horizontalCenter: parent.horizontalCenter
            radius: width / 2
            color: colorLightGrey
            border.color: colorDarkGrey
            border.width: 1

            Connections {
                target: ros2Bridge  
                function onAlarmDetectedLedOnSignal() {  
                    led1.color = "#17a81a"
                }
                function onAlarmDetectedLedOffSignal() {  
                    led1.color = "red"
                }
            }

            Text {
                text: qsTr("Alarm Detected")
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: led1.bottom
                anchors.topMargin: 10
            }
        }

        // Alarm AS signal
        Rectangle {
            id: led2
            width: 20
            height: 20
            anchors.top : led1.bottom
            anchors.topMargin : 30
            anchors.horizontalCenter: parent.horizontalCenter
            radius: width / 2
            color: colorLightGrey
            border.color: colorDarkGrey
            border.width: 1

            Connections {
                target: ros2Bridge  
                function onAlarmSignalLedOnSignal() {  
                    led2.color = "#17a81a"
                }
                function onAlarmSignalLedOffSignal() {  
                    led2.color = "red"
                }
            }

            Text {
                text: qsTr("Alarm Signal")
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: led2.bottom
                anchors.topMargin: 10
            }
        }

        // Alarm interior
        Rectangle {
            id: led3
            width: 20
            height: 20
            anchors.top : led2.bottom
            anchors.topMargin : 30
            anchors.horizontalCenter: parent.horizontalCenter
            radius: width / 2
            color: colorLightGrey
            border.color: colorDarkGrey
            border.width: 1

            Connections {
                target: ros2Bridge  
                function onAlarmInteriorLedOnSignal() {  
                    led3.color = "#17a81a"
                }
                function onAlarmInteriorLedOffSignal() {  
                    led3.color = "red"
                }
            }

            Text {
                text: qsTr("Alarm Interior")
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: led3.bottom
                anchors.topMargin: 10
            }
        }

        // Alarm active
        Rectangle {
            id: led4
            width: 20
            height: 20
            anchors.top : led3.bottom
            anchors.topMargin : 30
            anchors.horizontalCenter: parent.horizontalCenter
            radius: width / 2
            color: colorLightGrey
            border.color: colorDarkGrey
            border.width: 1

            Connections {
                target: ros2Bridge  
                function onAlarmActivationLedOnSignal() {  
                    led4.color = "#17a81a"
                }
                function onAlarmActivationLedOffSignal() {  
                    led4.color = "red"
                }
            }

            Text {
                text: qsTr("Alarm Activation")
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: led4.bottom
                anchors.topMargin: 10
            }
        }

        //  7
        Rectangle {
            id: led7
            width: 20
            height: 20
            anchors.top : led4.bottom
            anchors.topMargin : 30
            anchors.horizontalCenter: parent.horizontalCenter
            radius: width / 2
            color: colorLightGrey
            border.color: colorDarkGrey
            border.width: 1

            Connections {
                target: ros2Bridge  
                function onCentralLockLedOnSignal() {  
                    led7.color = "#17a81a"
                }
                function onCentralLockLedOffSignal() {  
                    led7.color = "red"
                }
            }

            Text {
                text: qsTr("Central Lock")
                font.pixelSize: 12
                color: "white"
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: led7.bottom
                anchors.topMargin: 10
            }
        }
        
    }
        
}
