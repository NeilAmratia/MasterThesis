import QtQuick 2.12
import QtQuick.Controls 2.12
import QtGraphicalEffects 1.12

Item {
    id: hmi
    width: parent.width
    height: parent.height

    property bool isWindowUpPressed: false
    property bool isWindowDownPressed: false
    property bool isMirrorUpPressed: false
    property bool isMirrorDownPressed: false
    property bool isMirrorLeftPressed: false
    property bool isMirrorRightPressed: false
    property bool alarmActivateIsPressed: false
    property bool alarmDeactivateIsPressed: false
    property bool alarmConfirmedIsPressed: false
    property bool releaseWindowUpIsPressed: false
    property bool releaseWindowDownIsPressed: false
    
    // Window Controls Section
    Rectangle {
        id: windowInput
        width: parent.width / 4
        height: parent.height 
        anchors.left: parent.left
        color: "transparent"

        Image {
            id: windowUpImage
            anchors.top: parent.top
            anchors.margins: 75
            anchors.horizontalCenter: parent.horizontalCenter
            source: isWindowUpPressed ? "qrc:/assets/window_up_on.png" : "qrc:/assets/window_up_off.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isWindowUpPressed = true
                    ros2Bridge.publishWindowUp()
                }
                onReleased: {
                    isWindowUpPressed = false
                    ros2Bridge.publishReleaseWindowUp()
                }
            }
        }

        Text {
            id: windowText
            text: "Window"
            font.pixelSize: 10
            font.bold: true
            color: "white"
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Image {
            id: windowDownImage
            anchors.bottom: parent.bottom
            anchors.margins: 75
            anchors.horizontalCenter: parent.horizontalCenter
            source: isWindowDownPressed ? "qrc:/assets/window_down_on.png" : "qrc:/assets/window_down_off.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isWindowDownPressed = true
                    ros2Bridge.publishWindowDown()
                }
                onReleased: {
                    isWindowDownPressed = false
                    ros2Bridge.publishReleaseWindowDown()
                }
            }
        }
        
    }

    // Mirror Controls Section
    Rectangle {
        id: mirrorControls
        anchors.left: windowInput.right
        anchors.verticalCenter: parent.verticalCenter
        width: parent.width / 3
        height: parent.height / 2.5
        radius: width / 2 
        color: "transparent"
        border.color: colorGlow   
        border.width: 2

        Image {
            id: mirrorUpImage
            anchors.top: parent.top
            anchors.margins: 10
            anchors.horizontalCenter: parent.horizontalCenter
            source: isMirrorUpPressed ? "qrc:/assets/mirror_up_off.png" : "qrc:/assets/mirror_up_on.png"
            width: 28
            height: 28

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isMirrorUpPressed = true
                    ros2Bridge.publishMirrorUp()
                }
                onReleased: {
                    isMirrorUpPressed = false
                }
            }
        }

        Image {
            id: mirrorDownImage
            anchors.bottom: parent.bottom
            anchors.margins: 10
            anchors.horizontalCenter: parent.horizontalCenter
            source: isMirrorDownPressed ? "qrc:/assets/mirror_down_off.png" : "qrc:/assets/mirror_down_on.png"
            width: 28
            height: 28

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isMirrorDownPressed = true
                    ros2Bridge.publishMirrorDown()
                }
                onReleased: {
                    isMirrorDownPressed = false
                }
            }
        }

        Image {
            id: mirrorLeftImage
            anchors.left: parent.left
            anchors.margins: 10
            anchors.verticalCenter: parent.verticalCenter
            source: isMirrorLeftPressed ? "qrc:/assets/mirror_left_off.png" : "qrc:/assets/mirror_left_on.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isMirrorLeftPressed = true
                    ros2Bridge.publishMirrorLeft()
                }
                onReleased: {
                    isMirrorLeftPressed = false
                }
            }
        }

        Image {
            id: mirrorRightImage
            anchors.right: parent.right
            anchors.margins: 10
            anchors.verticalCenter: parent.verticalCenter
            source: isMirrorRightPressed ? "qrc:/assets/mirror_right_off.png" : "qrc:/assets/mirror_right_on.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isMirrorRightPressed = true
                    ros2Bridge.publishMirrorRight()
                }
                onReleased: {
                    isMirrorRightPressed = false
                }
            }
        }

        Text {
            id: mirrorText
            text: "Mirror"
            font.pixelSize: 10
            font.bold: true
            color: "white"
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }
        
    }

    // Alarm Controls Section
    Rectangle {
        id: alarmControls
        anchors.left: mirrorControls.right
        anchors.right: parent.right
        height: parent.height
        color: "transparent"

        Rectangle {
            id: alamActivate
            anchors.bottom: alamDeactivate.top
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottomMargin: 10
            width: 120
            height: 25
            radius: 10 
            color: alarmActivateIsPressed ? colorGlow : "transparent" 
            border.color: colorGlow
            border.width: 2

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    alarmActivateIsPressed = true
                }
                onReleased: {
                    alarmActivateIsPressed = false
                    ros2Bridge.publishAlarmActivate()
                }
            }

            Text {
                anchors.centerIn: parent
                text: "Activate Alarm"
                font.pixelSize: 8
                font.bold: true
                color: "white"
            }
        }

        Rectangle {
            id: alamDeactivate
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
            width: 120
            height: 25
            radius: 10 
            color: alarmDeactivateIsPressed ? colorGlow : "transparent" 
            border.color: colorGlow
            border.width: 2

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    alarmDeactivateIsPressed = true
                }
                onReleased: {
                    alarmDeactivateIsPressed = false
                    ros2Bridge.publishAlarmDeactivate()
                }
            }

            Text {
                anchors.centerIn: parent
                text: "Deactivate Alarm"
                font.pixelSize: 8
                font.bold: true
                color: "white"
            }
        }

        Rectangle {
            id: alarmConfirm
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: alamDeactivate.bottom
            anchors.topMargin: 10
            width: 120
            height: 25
            radius: 10 
            color: alarmConfirmedIsPressed ? colorGlow : "transparent" 
            border.color: colorGlow
            border.width: 2

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    alarmConfirmedIsPressed = true
                }
                onReleased: {
                    alarmConfirmedIsPressed = false
                    ros2Bridge.publishAlarmConfirm()
                }
            }

            Text {
                anchors.centerIn: parent
                text: "Confirm Alarm"
                font.pixelSize: 8
                font.bold: true
                color: "white"
            }
        }
        
    }
    
}

