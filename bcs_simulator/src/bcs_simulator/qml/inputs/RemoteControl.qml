import QtQuick 2.12
import QtQuick.Controls 2.12

Item {
    id: remoteControl
    width: parent.width * 0.85
    height: parent.height * 0.85

    property bool isRemoteWindowUpPressed: false
    property bool isRemoteWindowDownPressed: false
    property bool isRemoteLockPressed: false
    property bool isRemoteUnlockPressed: false
    property bool isCentralLockPressed: false
    property bool isCentralUnlockPressed: false
    property bool alarmActivatedIsPressed: false
    property bool alarmDeactivatedIsPressed: false

    // Remote Control
    Rectangle {
        id: remoteControlKeyController
        anchors.left: parent.left
        anchors.leftMargin: 30
        anchors.top: parent.top
        anchors.topMargin: 25
        width: parent.width * 0.45
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 10
        color: "transparent"
        border.color: colorGlow   
        border.width: 2

        Text {
            id: remoteText
            text: "Remote Control Key"
            font.pixelSize: 14
            font.bold: true
            color: "white"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            anchors.margins: 10
        }

        Image {
            id: remoteWindowUpImage
            anchors.bottom: parent.verticalCenter
            anchors.bottomMargin: 15
            anchors.left: parent.left
            anchors.leftMargin: 40
            source: isRemoteWindowUpPressed ? "qrc:/assets/window_up_on.png" : "qrc:/assets/window_up_off.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isRemoteWindowUpPressed = true
                    ros2Bridge.publishRemoteWindowUp()
                }
                onReleased: {
                    isRemoteWindowUpPressed = false
                    ros2Bridge.publishReleaseWindowUp()
                }
            }
        }

        Text {
            id: remoteWindowText
            text: "Window"
            font.pixelSize: 10
            font.bold: true
            color: "white"
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: parent.left
            anchors.leftMargin: 30
        }

        Image {
            id: remoteWindowDownImage
            anchors.top: parent.verticalCenter
            anchors.topMargin: 15
            anchors.left: parent.left
            anchors.leftMargin: 40
            source: isRemoteWindowDownPressed ? "qrc:/assets/window_down_on.png" : "qrc:/assets/window_down_off.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isRemoteWindowDownPressed = true
                    ros2Bridge.publishRemoteWindowDown()
                }
                onReleased: {
                    isRemoteWindowDownPressed = false
                    ros2Bridge.publishReleaseWindowDown()
                }
            }
        }

        Image {
            id: remoteLockPressed
            anchors.bottom: parent.verticalCenter
            anchors.bottomMargin: 15
            anchors.right: parent.right
            anchors.rightMargin: 40
            source: isRemoteLockPressed ? "qrc:/assets/lock_on.png" : "qrc:/assets/lock_off.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isRemoteLockPressed = true
                    ros2Bridge.publishRemoteLock()
                }
                onReleased: {
                    isRemoteLockPressed = false
                }
            }
        }

        Text {
            id: lockUnlock
            text: "Lock/Unlock"
            font.pixelSize: 10
            font.bold: true
            color: "white"
            anchors.verticalCenter: parent.verticalCenter
            anchors.right: parent.right
            anchors.rightMargin: 20
        }

        Image {
            id: remoteUnlockPressed
            anchors.top: parent.verticalCenter
            anchors.topMargin: 15
            anchors.right: parent.right
            anchors.rightMargin: 40
            source: isRemoteUnlockPressed ? "qrc:/assets/unlock_on" : "qrc:/assets/unlock_off.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isRemoteUnlockPressed = true
                    ros2Bridge.publishRemoteUnlock()
                }
                onReleased: {
                    isRemoteUnlockPressed = false
                }
            }
        }
    }

    // centralLockingSystem
    Rectangle {
        id: centralLockingSystem
        anchors.left: remoteControlKeyController.right
        anchors.right: parent.right
        anchors.rightMargin: 30
        anchors.top: parent.top
        anchors.topMargin: 25
        anchors.leftMargin: 10
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 10
        color: "transparent"
        border.color: colorGlow   
        border.width: 2

        Text {
            id: centralLockingText
            text: "Central Locking"
            font.pixelSize: 14
            font.bold: true
            color: "white"
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            anchors.margins: 10
        }

        Image {
            id: centralLockPressed
            anchors.bottom: parent.verticalCenter
            anchors.bottomMargin: 15
            anchors.horizontalCenter: parent.horizontalCenter
            source: isCentralLockPressed ? "qrc:/assets/lock_on.png" : "qrc:/assets/lock_off.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isCentralLockPressed = true
                    ros2Bridge.publishCentralLock()
                }
                onReleased: {
                    isCentralLockPressed = false
                }
            }
        }

        Text {
            id: centralLockUnlock
            text: "Lock/Unlock"
            font.pixelSize: 10
            font.bold: true
            color: "white"
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
        }

        Image {
            id: centralUnlockPressed
            anchors.top: parent.verticalCenter
            anchors.topMargin: 15
            anchors.horizontalCenter: parent.horizontalCenter
            source: isCentralUnlockPressed ? "qrc:/assets/unlock_on" : "qrc:/assets/unlock_off.png"
            width: 24
            height: 24

            MouseArea {
                anchors.fill: parent
                onPressed: {
                    isCentralUnlockPressed = true
                    ros2Bridge.publishCentralUnlock()
                }
                onReleased: {
                    isCentralUnlockPressed = false
                }
            }
        }
    }
}

