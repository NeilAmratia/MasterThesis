/****************************************************************************
** Meta object code from reading C++ file 'ROS2Bridge.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.18)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/ROS2Bridge.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ROS2Bridge.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.18. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ROS2Bridge_t {
    QByteArrayData data[124];
    char stringdata0[2420];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ROS2Bridge_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ROS2Bridge_t qt_meta_stringdata_ROS2Bridge = {
    {
QT_MOC_LITERAL(0, 0, 10), // "ROS2Bridge"
QT_MOC_LITERAL(1, 11, 15), // "messageReceived"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 7), // "message"
QT_MOC_LITERAL(4, 36, 18), // "windowMoveUpSignal"
QT_MOC_LITERAL(5, 55, 20), // "windowMoveDownSignal"
QT_MOC_LITERAL(6, 76, 25), // "windowPositionUpMaxSignal"
QT_MOC_LITERAL(7, 102, 27), // "windowPositionDownMaxSignal"
QT_MOC_LITERAL(8, 130, 22), // "windowAutoMoveUpSignal"
QT_MOC_LITERAL(9, 153, 24), // "windowAutoMoveDownSignal"
QT_MOC_LITERAL(10, 178, 24), // "windowAutoMoveStopSignal"
QT_MOC_LITERAL(11, 203, 18), // "mirrorMoveUpSignal"
QT_MOC_LITERAL(12, 222, 20), // "mirrorMoveDownSignal"
QT_MOC_LITERAL(13, 243, 20), // "mirrorMoveLeftSignal"
QT_MOC_LITERAL(14, 264, 21), // "mirrorMoveRightSignal"
QT_MOC_LITERAL(15, 286, 17), // "mirrorPosUpSignal"
QT_MOC_LITERAL(16, 304, 19), // "mirrorPosDownSignal"
QT_MOC_LITERAL(17, 324, 19), // "mirrorPosLeftSignal"
QT_MOC_LITERAL(18, 344, 20), // "mirrorPosRightSignal"
QT_MOC_LITERAL(19, 365, 21), // "mirrorHeatingOnSignal"
QT_MOC_LITERAL(20, 387, 22), // "mirrorHeatingOffSignal"
QT_MOC_LITERAL(21, 410, 19), // "windowUpLedOnSignal"
QT_MOC_LITERAL(22, 430, 20), // "windowUpLedOffSignal"
QT_MOC_LITERAL(23, 451, 21), // "windowDownLedOnSignal"
QT_MOC_LITERAL(24, 473, 22), // "windowDownLedOffSignal"
QT_MOC_LITERAL(25, 496, 27), // "fingerProtectionLedOnSignal"
QT_MOC_LITERAL(26, 524, 28), // "fingerProtectionLedOffSignal"
QT_MOC_LITERAL(27, 553, 26), // "centralWindowUpLedOnSignal"
QT_MOC_LITERAL(28, 580, 27), // "centralWindowUpLedOffSignal"
QT_MOC_LITERAL(29, 608, 19), // "mirrorUpLedOnSignal"
QT_MOC_LITERAL(30, 628, 20), // "mirrorUpLedOffSignal"
QT_MOC_LITERAL(31, 649, 21), // "mirrorDownLedOnSignal"
QT_MOC_LITERAL(32, 671, 22), // "mirrorDownLedOffSignal"
QT_MOC_LITERAL(33, 694, 21), // "mirrorLeftLedOnSignal"
QT_MOC_LITERAL(34, 716, 22), // "mirrorLeftLedOffSignal"
QT_MOC_LITERAL(35, 739, 22), // "mirrorRightLedOnSignal"
QT_MOC_LITERAL(36, 762, 23), // "mirrorRightLedOffSignal"
QT_MOC_LITERAL(37, 786, 23), // "mirrorHeaterLedOnSignal"
QT_MOC_LITERAL(38, 810, 24), // "mirrorHeaterLedOffSignal"
QT_MOC_LITERAL(39, 835, 24), // "alarmDetectedLedOnSignal"
QT_MOC_LITERAL(40, 860, 25), // "alarmDetectedLedOffSignal"
QT_MOC_LITERAL(41, 886, 19), // "alarmSignalOnSignal"
QT_MOC_LITERAL(42, 906, 20), // "alarmSignalOffSignal"
QT_MOC_LITERAL(43, 927, 21), // "alarmInteriorOnSignal"
QT_MOC_LITERAL(44, 949, 22), // "alarmInteriorOffSignal"
QT_MOC_LITERAL(45, 972, 23), // "alarmActivationOnSignal"
QT_MOC_LITERAL(46, 996, 24), // "alarmActivationOffSignal"
QT_MOC_LITERAL(47, 1021, 22), // "alarmSignalLedOnSignal"
QT_MOC_LITERAL(48, 1044, 23), // "alarmSignalLedOffSignal"
QT_MOC_LITERAL(49, 1068, 24), // "alarmInteriorLedOnSignal"
QT_MOC_LITERAL(50, 1093, 25), // "alarmInteriorLedOffSignal"
QT_MOC_LITERAL(51, 1119, 26), // "alarmActivationLedOnSignal"
QT_MOC_LITERAL(52, 1146, 27), // "alarmActivationLedOffSignal"
QT_MOC_LITERAL(53, 1174, 15), // "carLockedSignal"
QT_MOC_LITERAL(54, 1190, 17), // "carUnlockedSignal"
QT_MOC_LITERAL(55, 1208, 13), // "clsLockSignal"
QT_MOC_LITERAL(56, 1222, 15), // "clsUnlockSignal"
QT_MOC_LITERAL(57, 1238, 22), // "centralLockLedOnSignal"
QT_MOC_LITERAL(58, 1261, 23), // "centralLockLedOffSignal"
QT_MOC_LITERAL(59, 1285, 22), // "startAutoMoveRequested"
QT_MOC_LITERAL(60, 1308, 21), // "stopAutoMoveRequested"
QT_MOC_LITERAL(61, 1330, 21), // "sliderPositionChanged"
QT_MOC_LITERAL(62, 1352, 8), // "position"
QT_MOC_LITERAL(63, 1361, 29), // "mirrorVerticalPositionChanged"
QT_MOC_LITERAL(64, 1391, 31), // "mirrorHorizontalPositionChanged"
QT_MOC_LITERAL(65, 1423, 19), // "inputsLockedChanged"
QT_MOC_LITERAL(66, 1443, 14), // "sliderPosition"
QT_MOC_LITERAL(67, 1458, 24), // "mirrorHorizontalPosition"
QT_MOC_LITERAL(68, 1483, 22), // "mirrorVerticalPosition"
QT_MOC_LITERAL(69, 1506, 17), // "setSliderPosition"
QT_MOC_LITERAL(70, 1524, 25), // "setMirrorVerticalPosition"
QT_MOC_LITERAL(71, 1550, 27), // "setMirrorHorizontalPosition"
QT_MOC_LITERAL(72, 1578, 18), // "startAutoMoveTimer"
QT_MOC_LITERAL(73, 1597, 17), // "stopAutoMoveTimer"
QT_MOC_LITERAL(74, 1615, 14), // "updateAutoMove"
QT_MOC_LITERAL(75, 1630, 20), // "handleHeatingTimeout"
QT_MOC_LITERAL(76, 1651, 19), // "handleRemoteTimeout"
QT_MOC_LITERAL(77, 1671, 18), // "handleAlarmTimeout"
QT_MOC_LITERAL(78, 1690, 15), // "messageCallback"
QT_MOC_LITERAL(79, 1706, 32), // "std_msgs::msg::String::SharedPtr"
QT_MOC_LITERAL(80, 1739, 3), // "msg"
QT_MOC_LITERAL(81, 1743, 10), // "addMessage"
QT_MOC_LITERAL(82, 1754, 4), // "type"
QT_MOC_LITERAL(83, 1759, 9), // "timestamp"
QT_MOC_LITERAL(84, 1769, 15), // "publishWindowUp"
QT_MOC_LITERAL(85, 1785, 17), // "publishWindowDown"
QT_MOC_LITERAL(86, 1803, 23), // "publishWindowPositionUp"
QT_MOC_LITERAL(87, 1827, 25), // "publishWindowPositionDown"
QT_MOC_LITERAL(88, 1853, 14), // "fingerDetected"
QT_MOC_LITERAL(89, 1868, 9), // "detection"
QT_MOC_LITERAL(90, 1878, 15), // "publishEMPosTop"
QT_MOC_LITERAL(91, 1894, 3), // "top"
QT_MOC_LITERAL(92, 1898, 18), // "publishEMPosBottom"
QT_MOC_LITERAL(93, 1917, 6), // "bottom"
QT_MOC_LITERAL(94, 1924, 16), // "publishEMPosLeft"
QT_MOC_LITERAL(95, 1941, 4), // "left"
QT_MOC_LITERAL(96, 1946, 17), // "publishEMPosRight"
QT_MOC_LITERAL(97, 1964, 5), // "right"
QT_MOC_LITERAL(98, 1970, 13), // "publishEMCold"
QT_MOC_LITERAL(99, 1984, 4), // "cold"
QT_MOC_LITERAL(100, 1989, 15), // "publishMirrorUp"
QT_MOC_LITERAL(101, 2005, 17), // "publishMirrorDown"
QT_MOC_LITERAL(102, 2023, 17), // "publishMirrorLeft"
QT_MOC_LITERAL(103, 2041, 18), // "publishMirrorRight"
QT_MOC_LITERAL(104, 2060, 20), // "publishAlarmActivate"
QT_MOC_LITERAL(105, 2081, 22), // "publishAlarmDeactivate"
QT_MOC_LITERAL(106, 2104, 19), // "publishAlarmConfirm"
QT_MOC_LITERAL(107, 2124, 21), // "publishAlarmTriggered"
QT_MOC_LITERAL(108, 2146, 23), // "publishAlarmIMTriggered"
QT_MOC_LITERAL(109, 2170, 22), // "publishReleaseWindowUp"
QT_MOC_LITERAL(110, 2193, 24), // "publishReleaseWindowDown"
QT_MOC_LITERAL(111, 2218, 21), // "publishRemoteWindowUp"
QT_MOC_LITERAL(112, 2240, 23), // "publishRemoteWindowDown"
QT_MOC_LITERAL(113, 2264, 17), // "publishRemoteLock"
QT_MOC_LITERAL(114, 2282, 19), // "publishRemoteUnlock"
QT_MOC_LITERAL(115, 2302, 7), // "carDoor"
QT_MOC_LITERAL(116, 2310, 4), // "door"
QT_MOC_LITERAL(117, 2315, 10), // "carDriving"
QT_MOC_LITERAL(118, 2326, 7), // "driving"
QT_MOC_LITERAL(119, 2334, 18), // "publishCentralLock"
QT_MOC_LITERAL(120, 2353, 20), // "publishCentralUnlock"
QT_MOC_LITERAL(121, 2374, 12), // "messageModel"
QT_MOC_LITERAL(122, 2387, 19), // "QAbstractListModel*"
QT_MOC_LITERAL(123, 2407, 12) // "inputsLocked"

    },
    "ROS2Bridge\0messageReceived\0\0message\0"
    "windowMoveUpSignal\0windowMoveDownSignal\0"
    "windowPositionUpMaxSignal\0"
    "windowPositionDownMaxSignal\0"
    "windowAutoMoveUpSignal\0windowAutoMoveDownSignal\0"
    "windowAutoMoveStopSignal\0mirrorMoveUpSignal\0"
    "mirrorMoveDownSignal\0mirrorMoveLeftSignal\0"
    "mirrorMoveRightSignal\0mirrorPosUpSignal\0"
    "mirrorPosDownSignal\0mirrorPosLeftSignal\0"
    "mirrorPosRightSignal\0mirrorHeatingOnSignal\0"
    "mirrorHeatingOffSignal\0windowUpLedOnSignal\0"
    "windowUpLedOffSignal\0windowDownLedOnSignal\0"
    "windowDownLedOffSignal\0"
    "fingerProtectionLedOnSignal\0"
    "fingerProtectionLedOffSignal\0"
    "centralWindowUpLedOnSignal\0"
    "centralWindowUpLedOffSignal\0"
    "mirrorUpLedOnSignal\0mirrorUpLedOffSignal\0"
    "mirrorDownLedOnSignal\0mirrorDownLedOffSignal\0"
    "mirrorLeftLedOnSignal\0mirrorLeftLedOffSignal\0"
    "mirrorRightLedOnSignal\0mirrorRightLedOffSignal\0"
    "mirrorHeaterLedOnSignal\0"
    "mirrorHeaterLedOffSignal\0"
    "alarmDetectedLedOnSignal\0"
    "alarmDetectedLedOffSignal\0alarmSignalOnSignal\0"
    "alarmSignalOffSignal\0alarmInteriorOnSignal\0"
    "alarmInteriorOffSignal\0alarmActivationOnSignal\0"
    "alarmActivationOffSignal\0"
    "alarmSignalLedOnSignal\0alarmSignalLedOffSignal\0"
    "alarmInteriorLedOnSignal\0"
    "alarmInteriorLedOffSignal\0"
    "alarmActivationLedOnSignal\0"
    "alarmActivationLedOffSignal\0carLockedSignal\0"
    "carUnlockedSignal\0clsLockSignal\0"
    "clsUnlockSignal\0centralLockLedOnSignal\0"
    "centralLockLedOffSignal\0startAutoMoveRequested\0"
    "stopAutoMoveRequested\0sliderPositionChanged\0"
    "position\0mirrorVerticalPositionChanged\0"
    "mirrorHorizontalPositionChanged\0"
    "inputsLockedChanged\0sliderPosition\0"
    "mirrorHorizontalPosition\0"
    "mirrorVerticalPosition\0setSliderPosition\0"
    "setMirrorVerticalPosition\0"
    "setMirrorHorizontalPosition\0"
    "startAutoMoveTimer\0stopAutoMoveTimer\0"
    "updateAutoMove\0handleHeatingTimeout\0"
    "handleRemoteTimeout\0handleAlarmTimeout\0"
    "messageCallback\0std_msgs::msg::String::SharedPtr\0"
    "msg\0addMessage\0type\0timestamp\0"
    "publishWindowUp\0publishWindowDown\0"
    "publishWindowPositionUp\0"
    "publishWindowPositionDown\0fingerDetected\0"
    "detection\0publishEMPosTop\0top\0"
    "publishEMPosBottom\0bottom\0publishEMPosLeft\0"
    "left\0publishEMPosRight\0right\0publishEMCold\0"
    "cold\0publishMirrorUp\0publishMirrorDown\0"
    "publishMirrorLeft\0publishMirrorRight\0"
    "publishAlarmActivate\0publishAlarmDeactivate\0"
    "publishAlarmConfirm\0publishAlarmTriggered\0"
    "publishAlarmIMTriggered\0publishReleaseWindowUp\0"
    "publishReleaseWindowDown\0publishRemoteWindowUp\0"
    "publishRemoteWindowDown\0publishRemoteLock\0"
    "publishRemoteUnlock\0carDoor\0door\0"
    "carDriving\0driving\0publishCentralLock\0"
    "publishCentralUnlock\0messageModel\0"
    "QAbstractListModel*\0inputsLocked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ROS2Bridge[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
     106,   14, // methods
       5,  696, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      62,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  544,    2, 0x06 /* Public */,
       4,    0,  547,    2, 0x06 /* Public */,
       5,    0,  548,    2, 0x06 /* Public */,
       6,    0,  549,    2, 0x06 /* Public */,
       7,    0,  550,    2, 0x06 /* Public */,
       8,    0,  551,    2, 0x06 /* Public */,
       9,    0,  552,    2, 0x06 /* Public */,
      10,    0,  553,    2, 0x06 /* Public */,
      11,    0,  554,    2, 0x06 /* Public */,
      12,    0,  555,    2, 0x06 /* Public */,
      13,    0,  556,    2, 0x06 /* Public */,
      14,    0,  557,    2, 0x06 /* Public */,
      15,    0,  558,    2, 0x06 /* Public */,
      16,    0,  559,    2, 0x06 /* Public */,
      17,    0,  560,    2, 0x06 /* Public */,
      18,    0,  561,    2, 0x06 /* Public */,
      19,    0,  562,    2, 0x06 /* Public */,
      20,    0,  563,    2, 0x06 /* Public */,
      21,    0,  564,    2, 0x06 /* Public */,
      22,    0,  565,    2, 0x06 /* Public */,
      23,    0,  566,    2, 0x06 /* Public */,
      24,    0,  567,    2, 0x06 /* Public */,
      25,    0,  568,    2, 0x06 /* Public */,
      26,    0,  569,    2, 0x06 /* Public */,
      27,    0,  570,    2, 0x06 /* Public */,
      28,    0,  571,    2, 0x06 /* Public */,
      29,    0,  572,    2, 0x06 /* Public */,
      30,    0,  573,    2, 0x06 /* Public */,
      31,    0,  574,    2, 0x06 /* Public */,
      32,    0,  575,    2, 0x06 /* Public */,
      33,    0,  576,    2, 0x06 /* Public */,
      34,    0,  577,    2, 0x06 /* Public */,
      35,    0,  578,    2, 0x06 /* Public */,
      36,    0,  579,    2, 0x06 /* Public */,
      37,    0,  580,    2, 0x06 /* Public */,
      38,    0,  581,    2, 0x06 /* Public */,
      39,    0,  582,    2, 0x06 /* Public */,
      40,    0,  583,    2, 0x06 /* Public */,
      41,    0,  584,    2, 0x06 /* Public */,
      42,    0,  585,    2, 0x06 /* Public */,
      43,    0,  586,    2, 0x06 /* Public */,
      44,    0,  587,    2, 0x06 /* Public */,
      45,    0,  588,    2, 0x06 /* Public */,
      46,    0,  589,    2, 0x06 /* Public */,
      47,    0,  590,    2, 0x06 /* Public */,
      48,    0,  591,    2, 0x06 /* Public */,
      49,    0,  592,    2, 0x06 /* Public */,
      50,    0,  593,    2, 0x06 /* Public */,
      51,    0,  594,    2, 0x06 /* Public */,
      52,    0,  595,    2, 0x06 /* Public */,
      53,    0,  596,    2, 0x06 /* Public */,
      54,    0,  597,    2, 0x06 /* Public */,
      55,    0,  598,    2, 0x06 /* Public */,
      56,    0,  599,    2, 0x06 /* Public */,
      57,    0,  600,    2, 0x06 /* Public */,
      58,    0,  601,    2, 0x06 /* Public */,
      59,    0,  602,    2, 0x06 /* Public */,
      60,    0,  603,    2, 0x06 /* Public */,
      61,    1,  604,    2, 0x06 /* Public */,
      63,    1,  607,    2, 0x06 /* Public */,
      64,    1,  610,    2, 0x06 /* Public */,
      65,    0,  613,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      66,    0,  614,    2, 0x0a /* Public */,
      67,    0,  615,    2, 0x0a /* Public */,
      68,    0,  616,    2, 0x0a /* Public */,
      69,    1,  617,    2, 0x0a /* Public */,
      70,    1,  620,    2, 0x0a /* Public */,
      71,    1,  623,    2, 0x0a /* Public */,
      72,    0,  626,    2, 0x0a /* Public */,
      73,    0,  627,    2, 0x0a /* Public */,
      74,    0,  628,    2, 0x0a /* Public */,
      75,    0,  629,    2, 0x0a /* Public */,
      76,    0,  630,    2, 0x0a /* Public */,
      77,    0,  631,    2, 0x0a /* Public */,
      78,    1,  632,    2, 0x0a /* Public */,
      81,    3,  635,    2, 0x0a /* Public */,
      81,    2,  642,    2, 0x2a /* Public | MethodCloned */,

 // methods: name, argc, parameters, tag, flags
      84,    0,  647,    2, 0x02 /* Public */,
      85,    0,  648,    2, 0x02 /* Public */,
      86,    1,  649,    2, 0x02 /* Public */,
      87,    1,  652,    2, 0x02 /* Public */,
      88,    1,  655,    2, 0x02 /* Public */,
      90,    1,  658,    2, 0x02 /* Public */,
      92,    1,  661,    2, 0x02 /* Public */,
      94,    1,  664,    2, 0x02 /* Public */,
      96,    1,  667,    2, 0x02 /* Public */,
      98,    1,  670,    2, 0x02 /* Public */,
     100,    0,  673,    2, 0x02 /* Public */,
     101,    0,  674,    2, 0x02 /* Public */,
     102,    0,  675,    2, 0x02 /* Public */,
     103,    0,  676,    2, 0x02 /* Public */,
     104,    0,  677,    2, 0x02 /* Public */,
     105,    0,  678,    2, 0x02 /* Public */,
     106,    0,  679,    2, 0x02 /* Public */,
     107,    0,  680,    2, 0x02 /* Public */,
     108,    0,  681,    2, 0x02 /* Public */,
     109,    0,  682,    2, 0x02 /* Public */,
     110,    0,  683,    2, 0x02 /* Public */,
     111,    0,  684,    2, 0x02 /* Public */,
     112,    0,  685,    2, 0x02 /* Public */,
     113,    0,  686,    2, 0x02 /* Public */,
     114,    0,  687,    2, 0x02 /* Public */,
     115,    1,  688,    2, 0x02 /* Public */,
     117,    1,  691,    2, 0x02 /* Public */,
     119,    0,  694,    2, 0x02 /* Public */,
     120,    0,  695,    2, 0x02 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   62,
    QMetaType::Void, QMetaType::Int,   62,
    QMetaType::Void, QMetaType::Int,   62,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::Void, QMetaType::Int,   62,
    QMetaType::Void, QMetaType::Int,   62,
    QMetaType::Void, QMetaType::Int,   62,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 79,   80,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::QString,    3,   82,   83,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    3,   82,

 // methods: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   62,
    QMetaType::Void, QMetaType::Bool,   62,
    QMetaType::Void, QMetaType::Bool,   89,
    QMetaType::Void, QMetaType::Bool,   91,
    QMetaType::Void, QMetaType::Bool,   93,
    QMetaType::Void, QMetaType::Bool,   95,
    QMetaType::Void, QMetaType::Bool,   97,
    QMetaType::Void, QMetaType::Bool,   99,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,  116,
    QMetaType::Void, QMetaType::Bool,  118,
    QMetaType::Void,
    QMetaType::Void,

 // properties: name, type, flags
     121, 0x80000000 | 122, 0x00095409,
      66, QMetaType::Int, 0x00495103,
      67, QMetaType::Int, 0x00495103,
      68, QMetaType::Int, 0x00495103,
     123, QMetaType::Bool, 0x00495001,

 // properties: notify_signal_id
       0,
      58,
      60,
      59,
      61,

       0        // eod
};

void ROS2Bridge::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ROS2Bridge *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->messageReceived((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->windowMoveUpSignal(); break;
        case 2: _t->windowMoveDownSignal(); break;
        case 3: _t->windowPositionUpMaxSignal(); break;
        case 4: _t->windowPositionDownMaxSignal(); break;
        case 5: _t->windowAutoMoveUpSignal(); break;
        case 6: _t->windowAutoMoveDownSignal(); break;
        case 7: _t->windowAutoMoveStopSignal(); break;
        case 8: _t->mirrorMoveUpSignal(); break;
        case 9: _t->mirrorMoveDownSignal(); break;
        case 10: _t->mirrorMoveLeftSignal(); break;
        case 11: _t->mirrorMoveRightSignal(); break;
        case 12: _t->mirrorPosUpSignal(); break;
        case 13: _t->mirrorPosDownSignal(); break;
        case 14: _t->mirrorPosLeftSignal(); break;
        case 15: _t->mirrorPosRightSignal(); break;
        case 16: _t->mirrorHeatingOnSignal(); break;
        case 17: _t->mirrorHeatingOffSignal(); break;
        case 18: _t->windowUpLedOnSignal(); break;
        case 19: _t->windowUpLedOffSignal(); break;
        case 20: _t->windowDownLedOnSignal(); break;
        case 21: _t->windowDownLedOffSignal(); break;
        case 22: _t->fingerProtectionLedOnSignal(); break;
        case 23: _t->fingerProtectionLedOffSignal(); break;
        case 24: _t->centralWindowUpLedOnSignal(); break;
        case 25: _t->centralWindowUpLedOffSignal(); break;
        case 26: _t->mirrorUpLedOnSignal(); break;
        case 27: _t->mirrorUpLedOffSignal(); break;
        case 28: _t->mirrorDownLedOnSignal(); break;
        case 29: _t->mirrorDownLedOffSignal(); break;
        case 30: _t->mirrorLeftLedOnSignal(); break;
        case 31: _t->mirrorLeftLedOffSignal(); break;
        case 32: _t->mirrorRightLedOnSignal(); break;
        case 33: _t->mirrorRightLedOffSignal(); break;
        case 34: _t->mirrorHeaterLedOnSignal(); break;
        case 35: _t->mirrorHeaterLedOffSignal(); break;
        case 36: _t->alarmDetectedLedOnSignal(); break;
        case 37: _t->alarmDetectedLedOffSignal(); break;
        case 38: _t->alarmSignalOnSignal(); break;
        case 39: _t->alarmSignalOffSignal(); break;
        case 40: _t->alarmInteriorOnSignal(); break;
        case 41: _t->alarmInteriorOffSignal(); break;
        case 42: _t->alarmActivationOnSignal(); break;
        case 43: _t->alarmActivationOffSignal(); break;
        case 44: _t->alarmSignalLedOnSignal(); break;
        case 45: _t->alarmSignalLedOffSignal(); break;
        case 46: _t->alarmInteriorLedOnSignal(); break;
        case 47: _t->alarmInteriorLedOffSignal(); break;
        case 48: _t->alarmActivationLedOnSignal(); break;
        case 49: _t->alarmActivationLedOffSignal(); break;
        case 50: _t->carLockedSignal(); break;
        case 51: _t->carUnlockedSignal(); break;
        case 52: _t->clsLockSignal(); break;
        case 53: _t->clsUnlockSignal(); break;
        case 54: _t->centralLockLedOnSignal(); break;
        case 55: _t->centralLockLedOffSignal(); break;
        case 56: _t->startAutoMoveRequested(); break;
        case 57: _t->stopAutoMoveRequested(); break;
        case 58: _t->sliderPositionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 59: _t->mirrorVerticalPositionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 60: _t->mirrorHorizontalPositionChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 61: _t->inputsLockedChanged(); break;
        case 62: { int _r = _t->sliderPosition();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 63: { int _r = _t->mirrorHorizontalPosition();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 64: { int _r = _t->mirrorVerticalPosition();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 65: _t->setSliderPosition((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 66: _t->setMirrorVerticalPosition((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 67: _t->setMirrorHorizontalPosition((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 68: _t->startAutoMoveTimer(); break;
        case 69: _t->stopAutoMoveTimer(); break;
        case 70: _t->updateAutoMove(); break;
        case 71: _t->handleHeatingTimeout(); break;
        case 72: _t->handleRemoteTimeout(); break;
        case 73: _t->handleAlarmTimeout(); break;
        case 74: _t->messageCallback((*reinterpret_cast< const std_msgs::msg::String::SharedPtr(*)>(_a[1]))); break;
        case 75: _t->addMessage((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2])),(*reinterpret_cast< const QString(*)>(_a[3]))); break;
        case 76: _t->addMessage((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 77: _t->publishWindowUp(); break;
        case 78: _t->publishWindowDown(); break;
        case 79: _t->publishWindowPositionUp((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 80: _t->publishWindowPositionDown((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 81: _t->fingerDetected((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 82: _t->publishEMPosTop((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 83: _t->publishEMPosBottom((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 84: _t->publishEMPosLeft((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 85: _t->publishEMPosRight((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 86: _t->publishEMCold((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 87: _t->publishMirrorUp(); break;
        case 88: _t->publishMirrorDown(); break;
        case 89: _t->publishMirrorLeft(); break;
        case 90: _t->publishMirrorRight(); break;
        case 91: _t->publishAlarmActivate(); break;
        case 92: _t->publishAlarmDeactivate(); break;
        case 93: _t->publishAlarmConfirm(); break;
        case 94: _t->publishAlarmTriggered(); break;
        case 95: _t->publishAlarmIMTriggered(); break;
        case 96: _t->publishReleaseWindowUp(); break;
        case 97: _t->publishReleaseWindowDown(); break;
        case 98: _t->publishRemoteWindowUp(); break;
        case 99: _t->publishRemoteWindowDown(); break;
        case 100: _t->publishRemoteLock(); break;
        case 101: _t->publishRemoteUnlock(); break;
        case 102: _t->carDoor((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 103: _t->carDriving((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 104: _t->publishCentralLock(); break;
        case 105: _t->publishCentralUnlock(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ROS2Bridge::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::messageReceived)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowMoveUpSignal)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowMoveDownSignal)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowPositionUpMaxSignal)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowPositionDownMaxSignal)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowAutoMoveUpSignal)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowAutoMoveDownSignal)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowAutoMoveStopSignal)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorMoveUpSignal)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorMoveDownSignal)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorMoveLeftSignal)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorMoveRightSignal)) {
                *result = 11;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorPosUpSignal)) {
                *result = 12;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorPosDownSignal)) {
                *result = 13;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorPosLeftSignal)) {
                *result = 14;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorPosRightSignal)) {
                *result = 15;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorHeatingOnSignal)) {
                *result = 16;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorHeatingOffSignal)) {
                *result = 17;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowUpLedOnSignal)) {
                *result = 18;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowUpLedOffSignal)) {
                *result = 19;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowDownLedOnSignal)) {
                *result = 20;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::windowDownLedOffSignal)) {
                *result = 21;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::fingerProtectionLedOnSignal)) {
                *result = 22;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::fingerProtectionLedOffSignal)) {
                *result = 23;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::centralWindowUpLedOnSignal)) {
                *result = 24;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::centralWindowUpLedOffSignal)) {
                *result = 25;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorUpLedOnSignal)) {
                *result = 26;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorUpLedOffSignal)) {
                *result = 27;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorDownLedOnSignal)) {
                *result = 28;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorDownLedOffSignal)) {
                *result = 29;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorLeftLedOnSignal)) {
                *result = 30;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorLeftLedOffSignal)) {
                *result = 31;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorRightLedOnSignal)) {
                *result = 32;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorRightLedOffSignal)) {
                *result = 33;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorHeaterLedOnSignal)) {
                *result = 34;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorHeaterLedOffSignal)) {
                *result = 35;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmDetectedLedOnSignal)) {
                *result = 36;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmDetectedLedOffSignal)) {
                *result = 37;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmSignalOnSignal)) {
                *result = 38;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmSignalOffSignal)) {
                *result = 39;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmInteriorOnSignal)) {
                *result = 40;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmInteriorOffSignal)) {
                *result = 41;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmActivationOnSignal)) {
                *result = 42;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmActivationOffSignal)) {
                *result = 43;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmSignalLedOnSignal)) {
                *result = 44;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmSignalLedOffSignal)) {
                *result = 45;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmInteriorLedOnSignal)) {
                *result = 46;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmInteriorLedOffSignal)) {
                *result = 47;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmActivationLedOnSignal)) {
                *result = 48;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::alarmActivationLedOffSignal)) {
                *result = 49;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::carLockedSignal)) {
                *result = 50;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::carUnlockedSignal)) {
                *result = 51;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::clsLockSignal)) {
                *result = 52;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::clsUnlockSignal)) {
                *result = 53;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::centralLockLedOnSignal)) {
                *result = 54;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::centralLockLedOffSignal)) {
                *result = 55;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::startAutoMoveRequested)) {
                *result = 56;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::stopAutoMoveRequested)) {
                *result = 57;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::sliderPositionChanged)) {
                *result = 58;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorVerticalPositionChanged)) {
                *result = 59;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::mirrorHorizontalPositionChanged)) {
                *result = 60;
                return;
            }
        }
        {
            using _t = void (ROS2Bridge::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ROS2Bridge::inputsLockedChanged)) {
                *result = 61;
                return;
            }
        }
    } else if (_c == QMetaObject::RegisterPropertyMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QAbstractListModel* >(); break;
        }
    }

#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<ROS2Bridge *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QAbstractListModel**>(_v) = _t->messageModel(); break;
        case 1: *reinterpret_cast< int*>(_v) = _t->sliderPosition(); break;
        case 2: *reinterpret_cast< int*>(_v) = _t->mirrorHorizontalPosition(); break;
        case 3: *reinterpret_cast< int*>(_v) = _t->mirrorVerticalPosition(); break;
        case 4: *reinterpret_cast< bool*>(_v) = _t->inputsLocked(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<ROS2Bridge *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 1: _t->setSliderPosition(*reinterpret_cast< int*>(_v)); break;
        case 2: _t->setMirrorHorizontalPosition(*reinterpret_cast< int*>(_v)); break;
        case 3: _t->setMirrorVerticalPosition(*reinterpret_cast< int*>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

QT_INIT_METAOBJECT const QMetaObject ROS2Bridge::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ROS2Bridge.data,
    qt_meta_data_ROS2Bridge,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ROS2Bridge::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ROS2Bridge::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ROS2Bridge.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ROS2Bridge::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 106)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 106;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 106)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 106;
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 5;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 5;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void ROS2Bridge::messageReceived(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ROS2Bridge::windowMoveUpSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void ROS2Bridge::windowMoveDownSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void ROS2Bridge::windowPositionUpMaxSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void ROS2Bridge::windowPositionDownMaxSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void ROS2Bridge::windowAutoMoveUpSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void ROS2Bridge::windowAutoMoveDownSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 6, nullptr);
}

// SIGNAL 7
void ROS2Bridge::windowAutoMoveStopSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 7, nullptr);
}

// SIGNAL 8
void ROS2Bridge::mirrorMoveUpSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 8, nullptr);
}

// SIGNAL 9
void ROS2Bridge::mirrorMoveDownSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 9, nullptr);
}

// SIGNAL 10
void ROS2Bridge::mirrorMoveLeftSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 10, nullptr);
}

// SIGNAL 11
void ROS2Bridge::mirrorMoveRightSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 11, nullptr);
}

// SIGNAL 12
void ROS2Bridge::mirrorPosUpSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 12, nullptr);
}

// SIGNAL 13
void ROS2Bridge::mirrorPosDownSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 13, nullptr);
}

// SIGNAL 14
void ROS2Bridge::mirrorPosLeftSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 14, nullptr);
}

// SIGNAL 15
void ROS2Bridge::mirrorPosRightSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 15, nullptr);
}

// SIGNAL 16
void ROS2Bridge::mirrorHeatingOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 16, nullptr);
}

// SIGNAL 17
void ROS2Bridge::mirrorHeatingOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 17, nullptr);
}

// SIGNAL 18
void ROS2Bridge::windowUpLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 18, nullptr);
}

// SIGNAL 19
void ROS2Bridge::windowUpLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 19, nullptr);
}

// SIGNAL 20
void ROS2Bridge::windowDownLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 20, nullptr);
}

// SIGNAL 21
void ROS2Bridge::windowDownLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 21, nullptr);
}

// SIGNAL 22
void ROS2Bridge::fingerProtectionLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 22, nullptr);
}

// SIGNAL 23
void ROS2Bridge::fingerProtectionLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 23, nullptr);
}

// SIGNAL 24
void ROS2Bridge::centralWindowUpLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 24, nullptr);
}

// SIGNAL 25
void ROS2Bridge::centralWindowUpLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 25, nullptr);
}

// SIGNAL 26
void ROS2Bridge::mirrorUpLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 26, nullptr);
}

// SIGNAL 27
void ROS2Bridge::mirrorUpLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 27, nullptr);
}

// SIGNAL 28
void ROS2Bridge::mirrorDownLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 28, nullptr);
}

// SIGNAL 29
void ROS2Bridge::mirrorDownLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 29, nullptr);
}

// SIGNAL 30
void ROS2Bridge::mirrorLeftLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 30, nullptr);
}

// SIGNAL 31
void ROS2Bridge::mirrorLeftLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 31, nullptr);
}

// SIGNAL 32
void ROS2Bridge::mirrorRightLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 32, nullptr);
}

// SIGNAL 33
void ROS2Bridge::mirrorRightLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 33, nullptr);
}

// SIGNAL 34
void ROS2Bridge::mirrorHeaterLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 34, nullptr);
}

// SIGNAL 35
void ROS2Bridge::mirrorHeaterLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 35, nullptr);
}

// SIGNAL 36
void ROS2Bridge::alarmDetectedLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 36, nullptr);
}

// SIGNAL 37
void ROS2Bridge::alarmDetectedLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 37, nullptr);
}

// SIGNAL 38
void ROS2Bridge::alarmSignalOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 38, nullptr);
}

// SIGNAL 39
void ROS2Bridge::alarmSignalOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 39, nullptr);
}

// SIGNAL 40
void ROS2Bridge::alarmInteriorOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 40, nullptr);
}

// SIGNAL 41
void ROS2Bridge::alarmInteriorOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 41, nullptr);
}

// SIGNAL 42
void ROS2Bridge::alarmActivationOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 42, nullptr);
}

// SIGNAL 43
void ROS2Bridge::alarmActivationOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 43, nullptr);
}

// SIGNAL 44
void ROS2Bridge::alarmSignalLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 44, nullptr);
}

// SIGNAL 45
void ROS2Bridge::alarmSignalLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 45, nullptr);
}

// SIGNAL 46
void ROS2Bridge::alarmInteriorLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 46, nullptr);
}

// SIGNAL 47
void ROS2Bridge::alarmInteriorLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 47, nullptr);
}

// SIGNAL 48
void ROS2Bridge::alarmActivationLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 48, nullptr);
}

// SIGNAL 49
void ROS2Bridge::alarmActivationLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 49, nullptr);
}

// SIGNAL 50
void ROS2Bridge::carLockedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 50, nullptr);
}

// SIGNAL 51
void ROS2Bridge::carUnlockedSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 51, nullptr);
}

// SIGNAL 52
void ROS2Bridge::clsLockSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 52, nullptr);
}

// SIGNAL 53
void ROS2Bridge::clsUnlockSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 53, nullptr);
}

// SIGNAL 54
void ROS2Bridge::centralLockLedOnSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 54, nullptr);
}

// SIGNAL 55
void ROS2Bridge::centralLockLedOffSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 55, nullptr);
}

// SIGNAL 56
void ROS2Bridge::startAutoMoveRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 56, nullptr);
}

// SIGNAL 57
void ROS2Bridge::stopAutoMoveRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 57, nullptr);
}

// SIGNAL 58
void ROS2Bridge::sliderPositionChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 58, _a);
}

// SIGNAL 59
void ROS2Bridge::mirrorVerticalPositionChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 59, _a);
}

// SIGNAL 60
void ROS2Bridge::mirrorHorizontalPositionChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 60, _a);
}

// SIGNAL 61
void ROS2Bridge::inputsLockedChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 61, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
