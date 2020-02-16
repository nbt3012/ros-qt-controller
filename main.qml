import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 2.12

Window {
    visible: true
    width: 640
    height: 480
    title: qsTr("QT - ROS Test App")

    Column {
        anchors.centerIn: parent
        spacing: 30
        property int size: 100
        LabelBox {
            id: topic
            labelSize: parent.size
            labelTxt: "Topic Name"
            labelValue: "default_topic"
        }
        LabelBox {
            id: xVal
            labelSize: parent.size
            labelTxt: "Linear x"
        }
        LabelBox {
            id: yVal
            labelSize: parent.size
            labelTxt: "Linear y"
        }
        LabelBox {
            id: zVal
            labelSize: parent.size
            labelTxt: "Linear z"
        }
        Button {
            text: "Send"
            width: parent.width/2
            anchors.left: parent.left
            anchors.leftMargin: parent.size*2 - width/2
            onClicked: {
                console.log("Button clicked!")
                RosController.sendMsg(topic.labelValue, parseInt(xVal.labelValue), parseInt(yVal.labelValue), parseInt(zVal.labelValue))
            }
        }
    }
}
