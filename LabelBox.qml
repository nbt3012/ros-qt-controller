import QtQuick 2.12

Row {
    property string labelTxt: "Label"
    property alias labelSize: label.width
    property alias labelValue: labelContent.text
    Text {
        id: label
        text: labelTxt
    }
    TextInput {
        id: labelContent
        width: label.width * 2
        height: label.contentHeight
        Rectangle {
            border.width: 1
            border.color: "black"
            anchors.centerIn: parent
            width: parent.width + 10
            height: parent.height + 10
            z: -1
        }
    }
}
