import QtQuick 2.12
import QtQuick.Controls 2.12

Rectangle {
  id: hud
  color: "#ed14212f"
  radius: 8
  anchors.fill: parent
  implicitWidth: 16 + RobotSpeedPanel.rows.length * 104
                 + Math.max(0, RobotSpeedPanel.rows.length - 1) * 6
  implicitHeight: 108

  Column {
    anchors.fill: parent
    anchors.margins: 8
    spacing: 4

    Item {
      width: parent.width
      height: 16
      Text {
        anchors.left: parent.left
        anchors.verticalCenter: parent.verticalCenter
        text: "현재 속도 · m/s"
        color: "#f3f7fb"
        font.family: "Noto Sans CJK KR"
        font.pixelSize: 12
        font.bold: true
      }
      Text {
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        text: "최대 " + RobotSpeedPanel.speedLimit.toFixed(1) + " · — 대기/만료"
        color: "#afc2d3"
        font.family: "Noto Sans CJK KR"
        font.pixelSize: 10
      }
    }

    Row {
      spacing: 6
      height: 40
      Repeater {
        model: RobotSpeedPanel.rows
        delegate: Rectangle {
          width: 104
          height: 40
          radius: 5
          color: "#26394b"
          Rectangle {
            width: 3
            height: parent.height
            radius: 2
            color: modelData.color
          }
          Text {
            anchors.left: parent.left
            anchors.leftMargin: 9
            anchors.verticalCenter: parent.verticalCenter
            text: modelData.name.replace("robot_", "").toUpperCase()
            color: modelData.color
            font.family: "Noto Sans CJK KR"
            font.pixelSize: 13
            font.bold: true
          }
          Text {
            anchors.right: parent.right
            anchors.rightMargin: 8
            anchors.verticalCenter: parent.verticalCenter
            text: modelData.fresh ? modelData.speed.toFixed(2) : "—"
            color: modelData.fresh ? "#ffffff" : "#8192a3"
            font.family: "monospace"
            font.pixelSize: 22
            font.bold: true
          }
        }
      }
    }

    Item {
      width: parent.width
      height: 24
      Row {
        anchors.left: parent.left
        spacing: 8
        CheckBox {
          text: "계획 경로"
          height: 24
          padding: 0
          spacing: 4
          font.family: "Noto Sans CJK KR"
          font.pixelSize: 11
          checked: RobotSpeedPanel.showPlans
          onToggled: RobotSpeedPanel.showPlans = checked
        }
        CheckBox {
          text: "상대 예측"
          height: 24
          padding: 0
          spacing: 4
          font.family: "Noto Sans CJK KR"
          font.pixelSize: 11
          checked: RobotSpeedPanel.showPredictions
          onToggled: RobotSpeedPanel.showPredictions = checked
        }
      }
      Text {
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        text: "실선: 계획 · 점선/원: 예측"
        color: "#afc2d3"
        font.family: "Noto Sans CJK KR"
        font.pixelSize: 10
      }
    }
  }
}
