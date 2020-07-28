/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.0
import "qrc:/QoSConfig"

Item {
  Layout.minimumWidth: 250
  Layout.minimumHeight: 395
  anchors.fill: parent
  anchors.margins: 10
  Column {
    width: parent.width

    RowLayout {
      width: parent.width
      RoundButton {
        text: "\u21bb"
        Material.background: Material.primary
        onClicked: {
          PointStampedDisplay.onRefresh();
        }
      }

      ComboBox {
        id: combo
        Layout.fillWidth: true
        model: PointStampedDisplay.topicList
        currentIndex: 0
        editable: true
        editText: currentText
        displayText: currentText
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }

          PointStampedDisplay.setTopic(textAt(currentIndex));
        }

        Component.onCompleted: {
          combo.editText = "/point"
          combo.displayText = "/point"
        }

        Connections {
          target: PointStampedDisplay
          onSetCurrentIndex: {
            combo.currentIndex = index
          }
        }
      }
    }

    QoSConfig {
      onProfileUpdate: {
        PointStampedDisplay.updateQoS(depth, history, reliability, durability)
      }
    }

    RowLayout {
      width: parent.width
      spacing: 10

      Text {
        width: 75
        Layout.minimumWidth: 75
        text: "Color"
        font.pointSize: 10.5
      }

      Button {
        Layout.preferredWidth: 20
        Layout.preferredHeight: 20
        onClicked: colorDialog.open()
        background: Rectangle {
          width: 20
          height: 20
          id: bgColor
          color: "#cc29cc"
          border.color: "#000000"
          border.width: 2
        }
      }

      TextField {
        id: colorTextField
        text: "#cc29cc"
        Layout.fillWidth: true
        validator: RegExpValidator {
          regExp: /#([\da-f]{3}){1,2}/ig
        }
        onAccepted: {
          colorDialog.color = text
          bgColor.color = text
          PointStampedDisplay.setColor(text);
        }
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Alpha"
        font.pointSize: 10.5
      }

      TextField {
        id: alphaTextField
        Layout.fillWidth: true
        text: "1.0"
        validator: RegExpValidator {
          // Integer and floating point numbers
          regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
        }
        onAccepted: {
          colorDialog.color.a = alphaTextField.text;
          bgColor.color = colorDialog.color
          PointStampedDisplay.setColor(colorDialog.color);
        }
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Radius"
        font.pointSize: 10.5
      }

      TextField {
        id: radiusTextField
        Layout.fillWidth: true
        text: "0.2"
        validator: RegExpValidator {
          // Integer and floating point numbers
          regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
        }
        onAccepted: {
          PointStampedDisplay.setRadius(radiusTextField.text);
        }
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "History Length"
        font.pointSize: 10.5
      }

      TextField {
        id: historyTextField
        Layout.fillWidth: true
        text: "5"
        validator: IntValidator { 
          bottom: 1
          top: 1000 
        }
        onAccepted: {
          PointStampedDisplay.setHistoryLength(historyTextField.text);
        }
      }
    }
  }

  ColorDialog {
    id: colorDialog
    title: "Select visual color"
    color: "#cc29cc"
    showAlphaChannel: false
    onAccepted: {
      bgColor.color = colorDialog.color
      colorTextField.text = colorDialog.color
      colorDialog.color.a = alphaTextField.text
      PointStampedDisplay.setColor(colorDialog.color);
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }
}
