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
  Layout.minimumHeight: 475
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
          PoseArrayDisplay.onRefresh();
        }
      }

      ComboBox {
        id: combo
        Layout.fillWidth: true
        model: PoseArrayDisplay.topicList
        currentIndex: 0
        editable: true
        editText: currentText
        displayText: currentText
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }

          PoseArrayDisplay.setTopic(textAt(currentIndex));
        }

        Component.onCompleted: {
          combo.editText = "/pose_array"
          combo.displayText = "/pose_array"
        }

        Connections {
          target: PoseArrayDisplay
          onSetCurrentIndex: {
            combo.currentIndex = index
          }
        }
      }
    }

    QoSConfig {
      onProfileUpdate: {
        PoseArrayDisplay.updateQoS(depth, history, reliability, durability)
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Shape"
        font.pointSize: 10.5
      }

      ComboBox {
        id: shapeCombo
        Layout.fillWidth: true
        currentIndex: 0
        model: [ "Arrow", "Axis" ]
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }
          PoseArrayDisplay.setShape(currentIndex === 0)
        }
      }
    }

    ColumnLayout {
      id: arrowConfig
      visible: shapeCombo.currentIndex === 0
      width: parent.width
      Layout.fillWidth: true

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 80
          Layout.minimumWidth: 80
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
            color: "#ff1900"
            border.color: "#000000"
            border.width: 2
          }
        }

        TextField {
          id: colorTextField
          text: "#ff1900"
          Layout.fillWidth: true
          validator: RegExpValidator {
            regExp: /#([\da-f]{3}){1,2}/ig
          }
          onAccepted: {
            colorDialog.color = text
            bgColor.color = text
            PoseArrayDisplay.setColor(text);
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10
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
            PoseArrayDisplay.setColor(colorDialog.color);
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Shaft Length"
          font.pointSize: 10.5
        }

        TextField {
          id: shaftLength
          Layout.fillWidth: true
          width: 150
          text: "0.23"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseArrayDisplay.setArrowDimensions(shaftLength.text, shaftRadius.text, headLength.text, headRadius.text)
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Shaft Radius"
          font.pointSize: 10.5
        }

        TextField {
          id: shaftRadius
          Layout.fillWidth: true
          width: 150
          text: "0.01"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseArrayDisplay.setArrowDimensions(shaftLength.text, shaftRadius.text, headLength.text, headRadius.text)
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Head Length"
          font.pointSize: 10.5
        }

        TextField {
          id: headLength
          Layout.fillWidth: true
          width: 150
          text: "0.07"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseArrayDisplay.setArrowDimensions(shaftLength.text, shaftRadius.text, headLength.text, headRadius.text)
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Head Radius"
          font.pointSize: 10.5
        }

        TextField {
          id: headRadius
          Layout.fillWidth: true
          width: 150
          text: "0.03"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseArrayDisplay.setArrowDimensions(shaftLength.text, shaftRadius.text, headLength.text, headRadius.text)
          }
        }
      }
    }

    ColumnLayout {
      id: axisConfig
      visible: shapeCombo.currentIndex === 1
      width: parent.width
      Layout.fillWidth: true

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Length"
          font.pointSize: 10.5
        }

        TextField {
          id: axisLengthField
          Layout.fillWidth: true
          width: 150
          text: "0.3"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseArrayDisplay.setAxisDimensions(axisLengthField.text, axisRadiusField.text)
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Radius"
          font.pointSize: 10.5
        }

        TextField {
          id: axisRadiusField
          Layout.fillWidth: true
          width: 150
          text: "0.03"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseArrayDisplay.setAxisDimensions(axisLengthField.text, axisRadiusField.text)
          }
        }
      }

      CheckBox {
        checked: false
        text: qsTr("Show axis head")
        onClicked: { PoseArrayDisplay.setAxisHeadVisibility(checked) }
      }
    }
  }

  ColorDialog {
    id: colorDialog
    title: "Select arrow visual color"
    color: "#ff1900"
    showAlphaChannel: false
    onAccepted: {
      bgColor.color = colorDialog.color
      colorTextField.text = colorDialog.color
      colorDialog.color.a = alphaTextField.text
      PoseArrayDisplay.setColor(colorDialog.color);
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }
}
