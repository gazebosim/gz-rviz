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

Item {
  Layout.minimumWidth: 280
  Layout.minimumHeight: 220
  anchors.fill: parent
  anchors.margins: 10

  ColumnLayout {
    width: parent.width
    spacing: 20

    RowLayout {
      width: parent.width
      RoundButton {
        text: "\u21bb"
        Material.background: Material.primary
        onClicked: {
          GlobalOptions.onRefresh();
        }
      }

      ComboBox {
        id: combo
        Layout.fillWidth: true
        model: GlobalOptions.frameList
        currentIndex: 0
        editable: true
        editText: currentText
        displayText: currentText
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }

          GlobalOptions.setFrame(textAt(currentIndex));
        }

        Connections {
          target: GlobalOptions
          onSetCurrentIndex: {
            combo.currentIndex = index
            combo.editText = "world"
            combo.displayText = "world"
          }
        }
      }
    }

    RowLayout {
      width: parent.width
      spacing: 10

      Label {
        text: "Background Color"
        font.pointSize: 11
      }

      Button {
        Layout.preferredWidth: 20
        Layout.preferredHeight: 20
        onClicked: colorDialog.open()
        background: Rectangle {
          width: 20
          height: 20
          id: "bgColor"
          color: "#303030"
          border.color: "#000000"
          border.width: 2
        }
      }

      TextField {
        id: "bgColorHex"
        text: "#303030"
        Layout.fillWidth: true
        validator: RegExpValidator {
          regExp: /#([\da-f]{3}){1,2}/ig
        }
        onEditingFinished: {
          colorDialog.color = text
          bgColor.color = text
          GlobalOptions.setSceneBackground(text);
        }
      }
    }

    RowLayout {
      width: parent.width
      spacing: 10

      Label {
        id: "status"
        text: GlobalOptions.tfStatus.status
        color: GlobalOptions.tfStatus.color
        font.pointSize: 11
      }

      Label {
        id: "message"
        text: GlobalOptions.tfStatus.message
        color: GlobalOptions.tfStatus.color
        font.pointSize: 11
        Layout.fillWidth: true
        elide: Label.ElideRight
      }
    }
  }

  ColorDialog {
    id: colorDialog
    title: "Select scene background color"
    onAccepted: {
      bgColor.color = colorDialog.color
      bgColorHex.text = colorDialog.color
      GlobalOptions.setSceneBackground(colorDialog.color);
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }
}
