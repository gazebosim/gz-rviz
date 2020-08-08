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
  Layout.minimumHeight: 350
  anchors.fill: parent
  anchors.margins: 10

  ColumnLayout {
    width: parent.width

    CheckBox {
      checked: true
      text: "Visual Enabled"
      onClicked: {
        RobotModelDisplay.visualEnabled(checked)
      }
    }

    CheckBox {
      checked: false
      text: "Collision Enabled"
      onClicked: {
        RobotModelDisplay.collisionEnabled(checked)
      }
    }

    RowLayout {
      Layout.fillWidth: true
      spacing: 10

      Text {
        text: "Alpha"
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
          RobotModelDisplay.setAlpha(text);
        }
      }
    }

    RowLayout {
      Layout.fillWidth: true
      spacing: 10

      Text {
        text: "Description Source"
      }

      ComboBox {
        id: sourceCombo
        Layout.fillWidth: true
        model: [ "Topic", "File" ]
        currentIndex: 0
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }

          RobotModelDisplay.sourceChanged(currentIndex);

          if(currentIndex === 1) {
            RobotModelDisplay.openFile(fileText.text);
          }
        }
      }
    }

    ColumnLayout {
      visible: sourceCombo.currentIndex === 0
      RowLayout {
        width: parent.width
        RoundButton {
          text: "\u21bb"
          Material.background: Material.primary
          onClicked: {
            RobotModelDisplay.onRefresh();
          }
        }

        ComboBox {
          id: combo
          Layout.fillWidth: true
          model: RobotModelDisplay.topicList
          currentIndex: 0
          editable: true
          editText: currentText
          displayText: currentText
          onCurrentIndexChanged: {
            if (currentIndex < 0) {
              return;
            }

            RobotModelDisplay.setTopic(textAt(currentIndex));
          }

          Component.onCompleted: {
            combo.editText = "/robot_description"
            combo.displayText = "/robot_description"
          }

          Connections {
            target: RobotModelDisplay
            onSetCurrentIndex: {
              combo.currentIndex = index
            }
          }
        }
      }
    }

    RowLayout {
      visible: sourceCombo.currentIndex === 1
      width: parent.width
      spacing: 10

      Text {
        text: "Description File"
      }

      TextField {
        Layout.fillWidth: true
        id: fileText
        text: ""
        onAccepted: {
          RobotModelDisplay.openFile(text);
        }
      }

      Button {
        id: fileButton
        text: "\u2505"
        font.pointSize: 11
        Layout.preferredWidth: 30
        Layout.preferredHeight: 30
        onClicked: {
          fileDialog.open();
        }
      }
    }
  }

  FileDialog {
    id: fileDialog
    title: "Please choose a file"
    folder: shortcuts.home
    selectMultiple: false
    nameFilters: [ "URDF (*.urdf)", "All files (*)" ]
    onAccepted: {
      fileText.text = fileDialog.fileUrl
      RobotModelDisplay.openFile(fileText.text)
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }

}
