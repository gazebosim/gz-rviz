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
import QtQml.Models 2.2
import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Dialogs 1.0

Item {
  // Tree Properties
  property int rowHeight: 30;

  property color highlightColor: Material.accentColor;

  property color evenColor: (Material.theme == Material.Light) ?
                            Material.color(Material.Grey, Material.Shade100):
                            Material.color(Material.Grey, Material.Shade800);

  property color oddColor: (Material.theme == Material.Light) ?
                            Material.color(Material.Grey, Material.Shade200):
                            Material.color(Material.Grey, Material.Shade900);

  Layout.minimumWidth: 280
  Layout.minimumHeight: 475
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

    TreeView {
      id: tree
      Layout.fillWidth: true
      Layout.fillHeight: true
      model: RobotModelDisplay.robotLinkModel

      headerVisible: false
      headerDelegate: Rectangle {
        visible: false
      }

      frameVisible: false

      verticalScrollBarPolicy: Qt.ScrollBarAsNeeded
      horizontalScrollBarPolicy: Qt.ScrollBarAsNeeded

      // Selection
      selection: ItemSelectionModel {
        model: tree.model;
      }

      selectionMode: SelectionMode.SingleSelection;

      style: TreeViewStyle {
        transientScrollBars: true
      }

      TableViewColumn {
        role: "name"
        delegate: CheckDelegate {
          text: model === null ? false : model.name
          checked: model === null ? false : model.checked
          onClicked: {
            model.checked = checked;
            RobotModelDisplay.setLinkVisibility(model.name, model.checked);
          }
        }
      }

      // Delegates
      rowDelegate: Rectangle
      {
        id: row
        color: (styleData.selected)? highlightColor : (styleData.row % 2 == 0) ? evenColor : oddColor;
        height: rowHeight;
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
