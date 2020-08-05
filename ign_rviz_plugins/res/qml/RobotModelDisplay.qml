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
  Layout.minimumHeight: 200
  anchors.fill: parent
  anchors.margins: 10

  RowLayout {
    width: parent.width
    Button {
      id: fileButton
      text: "Open"
      onClicked: {
        fileDialog.open();
      }
    }

    Text {
      Layout.fillWidth: true
      id: fileText
      text: ""
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
      RobotModelDisplay.openFile(fileDialog.fileUrl)
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }

}
