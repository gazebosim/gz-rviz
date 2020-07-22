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

Item {
  Layout.minimumWidth: 250
  Layout.minimumHeight: 180
  anchors.fill: parent
  anchors.margins: 10
  ColumnLayout {
    width: parent.width

    RowLayout {
      width: parent.width
      RoundButton {
        text: "\u21bb"
        Material.background: Material.primary
        onClicked: {
          LaserScanDisplay.onRefresh();
        }
      }

      ComboBox {
        id: combo
        Layout.fillWidth: true
        model: LaserScanDisplay.topicList
        currentIndex: 0
        editable: true
        editText: currentText
        displayText: currentText
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }

          LaserScanDisplay.setTopic(textAt(currentIndex));
        }

        Component.onCompleted: {
          combo.editText = "/scan"
          combo.displayText = "/scan"
        }

        Connections {
          target: LaserScanDisplay
          onSetCurrentIndex: {
            combo.currentIndex = index
          }
        }
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 50
        Layout.minimumWidth: 50
        anchors.left: parent.left
        anchors.leftMargin: 2
        text: "Type"
        font.pointSize: 10.5
      }

      ComboBox {
        id: typeCombo
        Layout.fillWidth: true
        currentIndex: 0
        model: [ "Points", "Rays", "Triangles" ]
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }

          LaserScanDisplay.setVisualType(currentIndex);
        }
      }
    }
  }
}
