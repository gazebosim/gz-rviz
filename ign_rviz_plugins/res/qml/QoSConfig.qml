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

ColumnLayout {
  id: qosConfig
  signal profileUpdate(int depth, int history, int reliability, int durability);

  width: parent.width
  Layout.fillWidth: true

  RoundButton {
    Layout.fillWidth: true
    Layout.preferredHeight: 30
    text: configColumn.visible ? "\u25B4" : "\u25BE"
    Material.background: "#fafafa"
    onClicked: {
      configColumn.visible = !configColumn.visible;
    }
  }
  
  ColumnLayout {
    id: configColumn
    Layout.fillWidth: true
    visible: false

    RowLayout {
      width: parent.width

      Text {
        width: 75
        Layout.minimumWidth: 75
        text: "Depth"
        font.pointSize: 10.5
      }

      TextField {
        id: depthText
        Layout.fillWidth: true
        text: "5"
        validator: IntValidator {
          bottom: 0
        }
        onEditingFinished: {
          qosConfig.profileUpdate(depthText.text, historyCombo.currentIndex, reliabilityCombo.currentIndex, durabilityCombo.currentIndex)
        }
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 75
        Layout.minimumWidth: 75
        text: "History"
        font.pointSize: 10.5
      }

      ComboBox {
        id: historyCombo
        Layout.fillWidth: true
        currentIndex: 1
        model: [ "System Default", "Keep Last", "Keep All" ]
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }
          qosConfig.profileUpdate(depthText.text, historyCombo.currentIndex, reliabilityCombo.currentIndex, durabilityCombo.currentIndex)
        }
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 75
        Layout.minimumWidth: 75
        text: "Reliability"
        font.pointSize: 10.5
      }

      ComboBox {
        id: reliabilityCombo
        Layout.fillWidth: true
        currentIndex: 1
        model: [ "System Default", "Reliable", "Best Effort" ]
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }
          qosConfig.profileUpdate(depthText.text, historyCombo.currentIndex, reliabilityCombo.currentIndex, durabilityCombo.currentIndex)
        }
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 75
        Layout.minimumWidth: 75
        text: "Durability"
        font.pointSize: 10.5
      }

      ComboBox {
        id: durabilityCombo
        Layout.fillWidth: true
        currentIndex: 2
        model: [ "System Default", "Transient Local", "Volatile" ]
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }
          qosConfig.profileUpdate(depthText.text, historyCombo.currentIndex, reliabilityCombo.currentIndex, durabilityCombo.currentIndex)
        }
      }
    }
  }
}
