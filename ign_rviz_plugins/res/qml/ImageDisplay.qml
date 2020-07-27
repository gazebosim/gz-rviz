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
import "qrc:/QoSConfig"

Item {
  Layout.minimumWidth: 280
  Layout.minimumHeight: 345
  anchors.topMargin: 5
  anchors.leftMargin: 5
  anchors.rightMargin: 5
  anchors.fill: parent
  id:imageDisplay

  property string uniqueName: ""

  onParentChanged: {
    if (undefined === parent)
      return;

    uniqueName = parent.card().objectName + "imagedisplay";
    image.reload();
  }

  Connections {
    target: ImageDisplay
    onNewImage: image.reload();
  }

  RowLayout {
    width: parent.width
    id: configRow

    Layout.fillWidth: true
    Layout.fillHeight: true

    RoundButton {
      text: "\u21bb"
      Material.background: Material.primary
      onClicked: {
        ImageDisplay.onRefresh();
      }
    }

    ComboBox {
      id: combo
      Layout.fillWidth: true
      model: ImageDisplay.topicList
      currentIndex: 0
      editable: true
      editText: currentText
      displayText: currentText
      onCurrentIndexChanged: {
        if (currentIndex < 0) {
          return;
        }

        ImageDisplay.setTopic(textAt(currentIndex));
      }

      Component.onCompleted: {
        combo.editText = "/image"
        combo.displayText = "/image"
      }

      Connections {
        target: ImageDisplay
        onSetCurrentIndex: {
          combo.currentIndex = index
        }
      }
    }
  }

  QoSConfig {
    id: qos
    anchors.top: configRow.bottom
    onProfileUpdate: {
      ImageDisplay.updateQoS(depth, history, reliability, durability)
    }
  }

  Image {
    id: image
    anchors.top: qos.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right

    anchors.topMargin: 2
    anchors.leftMargin: -5
    anchors.rightMargin: -5

    Layout.fillHeight: true
    Layout.fillWidth: true

    fillMode: Image.PreserveAspectFit
    verticalAlignment: Image.AlignTop
    function reload() {
      // Force image request to C++
      source = "image://" + uniqueName + "/" + Math.random().toString(36).substr(2, 5);
    }
  }
}
