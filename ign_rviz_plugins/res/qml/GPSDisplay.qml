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
import QtLocation 5.6
import QtPositioning 5.6
import "qrc:/QoSConfig"

Item {
  property double lat: 0.0
  property double lng: 0.0
  property bool centering: true

  Layout.minimumWidth: 280
  Layout.minimumHeight: 455
  anchors.topMargin: 5
  anchors.leftMargin: 5
  anchors.rightMargin: 5
  anchors.fill: parent
  id: gpsDisplay

  ColumnLayout {
    id: configColumn
    width: parent.width

    RowLayout {
      width: parent.width

      Layout.fillWidth: true
      Layout.fillHeight: true

      RoundButton {
        text: "\u21bb"
        Material.background: Material.primary
        onClicked: {
          GPSDisplay.onRefresh();
        }
      }

      ComboBox {
        id: combo
        Layout.fillWidth: true
        model: GPSDisplay.topicList
        currentIndex: 0
        editable: true
        editText: currentText
        displayText: currentText
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }

          GPSDisplay.setTopic(textAt(currentIndex));
        }

        Component.onCompleted: {
          combo.editText = "/gps"
          combo.displayText = "/gps"
        }

        Connections {
          target: GPSDisplay
          onSetCurrentIndex: {
            combo.currentIndex = index
          }
        }
      }
    }

    QoSConfig {
      id: qos
      onProfileUpdate: {
        GPSDisplay.updateQoS(depth, history, reliability, durability)
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 75
        Layout.minimumWidth: 75
        text: "Map Type"
        font.pointSize: 10.5
      }

      ComboBox {
        id: typeCombo
        Layout.fillWidth: true
        currentIndex: 0
        model: [ "Default", "Satellite" ]
        onCurrentIndexChanged: {
          if (currentIndex == 0) {
            map.activeMapType = map.supportedMapTypes[2]
          } else if (currentIndex == 1) {
            map.activeMapType = map.supportedMapTypes[5]
          }
        }
      }
    }

    RowLayout {
      width: parent.width
      Text {
        Layout.fillWidth: true
        text: "<b>Lat: </b>" + lat.toFixed(6)
      }

      Text {
        Layout.fillWidth: true
        text: "<b>Lng: </b>" + lng.toFixed(6)
      }
    }
  }

  Map {
    id: map
    anchors.top: configColumn.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right

    anchors.topMargin: 5
    anchors.leftMargin: -5
    anchors.rightMargin: -5

    Layout.fillHeight: true
    Layout.fillWidth: true

    MapCircle {
      id: outerCircle
      center {
        latitude: lat
        longitude: lng
      }
      radius: 0.0
      color: '#4285f4'
      opacity: 0.25
      border.width: 3
      border.color: "#4285f4"
    }

    MapCircle {
      id: circle
      center {
        latitude: lat
        longitude: lng
      }
      radius: 23.0
      color: '#4285f4'
      border.width: 3
      border.color: "white"
    }

    plugin: Plugin {
      id: mapPlugin
      name: "mapboxgl"
    }
    activeMapType: supportedMapTypes[2]

    center: centering ? QtPositioning.coordinate(lat, lng) : center
    copyrightsVisible: false
    zoomLevel: 16

    gesture.onPanStarted: {
      centering = false
    }

    onZoomLevelChanged: {
      // Scaling the location marker according to zoom level.
      // Marker radius = meters per pixel * 10
      circle.radius = 156543.03392 * Math.cos(lat * Math.PI / 180) / Math.pow(2, map.zoomLevel) * 10
    }
  }

  RoundButton {
    height: 40
    width: 40
    anchors.top: map.top
    anchors.right: map.right

    text: "\u2316"
    font.pixelSize: 25

    Material.background: "#fafafa"
    Material.foreground: centering ? "#4285f4" : "black"

    onClicked: {
      centering = true
    }

    hoverEnabled: true

    ToolTip.delay: 1000
    ToolTip.timeout: 5000
    ToolTip.visible: hovered
    ToolTip.text: qsTr("Center")
  }

  Connections {
    target: GPSDisplay
    onCoordinateChanged: {
      lat = latitude
      lng = longitude
      outerCircle.radius = Math.sqrt(covariance)
    }
  }
}
