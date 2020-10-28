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
import QtQuick.Controls.Material 2.1
import QtQuick.Window 2.0

Rectangle {
  id: displayDrawer
  anchors.fill: parent

  function onAction(action) {
    switch(action) {
      case "loadPluginByTopic":
        loadPluginByTopic();
        break;
      case "addAxesDisplay":
        RViz.addAxesDisplay();
        break;
      case "addGrid3D":
        RViz.addGrid3D();
        break;
      case "addGPSDisplay":
        RViz.addGPSDisplay();
        break;
      case "addImageDisplay":
        RViz.addImageDisplay();
        break;
      case "addLaserScanDisplay":
        RViz.addLaserScanDisplay();
        break;
      case "addMarkerDisplay":
        RViz.addMarkerDisplay();
        break;
      case "addMarkerArrayDisplay":
        RViz.addMarkerArrayDisplay();
        break;
      case "addPathDisplay":
        RViz.addPathDisplay();
        break;
      case "addPointStampedDisplay":
        RViz.addPointStampedDisplay();
        break;
      case "addPolygonDisplay":
        RViz.addPolygonDisplay();
        break;
      case "addPoseDisplay":
        RViz.addPoseDisplay();
        break;
      case "addPoseArrayDisplay":
        RViz.addPoseArrayDisplay();
        break;
      case "addRobotModelDisplay":
        RViz.addRobotModelDisplay();
        break;
      case "addTFDisplay":
        RViz.addTFDisplay();
        break;
      default:
        parent.onAction(action);
        break;
    }
  }

  ListModel {
    id: displayDrawerModel

    ListElement {
      title: "Add By Topic"
      icon: "icons/Add.png"
      actionElement: "loadPluginByTopic"
    }

    ListElement {
      title: "Axes"
      icon: "icons/Axes.png"
      actionElement: "addAxesDisplay"
    }

    ListElement {
      title: "Grid"
      icon: "icons/Grid.png"
      actionElement: "addGrid3D"
    }

    ListElement {
      title: "GPS"
      icon: "icons/NavSatFix.png"
      actionElement: "addGPSDisplay"
    }

    ListElement {
      title: "Image"
      icon: "icons/Image.png"
      actionElement: "addImageDisplay"
    }

    ListElement {
      title: "LaserScan"
      icon: "icons/LaserScan.png"
      actionElement: "addLaserScanDisplay"
    }

    ListElement {
      title: "Marker"
      icon: "icons/Marker.png"
      actionElement: "addMarkerDisplay"
    }

    ListElement {
      title: "MarkerArray"
      icon: "icons/MarkerArray.png"
      actionElement: "addMarkerArrayDisplay"
    }

    ListElement {
      title: "Path"
      icon: "icons/Path.png"
      actionElement: "addPathDisplay"
    }

    ListElement {
      title: "PointStamped"
      icon: "icons/PointStamped.png"
      actionElement: "addPointStampedDisplay"
    }

    ListElement {
      title: "Polygon"
      icon: "icons/Polygon.png"
      actionElement: "addPolygonDisplay"
    }

    ListElement {
      title: "Pose"
      icon: "icons/Pose.png"
      actionElement: "addPoseDisplay"
    }

    ListElement {
      title: "PoseArray"
      icon: "icons/PoseArray.png"
      actionElement: "addPoseArrayDisplay"
    }

    ListElement {
      title: "RobotModel"
      icon: "icons/RobotModel.png"
      actionElement: "addRobotModelDisplay"
    }

    ListElement {
      title: "TF"
      icon: "icons/TF.png"
      actionElement: "addTFDisplay"
    }
  }

  ListView {
    id: displayListView
    anchors.fill: parent

    delegate: Rectangle {
      id: delegateItem
      width: parent.width;
      height: 50
      color: delegateArea.containsMouse ? Material.color(Material.Grey, Material.Shade200) : "#fff"

      Image {
        id: imageItem
        height: 15
        width: 15
        anchors.left: parent.left
        anchors.verticalCenter: parent.verticalCenter
        anchors.leftMargin: 10
        source: icon
      }

      Text {
        id: textItem
        anchors.left: imageItem.right
        anchors.leftMargin: 10
        anchors.verticalCenter: parent.verticalCenter
        text: title
      }

      MouseArea {
        id: delegateArea
        anchors.fill: parent
        hoverEnabled: true
        onClicked: {
          displayDrawer.onAction(actionElement);
          displayDrawer.parent.closeDrawer();
        }
      }
    }

    model: displayDrawerModel
    ScrollIndicator.vertical: ScrollIndicator { }
  }

  /**
   *  @brief Load display by available topic
   *  Refresh topic list with available display topics
   */
  function loadPluginByTopic() {
    RViz.refreshTopicList();
    topicWindow.show();
  }

  /**
   * @brief Load plugin on selection
   * @param[in] _name Topic name
   * @param[in] _msgType Message type
   */
  function loadPlugin(_name, _msgType) {
    switch(_msgType) {
      case "geometry_msgs/msg/PointStamped": {
        RViz.addPointStampedDisplay(_name)
        break;
      }
      case "geometry_msgs/msg/PolygonStamped": {
        RViz.addPolygonDisplay(_name)
        break;
      }
      case "geometry_msgs/msg/PoseStamped": {
        RViz.addPoseDisplay(_name)
        break;
      }
      case "geometry_msgs/msg/PoseArray": {
        RViz.addPoseArrayDisplay(_name)
        break;
      }
      case "nav_msgs/msg/Path": {
        RViz.addPathDisplay(_name)
        break;
      }
      case "sensor_msgs/msg/Image": {
        RViz.addImageDisplay(_name)
        break;
      }
      case "sensor_msgs/msg/LaserScan": {
        RViz.addLaserScanDisplay(_name)
        break;
      }
      case "visualization_msgs/msg/Marker": {
        RViz.addMarkerDisplay(_name)
        break;
      }
      case "visualization_msgs/msg/MarkerArray": {
        RViz.addMarkerArrayDisplay(_name)
        break;
      }
      case "sensor_msgs/msg/NavSatFix": {
        RViz.addGPSDisplay(_name)
        break;
      }
    }
  }

  // Select topic window
  ApplicationWindow {
    id: topicWindow
    title: "Select Topic"
    width: 500
    height: 320
    color: "#fff"

    Rectangle {
      anchors.fill: parent
      color: "#fff"
      visible: topicList.count == 0
      Text {
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        text: "No topics available"
        font.pointSize: 14
      }
    }

    ListView {
      anchors.fill: parent
      model: RViz.topicModel
      id: topicList
      visible: topicList.count != 0
      delegate: Rectangle {
        height: 40
        width: parent.width
        color: topicArea.containsMouse ? Material.color(Material.Grey, Material.Shade200) : "#fff"
        Row {
          width: parent.width
          height: parent.height
          spacing: 10
          anchors.left: parent.left
          anchors.leftMargin: 10

          Text {
            width: parent.width - 160
            anchors.verticalCenter: parent.verticalCenter
            text: model.topic
            clip: true
          }

          Image {
            height: 15
            width: 15
            anchors.verticalCenter: parent.verticalCenter
            source: {
              var type = model.msgType.split("/")[2]
              if(type != "PointStamped") {
                type = type.replace("Stamped", "")
              }
              return "icons/" + type + ".png"
            }
          }

          Text {
            width: 145
            anchors.verticalCenter: parent.verticalCenter
            clip: true
            text: {
              var type = model.msgType.split("/")[2]
              return (type == "PointStamped") ? type : type.replace("Stamped", "")
            }
          }
        }

        MouseArea {
          id: topicArea
          anchors.fill: parent
          hoverEnabled: true
          onClicked: {
            loadPlugin(model.topic, model.msgType);
            topicWindow.close();
          }
        }
      }

      ScrollIndicator.vertical: ScrollIndicator { }
    }

    // Center on sceen
    Component.onCompleted: {
      setX(Screen.width / 2 - width / 2);
      setY(Screen.height / 2 - height / 2);
    }
  }
}
