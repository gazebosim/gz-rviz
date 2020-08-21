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
      case "addImageDisplay":
        RViz.addImageDisplay();
        break;
      case "addLaserScanDisplay":
        RViz.addLaserScanDisplay();
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

  // Select topic window
  ApplicationWindow {
    id: topicWindow
    title: "Select Topic"
    width: 550
    height: 300

    ListView {
      anchors.fill: parent
      model: RViz.topicModel
      delegate: ItemDelegate {
        width: parent.width
        text: model.topic + "  :  " + model.msgType
        onClicked: {
          switch(model.msgType) {
            case "geometry_msgs/msg/PointStamped": {
              RViz.addPointStampedDisplay(model.topic)
              break;
            }
            case "geometry_msgs/msg/PolygonStamped": {
              RViz.addPolygonDisplay(model.topic)
              break;
            }
            case "geometry_msgs/msg/PoseStamped": {
              RViz.addPoseDisplay(model.topic)
              break;
            }
            case "geometry_msgs/msg/PoseArray": {
              RViz.addPoseArrayDisplay(model.topic)
              break;
            }
            case "nav_msgs/msg/Path": {
              RViz.addPathDisplay(model.topic)
              break;
            }
            case "sensor_msgs/msg/Image": {
              RViz.addImageDisplay(model.topic)
              break;
            }
            case "sensor_msgs/msg/LaserScan": {
              RViz.addLaserScanDisplay(model.topic)
              break;
            }
          }
          topicWindow.close();
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
