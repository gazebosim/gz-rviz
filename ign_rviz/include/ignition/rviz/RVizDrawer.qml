import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1

Rectangle {
    id: displayDrawer
    anchors.fill: parent

    function onAction(action) {
        switch(action) {
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
            case "addPointStampedDisplay":
                RViz.addPointStampedDisplay();
                break;
            case "addPolygonDisplay":
                RViz.addPolygonDisplay();
                break;
            case "addPoseDisplay":
                RViz.addPoseDisplay();
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
}
