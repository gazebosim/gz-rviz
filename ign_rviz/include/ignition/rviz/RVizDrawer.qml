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
            actionElement: "addAxesDisplay"
        }

        ListElement {
            title: "Grid"
            actionElement: "addGrid3D"
        }

        ListElement {
            title: "Image"
            actionElement: "addImageDisplay"
        }

        ListElement {
            title: "LaserScan"
            actionElement: "addLaserScanDisplay"
        }

        ListElement {
            title: "PointStamped"
            actionElement: "addPointStampedDisplay"
        }

        ListElement {
            title: "Polygon"
            actionElement: "addPolygonDisplay"
        }

        ListElement {
            title: "Pose"
            actionElement: "addPoseDisplay"
        }

        ListElement {
            title: "RobotModel"
            actionElement: "addRobotModelDisplay"
        }

        ListElement {
            title: "TF"
            actionElement: "addTFDisplay"
        }
    }

    ListView {
        id: displayListView
        anchors.fill: parent

        delegate: ItemDelegate {
            Material.theme: Material.theme
            width:parent.width
            text: title
            highlighted: ListView.isCurrentItem
            onClicked: {
                displayDrawer.onAction(actionElement);
                displayDrawer.parent.closeDrawer();
            }
        }

        model: displayDrawerModel
        ScrollIndicator.vertical: ScrollIndicator { }
    }
}
