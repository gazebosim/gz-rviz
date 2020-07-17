import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1

Rectangle {
    id: displayDrawer
    anchors.fill: parent

    function onAction(action) {
        switch(action) {
            case "addGrid3D":
                RViz.addGrid3D();
                break;
            case "addTFDisplay":
                RViz.addTFDisplay();
                break;
            case "addLaserScanDisplay":
                RViz.addLaserScanDisplay();
                break;
            case "addAxesDisplay":
                RViz.addAxesDisplay();
                break;
            case "addImageDisplay":
                RViz.addImageDisplay();
                break;
            default:
                parent.onAction(action);
                break;
        }
    }

    ListModel {
        id: displayDrawerModel

        ListElement {
            title: "Grid"
            actionElement: "addGrid3D"
        }

        ListElement {
            title: "TF"
            actionElement: "addTFDisplay"
        }

        ListElement {
            title: "LaserScan"
            actionElement: "addLaserScanDisplay"
        }

        ListElement {
            title: "Axes"
            actionElement: "addAxesDisplay"
        }

        ListElement {
            title: "Image"
            actionElement: "addImageDisplay"
        }

        ListElement {
            title: "Exit"
            actionElement: "close"
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
