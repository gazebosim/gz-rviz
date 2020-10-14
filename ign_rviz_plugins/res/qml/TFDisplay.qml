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
import QtQml.Models 2.2
import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3

Item {
  // Tree Properties
  property int rowHeight: 30;

  property color highlightColor: Material.accentColor;

  property color evenColor: (Material.theme == Material.Light) ?
                            Material.color(Material.Grey, Material.Shade100):
                            Material.color(Material.Grey, Material.Shade800);

  property color oddColor: (Material.theme == Material.Light) ?
                            Material.color(Material.Grey, Material.Shade200):
                            Material.color(Material.Grey, Material.Shade900);

  Layout.minimumWidth: 250
  Layout.minimumHeight: 500
  anchors.fill: parent
  anchors.margins: 10

  id: tfDisplay

  ColumnLayout {
    width: parent.width
    Layout.fillWidth: true
    spacing: 2
    id: configColumn

    CheckBox {
      checked: true
      text: "Show Names"
      onClicked: { TFDisplay.showNames(checked) }
    }

    CheckBox {
      checked: true
      text: "Show Axes"
      onClicked: { TFDisplay.showAxes(checked) }
    }

    CheckBox {
      checked: false
      text: "Show Axes Head"
      onClicked: { TFDisplay.showAxesHead(checked) }
    }

    CheckBox {
      checked: true
      text: "Show Arrows"
      onClicked: { TFDisplay.showArrows(checked) }
    }

    RowLayout {
      width: parent.width
      spacing: 10

      Text {
        width: 80
        Layout.minimumWidth: 50
        text: "Marker Scale"
        font.pointSize: 10.5
      }

      TextField {
        id: markerScale
        Layout.fillWidth: true
        Layout.minimumWidth: 50
        width: 150
        placeholderText: "1.0"

        validator: RegExpValidator {
          // Integer and floating point numbers
          regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
        }

        onEditingFinished: {
          TFDisplay.setMarkerScale(markerScale.text)
        }
      }
    }
  }

  TreeView {
    id: tree
    model: TFDisplay.frameModel
    anchors.left: parent.left
    anchors.right: parent.right
    anchors.bottom: tfDisplay.bottom
    anchors.top: configColumn.bottom
    anchors.leftMargin: -10
    anchors.rightMargin: -10
    anchors.bottomMargin: -10
    anchors.topMargin: 10

    Layout.fillWidth: true
    Layout.fillHeight: true
    width: parent.width

    headerVisible: false
    headerDelegate: Rectangle {
      visible: false
    }

    frameVisible: false

    verticalScrollBarPolicy: Qt.ScrollBarAsNeeded
    horizontalScrollBarPolicy: Qt.ScrollBarAsNeeded

    // Selection
    selection: ItemSelectionModel {
      model: tree.model;
    }

    selectionMode: SelectionMode.SingleSelection;

    style: TreeViewStyle {
      transientScrollBars: true
    }

    TableViewColumn {
      role: "name"
      delegate: CheckDelegate {
        text: model.name
        checked: model.checked
        onClicked: {
          model.checked = checked;
          TFDisplay.setFrameVisibility(model.name, model.checked);
        }
      }
    }

    // Delegates
    rowDelegate: Rectangle
    {
      id: row
      color: (styleData.selected)? highlightColor : (styleData.row % 2 == 0) ? evenColor : oddColor;
      height: rowHeight;
    }
  }
}