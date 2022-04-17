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
import QtQuick.Dialogs 1.0
import "qrc:/QoSConfig"

Item {
  Layout.minimumWidth: 250
  Layout.minimumHeight: 1175
  anchors.fill: parent
  anchors.margins: 10
  Column {
    width: parent.width

    RowLayout {
      width: parent.width
      RoundButton {
        text: "\u21bb"
        Material.background: Material.primary
        onClicked: {
          PoseWithCovarianceDisplay.onRefresh();
        }
      }

      ComboBox {
        id: combo
        Layout.fillWidth: true
        model: PoseWithCovarianceDisplay.topicList
        currentIndex: 0
        editable: true
        editText: currentText
        displayText: currentText
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }

          PoseWithCovarianceDisplay.setTopic(textAt(currentIndex));
        }

        Component.onCompleted: {
          combo.editText = "/posewithcov"
          combo.displayText = "/posewithcov"
        }

        Connections {
          target: PoseWithCovarianceDisplay
          onSetCurrentIndex: {
            combo.currentIndex = index
          }
        }
      }
    }

    QoSConfig {
      onProfileUpdate: {
        PoseWithCovarianceDisplay.updateQoS(depth, history, reliability, durability)
      }
    }

    RowLayout {
      width: parent.width

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Shape"
        font.pointSize: 10.5
      }

      ComboBox {
        id: shapeCombo
        Layout.fillWidth: true
        currentIndex: 0
        model: [ "Arrow", "Axis" ]
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            return;
          }
          PoseWithCovarianceDisplay.setShape(currentIndex === 0)
        }
      }
    }

    ColumnLayout {
      id: arrowConfig
      visible: shapeCombo.currentIndex === 0
      width: parent.width
      Layout.fillWidth: true

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 80
          Layout.minimumWidth: 80
          text: "Color"
          font.pointSize: 10.5
        }

        Button {
          Layout.preferredWidth: 20
          Layout.preferredHeight: 20
          onClicked: colorDialog.open()
          background: Rectangle {
            width: 20
            height: 20
            id: bgColor
            color: "#ff1900"
            border.color: "#000000"
            border.width: 2
          }
        }

        TextField {
          id: colorTextField
          text: "#ff1900"
          Layout.fillWidth: true
          validator: RegExpValidator {
            regExp: /#([\da-f]{3}){1,2}/ig
          }
          onAccepted: {
            colorDialog.color = text
            bgColor.color = text
            PoseWithCovarianceDisplay.setColor(text);
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10
        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Alpha"
          font.pointSize: 10.5
        }

        TextField {
          id: alphaTextField
          Layout.fillWidth: true
          text: "1.0"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            colorDialog.color.a = alphaTextField.text;
            bgColor.color = colorDialog.color
            PoseWithCovarianceDisplay.setColor(colorDialog.color);
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Shaft Length"
          font.pointSize: 10.5
        }

        TextField {
          id: shaftLength
          Layout.fillWidth: true
          width: 150
          text: "1.0"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseWithCovarianceDisplay.setArrowDimensions(shaftLength.text, shaftRadius.text, headLength.text, headRadius.text)
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Shaft Radius"
          font.pointSize: 10.5
        }

        TextField {
          id: shaftRadius
          Layout.fillWidth: true
          width: 150
          text: "0.05"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseWithCovarianceDisplay.setArrowDimensions(shaftLength.text, shaftRadius.text, headLength.text, headRadius.text)
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Head Length"
          font.pointSize: 10.5
        }

        TextField {
          id: headLength
          Layout.fillWidth: true
          width: 150
          text: "0.25"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseWithCovarianceDisplay.setArrowDimensions(shaftLength.text, shaftRadius.text, headLength.text, headRadius.text)
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Head Radius"
          font.pointSize: 10.5
        }

        TextField {
          id: headRadius
          Layout.fillWidth: true
          width: 150
          text: "0.1"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseWithCovarianceDisplay.setArrowDimensions(shaftLength.text, shaftRadius.text, headLength.text, headRadius.text)
          }
        }
      }
    }

    ColumnLayout {
      id: axisConfig
      visible: shapeCombo.currentIndex === 1
      width: parent.width
      Layout.fillWidth: true

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Length"
          font.pointSize: 10.5
        }

        TextField {
          id: axisLengthField
          Layout.fillWidth: true
          width: 150
          text: "1.0"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseWithCovarianceDisplay.setAxisDimensions(axisLengthField.text, axisRadiusField.text)
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          width: 110
          Layout.minimumWidth: 110
          text: "Radius"
          font.pointSize: 10.5
        }

        TextField {
          id: axisRadiusField
          Layout.fillWidth: true
          width: 150
          text: "0.1"
          validator: RegExpValidator {
            // Integer and floating point numbers
            regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
          }
          onAccepted: {
            PoseWithCovarianceDisplay.setAxisDimensions(axisLengthField.text, axisRadiusField.text)
          }
        }
      }

      CheckBox {
        checked: false
        text: qsTr("Show axis head")
        onClicked: { PoseWithCovarianceDisplay.setAxisHeadVisibility(checked) }
      }
    }
    
    // TODO: Create proper, reusable collapsible menu
    // covariance visibility
    CheckBox {
      checked: true
      text: qsTr("Visualize covariance")
      onClicked: { PoseWithCovarianceDisplay.setCovVisible(checked) }
    }
    
    // position covariance visibility
    CheckBox {
      checked: true
      text: qsTr("Visualize position covariance")
      onClicked: { PoseWithCovarianceDisplay.setPosCovVisible(checked) }
    }
    
    // Local/Fixed frame position cov
    RowLayout {
      width: parent.width

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Frame"
        font.pointSize: 10.5
      }

      ComboBox {
        id: posCovFrameCombo
        Layout.fillWidth: true
        currentIndex: 0
        model: [ "Local", "Fixed" ]
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            console.log("invalid position covariance combobox index " + currentIndex);
            return;
          }
          PoseWithCovarianceDisplay.setPosCovFrame(currentIndex === 0)
        }
      }
    }

    // Position covariance ellipse color
    RowLayout {
      Layout.fillWidth: true
      spacing: 10

      Text {
        width: 80
        Layout.minimumWidth: 80
        text: "Color"
        font.pointSize: 10.5
      }

      Button {
        Layout.preferredWidth: 20
        Layout.preferredHeight: 20
        onClicked: posCovColorDialog.open()
        background: Rectangle {
          width: 20
          height: 20
          id: posCovColorBtn
          color: "#cc33cc"
          border.color: "#000000"
          border.width: 2
        }
      }

      TextField {
        id: posCovColorTxt
        text: "#cc33cc"
        Layout.fillWidth: true
        validator: RegExpValidator {
          regExp: /#([\da-f]{3}){1,2}/ig
        }
        onAccepted: {
          posCovColorDialog.color = text
          posCovColorBtn.color = text
          PoseWithCovarianceDisplay.setPosCovColor(text);
        }
      }
    }

    // Position covariance ellipse alpha
    RowLayout {
      Layout.fillWidth: true
      spacing: 10
      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Alpha"
        font.pointSize: 10.5
      }

      TextField {
        id: posCovColorAlpha
        Layout.fillWidth: true
        text: "0.3"
        validator: RegExpValidator {
          // Integer and floating point numbers
          regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
        }
        onAccepted: {
          posCovColorDialog.color.a = posCovColorAlpha.text;
          posCovColorBtn.color = posCovColorDialog.color
          PoseWithCovarianceDisplay.setPosCovColor(posCovColorDialog.color);
        }
      }
    }

    // position covariance ellipse scale
    RowLayout {
      Layout.fillWidth: true
      spacing: 10

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Scale"
        font.pointSize: 10.5
      }

      TextField {
        id: posCovScale
        Layout.fillWidth: true
        width: 150
        text: "1"
        validator: RegExpValidator {
          // Integer and floating point numbers
          regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
        }
        onAccepted: {
          PoseWithCovarianceDisplay.setPosCovScale(posCovScale.text)
        }
      }
    }

    // orientation covariance visibility
    CheckBox {
      checked: true
      text: qsTr("Visualize orientation covariance")
      onClicked: { PoseWithCovarianceDisplay.setRotCovVisible(checked) }
    }

    // Local/Fixed frame orientation cov
    RowLayout {
      width: parent.width

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Frame"
        font.pointSize: 10.5
      }

      ComboBox {
        id: rotCovFrameCombo
        Layout.fillWidth: true
        currentIndex: 0
        model: [ "Local", "Fixed" ]
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            console.log("invalid orientation covariance frame combobox index " + currentIndex);
            return;
          }
          PoseWithCovarianceDisplay.setRotCovFrame(currentIndex === 0)
        }
      }
    }
    
    // Unique/RGB orientation cov color
    RowLayout {
      width: parent.width

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Color Style"
        font.pointSize: 10.5
      }

      ComboBox {
        id: rotCovColorStyleCombo
        Layout.fillWidth: true
        currentIndex: 0
        model: [ "Unique", "RGB" ]
        onCurrentIndexChanged: {
          if (currentIndex < 0) {
            console.log("invalid orientation covariance color style combobox index " + currentIndex);
            return;
          }
          PoseWithCovarianceDisplay.setRotCovColorStyle(currentIndex === 0)
        }
      }
    }
    
    // Orientation covariance ellipse color
    RowLayout {
      Layout.fillWidth: true
      spacing: 10

      Text {
        width: 80
        Layout.minimumWidth: 80
        text: "Color"
        font.pointSize: 10.5
      }

      Button {
        Layout.preferredWidth: 20
        Layout.preferredHeight: 20
        onClicked: rotCovColorDialog.open()
        background: Rectangle {
          width: 20
          height: 20
          id: rotCovColorBtn
          color: "#ffff7f"
          border.color: "#000000"
          border.width: 2
        }
      }

      TextField {
        id: rotCovColorTxt
        text: "#ffff7f"
        Layout.fillWidth: true
        validator: RegExpValidator {
          regExp: /#([\da-f]{3}){1,2}/ig
        }
        onAccepted: {
          rotCovColorDialog.color = text
          rotCovColorBtn.color = text
          PoseWithCovarianceDisplay.setRotCovColor(text);
        }
      }
    }

    // Orientation covariance ellipse alpha
    RowLayout {
      Layout.fillWidth: true
      spacing: 10
      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Alpha"
        font.pointSize: 10.5
      }

      TextField {
        id: rotCovColorAlpha
        Layout.fillWidth: true
        text: "0.3"
        validator: RegExpValidator {
          // Integer and floating point numbers
          regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
        }
        onAccepted: {
          rotCovColorDialog.color.a = rotCovColorAlpha.text;
          rotCovColorBtn.color = rotCovColorDialog.color
          PoseWithCovarianceDisplay.setRotCovColor(rotCovColorDialog.color);
        }
      }
    }
    
    // orientation covariance ellipse offset
    RowLayout {
      Layout.fillWidth: true
      spacing: 10

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Offset"
        font.pointSize: 10.5
      }

      TextField {
        id: rotCovOffset
        Layout.fillWidth: true
        width: 150
        text: "1"
        validator: RegExpValidator {
          // Integer and floating point numbers
          regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
        }
        onAccepted: {
          PoseWithCovarianceDisplay.setRotCovOffset(rotCovOffset.text)
        }
      }
    }
    
    // orientation covariance ellipse scale
    RowLayout {
      Layout.fillWidth: true
      spacing: 10

      Text {
        width: 110
        Layout.minimumWidth: 110
        text: "Scale"
        font.pointSize: 10.5
      }

      TextField {
        id: rotCovScale
        Layout.fillWidth: true
        width: 150
        text: "1"
        validator: RegExpValidator {
          // Integer and floating point numbers
          regExp: /^([0-9]*\.[0-9]+|[0-9]+)$/g
        }
        onAccepted: {
          PoseWithCovarianceDisplay.setRotCovScale(rotCovScale.text)
        }
      }
    }

  }

  ColorDialog {
    id: colorDialog
    title: "Select arrow visual color"
    color: "#ff1900"
    showAlphaChannel: false
    onAccepted: {
      bgColor.color = colorDialog.color
      colorTextField.text = colorDialog.color
      colorDialog.color.a = alphaTextField.text
      PoseWithCovarianceDisplay.setColor(colorDialog.color);
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }

  ColorDialog {
    id: posCovColorDialog
    title: "Select position covariance ellipse visual color"
    color: "#cc33cc"
    showAlphaChannel: false
    onAccepted: {
      posCovColorBtn.color = posCovColorDialog.color
      posCovColorTxt.text = posCovColorDialog.color
      posCovColorDialog.color.a = posCovColorAlpha.text
      PoseWithCovarianceDisplay.setPosCovColor(posCovColorDialog.color);
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }

  ColorDialog {
    id: rotCovColorDialog
    title: "Select orientation covariance ellipse visual color"
    color: "#ffff7f"
    showAlphaChannel: false
    onAccepted: {
      rotCovColorBtn.color = rotCovColorDialog.color
      rotCovColorTxt.text = rotCovColorDialog.color
      rotCovColorDialog.color.a = rotCovColorAlpha.text
      PoseWithCovarianceDisplay.setRotCovColor(rotCovColorDialog.color);
    }
    onRejected: {
      console.log("Canceled")
    }
    Component.onCompleted: visible = false
  }
}
