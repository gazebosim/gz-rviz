// Copyright (c) 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ignition/rviz/plugins/ImageDisplay.hpp"

#include <ignition/gui/Application.hh>
#include <ignition/plugin/Register.hh>

#include <string>
#include <utility>

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
ImageDisplay::ImageDisplay()
: MessageDisplay() {}

////////////////////////////////////////////////////////////////////////////////
ImageDisplay::~ImageDisplay() {}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->node = std::move(_node);
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::subscribe()
{
  this->subscriber = this->node->create_subscription<sensor_msgs::msg::Image>(
    this->topic_name,
    10,
    std::bind(&ImageDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::setTopic(std::string topic_name)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->topic_name = topic_name;

  this->subscribe();
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::callback(const sensor_msgs::msg::Image::SharedPtr _msg)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->msg = std::move(_msg);

  if (_msg->encoding == "bgr8") {
    updateFromBGR8();
  } else if (_msg->encoding == "rgb8") {
    updateFromRGB8();
  } else if (_msg->encoding == "mono8") {
    updateFromMONO8();
  } else {
    RCLCPP_ERROR(
      this->node->get_logger(), "Unsupported image encoding: %s",
      _msg->encoding.c_str());
  }
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::updateFromBGR8()
{
  QImage image(&msg->data[0], msg->width, msg->height, msg->step, QImage::Format_RGB888);
  image = image.rgbSwapped();
  this->provider->SetImage(image);
  this->newImage();
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::updateFromRGB8()
{
  QImage image(&msg->data[0], msg->width, msg->height, msg->step, QImage::Format_RGB888);
  this->provider->SetImage(image);
  this->newImage();
}

void ImageDisplay::updateFromMONO8()
{
  QImage image(&msg->data[0], msg->width, msg->height, msg->step, QImage::Format_Grayscale8);
  this->provider->SetImage(image);
  this->newImage();
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::reset() {}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "Image Display";
  }

  this->provider = new ImageProvider();
  gui::App()->Engine()->addImageProvider(
    this->CardItem()->objectName() + "imagedisplay", this->provider);
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition


IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::ImageDisplay,
  ignition::gui::Plugin)
