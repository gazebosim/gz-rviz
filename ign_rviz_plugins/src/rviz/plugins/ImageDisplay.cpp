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

#include <limits>
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
    this->qos,
    std::bind(&ImageDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::setTopic(const std::string & topic_name)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->topic_name = topic_name;

  this->subscribe();

  // Refresh combo-box on plugin load
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::setTopic(const QString & topic_name)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->topic_name = topic_name.toStdString();

  // Destroy previous subscription
  this->unsubscribe();

  // Create new subscription
  this->subscribe();
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::callback(const sensor_msgs::msg::Image::SharedPtr _msg)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  if (!_msg) {
    return;
  }

  this->msg = std::move(_msg);

  if (_msg->encoding == "bgr8") {
    updateFromBGR8();
  } else if (_msg->encoding == "rgb8") {
    updateFromRGB8();
  } else if (_msg->encoding == "mono8") {
    updateFromMONO8();
  } else if (_msg->encoding == "mono16") {
    updateFromMONO16();
  } else if (_msg->encoding == "32FC1") {
    updateFromFloat32();
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

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::updateFromMONO8()
{
  QImage image(&msg->data[0], msg->width, msg->height, msg->step, QImage::Format_Grayscale8);
  this->provider->SetImage(image);
  this->newImage();
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::updateFromMONO16()
{
  unsigned int height = msg->height;
  unsigned int width = msg->width;

  QImage image = QImage(width, height, QImage::Format_RGB888);

  unsigned int samples = width * height;

  unsigned int bufferSize = samples * sizeof(uint16_t);

  uint16_t * buffer = new uint16_t[samples];
  memcpy(buffer, &msg->data[0], bufferSize);

  // Get min and max of temperature values
  uint16_t min = std::numeric_limits<uint16_t>::max();
  uint16_t max = 0;
  for (unsigned int i = 0; i < samples; ++i) {
    uint16_t temp = buffer[i];
    if (temp > max) {
      max = temp;
    }
    if (temp < min) {
      min = temp;
    }
  }

  // Convert temperature to grayscale image
  double range = static_cast<double>(max - min);
  if (ignition::math::equal(range, 0.0)) {
    range = 1.0;
  }
  unsigned int idx = 0;
  for (unsigned int j = 0; j < height; ++j) {
    for (unsigned int i = 0; i < width; ++i) {
      uint16_t temp = buffer[idx++];
      double t = static_cast<double>(temp - min) / range;
      int r = 255 * t;
      int g = r;
      int b = r;
      QRgb value = qRgb(r, g, b);
      image.setPixel(i, j, value);
    }
  }

  this->provider->SetImage(image);
  this->newImage();

  delete[] buffer;
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::updateFromFloat32()
{
  unsigned int height = msg->height;
  unsigned int width = msg->width;

  QImage image = QImage(width, height, QImage::Format_RGB888);

  unsigned int depthSamples = width * height;

  unsigned int depthBufferSize = depthSamples * sizeof(float);

  float * depthBuffer = new float[depthSamples];

  memcpy(depthBuffer, &msg->data[0], depthBufferSize);

  float maxDepth = 0;
  for (unsigned int i = 0; i < depthSamples; ++i) {
    if (depthBuffer[i] > maxDepth && !std::isinf(depthBuffer[i])) {
      maxDepth = depthBuffer[i];
    }
  }
  unsigned int idx = 0;
  double factor = 255 / maxDepth;
  for (unsigned int j = 0; j < height; ++j) {
    for (unsigned int i = 0; i < width; ++i) {
      float d = depthBuffer[idx++];
      d = 255 - (d * factor);
      QRgb value = qRgb(d, d, d);
      image.setPixel(i, j, value);
    }
  }

  this->provider->SetImage(image);
  this->newImage();

  delete[] depthBuffer;
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::reset() {}

////////////////////////////////////////////////////////////////////////////////
QStringList ImageDisplay::getTopicList() const
{
  return this->topicList;
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::onRefresh()
{
  std::lock_guard<std::recursive_mutex>(this->lock);

  // Clear
  this->topicList.clear();

  int index = 0, position = 0;

  // Get topic list
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (topicType == "sensor_msgs/msg/Image") {
        this->topicList.push_back(QString::fromStdString(topic.first));
        if (topic.first == this->topic_name) {
          position = index;
        }
        index++;
      }
    }
  }
  // Update combo-box
  this->topicListChanged();
  emit setCurrentIndex(position);
}

////////////////////////////////////////////////////////////////////////////////
void ImageDisplay::updateQoS(
  const int & _depth, const int & _history, const int & _reliability,
  const int & _durability)
{
  std::lock_guard<std::recursive_mutex>(this->lock);
  this->setHistoryDepth(_depth);
  this->setHistoryPolicy(_history);
  this->setReliabilityPolicy(_reliability);
  this->setDurabilityPolicy(_durability);

  // Resubscribe with updated QoS profile
  this->unsubscribe();
  this->reset();
  this->subscribe();
}

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
