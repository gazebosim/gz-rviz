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

#ifndef IGNITION__RVIZ__PLUGINS__IMAGEDISPLAY_HPP_
#define IGNITION__RVIZ__PLUGINS__IMAGEDISPLAY_HPP_

#include <sensor_msgs/msg/image.hpp>

#include <QQuickImageProvider>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <utility>

#include "ignition/rviz/plugins/message_display_base.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
////////////////////////////////////////////////////////////////////////////////
class ImageProvider : public QQuickImageProvider
{
public:
  ImageProvider()
  : QQuickImageProvider(QQuickImageProvider::Image) {}

  QImage requestImage(const QString &, QSize *, const QSize &) override
  {
    if (!this->img.isNull()) {
      // Must return a copy
      QImage copy(this->img);
      return copy;
    }

    // Placeholder in case we have no image yet
    QImage i(400, 400, QImage::Format_RGB888);
    i.fill(QColor(128, 128, 128, 100));
    return i;
  }

public:
  void SetImage(const QImage & _image)
  {
    this->img = _image;
  }

private:
  QImage img;
};

////////////////////////////////////////////////////////////////////////////////
class ImageDisplay : public MessageDisplay<sensor_msgs::msg::Image>
{
  Q_OBJECT

  /**
   *  @brief Topic List
   */
  Q_PROPERTY(
    QStringList topicList
    READ getTopicList
    NOTIFY topicListChanged
  )

public:
  // Constructor
  ImageDisplay();

  // Destructor
  ~ImageDisplay();

  // Documentation Inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation Inherited
  void initialize(rclcpp::Node::SharedPtr _node) override;

  // Documentation Inherited
  void callback(const sensor_msgs::msg::Image::SharedPtr _msg) override;

  // Documentation inherited
  void setTopic(std::string topic_name) override;

  // Documentation inherited
  void subscribe() override;

  // Documentation inherited
  void reset() override;

  /**
   * @brief Set ROS Subscriber topic through GUI
   * @param[in] topic_name ROS Topic Name
   */
  Q_INVOKABLE void setTopic(QString topic_name);

public slots:
  /**
   * @brief Callback when refresh button is pressed.
   */
  void onRefresh();

  /**
   * @brief Get the topic list as a string
   * @return List of topics
   */
  Q_INVOKABLE QStringList getTopicList() const;

signals:
  /**
   * @brief Notify that topic list has changed
   */
  void topicListChanged();

signals:
  /**
   * @brief Set combo box index
   * @param index Combo box index
   */
  void setCurrentIndex(const int index);

signals:
  /**
   * @brief  Notify that a new image has been received
   */
  void newImage();

private:
  // Handle image with rgb8 encoding
  void updateFromRGB8();

  // Handle image with bgr8 encoding
  void updateFromBGR8();

  // Handle image with mono8 encoding
  void updateFromMONO8();

public:
  ImageProvider * provider{nullptr};

private:
  std::recursive_mutex lock;
  sensor_msgs::msg::Image::SharedPtr msg;
  QStringList topicList;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition
#endif  // IGNITION__RVIZ__PLUGINS__IMAGEDISPLAY_HPP_
