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

#ifndef GZ__RVIZ__PLUGINS__POINTSTAMPEDDISPLAY_HPP_
#define GZ__RVIZ__PLUGINS__POINTSTAMPEDDISPLAY_HPP_


#include <QColor>

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <gz/rendering.hh>

#include "gz/rviz/plugins/message_display_base.hpp"

namespace gz
{
namespace rviz
{
namespace plugins
{
/**
 * @brief Renders data from geometry_msgs::msg::PointStamped message as spheres
 */
class PointStampedDisplay : public MessageDisplay<geometry_msgs::msg::PointStamped>
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
  /**
   * Constructor for PointStamped visualization plugin
   */
  PointStampedDisplay();

  // Destructor
  ~PointStampedDisplay();

  // Documentation Inherited
  void LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/) override;

  // Documentation Inherited
  void initialize(rclcpp::Node::SharedPtr _node) override;

  // Documentation Inherited
  void callback(const geometry_msgs::msg::PointStamped::SharedPtr _msg) override;

  // Documentation inherited
  void setTopic(const std::string & topic_name) override;

  // Documentation inherited
  void subscribe() override;

  // Documentation inherited
  void reset() override;

  /**
   * @brief Set ROS Subscriber topic through GUI
   * @param[in] topic_name ROS Topic Name
   */
  Q_INVOKABLE void setTopic(const QString & topic_name);

  /**
   * @brief Update subscription Quality of Service
   * @param[in] _depth Queue size of keep last history policy
   * @param[in] _history Index of history policy
   * @param[in] _reliability Index of reliability policy
   * @param[in] _durability Index of durability policy
   */
  Q_INVOKABLE void updateQoS(
    const int & _depth, const int & _history, const int & _reliability,
    const int & _durability);

  /**
   * @brief Qt eventFilters. Original documentation can be found
   * <a href="https://doc.qt.io/qt-5/qobject.html#eventFilter">here</a>
   */
  bool eventFilter(QObject * _object, QEvent * _event);

  // Documentation inherited
  void setFrameManager(std::shared_ptr<common::FrameManager> _frameManager) override;

  /**
   * @brief Get the topic list as a string
   * @return List of topics
   */
  Q_INVOKABLE QStringList getTopicList() const;

  /**
   * @brief Set PointStamped visualization history length
   * @param[in] _length History length
   */
  Q_INVOKABLE void setHistoryLength(const int & _length);

  /**
   * @brief Set point visual radius
   * @param[in] _radius Size of point visual
   */
  Q_INVOKABLE void setRadius(const float & _radius);

  /**
   * @brief Set point visual color and transparency
   * @param _color Color and transparency of point visual
   */
  Q_INVOKABLE void setColor(const QColor & _color);

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

public slots:
  /**
   * @brief Callback when refresh button is pressed.
   */
  void onRefresh();

protected:
  /**
   * @brief Update PointStamped data visualization
   */
  void update() override;

  /**
   * @brief Create a new PointStamped visual
   * @param[in] _msg PointStamped data to visualize
   */
  void createNewPointVisual(const geometry_msgs::msg::PointStamped::SharedPtr _msg);

  /**
   * @brief Removes the oldest PointStamped visual
   */
  void removeOldestPointVisual();

private:
  rendering::RenderEngine * engine;
  rendering::ScenePtr scene;
  rendering::VisualPtr rootVisual;
  rendering::MaterialPtr mat;
  std::deque<rendering::VisualPtr> visuals;
  std::mutex lock;
  geometry_msgs::msg::PointStamped::SharedPtr msg;
  QStringList topicList;
  std::size_t historyLength;
  float radius;
};

}  // namespace plugins
}  // namespace rviz
}  // namespace gz

#endif  // GZ__RVIZ__PLUGINS__POINTSTAMPEDDISPLAY_HPP_
