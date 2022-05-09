// Copyright (c) 2022 Open Source Robotics Foundation, Inc.
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

#include "ignition/rviz/plugins/PoseWithCovarianceDisplay.hpp"
#include "ignition/rviz/plugins/CovarianceVisual.hpp"

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/plugin/Register.hh>

#include <string>
#include <utility>
#include <memory>
#include <vector>

namespace ignition
{
namespace rviz
{
namespace plugins
{
void ArrowVisualPrivate::updateVisual()
{
  visual->Shaft()->SetLocalScale(shaftRadius * 2.0, shaftRadius * 2.0, shaftLength);
  visual->SetOrigin(0, 0, -shaftLength);
  visual->Head()->SetLocalScale(headRadius * 2.0, headRadius * 2.0, headLength * 2.0);
}

void AxisVisualPrivate::updateVisual()
{
  for (size_t i = 0; i < visual->ChildCount(); ++i) {
    auto arrow = std::dynamic_pointer_cast<rendering::ArrowVisual>(visual->ChildByIndex(i));
    arrow->SetLocalScale(radius * 20, radius * 20, length * 2);
  }
}

class PoseWithCovarianceDisplay::Implementation
{
  public:
    ignition::rendering::RenderEngine * engine;
    ignition::rendering::ScenePtr scene;
    ignition::rendering::VisualPtr rootVisual;
    std::shared_ptr<std::mutex> lock;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg;
    QStringList topicList;
    ArrowVisualPrivate arrow;
    AxisVisualPrivate axis;
    CovarianceVisualPtr covVisual;
    bool visualShape = true;  // True: Arrow; False: Axis
    bool dirty = true;
};

////////////////////////////////////////////////////////////////////////////////
PoseWithCovarianceDisplay::PoseWithCovarianceDisplay()
: MessageDisplay(), dataPtr(utils::MakeImpl<Implementation>())
{
  // Get reference to scene
  this->dataPtr->engine = ignition::rendering::engine("ogre");
  this->dataPtr->scene = this->dataPtr->engine->SceneByName("scene");

  this->dataPtr->rootVisual = this->dataPtr->scene->CreateVisual();
  this->dataPtr->scene->RootVisual()->AddChild(this->dataPtr->rootVisual);

  this->dataPtr->arrow.mat = this->dataPtr->scene->CreateMaterial();
  this->dataPtr->arrow.mat->SetAmbient(1.0, 0.098, 0.0);
  this->dataPtr->arrow.mat->SetDiffuse(1.0, 0.098, 0.0);
  this->dataPtr->arrow.mat->SetEmissive(1.0, 0.098, 0.0);

  this->dataPtr->lock = std::make_shared<std::mutex>();
}

////////////////////////////////////////////////////////////////////////////////
PoseWithCovarianceDisplay::~PoseWithCovarianceDisplay()
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  // Delete visual
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>()->removeEventFilter(this);
  this->dataPtr->scene->DestroyVisual(this->dataPtr->rootVisual, true);
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::initialize(rclcpp::Node::SharedPtr _node)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->node = std::move(_node);
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::subscribe()
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);

  this->subscriber = this->node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    this->topic_name,
    this->qos,
    std::bind(&PoseWithCovarianceDisplay::callback, this, std::placeholders::_1));
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::setTopic(const std::string & topic_name)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->topic_name = topic_name;

  this->subscribe();

  // Refresh combo-box on plugin load
  this->onRefresh();
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::setTopic(const QString & topic_name)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->topic_name = topic_name.toStdString();

  // Destroy previous subscription
  this->unsubscribe();
  // Reset visualization
  this->reset();
  // Create new subscription
  this->subscribe();
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr _msg)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->dataPtr->msg = std::move(_msg);
}

////////////////////////////////////////////////////////////////////////////////
bool PoseWithCovarianceDisplay::eventFilter(QObject * _object, QEvent * _event)
{
  if (_event->type() == gui::events::Render::kType) {
    update();
  }

  return QObject::eventFilter(_object, _event);
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::reset()
{
  if (this->dataPtr->arrow.visual != nullptr)
    this->dataPtr->arrow.visual->SetLocalPose(math::Pose3d::Zero);
  if (this->dataPtr->axis.visual != nullptr)
    this->dataPtr->axis.visual->SetLocalPose(math::Pose3d::Zero);
  this->dataPtr->msg.reset();
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::update()
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  // Create axis
  if (this->dataPtr->axis.visual == nullptr) {
    this->dataPtr->axis.visual = this->dataPtr->scene->CreateAxisVisual();
    this->dataPtr->rootVisual->AddChild(this->dataPtr->axis.visual);
  }

  // Create arrow
  if (this->dataPtr->arrow.visual == nullptr) {
    this->dataPtr->arrow.visual = this->dataPtr->scene->CreateArrowVisual();
    this->dataPtr->arrow.visual->SetMaterial(this->dataPtr->arrow.mat);
    this->dataPtr->rootVisual->AddChild(this->dataPtr->arrow.visual);
  }

  if (this->dataPtr->covVisual == nullptr)
  {
    CovarianceUserData covUserData;
    covUserData.visible = true;
    covUserData.position_visible = true;
    covUserData.position_frame = Frame::Local;
    covUserData.position_color = ignition::math::Color(0.8, 0.2, 0.8, 0.3);
    covUserData.position_scale = 1.0;
    covUserData.orientation_visible = true;
    covUserData.orientation_frame = Frame::Local;
    covUserData.orientation_color_style = ColorStyle::Unique;
    covUserData.orientation_color = ignition::math::Color(1.0, 1.0, 0.5, 0.3);
    covUserData.orientation_offset = 1.0;
    covUserData.orientation_scale = 1.0;

    this->dataPtr->covVisual = std::make_shared<CovarianceVisual>(this->dataPtr->rootVisual, 
      covUserData, this->node->get_logger().get_name());
  }

  if (this->dataPtr->dirty) {
    // Update Arrow
    this->dataPtr->arrow.visual->SetVisible(this->dataPtr->visualShape);
    this->dataPtr->arrow.updateVisual();

    // Update Axis
    this->dataPtr->axis.visual->SetVisible(!this->dataPtr->visualShape);
    this->dataPtr->axis.visual->ShowAxisHead(!this->dataPtr->visualShape && this->dataPtr->axis.headVisible);
    this->dataPtr->axis.updateVisual();

    this->dataPtr->dirty = false;
  }

  if (!this->dataPtr->msg) {
    return;
  }

  // update pose
  math::Pose3d pose;
  bool poseAvailable = this->frameManager->getFramePose(this->dataPtr->msg->header.frame_id, pose);

  if (!poseAvailable) {
    RCLCPP_ERROR(
      this->node->get_logger(), "Unable to get frame pose: %s",
      this->dataPtr->msg->header.frame_id.c_str());
    return;
  }

  this->dataPtr->rootVisual->SetLocalPose(pose);

  math::Pose3d localPose(dataPtr->msg->pose.pose.position.x, dataPtr->msg->pose.pose.position.y,
    dataPtr->msg->pose.pose.position.z, dataPtr->msg->pose.pose.orientation.w,
    dataPtr->msg->pose.pose.orientation.x, dataPtr->msg->pose.pose.orientation.y, 
    dataPtr->msg->pose.pose.orientation.z);

  this->dataPtr->axis.visual->SetLocalPose(localPose);

  this->dataPtr->arrow.visual->SetLocalPosition(localPose.Pos());
  this->dataPtr->arrow.visual->SetLocalRotation(localPose.Rot() * math::Quaterniond(0, 1.57, 0));

  // update covariance 
  this->dataPtr->covVisual->updateUserData();
  if (this->dataPtr->covVisual->Visible())
  {
    this->dataPtr->covVisual->setPose(localPose);
    for (unsigned i = 0; i < 36; ++i) {
      // check for NaN in covariance
      if (std::isnan(this->dataPtr->msg->pose.covariance[i])) {
        RCLCPP_WARN(this->node->get_logger(), "covariance contains NaN at position %d", i);
        return;
      }
    }
    // Map covariance to a Eigen::Matrix
    Eigen::Map<const Eigen::Matrix<double, 6, 6>> covariance(this->dataPtr->msg->pose.covariance.data());
    this->dataPtr->covVisual->setCovariance(covariance);
  }
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::setShape(const bool & _shape)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->dataPtr->visualShape = _shape;
  this->dataPtr->dirty = true;
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::setAxisHeadVisibility(const bool & _visible)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->dataPtr->axis.headVisible = _visible;
  this->dataPtr->dirty = true;
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::setAxisDimensions(const float & _length, const float & _radius)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->dataPtr->axis.length = _length;
  this->dataPtr->axis.radius = _radius;
  this->dataPtr->dirty = true;
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::setArrowDimensions(
  const float & _shaftLength, const float & _shaftRadius,
  const float & _headLength, const float & _headRadius)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->dataPtr->arrow.shaftLength = _shaftLength;
  this->dataPtr->arrow.shaftRadius = _shaftRadius;
  this->dataPtr->arrow.headLength = _headLength;
  this->dataPtr->arrow.headRadius = _headRadius;
  this->dataPtr->dirty = true;
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::setColor(const QColor & _color)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->dataPtr->arrow.mat->SetAmbient(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());
  this->dataPtr->arrow.mat->SetDiffuse(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());
  this->dataPtr->arrow.mat->SetEmissive(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());

  if (this->dataPtr->arrow.visual != nullptr)
    this->dataPtr->arrow.visual->SetMaterial(this->dataPtr->arrow.mat);
}
  
void PoseWithCovarianceDisplay::setCovVisible(const bool& _visible)
{
  std::lock_guard<std::mutex>(*dataPtr->lock);
  dataPtr->covVisual->setCovVisible(_visible);
}
  
void PoseWithCovarianceDisplay::setPosCovVisible(const bool& _visible)
{
  std::lock_guard<std::mutex>(*dataPtr->lock);
  dataPtr->covVisual->setPosCovVisible(_visible);
}

void PoseWithCovarianceDisplay::setRotCovVisible(const bool& _visible)
{
  std::lock_guard<std::mutex>(*dataPtr->lock);
  dataPtr->covVisual->setRotCovVisible(_visible);
}

void PoseWithCovarianceDisplay::setPosCovFrame(const bool& _local)
{
  std::lock_guard<std::mutex>(*dataPtr->lock);
  dataPtr->covVisual->setPosCovFrame(_local);
}

void PoseWithCovarianceDisplay::setRotCovFrame(const bool& _local)
{
  std::lock_guard<std::mutex>(*dataPtr->lock);
  dataPtr->covVisual->setRotCovFrame(_local);
}

void PoseWithCovarianceDisplay::setPosCovColor(const QColor& _color)
{ 
  std::lock_guard<std::mutex>(*dataPtr->lock); 
  dataPtr->covVisual->setPosCovColor(ignition::math::Color(_color.redF(), _color.greenF(),
    _color.blueF(), _color.alphaF()));
}

void PoseWithCovarianceDisplay::setRotCovColor(const QColor& _color)
{ 
  std::lock_guard<std::mutex>(*dataPtr->lock); 
  dataPtr->covVisual->setRotCovColor(ignition::math::Color(_color.redF(), _color.greenF(),
    _color.blueF(), _color.alphaF()));
}

void PoseWithCovarianceDisplay::setRotCovColorStyle(const bool& _unique)
{
  std::lock_guard<std::mutex>(*dataPtr->lock);
  dataPtr->covVisual->setRotCovColorStyle(_unique);
}

void PoseWithCovarianceDisplay::setPosCovScale(const float& _scale)
{
  std::lock_guard<std::mutex>(*dataPtr->lock);
  dataPtr->covVisual->setPosCovScale(_scale);
}

void PoseWithCovarianceDisplay::setRotCovScale(const float& _scale)
{
  std::lock_guard<std::mutex>(*dataPtr->lock);
  dataPtr->covVisual->setRotCovScale(_scale);
}

void PoseWithCovarianceDisplay::setRotCovOffset(const float& _offset)
{
  std::lock_guard<std::mutex>(*dataPtr->lock);
  dataPtr->covVisual->setRotCovOffset(_offset);
}


////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::setFrameManager(std::shared_ptr<common::FrameManager> _frameManager)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
  this->frameManager = std::move(_frameManager);
}

////////////////////////////////////////////////////////////////////////////////
QStringList PoseWithCovarianceDisplay::getTopicList() const
{
  return this->dataPtr->topicList;
}

////////////////////////////////////////////////////////////////////////////////
void PoseWithCovarianceDisplay::onRefresh()
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);

  // Clear
  this->dataPtr->topicList.clear();

  int index = 0, position = 0;

  // Get topic list
  auto topics = this->node->get_topic_names_and_types();
  for (const auto & topic : topics) {
    for (const auto & topicType : topic.second) {
      if (topicType == "geometry_msgs/msg/PoseWithCovarianceStamped") {
        this->dataPtr->topicList.push_back(QString::fromStdString(topic.first));
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
void PoseWithCovarianceDisplay::updateQoS(
  const int & _depth, const int & _history, const int & _reliability,
  const int & _durability)
{
  std::lock_guard<std::mutex>(*this->dataPtr->lock);
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
void PoseWithCovarianceDisplay::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty()) {
    this->title = "PoseWithCovariance";
  }
}

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

IGNITION_ADD_PLUGIN(
  ignition::rviz::plugins::PoseWithCovarianceDisplay,
  ignition::gui::Plugin)
