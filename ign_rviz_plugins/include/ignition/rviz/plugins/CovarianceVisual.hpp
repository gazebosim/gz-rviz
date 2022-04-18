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

#ifndef IGNITION__RVIZ__PLUGINS__COVARIANCEVISUAL_HPP_
#define IGNITION__RVIZ__PLUGINS__COVARIANCEVISUAL_HPP_

#include <ignition/rendering.hh>
#include <ignition/math.hh>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

namespace Eigen
{
using Matrix6d = Matrix<double, 6, 6>;
}

namespace ignition
{
namespace rviz
{
namespace plugins
{

enum Frame
{
  Local, // Renders covariances w.r.t. local frame
  Fixed, // Renders covariances w.r.t. fixed frame
};

enum ColorStyle
{
  Unique,
  RGB,
};

struct CovarianceUserData
{
  bool visible;

  bool position_visible;
  Frame position_frame;
  ignition::math::Color position_color;
  float position_scale;

  bool orientation_visible;
  Frame orientation_frame;
  ColorStyle orientation_color_style;
  ignition::math::Color orientation_color;
  float orientation_offset;
  float orientation_scale;
};

/**
 * @brief Visualizes the covariance of a pose (orientation+position) using 2D and 3D ellipses
 * 
 * One 3D ellipsoid indicates position covariance and 3 2D ellipsoids indicate the uncertainty of orientation along rotation at each axis.
 * In 2D mode, the position ellipsoid is flattened to a 2D ellipse and a cone is used for orientation uncertainty.
 * Choose Local/Fixed frame depending on whether the covariances are expressed w.r.t. the local or the fixed frame.
 */
class CovarianceVisual
{
public:
  enum ShapeIndex
  {
    kRoll = 0,
    kPitch = 1,
    kYaw = 2,
    kYaw2D = 3, // only used with 2D covariances
    kNumOrientationShapes
  };
  
  enum MaterialIndex
  {
    kPos = 0,
    kRotX = 1,
    kRotY = 2,
    kRotZ = 3,
    kRotZ2D = 4,
    kNumMaterials
  };

  CovarianceVisual(ignition::rendering::VisualPtr parent_visual, CovarianceUserData user_data, std::string);

  virtual ~CovarianceVisual();

  /**
   * @brief Initializes all materials
   */
  void createMaterials();

  //////////////////// functions safe to call outside render thread //////////////////////////////
  inline void setCovVisible(bool visible) { user_data_.visible = visible; }
  inline bool Visible() const { return user_data_.visible && (user_data_.position_visible || user_data_.orientation_visible); }
  inline void setPosCovVisible(bool visible) { user_data_.position_visible = visible; }
  inline void setRotCovVisible(bool visible) { user_data_.orientation_visible = visible; }
  inline void setPosCovFrame(bool local) {
    if (local) user_data_.position_frame = Frame::Local;
    else user_data_.position_frame = Frame::Fixed;
  }
  inline void setRotCovFrame(bool local) {
    if (local) user_data_.orientation_frame = Frame::Local;
    else user_data_.orientation_frame = Frame::Fixed;
  }
  inline void setRotCovColorStyle(bool unique) {
    if (unique) user_data_.orientation_color_style = ColorStyle::Unique;
    else user_data_.orientation_color_style = ColorStyle::RGB;
  }
  inline void setPosCovColor(const ignition::math::Color& color) { user_data_.position_color = color; }
  inline void setRotCovColor(const ignition::math::Color& color) { user_data_.orientation_color = color; }
  inline void setPosCovScale(double scale) { user_data_.position_scale = scale; }
  inline void setRotCovScale(double scale) { user_data_.orientation_scale = scale; }
  inline void setRotCovOffset(double offset) { user_data_.orientation_offset = offset; }

  //////////////////// functions unsafe to call outside render thread //////////////////////////////
  void setPose(const ignition::math::Pose3d& pose);
  void setCovariance(const Eigen::Matrix6d& cov);

  void updatePosVisual(const Eigen::Matrix6d& cov);
  void updateRotVisual(const Eigen::Matrix6d& cov, ShapeIndex shapeIdx);
  void updatePosVisualScale();
  void updateRotVisualScale(ShapeIndex shapeIdx);
  void updateRotVisualScales();
  void updateRotVisualOffsets();
  void updateMaterialColor(int idx, const ignition::math::Color& color);
  void updatePosCovColor();
  void updateOrientationVisibility();
  void updateRotCovColor();
  void updateUserData();

private:
  ignition::rendering::ScenePtr scene_;
  // Frame placed on pose's frame, oriented parallel to fixed frame
  ignition::rendering::VisualPtr root_visual_;
  ignition::rendering::VisualPtr fixed_orientation_visual_;

  // children of either fixed or root nodes (Fixed/Local)
  ignition::rendering::VisualPtr position_root_visual_;
  ignition::rendering::VisualPtr orientation_root_visual_;
  std::array<ignition::rendering::NodePtr, kNumOrientationShapes> orientation_offset_nodes_; // reference of orientation ellipse scalings

  // Ellipse used for the position covariance
  ignition::rendering::VisualPtr position_visual_;
  ignition::rendering::GeometryPtr position_ellipse_;
  // Cylinders and cone used for the orientation covariance
  std::array<ignition::rendering::VisualPtr, kNumOrientationShapes> orientation_visuals_;
  std::array<ignition::rendering::GeometryPtr, kNumOrientationShapes> orientation_shapes_;
  // Materials
  std::array<ignition::rendering::MaterialPtr, kNumMaterials> materials_;
  // Scales computed with covariance matrix (do not include user applied scales)
  ignition::math::Vector3d current_position_scale_;
  std::array<ignition::math::Vector3d, kNumOrientationShapes> current_orientation_scales_;

  CovarianceUserData user_data_;

  // flag on whether we are on 2D or 3D covariance visualization mode
  bool cov_2d_;

  static constexpr float kMaxDegrees = 89.0f;

  rclcpp::Logger logger_;
};

typedef std::shared_ptr<CovarianceVisual> CovarianceVisualPtr;

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif // IGNITION__RVIZ__PLUGINS__COVARIANCEVISUAL_HPP_