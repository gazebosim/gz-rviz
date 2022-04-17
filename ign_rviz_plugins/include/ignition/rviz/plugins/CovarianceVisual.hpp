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
  // Maybe add parent frame option?
};

// TODO: support unique color style
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

  CovarianceVisual(ignition::rendering::VisualPtr parent_visual, CovarianceUserData user_data);

  virtual ~CovarianceVisual();

  /**
   * @brief Initializes all materials
   */
  void createMaterials();

  void setCovariance(const Eigen::Matrix6d& cov);

  inline void setCovVisible(bool visible) {
    user_data_.visible = visible;
    setPosCovVisible(visible);
    setRotCovVisible(visible);
  }

  inline void setPosCovVisible(bool visible) {
    user_data_.position_visible = visible;
    position_root_visual_->SetVisible(visible);
  }

  inline void setRotCovVisible(bool visible) {
    user_data_.orientation_visible = visible;
    orientation_root_visual_->SetVisible(visible);
  }
  
  inline void setPosCovFrame(bool local) {
    if (local == (user_data_.position_frame == Frame::Local)) return; // nothing to change
    if (local)
    {
      user_data_.position_frame = Frame::Local;
      this->root_visual_->AddChild(fixed_orientation_visual_->RemoveChild(position_root_visual_));
    }
    else
    {
      user_data_.position_frame = Frame::Fixed;
      fixed_orientation_visual_->AddChild(this->root_visual_->RemoveChild(position_root_visual_));
    }
  }
  
  inline void setRotCovFrame(bool local) {
    if (local == (user_data_.orientation_frame == Frame::Local)) return; // nothing to change
    if (local)
    {
      user_data_.orientation_frame = Frame::Local;
      this->root_visual_->AddChild(fixed_orientation_visual_->RemoveChild(orientation_root_visual_));
    }
    else
    {
      user_data_.orientation_frame = Frame::Fixed;
      fixed_orientation_visual_->AddChild(this->root_visual_->RemoveChild(orientation_root_visual_));
    }
  }

  inline void updateMaterialColor(int idx, const ignition::math::Color& color)
  {
    this->materials_[idx]->SetAmbient(color);
    this->materials_[idx]->SetDiffuse(color);
    this->materials_[idx]->SetEmissive(color);
  }

  inline void setRotCovColorStyle(bool unique)
  {
    if (unique)
      user_data_.orientation_color_style = ColorStyle::Unique;
    else
      user_data_.orientation_color_style = ColorStyle::RGB;

    this->updateRotCovColor();
  }

  inline void setPosCovColor(const ignition::math::Color& color) {
    user_data_.position_color = color;
    this->updatePosCovColor();
    this->position_visual_->SetMaterial(this->materials_[kPos]);
  }

  inline void updatePosCovColor() { updateMaterialColor(kPos, user_data_.position_color); }

  inline void setRotCovColor(const ignition::math::Color& color)
  {
    user_data_.orientation_color = color;
    this->updateRotCovColor();
    this->orientation_visuals_[kRoll]->SetMaterial(this->materials_[kRotX]);
    this->orientation_visuals_[kPitch]->SetMaterial(this->materials_[kRotY]);
    this->orientation_visuals_[kYaw]->SetMaterial(this->materials_[kRotZ]);
    this->orientation_visuals_[kYaw2D]->SetMaterial(this->materials_[kRotZ2D]);
  }

  inline void setRotCovColorToRGB(double alpha)
  {
    user_data_.orientation_color.A(alpha);
    this->updateRotCovColor();
  }
  
  inline void updateRotCovColor()
  {
    if (user_data_.orientation_color_style == Unique)
    {
      const ignition::math::Color& color = user_data_.orientation_color;
      updateMaterialColor(kRotX, color);
      updateMaterialColor(kRotY, color);
      updateMaterialColor(kRotZ, color);
      updateMaterialColor(kRotZ2D, color);
    }
    else
    {
      double alpha = user_data_.orientation_color.A();
      updateMaterialColor(kRotX, ignition::math::Color(1.0, 0.0, 0.0, alpha));
      updateMaterialColor(kRotY, ignition::math::Color(0.0, 1.0, 0.0, alpha));
      updateMaterialColor(kRotZ, ignition::math::Color(0.0, 0.0, 1.0, alpha));
      updateMaterialColor(kRotZ2D, ignition::math::Color(0.0, 0.0, 1.0, alpha));
    }
  }

  inline void setPosCovScale(double scale)
  {
    user_data_.position_scale = scale;
    this->position_visual_->SetLocalScale(scale * this->current_position_scale_);
  }
  
  inline void setRotCovScale(double scale)
  {
    user_data_.orientation_scale = scale;
    // TODO: Do not scale towards the flat dimension
    for (size_t i = 0; i < kNumOrientationShapes; i++)
        this->orientation_visuals_[i]->SetLocalScale(scale * this->current_orientation_scales_[i]);
  }

  inline void setRotCovOffset(double offset)
  {
    user_data_.orientation_offset = offset;
    // TODO: Check if offset at the desired direction
    this->orientation_visuals_[kRoll]->SetLocalPosition(offset * ignition::math::Vector3d::UnitX);
    this->orientation_visuals_[kPitch]->SetLocalPosition(offset * ignition::math::Vector3d::UnitY);
    this->orientation_visuals_[kYaw]->SetLocalPosition(offset * ignition::math::Vector3d::UnitZ);
  }

  inline void setPose(const ignition::math::Pose3d& pose) { this->root_visual_->SetLocalPose(pose); }

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
};

typedef std::shared_ptr<CovarianceVisual> CovarianceVisualPtr;

}  // namespace plugins
}  // namespace rviz
}  // namespace ignition

#endif // IGNITION__RVIZ__PLUGINS__COVARIANCEVISUAL_HPP_