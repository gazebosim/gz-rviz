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

#include "ignition/rviz/plugins/CovarianceVisual.hpp"

namespace ignition
{
namespace rviz
{
namespace plugins
{
CovarianceVisual::CovarianceVisual(ignition::rendering::VisualPtr parent_node, CovarianceUserData user_data) : user_data_(user_data) 
{
  this->scene_ = parent_node->Scene();
  this->root_visual_ = this->scene_->CreateVisual();
  parent_node->AddChild(this->root_visual_);

  this->fixed_orientation_visual_ = this->scene_->CreateVisual();
  this->root_visual_->AddChild(this->fixed_orientation_visual_);

  this->position_root_visual_ = this->scene_->CreateVisual();
  if (this->user_data_.position_frame == Frame::Local)
    this->root_visual_->AddChild(this->position_root_visual_);
  else
    this->fixed_orientation_visual_->AddChild(this->position_root_visual_);

  this->orientation_root_visual_ = this->scene_->CreateVisual();
  if (this->user_data_.orientation_frame == Frame::Local)
    this->root_visual_->AddChild(this->orientation_root_visual_);
  else
    this->fixed_orientation_visual_->AddChild(this->orientation_root_visual_);

  this->createMaterials();

  this->position_visual_ = this->scene_->CreateVisual();
  this->position_visual_->SetInheritScale(false);
  this->position_ellipse_ = this->scene_->CreateSphere(); 
  this->position_visual_->AddGeometry(this->position_ellipse_);
  this->position_visual_->SetMaterial(this->materials_[kPos], false);
  this->position_root_visual_->AddChild(this->position_visual_);
  this->position_visual_->SetLocalPosition(1.0 * math::Vector3d::UnitZ);
  this->position_visual_->SetLocalRotation(math::Quaterniond(0, 0, 0));

  for (int i = 0; i < kNumOrientationShapes; i++) {
    // Visual to position and orient the shape along the axis. One for each axis.
    this->orientation_visuals_[i] = this->scene_->CreateVisual();
    // Does not inherit scale from the parent.
    // This is needed to keep the cylinders with the same height.
    // The scale is set by setOrientationScale()
    this->orientation_visuals_[i]->SetInheritScale(false);

    if (i != kYaw2D) {
      this->orientation_shapes_[i] = this->scene_->CreateCylinder();
    } else {
      this->orientation_shapes_[i] = this->scene_->CreateCone(); // TODO: use pie slice instead
    }
    this->orientation_visuals_[i]->AddGeometry(this->orientation_shapes_[i]);
    this->orientation_visuals_[i]->SetMaterial(this->materials_[i+1], false);
    this->orientation_root_visual_->AddChild(this->orientation_visuals_[i]);

    // Initialize all current scales to 0
    // current_orientation_scales_[i] = Ogre::Vector3::ZERO;
  }
  
  // Position the cylinders at position 1.0 in the respective axis, and perpendicular to the axis.
  double offset = user_data_.orientation_offset;
  // x-axis (roll)
  orientation_visuals_[kRoll]->SetLocalPosition(offset * math::Vector3d::UnitX);
  orientation_visuals_[kRoll]->SetLocalRotation(math::Quaterniond(math::Vector3d::UnitY, math::Angle::HalfPi.Radian()));
  // y-axis (pitch)
  orientation_visuals_[kPitch]->SetLocalPosition(offset * math::Vector3d::UnitY);
  orientation_visuals_[kPitch]->SetLocalRotation(math::Quaterniond(math::Vector3d::UnitX, math::Angle::HalfPi.Radian()));
  // z-axis (yaw)
  orientation_visuals_[kYaw]->SetLocalPosition(offset * math::Vector3d::UnitZ);
  orientation_visuals_[kYaw]->SetLocalRotation(math::Quaterniond(0, 0, 0)); // no rotation needed
  // z-axis (yaw 2D)
  // NOTE: rviz use a cone defined by the file rviz_rendering/ogre_media/models/rviz_cone.mesh, and
  //       it's origin is not at the top of the cone. Since we want the top to be at the origin of
  //       the pose we need to use an offset here.
  // WARNING: This number was found by trial-and-error on rviz and it's not the correct
  //          one, so changes on scale are expected to cause the top of the cone to move
  //          from the pose origin, although it's only noticeable with big scales.
  // TODO(anonymous): Find the right value from the cone.mesh file, or implement a class that draws
  //        something like a 2D "pie slice" and use it instead of the cone.
  static const float cone_origin_to_top = 0.49115f;
  orientation_visuals_[kYaw2D]->SetLocalPosition(cone_origin_to_top * math::Vector3d::UnitX);
  orientation_visuals_[kYaw2D]->SetLocalRotation(math::Quaterniond(math::Vector3d::UnitZ, math::Angle::HalfPi.Radian()));

  // TODO: Compute scales properly
  this->current_position_scale_ = ignition::math::Vector3d(0.1, 0.1, 0.1);
  for (size_t i = 0; i < kNumOrientationShapes; i++) {
    this->current_orientation_scales_[i] = ignition::math::Vector3d(0.1, 0.1, 0.1);
  }
}

void CovarianceVisual::createMaterials()
{
  this->materials_[kPos] = this->scene_->CreateMaterial();
  this->materials_[kRotX] = this->scene_->CreateMaterial();
  this->materials_[kRotY] = this->scene_->CreateMaterial();
  this->materials_[kRotZ] = this->scene_->CreateMaterial();
  this->materials_[kRotZ2D] = this->scene_->CreateMaterial();

  //this->updatePosCovColor();
  this->updateRotCovColor();
}

void CovarianceVisual::setCovariance(const Eigen::Matrix6d& cov)
{
  // The 3rd, 4th, and 5th diagonal entries are empty for 2D case
  cov_2d_ = (cov(3,3) <= 0 && cov(4,4) <= 0 && cov(5,5) <= 0);
}

CovarianceVisual::~CovarianceVisual()
{
  this->scene_->DestroyVisual(this->position_root_visual_, true);
  this->scene_->DestroyVisual(this->orientation_root_visual_, true);
  this->scene_->DestroyVisual(this->fixed_orientation_visual_, true);
}
}  // namespace plugins
}  // namespace rviz
}  // namespace ignition