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

// diagonalization utilities
namespace
{
double deg2rad(double degrees)
{
  return degrees * 3.14159265358979 / 180.0;
}

  // Local function to force the axis to be right handed for 3D. Taken from ecl_statistics
void makeRightHanded(Eigen::Matrix3d & eigenvectors, Eigen::Vector3d & eigenvalues)
{
  // Note that sorting of eigenvalues may end up with left-hand coordinate system.
  // So here we correctly sort it so that it does end up being right-handed and normalised.
  Eigen::Vector3d c0 = eigenvectors.block<3, 1>(0, 0);  c0.normalize();
  Eigen::Vector3d c1 = eigenvectors.block<3, 1>(0, 1);  c1.normalize();
  Eigen::Vector3d c2 = eigenvectors.block<3, 1>(0, 2);  c2.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc.dot(c2) < 0) {
    eigenvectors << c1, c0, c2;
    double e = eigenvalues[0];  eigenvalues[0] = eigenvalues[1];  eigenvalues[1] = e;
  } else {
    eigenvectors << c0, c1, c2;
  }
}

// Local function to force the axis to be right handed for 2D. Based on the one from ecl_statistics
void makeRightHanded(Eigen::Matrix2d & eigenvectors, Eigen::Vector2d & eigenvalues)
{
  // Note that sorting of eigenvalues may end up with left-hand coordinate system.
  // So here we correctly sort it so that it does end up being right-handed and normalised.
  Eigen::Vector3d c0;  c0.setZero();  c0.head<2>() = eigenvectors.col(0);  c0.normalize();
  Eigen::Vector3d c1;  c1.setZero();  c1.head<2>() = eigenvectors.col(1);  c1.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc[2] < 0) {
    eigenvectors << c1.head<2>(), c0.head<2>();
    double e = eigenvalues[0];  eigenvalues[0] = eigenvalues[1];  eigenvalues[1] = e;
  } else {
    eigenvectors << c0.head<2>(), c1.head<2>();
  }
}

struct Cov2DSolverParams
{
  static const size_t dimension = 2;
  using EigenvaluesType = Eigen::Vector2d;
  using EigenvectorsType = Eigen::Matrix2d;
  using ResultNumberType = double;
};
struct Cov3DSolverParams
{
  static const size_t dimension = 3;
  using EigenvaluesType = Eigen::Vector3d;
  using EigenvectorsType = Eigen::Matrix3d;
  using ResultNumberType = double;
};

template<typename SolverParams>
std::tuple<typename SolverParams::EigenvaluesType, typename SolverParams::EigenvectorsType>
diagonalizeCovariance(
  const Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension>& covariance, rclcpp::Logger logger)
{
  Eigen::Matrix<double, SolverParams::dimension, 1> eigenvalues =
    Eigen::Matrix<double, SolverParams::dimension, 1>::Identity();
  Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension> eigenvectors =
    Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension>::Zero();

  // NOTE: The SelfAdjointEigenSolver only
  // references the lower triangular part of the covariance matrix
  Eigen::SelfAdjointEigenSolver<
    Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension>>
  eigensolver(covariance);
  // Compute eigenvectors and eigenvalues
  bool covariance_valid = true;
  if (eigensolver.info() == Eigen::Success) {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();

    if (eigenvalues.minCoeff() < 0) {
      RCLCPP_WARN(logger,
        "Negative eigenvalue found for position. Is the "
        "covariance matrix correct (positive semidefinite)?");
      covariance_valid = false;
    }
  } else {
    RCLCPP_WARN(logger,
      "failed to compute eigen vectors/values for position. Is the "
      "covariance matrix correct?");
    covariance_valid = false;
  }
  if (!covariance_valid) {
    eigenvalues = Eigen::Matrix<double, SolverParams::dimension, 1>::Zero();  // Setting the scale
    // to zero will hide it on the screen
    eigenvectors =
      Eigen::Matrix<double, SolverParams::dimension, SolverParams::dimension>::Identity();
  }

  // Be sure we have a right-handed orientation system
  makeRightHanded(eigenvectors, eigenvalues);

  return std::make_tuple(eigenvalues, eigenvectors);
}

std::tuple<ignition::math::Vector3d, ignition::math::Quaterniond>
computeShapeScaleAndOrientation3D(const Eigen::Matrix3d& covariance, rclcpp::Logger logger)
{
  Cov3DSolverParams::EigenvaluesType eigenvalues;
  Cov3DSolverParams::EigenvectorsType eigenvectors;
  std::tie(eigenvalues, eigenvectors) =
    diagonalizeCovariance<Cov3DSolverParams>(covariance, logger);

  // Define the rotation
  Eigen::Quaterniond ori_eigen(eigenvectors); // implicitly converts rotation mat -> quaternion
  ignition::math::Quaterniond orientation(ori_eigen.w(), ori_eigen.x(), ori_eigen.y(), ori_eigen.z());

  // Define the scale. eigenvalues are the variances,
  // so we take the sqrt to draw the standard deviation
  ignition::math::Vector3d scale(
    2.f * std::sqrt(eigenvalues[0]),
    2.f * std::sqrt(eigenvalues[1]),
    2.f * std::sqrt(eigenvalues[2]));
  return std::make_tuple(scale, orientation);
}

std::tuple<ignition::math::Vector3d, ignition::math::Quaterniond>
computeShapeScaleAndOrientation2D(const Eigen::Matrix2d & covariance, rclcpp::Logger logger)
{
  Cov2DSolverParams::EigenvaluesType eigenvalues;
  Cov2DSolverParams::EigenvectorsType eigenvectors;
  std::tie(eigenvalues, eigenvectors) =
    diagonalizeCovariance<Cov2DSolverParams>(covariance, logger);

  // Define the rotation and scale of the plane
  // The Eigenvalues are the variances. The scales are two times the standard
  // deviation. The scale of the missing dimension is set to zero.
  ignition::math::Vector3d scale;
  scale.X(2. * std::sqrt(eigenvalues[0]));
  scale.Y(2. * std::sqrt(eigenvalues[1]));
  scale.Z(0);

  Eigen::Matrix3d ori_mat = Eigen::Matrix3d::Identity();
  ori_mat.topLeftCorner<2,2>() = eigenvectors;
  Eigen::Quaterniond ori_eigen(ori_mat); // implicitly converts rotation mat -> quaternion
  ignition::math::Quaterniond orientation(ori_eigen.w(), ori_eigen.x(), ori_eigen.y(), ori_eigen.z());

  return std::make_tuple(scale, orientation);
}

double radianScaleToMetricScaleBounded(double radian_scale, double max_degrees)
{
  if (radian_scale > 2.0 * deg2rad(max_degrees)) {
    return 2.0 * tanf(deg2rad(max_degrees));
  } else {
    return 2.0 * tanf(radian_scale * 0.5);
  }
}
} // namespace

namespace ignition
{
namespace rviz
{
namespace plugins
{
CovarianceVisual::CovarianceVisual(ignition::rendering::VisualPtr parent_node, CovarianceUserData user_data, std::string logger_name) 
  : user_data_(user_data), logger_(rclcpp::get_logger(logger_name))
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
  this->position_visual_->SetMaterial(this->materials_[kPos]);
  this->position_root_visual_->AddChild(this->position_visual_);
  this->position_visual_->SetLocalRotation(math::Quaterniond(0, 0, 0));

  for (int i = 0; i < kNumOrientationShapes; i++) {
    // Visual to position and orient the shape along the axis. One for each axis.
    this->orientation_visuals_[i] = this->scene_->CreateVisual();
    // Does not inherit scale from the parent.
    // This is needed to keep the cylinders with the same height.
    // The scale is set by setOrientationScale()
    //this->orientation_visuals_[i]->SetInheritScale(false);

    if (i != kYaw2D) {
      this->orientation_shapes_[i] = this->scene_->CreateCylinder();
    } else {
      this->orientation_shapes_[i] = this->scene_->CreateCone(); // TODO: use pie slice instead
    }
    this->orientation_visuals_[i]->AddGeometry(this->orientation_shapes_[i]);
    this->orientation_visuals_[i]->SetMaterial(this->materials_[i+1]);
    this->orientation_root_visual_->AddChild(this->orientation_visuals_[i]);
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
  // NOTE: cone's origin is not at the top of the cone. Since we want the top to be at the origin of
  //       the pose we need to use an offset here.
  // WARNING: This number was found by trial-and-error on rviz and it's not the correct
  //          one, so changes on scale are expected to cause the top of the cone to move
  //          from the pose origin, although it's only noticeable with big scales.
  // TODO(Thodoris): Implement something like a 2D "pie slice" and use it instead of the cone.
  static const float cone_origin_to_top = 0.49115f;
  orientation_visuals_[kYaw2D]->SetLocalPosition(cone_origin_to_top * math::Vector3d::UnitX);
  orientation_visuals_[kYaw2D]->SetLocalRotation(math::Quaterniond(math::Vector3d::UnitY, -math::Angle::HalfPi.Radian()));

  // Initialize all scales to zero
  this->current_position_scale_ = ignition::math::Vector3d::Zero;
  for (size_t i = 0; i < kNumOrientationShapes; i++) {
    this->current_orientation_scales_[i] = ignition::math::Vector3d::Zero;
  }
  this->updateUserData();
}

void CovarianceVisual::createMaterials()
{
  this->materials_[kPos] = this->scene_->CreateMaterial();
  this->materials_[kRotX] = this->scene_->CreateMaterial();
  this->materials_[kRotY] = this->scene_->CreateMaterial();
  this->materials_[kRotZ] = this->scene_->CreateMaterial();
  this->materials_[kRotZ2D] = this->scene_->CreateMaterial();
}

void CovarianceVisual::setPose(const ignition::math::Pose3d& pose)
{ 
  this->root_visual_->SetLocalPose(pose);
  // Set the orientation of the fixed node. Since this node is attached to the root node,
  // it's orientation will be the inverse of pose's orientation.
  this->fixed_orientation_visual_->SetLocalRotation(pose.Rot().Inverse());
}


void CovarianceVisual::setCovariance(const Eigen::Matrix6d& cov)
{
  // The 3rd, 4th, and 5th diagonal entries are empty for 2D case
  this->cov_2d_ = (cov(2,2) <= 0 && cov(3,3) <= 0 && cov(4,4) <= 0);

  this->updateOrientationVisibility();

  this->updatePosVisual(cov);
  if (this->cov_2d_)
  {
    this->updateRotVisual(cov, kYaw2D);
  }
  else
  {
    this->updateRotVisual(cov, kRoll);
    this->updateRotVisual(cov, kPitch);
    this->updateRotVisual(cov, kYaw);
  }
}
  
void CovarianceVisual::updatePosVisual(const Eigen::Matrix6d& cov)
{
  math::Vector3d shape_scale;
  math::Quaterniond shape_orientation;
  if (this->cov_2d_) {
    std::tie(shape_scale, shape_orientation) =
      computeShapeScaleAndOrientation2D(cov.topLeftCorner<2, 2>(), this->logger_);
    // Make the scale in z minimal for better visualization
    shape_scale.Z(0.001);
  } else {
    std::tie(shape_scale, shape_orientation) =
      computeShapeScaleAndOrientation3D(cov.topLeftCorner<3, 3>(), this->logger_);
  }

  position_visual_->SetLocalRotation(shape_orientation);
  if (shape_scale.IsFinite())
  {
    this->current_position_scale_ = shape_scale;
    this->updatePosVisualScale();
  }
  else
  {
    RCLCPP_WARN(this->logger_, "Covariance Visual: computed infinite eigenvalues (is covariance matrix finite?)");
  }
}

void CovarianceVisual::updateRotVisual(const Eigen::Matrix6d& cov, ShapeIndex shapeIdx)
{
  math::Vector3d shape_scale;
  math::Quaterniond shape_orientation;

  if (this->cov_2d_)
  {
    // We should only enter on this scope if the index is kYaw2D
    assert(shapeIdx == kYaw2D);

    // 2D poses only depend on yaw.
    // scale cone width
    // The computed scale is equivalent to twice the standard deviation _in radians_.
    // So we need to convert it to the linear scale of the shape using tan().
    // Also, we bound the maximum std. TODO: Use pie slice without metric conversion
    shape_scale.Y(2 * std::sqrt(cov(5,5)));
    shape_scale.Z(1.0);
    shape_orientation = math::Quaterniond(math::Vector3d::UnitY, -math::Angle::HalfPi.Radian()); 
    shape_scale.Y(radianScaleToMetricScaleBounded(shape_scale.Y(), kMaxDegrees));
  }
  else
  {
    // We should only enter on this scope if the index is kYaw2D
    assert(shapeIdx != kYaw2D);

    // Cylinder geometry is aligned on its Z-axis. 
    // I will twist each orientaiton visual frame s.t. its Z axis is parallel to its corresponding axis,
    // and that its other two axis are alligned with the parent's other axis. Then I will scale the ellipse such that
    // it represents the region that the tip of the parent's axis would stay inside given the covariance of the other two axis.
    // Order of variables entered in the matrix to be diagonalized matters!
    Eigen::Matrix2d covariancePlane;
    if (shapeIdx == kRoll)
    {
      covariancePlane = cov.bottomRightCorner<2,2>();
    }
    else if (shapeIdx == kPitch)
    {
      covariancePlane << cov(5,5), cov(3,5), cov(5,3), cov(3,3);
    }
    else if (shapeIdx == kYaw)
    {
      covariancePlane << cov(4,4), cov(3,4), cov(4,3), cov(3,3);
    }
    std::tie(shape_scale, shape_orientation) = computeShapeScaleAndOrientation2D(covariancePlane, this->logger_);
    if (shapeIdx == kRoll)
    {
      shape_orientation = math::Quaterniond(math::Vector3d::UnitY, math::Angle::HalfPi.Radian()) * shape_orientation;
    }
    else if (shapeIdx == kPitch)
    {
      shape_orientation = math::Quaterniond(math::Vector3d::UnitX, -math::Angle::HalfPi.Radian()) * shape_orientation;
    }

    // The computed scale is equivalent to twice the standard deviation _in radians_.
    // So we need to convert it to the linear scale of the shape using tan().
    // Also, we bound the maximum std.
    shape_scale.X(radianScaleToMetricScaleBounded(shape_scale.X(), kMaxDegrees));
    shape_scale.Y(radianScaleToMetricScaleBounded(shape_scale.Y(), kMaxDegrees));
  }

  orientation_visuals_[shapeIdx]->SetLocalRotation(shape_orientation);
  if (shape_scale.IsFinite())
  {
    current_orientation_scales_[shapeIdx] = shape_scale;
    this->updateRotVisualScale(shapeIdx);
  }
}

void CovarianceVisual::updatePosVisualScale()
{
  if (this->cov_2d_)
    this->position_visual_->SetLocalScale(ignition::math::Vector3d(user_data_.position_scale * this->current_position_scale_.X(),
                                                                    user_data_.position_scale * this->current_position_scale_.Y(),
                                                                    0.001));
  else
    this->position_visual_->SetLocalScale(user_data_.position_scale * this->current_position_scale_);
}

void CovarianceVisual::updateRotVisualScale(ShapeIndex shapeIdx)
{
  if (this->cov_2d_)
    this->orientation_visuals_[shapeIdx]->SetLocalScale(
    ignition::math::Vector3d(0.001,
                              user_data_.orientation_scale * this->current_orientation_scales_[shapeIdx].Y(),
                              user_data_.orientation_scale * this->current_orientation_scales_[shapeIdx].Z()));
  else
    this->orientation_visuals_[shapeIdx]->SetLocalScale(
    ignition::math::Vector3d(user_data_.orientation_scale * this->current_orientation_scales_[shapeIdx].X(),
                              user_data_.orientation_scale * this->current_orientation_scales_[shapeIdx].Y(),
                              0.001));
}

void CovarianceVisual::updateRotVisualOffsets()
{
  double offset = this->user_data_.orientation_offset;
  orientation_visuals_[kRoll]->SetLocalPosition(offset * math::Vector3d::UnitX);
  orientation_visuals_[kPitch]->SetLocalPosition(offset * math::Vector3d::UnitY);
  orientation_visuals_[kYaw]->SetLocalPosition(offset * math::Vector3d::UnitZ);
}

void CovarianceVisual::updateRotVisualScales()
{
  this->updateRotVisualScale(kRoll);
  this->updateRotVisualScale(kPitch);
  this->updateRotVisualScale(kYaw);
  this->updateRotVisualScale(kYaw2D);
}

void CovarianceVisual::updateOrientationVisibility()
{
  bool orientation_visible_ = user_data_.orientation_visible && user_data_.visible;
  orientation_visuals_[kRoll]->SetVisible(orientation_visible_ && !cov_2d_);
  orientation_visuals_[kPitch]->SetVisible(orientation_visible_ && !cov_2d_);
  orientation_visuals_[kYaw]->SetVisible(orientation_visible_ && !cov_2d_);
  orientation_visuals_[kYaw2D]->SetVisible(orientation_visible_ && cov_2d_);
}

void CovarianceVisual::updateUserData() {
  position_root_visual_->SetVisible(user_data_.position_visible && user_data_.visible);
  this->updateOrientationVisibility();

  if (user_data_.position_frame == Frame::Local && fixed_orientation_visual_->HasChild(position_root_visual_))
  {
    this->root_visual_->AddChild(fixed_orientation_visual_->RemoveChild(position_root_visual_));
  }
  else if (user_data_.position_frame == Frame::Fixed && root_visual_->HasChild(position_root_visual_))
  {
    fixed_orientation_visual_->AddChild(this->root_visual_->RemoveChild(position_root_visual_));
  }
  if (user_data_.orientation_frame == Frame::Local && fixed_orientation_visual_->HasChild(orientation_root_visual_))
  {
    this->root_visual_->AddChild(fixed_orientation_visual_->RemoveChild(orientation_root_visual_));
  }
  else if (user_data_.orientation_frame == Frame::Fixed && root_visual_->HasChild(orientation_root_visual_))
  {
    fixed_orientation_visual_->AddChild(this->root_visual_->RemoveChild(orientation_root_visual_));
  }

  this->updatePosCovColor();
  this->updateRotCovColor();

  this->updatePosVisualScale();
  this->updateRotVisualScales();
  this->updateRotVisualOffsets();
}

void CovarianceVisual::updateMaterialColor(int idx, const ignition::math::Color& color)
{
  this->materials_[idx]->SetAmbient(color);
  this->materials_[idx]->SetDiffuse(color);
  this->materials_[idx]->SetEmissive(color);
}

void CovarianceVisual::updatePosCovColor()
{ 
  updateMaterialColor(kPos, user_data_.position_color);
  this->position_visual_->SetMaterial(this->materials_[kPos]);
}

void CovarianceVisual::updateRotCovColor()
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
  this->orientation_visuals_[kRoll]->SetMaterial(this->materials_[kRotX]);
  this->orientation_visuals_[kPitch]->SetMaterial(this->materials_[kRotY]);
  this->orientation_visuals_[kYaw]->SetMaterial(this->materials_[kRotZ]);
  this->orientation_visuals_[kYaw2D]->SetMaterial(this->materials_[kRotZ2D]);
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