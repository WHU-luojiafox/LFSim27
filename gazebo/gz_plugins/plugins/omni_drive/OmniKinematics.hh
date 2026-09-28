// Copyright 2026 RMOSS contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef RMOSS_OMNI_KINEMATICS_HH
#define RMOSS_OMNI_KINEMATICS_HH

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace rmoss
{
struct OmniWheel
{
  double x, y, rollingAngle, radius, maxAngularSpeed;
};

// Body frame: x forward, y left, positive yaw counterclockwise.
// rollingAngle is the driven direction, NOT a mecanum roller angle.
class OmniKinematics
{
public:
  explicit OmniKinematics(const std::vector<OmniWheel> &wheels)
  : wheels_(wheels), jacobian_(wheels.size(), 3)
  {
    if (wheels.size() < 3) throw std::invalid_argument("at least three wheels required");
    for (size_t i = 0; i < wheels.size(); ++i)
    {
      const auto &w = wheels[i];
      if (!std::isfinite(w.x) || !std::isfinite(w.y) ||
          !std::isfinite(w.rollingAngle) || !std::isfinite(w.radius) ||
          !std::isfinite(w.maxAngularSpeed) || w.radius <= 0 || w.maxAngularSpeed <= 0)
        throw std::invalid_argument("invalid wheel geometry or speed limit");
      const double c = std::cos(w.rollingAngle), s = std::sin(w.rollingAngle);
      jacobian_.row(i) << c / w.radius, s / w.radius, (w.x * s - w.y * c) / w.radius;
    }
    if (jacobian_.fullPivLu().rank() != 3)
      throw std::invalid_argument("wheel layout cannot control all three body axes");
  }

  Eigen::VectorXd wheelSpeeds(const Eigen::Vector3d &body) const
  {
    if (!body.allFinite()) throw std::invalid_argument("non-finite body command");
    return jacobian_ * body;
  }

  // Scale all axes together, preserving the requested direction and curvature.
  Eigen::Vector3d limit(const Eigen::Vector3d &body) const
  {
    const auto speeds = wheelSpeeds(body);
    double scale = 1.0;
    for (size_t i = 0; i < wheels_.size(); ++i)
      if (std::abs(speeds[i]) > wheels_[i].maxAngularSpeed)
        scale = std::min(scale, wheels_[i].maxAngularSpeed / std::abs(speeds[i]));
    return scale * body;
  }

private:
  std::vector<OmniWheel> wheels_;
  Eigen::MatrixXd jacobian_;
};
}  // namespace rmoss
#endif
