/// @file TrajectoryTransformMath.cpp
/// @brief TrajectoryTransformMath 实现

#include "TrajectoryTransformMath.h"

#include <cmath>

#include <Eigen/Geometry>

namespace trajectory_algo
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

engine::RigidTransform rigidDeltaFromTranslate(const RobotInstruction::TranslateParams& translate)
{
	return engine::RigidTransform::fromTranslationQuat(Eigen::Vector3d(translate.dxMm, translate.dyMm, translate.dzMm),
													   Eigen::Quaterniond::Identity());
}

engine::RigidTransform rigidDeltaFromRotate(const RobotInstruction::RotateParams& rotate)
{
	Eigen::Vector3d axis(rotate.axisX, rotate.axisY, rotate.axisZ);
	if (axis.norm() < 1e-9)
	{
		axis = Eigen::Vector3d::UnitZ();
	}
	axis.normalize();
	const double rad = rotate.angleDeg * kPi / 180.0;
	return engine::RigidTransform::fromTranslationQuat(Eigen::Vector3d::Zero(),
													   Eigen::Quaterniond(Eigen::AngleAxisd(rad, axis)));
}

engine::RigidTransform applyTransformDelta(const engine::RigidTransform& target, const engine::RigidTransform& delta,
										   RobotInstruction::TransformReferenceFrame frame)
{
	if (frame == RobotInstruction::TransformReferenceFrame::Body)
	{
		return target.composeColumn(delta);
	}
	return delta.composeColumn(target);
}

} // namespace trajectory_algo
