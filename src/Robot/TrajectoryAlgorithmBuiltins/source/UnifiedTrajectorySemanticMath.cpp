/// @file UnifiedTrajectorySemanticMath.cpp
/// @brief UnifiedTrajectorySemanticMath 实现

#include "UnifiedTrajectorySemanticMath.h"

#include "TrajectoryOpParamsParse.h"
#include "TrajectoryTransformMath.h"
#include "TrajectoryUnifiedScope.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace trajectory_algo
{
namespace
{
double lerp(const double a, const double b, const double t)
{
	return a + (b - a) * t;
}

RobotInstruction::TrajectoryOpDescriptor interpolateDescriptor(const RobotInstruction::TrajectoryOpDescriptor& op,
															   const double t)
{
	RobotInstruction::TrajectoryOpDescriptor out = op;
	interpolateTransformParamsInPlace(out, t);
	return out;
}

Eigen::Vector3d tangentDirection(const RobotInstruction::UnifiedTrajectory& traj, const std::size_t index)
{
	if (traj.points.size() < 2)
	{
		return Eigen::Vector3d::UnitX();
	}
	const std::size_t prev = index == 0 ? 0 : index - 1;
	const std::size_t next = index + 1 >= traj.points.size() ? traj.points.size() - 1 : index + 1;
	Eigen::Vector3d dir(traj.points[next].poseMm.x - traj.points[prev].poseMm.x,
						traj.points[next].poseMm.y - traj.points[prev].poseMm.y,
						traj.points[next].poseMm.z - traj.points[prev].poseMm.z);
	if (dir.norm() < 1e-9)
	{
		return Eigen::Vector3d::UnitX();
	}
	return dir.normalized();
}

Eigen::Vector3d toolZDirection(const RobotInstruction::UnifiedTrajectoryPoint& point)
{
	const engine::RigidTransform tf = rigidFromPoint(point);
	Eigen::Matrix3d rot = tf.rotation().toRotationMatrix();
	Eigen::Vector3d z = rot.col(2);
	if (z.norm() < 1e-9)
	{
		return Eigen::Vector3d::UnitZ();
	}
	return z.normalized();
}

Eigen::Vector3d normalizeOrFallback(const Eigen::Vector3d& v, const Eigen::Vector3d& fallback)
{
	if (v.norm() < 1e-9)
	{
		return fallback.normalized();
	}
	return v.normalized();
}

Eigen::Vector3d directionByMode(const RobotInstruction::UnifiedTrajectory& traj, const std::size_t index,
								const RobotInstruction::ApproachDirectionMode mode, const double customX,
								const double customY, const double customZ)
{
	switch (mode)
	{
	case RobotInstruction::ApproachDirectionMode::PathTangent:
		return tangentDirection(traj, index);
	case RobotInstruction::ApproachDirectionMode::ToolZ:
		return toolZDirection(traj.points[index]);
	case RobotInstruction::ApproachDirectionMode::Custom:
		return normalizeOrFallback(Eigen::Vector3d(customX, customY, customZ), Eigen::Vector3d::UnitZ());
	case RobotInstruction::ApproachDirectionMode::SurfaceNormal:
	default:
		return toolZDirection(traj.points[index]);
	}
}

Eigen::Vector3d resolveApproachDirection(const RobotInstruction::UnifiedTrajectory& traj, const std::size_t anchorIndex,
										 const RobotInstruction::ApproachDirectionMode mode,
										 const RobotInstruction::TransformReferenceFrame frame, const double customX,
										 const double customY, const double customZ)
{
	Eigen::Vector3d dir = directionByMode(traj, anchorIndex, mode, customX, customY, customZ);
	if (frame == RobotInstruction::TransformReferenceFrame::Body && anchorIndex < traj.points.size())
	{
		const engine::RigidTransform tf = rigidFromPoint(traj.points[anchorIndex]);
		dir = tf.rotation().toRotationMatrix() * dir;
		dir = normalizeOrFallback(dir, Eigen::Vector3d::UnitZ());
	}
	return dir;
}

bool segmentIndexInRange(const int segmentIndex1Based, const RobotInstruction::SegmentSelectMode mode, const int from,
						 const int to)
{
	if (mode == RobotInstruction::SegmentSelectMode::AllSegments)
	{
		return true;
	}
	return segmentIndex1Based >= from && segmentIndex1Based <= to;
}

void appendApproachAnchors(const std::vector<std::size_t>& indices, const RobotInstruction::InsertMode insertMode,
						   const RobotInstruction::SegmentSelectMode segmentMode, const int segmentFrom,
						   const int segmentTo, std::vector<std::size_t>& outAnchors)
{
	if (indices.empty())
	{
		return;
	}
	if (insertMode == RobotInstruction::InsertMode::Trajectory)
	{
		outAnchors.push_back(indices.front());
		return;
	}
	if (indices.size() == 1)
	{
		if (segmentIndexInRange(1, segmentMode, segmentFrom, segmentTo))
		{
			outAnchors.push_back(indices.front());
		}
		return;
	}
	for (std::size_t s = 0; s + 1 < indices.size(); ++s)
	{
		const int segNum = static_cast<int>(s + 1);
		if (segmentIndexInRange(segNum, segmentMode, segmentFrom, segmentTo))
		{
			outAnchors.push_back(indices[s]);
		}
	}
}

void appendRetractAnchors(const std::vector<std::size_t>& indices, const RobotInstruction::InsertMode insertMode,
						  const RobotInstruction::SegmentSelectMode segmentMode, const int segmentFrom,
						  const int segmentTo, std::vector<std::size_t>& outAnchors)
{
	if (indices.empty())
	{
		return;
	}
	if (insertMode == RobotInstruction::InsertMode::Trajectory)
	{
		outAnchors.push_back(indices.back());
		return;
	}
	if (indices.size() == 1)
	{
		if (segmentIndexInRange(1, segmentMode, segmentFrom, segmentTo))
		{
			outAnchors.push_back(indices.front());
		}
		return;
	}
	for (std::size_t s = 0; s + 1 < indices.size(); ++s)
	{
		const int segNum = static_cast<int>(s + 1);
		if (segmentIndexInRange(segNum, segmentMode, segmentFrom, segmentTo))
		{
			outAnchors.push_back(indices[s + 1]);
		}
	}
}

void applyAxisReverse(RobotInstruction::UnifiedTrajectoryPoint& point, const int mirrorAxis)
{
	engine::RigidTransform tf = rigidFromPoint(point);
	Eigen::Matrix3d rot = tf.rotation().toRotationMatrix();
	Eigen::Vector3d axes[3] = {rot.col(0), rot.col(1), rot.col(2)};
	const int reversedAxis = std::max(0, std::min(2, mirrorAxis));
	const int keptAxis = (reversedAxis + 1) % 3;
	const int rebuiltAxis = 3 - reversedAxis - keptAxis;
	axes[reversedAxis] = -axes[reversedAxis];
	axes[rebuiltAxis] = axes[reversedAxis].cross(axes[keptAxis]);
	if (axes[rebuiltAxis].norm() < 1e-9)
	{
		return;
	}
	axes[rebuiltAxis].normalize();
	axes[keptAxis] = axes[rebuiltAxis].cross(axes[reversedAxis]);
	if (axes[keptAxis].norm() < 1e-9)
	{
		return;
	}
	axes[reversedAxis].normalize();
	axes[keptAxis].normalize();
	Eigen::Matrix3d updatedRot;
	updatedRot.col(0) = axes[0];
	updatedRot.col(1) = axes[1];
	updatedRot.col(2) = axes[2];
	pointFromRigid(engine::RigidTransform::fromTranslationQuat(tf.translationMm(), Eigen::Quaterniond(updatedRot)),
				   point);
}

} // namespace

engine::RigidTransform rigidFromPoint(const RobotInstruction::UnifiedTrajectoryPoint& point)
{
	return engine::RigidTransform::fromTranslationEulerDeg(point.poseMm.x, point.poseMm.y, point.poseMm.z,
														   point.eulerDeg.x, point.eulerDeg.y, point.eulerDeg.z);
}

void pointFromRigid(const engine::RigidTransform& tf, RobotInstruction::UnifiedTrajectoryPoint& point)
{
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	tf.translationMm(px, py, pz);
	tf.eulerDegForDisplay(ex, ey, ez);
	point.poseMm = RobotInstruction::Vec3{px, py, pz};
	point.eulerDeg = RobotInstruction::Vec3{ex, ey, ez};
}

bool applyTranslateRotateInScope(const RobotInstruction::TrajectoryOpDescriptor& op,
								 RobotInstruction::UnifiedTrajectory& traj,
								 const RobotInstruction::RobotProgram* program)
{
	if (traj.points.empty())
	{
		return false;
	}
	const std::vector<std::size_t> scoped = resolveScopedPointIndices(traj, op.scope, program);
	if (scoped.empty())
	{
		return true;
	}
	const std::size_t scopeCount = scoped.size();
	for (std::size_t j = 0; j < scopeCount; ++j)
	{
		const std::size_t i = scoped[j];
		const double t = scopeCount <= 1 ? 0.0 : static_cast<double>(j) / static_cast<double>(scopeCount - 1);
		const RobotInstruction::TrajectoryOpDescriptor current = interpolateDescriptor(op, t);
		engine::RigidTransform tf = rigidFromPoint(traj.points[i]);
		if (current.kind == RobotInstruction::TrajectoryOpKind::Translate)
		{
			const RobotInstruction::TranslateParams translate = parseTranslateParams(current.params);
			const engine::RigidTransform delta = rigidDeltaFromTranslate(translate);
			tf = applyTransformDelta(tf, delta, translate.frame);
		}
		else
		{
			const RobotInstruction::RotateParams rotate = parseRotateParams(current.params);
			const engine::RigidTransform delta = rigidDeltaFromRotate(rotate);
			tf = applyTransformDelta(tf, delta, rotate.frame);
		}
		pointFromRigid(tf, traj.points[i]);
	}
	return true;
}

bool applyMirrorInScope(const RobotInstruction::TrajectoryOpDescriptor& op, RobotInstruction::UnifiedTrajectory& traj,
						const RobotInstruction::RobotProgram* program)
{
	if (traj.points.empty())
	{
		return false;
	}
	const std::vector<std::size_t> scoped = resolveScopedPointIndices(traj, op.scope, program);
	for (const std::size_t i : scoped)
	{
		applyAxisReverse(traj.points[i], parseMirrorAxis(op.params));
	}
	return true;
}

bool applyReorderInScope(const RobotInstruction::TrajectoryOpDescriptor& op, RobotInstruction::UnifiedTrajectory& traj,
						 const RobotInstruction::RobotProgram* program)
{
	if (traj.points.empty())
	{
		return false;
	}
	const std::vector<std::size_t> scoped = resolveScopedPointIndices(traj, op.scope, program);
	if (scoped.empty())
	{
		return true;
	}
	RobotInstruction::Vec3 refEuler = traj.points[scoped.front()].eulerDeg;
	for (const std::size_t i : scoped)
	{
		traj.points[i].eulerDeg = refEuler;
	}
	return true;
}

void insertApproachInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::ApproachParams& params,
						   const RobotInstruction::OpScope& scope, const RobotInstruction::RobotProgram* program)
{
	if (traj.points.empty())
	{
		return;
	}
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	std::vector<std::size_t> anchors;
	appendApproachAnchors(indices, params.insertMode, params.segmentSelectMode, params.segmentFrom, params.segmentTo,
						  anchors);
	if (anchors.empty())
	{
		return;
	}
	std::sort(anchors.begin(), anchors.end());
	anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
	for (auto it = anchors.rbegin(); it != anchors.rend(); ++it)
	{
		const std::size_t anchor = *it;
		if (anchor >= traj.points.size())
		{
			continue;
		}
		RobotInstruction::UnifiedTrajectoryPoint point = traj.points[anchor];
		const Eigen::Vector3d dir =
			resolveApproachDirection(traj, anchor, params.directionMode, params.directionFrame, params.customDirectionX,
									 params.customDirectionY, params.customDirectionZ);
		point.poseMm.x -= dir.x() * params.distanceMm;
		point.poseMm.y -= dir.y() * params.distanceMm;
		point.poseMm.z -= dir.z() * params.distanceMm;
		point.sourceInstructionId.clear();
		if (params.overrideSpeedEnabled && params.speedMmPerSec > 0.0)
		{
			point.speedMmPerSec = params.speedMmPerSec;
		}
		traj.points.insert(traj.points.begin() + static_cast<std::ptrdiff_t>(anchor), point);
	}
}

void insertRetractInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::RetractParams& params,
						  const RobotInstruction::OpScope& scope, const RobotInstruction::RobotProgram* program)
{
	if (traj.points.empty())
	{
		return;
	}
	const std::vector<std::size_t> indices = resolveScopedPointIndices(traj, scope, program);
	std::vector<std::size_t> anchors;
	appendRetractAnchors(indices, params.insertMode, params.segmentSelectMode, params.segmentFrom, params.segmentTo,
						 anchors);
	if (anchors.empty())
	{
		return;
	}
	std::sort(anchors.begin(), anchors.end());
	anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
	for (auto it = anchors.rbegin(); it != anchors.rend(); ++it)
	{
		const std::size_t anchor = *it;
		if (anchor >= traj.points.size())
		{
			continue;
		}
		RobotInstruction::UnifiedTrajectoryPoint point = traj.points[anchor];
		const Eigen::Vector3d dir =
			resolveApproachDirection(traj, anchor, params.directionMode, params.directionFrame, params.customDirectionX,
									 params.customDirectionY, params.customDirectionZ);
		point.poseMm.x += dir.x() * params.distanceMm;
		point.poseMm.y += dir.y() * params.distanceMm;
		point.poseMm.z += dir.z() * params.distanceMm;
		point.sourceInstructionId.clear();
		if (params.overrideSpeedEnabled && params.speedMmPerSec > 0.0)
		{
			point.speedMmPerSec = params.speedMmPerSec;
		}
		traj.points.insert(traj.points.begin() + static_cast<std::ptrdiff_t>(anchor + 1U), point);
	}
}

} // namespace trajectory_algo
