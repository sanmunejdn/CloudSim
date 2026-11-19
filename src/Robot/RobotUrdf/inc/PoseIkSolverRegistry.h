#ifndef ROBOTURDF_POSEIKSOLVERREGISTRY_H
#define ROBOTURDF_POSEIKSOLVERREGISTRY_H

/// @file PoseIkSolverRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 解析优先 / 数值回退 / 竞速择近

#include "robot_urdf_global.h"

#include "IPoseIkSolver.h"
#include "PoseIkCandidate.h"
#include "UrdfIkSolverOptions.h"
#include "UrdfNumericalIk.h"

#include <QString>
#include <string>
#include <vector>

namespace UrdfRobotLoader
{
/// 按机型与 options 调度球形腕 + DLS；行为对齐原 solveArmPoseDampedLeastSquares
ROBOT_URDF_API std::vector<double> solveArmPoseViaRegistry(const QString& urdfPath, const QString& ikLink,
														   const UrdfPoseIkTarget& target,
														   std::vector<double> seedJointRad,
														   const UrdfIkSolverOptions& options,
														   std::string* failReason = nullptr,
														   IkConvergenceStatus* status = nullptr);

/// 收齐解析多腕支（若适用）+ 可选数值；供 Pipeline 入池
ROBOT_URDF_API std::vector<PoseIkCandidate> collectPoseIkCandidates(const QString& urdfPath, const QString& ikLink,
																	const UrdfPoseIkTarget& target,
																	std::vector<double> seedJointRad,
																	const UrdfIkSolverOptions& options,
																	bool includeAnalytical, bool includeNumerical,
																	std::string* failReason = nullptr);

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_POSEIKSOLVERREGISTRY_H
