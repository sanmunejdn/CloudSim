#ifndef ROBOTURDF_IPOSEIKSOLVER_H
#define ROBOTURDF_IPOSEIKSOLVER_H

/// @file IPoseIkSolver.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 位姿 IK 可插拔接口（本波：球形腕解析 + DLS；Pinocchio 仅预留）

#include "robot_urdf_global.h"

#include "UrdfIkSolverOptions.h"
#include "UrdfNumericalIk.h"

#include <QString>
#include <string>
#include <vector>

namespace UrdfRobotLoader
{
/// 未来可接 PinocchioPoseIkSolver；本波不引入 SDK
class ROBOT_URDF_API IPoseIkSolver
{
public:
	virtual ~IPoseIkSolver() = default;

	virtual const char* name() const = 0;

	/// 不适用时返回空；调用方回退下一求解器
	virtual std::vector<double> solve(const QString& urdfPath, const QString& ikLink, const UrdfPoseIkTarget& target,
									  std::vector<double> seedJointRad, const UrdfIkSolverOptions& options,
									  std::string* failReason, IkConvergenceStatus* status) = 0;
};

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_IPOSEIKSOLVER_H
