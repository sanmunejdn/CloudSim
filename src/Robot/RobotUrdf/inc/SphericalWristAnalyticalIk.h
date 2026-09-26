#ifndef ROBOTURDF_SPHERICALWRISTANALYTICALIK_H
#define ROBOTURDF_SPHERICALWRISTANALYTICALIK_H

/// @file SphericalWristAnalyticalIk.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 球形腕 6R：腕心三点交汇时解析腕角，前三轴位置数值

#include "IPoseIkSolver.h"

namespace UrdfRobotLoader
{
class ROBOT_URDF_API SphericalWristAnalyticalIk : public IPoseIkSolver
{
public:
	const char* name() const override { return "analytical_spherical_wrist_6r"; }

	std::vector<double> solve(const QString& urdfPath, const QString& ikLink, const UrdfPoseIkTarget& target,
							  std::vector<double> seedJointRad, const UrdfIkSolverOptions& options,
							  std::string* failReason, IkConvergenceStatus* status) override;

	/// 6 转 + 末三轴近交于一点
	static bool isEligible(const QString& urdfPath);
};

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_SPHERICALWRISTANALYTICALIK_H
