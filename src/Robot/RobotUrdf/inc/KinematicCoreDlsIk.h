#ifndef ROBOTURDF_KINEMATICCOREDLSK_H
#define ROBOTURDF_KINEMATICCOREDLSK_H

/// @file KinematicCoreDlsIk.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief KinematicCore DLS 作为 IPoseIkSolver

#include "IPoseIkSolver.h"

namespace UrdfRobotLoader
{
class ROBOT_URDF_API KinematicCoreDlsIk : public IPoseIkSolver
{
public:
	const char* name() const override { return "kinematic_core_dls"; }

	std::vector<double> solve(const QString& urdfPath, const QString& ikLink, const UrdfPoseIkTarget& target,
							  std::vector<double> seedJointRad, const UrdfIkSolverOptions& options,
							  std::string* failReason, IkConvergenceStatus* status) override;
};

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_KINEMATICCOREDLSK_H
