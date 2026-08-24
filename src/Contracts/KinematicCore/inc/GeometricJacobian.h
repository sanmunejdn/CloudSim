#ifndef KINEMATICCORE_GEOMETRICJACOBIAN_H
#define KINEMATICCORE_GEOMETRICJACOBIAN_H

#include "KinematicGraph.h"
#include "kinematic_core_global.h"

#include <vector>

namespace kinematic_core
{
struct KINEMATIC_CORE_API JacobianOptions
{
	double axisEps = 1e-9;
	double orientationWeight = 1.0;
};

/// 位置雅可比 3×n（行主序 J[r*n+c]）
KINEMATIC_CORE_API bool computePositionJacobian(const KinematicGraph& graph, const double baseWorld[16],
												const double* q, std::size_t qCount, int targetLinkIdx,
												std::vector<double>& J_3xn, const JacobianOptions& opt = {});

/// 位姿雅可比 6×n：位置 3 行 + 姿态 3 行（旋转关节列为世界系轴；平移关节姿态行 0）
KINEMATIC_CORE_API bool computePoseJacobian(const KinematicGraph& graph, const double baseWorld[16], const double* q,
											std::size_t qCount, int targetLinkIdx, std::vector<double>& J_6xn,
											const JacobianOptions& opt = {});

} // namespace kinematic_core

#endif // KINEMATICCORE_GEOMETRICJACOBIAN_H
