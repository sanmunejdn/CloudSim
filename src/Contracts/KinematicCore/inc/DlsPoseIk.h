#ifndef KINEMATICCORE_DLSPOSEIK_H
#define KINEMATICCORE_DLSPOSEIK_H

#include "KinematicGraph.h"
#include "kinematic_core_global.h"

#include <vector>

namespace kinematic_core
{
struct KINEMATIC_CORE_API PoseIkTarget
{
	double positionMm[3]{0.0, 0.0, 0.0};
	bool hasOrientation = false;
	double quatXyzw[4]{0.0, 0.0, 0.0, 1.0};
};

struct KINEMATIC_CORE_API DlsPoseIkOptions
{
	int maxIterations = 80;
	double positionToleranceMm = 0.5;
	double orientationToleranceRad = 0.02;
	double orientationWeight = 300.0;
	double lambdaDamping = 0.05;
	double stepLimitRad = 0.25;
};

struct KINEMATIC_CORE_API DlsPoseIkResult
{
	bool success = false;
	int iterationsUsed = 0;
	double positionErrorMm = 0.0;
	double orientationErrorRad = 0.0;
};

/// 阻尼最小二乘位姿 IK（位置 + 可选姿态）
KINEMATIC_CORE_API DlsPoseIkResult solvePoseDampedLeastSquares(const KinematicGraph& graph, const double baseWorld[16],
															 int targetLinkIdx, const PoseIkTarget& target,
															 std::vector<double>& qInOut, const DlsPoseIkOptions& opt = {});

} // namespace kinematic_core

#endif // KINEMATICCORE_DLSPOSEIK_H
