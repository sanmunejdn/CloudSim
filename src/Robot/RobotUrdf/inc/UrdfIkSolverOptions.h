#ifndef ROBOTURDF_URDFIKSOLVEROPTIONS_H
#define ROBOTURDF_URDFIKSOLVEROPTIONS_H

/// @file UrdfIkSolverOptions.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 位姿 IK 容差/阻尼/迭代与硬软收敛策略

#include "robot_urdf_global.h"

namespace UrdfRobotLoader
{
/// 对外成功分级：Soft 仅当 allowApproximateOrientation
enum class ROBOT_URDF_API IkConvergenceStatus
{
	Failed = 0,
	HardConverged = 1,
	SoftAccepted = 2,
};

struct ROBOT_URDF_API UrdfIkSolverOptions
{
	double lambda = 1e-2;
	double positionToleranceMm = 1e-2;
	/// 硬收敛姿态容差（默认 0.1°）
	double orientationToleranceRad = 0.1 * 3.14159265358979323846 / 180.0;
	double orientationWeight = 300.0;
	/// 0：沿用内部默认步进
	double maxJointStepRad = 0.0;
	/// 0：默认迭代上限
	int maxIterations = 0;
	/// 0：默认 pos-then-ori 尝试次数
	int maxPosThenOriAttempts = 0;
	/// 仅顶层规划器开启；禁止与 UI 随机重启叠加
	bool allowInternalRandomRestart = false;
	/// false：SoftAccepted 视为失败
	bool allowApproximateOrientation = false;
	/// 内核 SoftAccepted 分类用（默认 2°）；对外验收见 IkAcceptanceGates（硬 0.5° / 软 5°）
	double softOrientationToleranceRad = 2.0 * 3.14159265358979323846 / 180.0;
};

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_URDFIKSOLVEROPTIONS_H
