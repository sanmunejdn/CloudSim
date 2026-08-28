#ifndef ROBOTSCENE_RUCKIGPTPTRAJECTORY_H
#define ROBOTSCENE_RUCKIGPTPTRAJECTORY_H

/// @file RuckigPtpTrajectory.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PTP 关节段：Ruckig state-to-state（头文件保持 C++17，实现侧 C++20）

#include "robot_scene_global.h"

#include <vector>

namespace RobotInstruction
{
struct ROBOT_SCENE_API RuckigPtpLimits
{
	/// rad/s；空则用默认
	std::vector<double> maxVelocityRadPerSec;
	std::vector<double> maxAccelerationRadPerSec2;
	std::vector<double> maxJerkRadPerSec3;
};

/// 生成 q0→q1 的限 jerk 关节轨迹；失败返回 false（调用方可回退 lerp）
ROBOT_SCENE_API bool buildRuckigPtpJointTrajectory(const std::vector<double>& q0,
												   const std::vector<double>& q1,
												   const RuckigPtpLimits& limits,
												   double sampleDtSec,
												   std::vector<std::vector<double>>& outTrajectoryRad,
												   double& outDurationSec);

/// 由指令 speed/accel（°/s、°/s²，与梯形时长同一套）生成关节限速
ROBOT_SCENE_API RuckigPtpLimits ruckigLimitsFromPtpDegScalars(size_t dof, double speedDegPerSec,
															  double accelDegPerSec2);

/// @deprecated 旧百分数映射；请用 ruckigLimitsFromPtpDegScalars
ROBOT_SCENE_API RuckigPtpLimits mapMotionScalarToRuckigLimits(size_t dof, double speed, double accel);

} // namespace RobotInstruction

#endif // ROBOTSCENE_RUCKIGPTPTRAJECTORY_H
