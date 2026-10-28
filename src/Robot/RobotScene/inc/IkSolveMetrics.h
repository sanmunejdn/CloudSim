#ifndef ROBOTSCENE_IKSOLVEMETRICS_H
#define ROBOTSCENE_IKSOLVEMETRICS_H

/// @file IkSolveMetrics.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 进程内 IK 成功/失败/耗时计数（排障调预算，非完整仪表盘）

#include "robot_scene_global.h"

#include "RobotInstructionAxisConfiguration.h"

#include <cstdint>
#include <string>

namespace RobotInstruction
{
struct ROBOT_SCENE_API IkSolveMetricsSnapshot
{
	std::uint64_t success = 0;
	std::uint64_t fail = 0;
	std::uint64_t failBudget = 0;
	std::uint64_t failWrist = 0;
	std::uint64_t failAxisClass = 0;
	std::uint64_t failOther = 0;
	std::uint64_t totalElapsedMs = 0;
	std::uint64_t samples = 0;
};

ROBOT_SCENE_API void recordIkSolveSuccess(int elapsedMs);
ROBOT_SCENE_API void recordIkSolveFailure(IkFailKind kind, int elapsedMs);
ROBOT_SCENE_API IkSolveMetricsSnapshot ikSolveMetricsSnapshot();
ROBOT_SCENE_API void resetIkSolveMetrics();
ROBOT_SCENE_API std::string formatIkSolveMetricsSummary();

} // namespace RobotInstruction

#endif // ROBOTSCENE_IKSOLVEMETRICS_H
