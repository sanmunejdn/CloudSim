#ifndef ROBOTSCENE_ROBOTINSTRUCTIONIKREQUEST_H
#define ROBOTSCENE_ROBOTINSTRUCTIONIKREQUEST_H

/// @file RobotInstructionIkRequest.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Pinocchio 形 IK 请求：目标 TCP + 显式种子策略（关节不进指令持久化）

#include "robot_scene_global.h"

#include <RigidTransform.h>

#include <string>
#include <vector>

namespace RobotInstruction
{
/// IK 种子来源；解析结果只进入 PlanRequest，不写指令
enum class ROBOT_SCENE_API IkSeedPolicy
{
	FromCurrentPose = 0,
	FromInstruction
};

struct ROBOT_SCENE_API IkRequest
{
	engine::RigidTransform targetTcpInBase{};
	IkSeedPolicy seedPolicy = IkSeedPolicy::FromInstruction;
	/// FromInstruction：指定路点 id；空表示使用调用方已解析的链式种子
	std::string seedInstructionId;
	std::vector<double> qSeedResolved;
};

/// 按策略填充 qSeedResolved：Current 用 currentQ；Instruction 优先 refQ，否则回退 currentQ
ROBOT_SCENE_API void resolveIkRequestSeed(IkRequest& req, const std::vector<double>& currentQ,
										  const std::vector<double>& refInstructionQ);

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONIKREQUEST_H
