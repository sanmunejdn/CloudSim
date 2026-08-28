/// @file RobotInstructionIkRequest.cpp
/// @brief IK 种子策略解析

#include "RobotInstructionIkRequest.h"

namespace RobotInstruction
{
void resolveIkRequestSeed(IkRequest& req, const std::vector<double>& currentQ,
						  const std::vector<double>& refInstructionQ)
{
	req.qSeedResolved.clear();
	if (req.seedPolicy == IkSeedPolicy::FromInstruction)
	{
		// 指定指令点：禁止静默降级到当前位姿
		if (!refInstructionQ.empty())
		{
			req.qSeedResolved = refInstructionQ;
		}
		return;
	}
	req.qSeedResolved = currentQ;
}
} // namespace RobotInstruction
