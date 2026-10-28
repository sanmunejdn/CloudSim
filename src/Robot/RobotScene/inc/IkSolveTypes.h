#ifndef ROBOTSCENE_IKSOLVETYPES_H
#define ROBOTSCENE_IKSOLVETYPES_H

/// @file IkSolveTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 轴配置 IK 强类型请求/结果（与 Plan 层 IkRequest 种子策略分离）

#include "robot_scene_global.h"

#include "RobotInstructionAxisConfiguration.h"

#include <string>
#include <vector>

namespace RobotInstruction
{
/// elbowSign: 0=肘角≥0，1=否则；wristBranch: 解析腕 0|1
inline int encodeIkBranchId(const int elbowSign, const int wristBranch)
{
	return ((elbowSign & 1) << 1) | (wristBranch & 1);
}

inline int ikBranchElbowSign(const int branchId)
{
	return (branchId >> 1) & 1;
}

inline int ikBranchWrist(const int branchId)
{
	return branchId & 1;
}

struct ROBOT_SCENE_API IkSolveRequest
{
	std::vector<double> qSeed;
	MotionAxisConfiguration cfg{};
	IkSelectionPolicy policy{};
	std::string urdfPath;
	std::vector<std::string> jointNames;
	int elbowJointIndex = -1;
	int wristJointIndex = -1;
	/// 转数参照；空则用 qSeed
	std::vector<double> classRefQ;
	bool sphericalEligible = false;
	bool dragProfile = false;
};

struct ROBOT_SCENE_API IkSolveResult
{
	std::vector<double> q;
	IkFailKind failKind = IkFailKind::None;
	std::string message;
	int branchId = -1;
	/// 例： [ikEscalate=preview]
	std::string escalateTag;

	bool ok() const { return !q.empty(); }
};

} // namespace RobotInstruction

#endif // ROBOTSCENE_IKSOLVETYPES_H
