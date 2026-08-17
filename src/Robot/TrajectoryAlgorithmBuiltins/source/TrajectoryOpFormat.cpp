/// @file TrajectoryOpFormat.cpp
/// @brief 算子格式

// TrajectoryOpFormat 实现
#include "TrajectoryOpFormat.h"

namespace trajectory_algo
{
std::string frameLabel(const RobotInstruction::TransformReferenceFrame frame, const bool chinese)
{
	if (frame == RobotInstruction::TransformReferenceFrame::Body)
	{
		return chinese ? "物体系" : "Body";
	}
	return chinese ? "世界系" : "World";
}

std::string scopeKindLabel(const RobotInstruction::OpScope::Kind kind, const bool chinese)
{
	switch (kind)
	{
	case RobotInstruction::OpScope::Kind::EntireProgram:
		return chinese ? "全程序" : "Program";
	case RobotInstruction::OpScope::Kind::PointIndexRange:
		return chinese ? "P范围" : "P range";
	case RobotInstruction::OpScope::Kind::Group:
	default:
		return chinese ? "分组" : "Group";
	}
}

} // namespace trajectory_algo
