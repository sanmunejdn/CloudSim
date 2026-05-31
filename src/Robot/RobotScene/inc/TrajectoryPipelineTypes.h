#pragma once

#include "robot_scene_global.h"

#include <string>
#include <vector>

namespace RobotInstruction
{

enum class ROBOT_SCENE_API TrajectoryOpKind
{
	Translate = 0,
	Rotate,
	Mirror,
	Delete,
	Duplicate,
	Reorder
};

/// 轨迹增量参考系（相对路点当前 T_base_target）
enum class ROBOT_SCENE_API TransformReferenceFrame : int
{
	World = 0,
	Body = 1
};

struct ROBOT_SCENE_API OpScope
{
	enum class Kind
	{
		EntireProgram = 0,
		Group,
		PointIndexRange,
		InstructionIds
	};

	Kind kind = Kind::Group;
	std::string groupId;
	int pointFrom = 1;
	int pointTo = 1;
	std::vector<std::string> instructionIds;
};

struct ROBOT_SCENE_API TranslateParams
{
	TransformReferenceFrame frame = TransformReferenceFrame::World;
	double dxMm = 0.0;
	double dyMm = 0.0;
	double dzMm = 0.0;
	double endDxMm = 0.0;
	double endDyMm = 0.0;
	double endDzMm = 0.0;
};

struct ROBOT_SCENE_API RotateParams
{
	TransformReferenceFrame frame = TransformReferenceFrame::World;
	double axisX = 0.0;
	double axisY = 0.0;
	double axisZ = 1.0;
	double angleDeg = 0.0;
	double endAngleDeg = 0.0;
};

struct ROBOT_SCENE_API TrajectoryOpDescriptor
{
	TrajectoryOpKind kind = TrajectoryOpKind::Translate;
	OpScope scope{};
	TranslateParams translate{};
	RotateParams rotate{};
	int duplicateCount = 1;
	int mirrorAxis = 0;
};

} // namespace RobotInstruction
