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
	double dxMm = 0.0;
	double dyMm = 0.0;
	double dzMm = 0.0;
};

struct ROBOT_SCENE_API RotateParams
{
	double axisX = 0.0;
	double axisY = 0.0;
	double axisZ = 1.0;
	double angleDeg = 0.0;
};

struct ROBOT_SCENE_API TrajectoryOpDescriptor
{
	TrajectoryOpKind kind = TrajectoryOpKind::Translate;
	OpScope scope{};
	TranslateParams translate{};
	RotateParams rotate{};
	int duplicateCount = 1;
};

} // namespace RobotInstruction
