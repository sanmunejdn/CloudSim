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
	Reorder,
	Approach,
	Retract,
	Resample,
	OffsetAlongNormal,
	OffsetLateral,
	SmoothPose,
	AssignBlend,
	AssignSpeedZone,
	Weave,
	ReachabilityFilter,
	ExternalAxisSearch
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

struct ROBOT_SCENE_API ResampleParams
{
	double stepMm = 5.0;
};

struct ROBOT_SCENE_API PathOffsetParams
{
	double offsetMm = 0.0;
	double lateralMm = 0.0;
};

struct ROBOT_SCENE_API WeaveParams
{
	double amplitudeMm = 2.0;
	double periodMm = 10.0;
};

struct ROBOT_SCENE_API AssignMotionParams
{
	double blendRadiusMm = 2.0;
	double speedMmPerSec = 100.0;
};

/// 方向模式：路径切向 / 法向 / 工具Z
enum class ROBOT_SCENE_API ApproachDirectionMode : int
{
	PathTangent = 0,
	SurfaceNormal = 1,
	ToolZ = 2
};

/// 插点范围：整条轨迹头尾 或 分段头尾
enum class ROBOT_SCENE_API InsertMode : int
{
	Trajectory = 0,
	Segment = 1
};

/// 分段选择：全部段 或 区间段
enum class ROBOT_SCENE_API SegmentSelectMode : int
{
	AllSegments = 0,
	IndexRange = 1
};

struct ROBOT_SCENE_API ApproachParams
{
	double distanceMm = 20.0;
	ApproachDirectionMode directionMode = ApproachDirectionMode::SurfaceNormal;
	InsertMode insertMode = InsertMode::Trajectory;
	SegmentSelectMode segmentSelectMode = SegmentSelectMode::AllSegments;
	int segmentFrom = 1;
	int segmentTo = 1;
	bool overrideSpeedEnabled = false;
	double speedMmPerSec = 100.0;
};

struct ROBOT_SCENE_API RetractParams
{
	double distanceMm = 20.0;
	ApproachDirectionMode directionMode = ApproachDirectionMode::SurfaceNormal;
	InsertMode insertMode = InsertMode::Trajectory;
	SegmentSelectMode segmentSelectMode = SegmentSelectMode::AllSegments;
	int segmentFrom = 1;
	int segmentTo = 1;
	bool overrideSpeedEnabled = false;
	double speedMmPerSec = 100.0;
};

struct ROBOT_SCENE_API TrajectoryOpDescriptor
{
	TrajectoryOpKind kind = TrajectoryOpKind::Translate;
	OpScope scope{};
	TranslateParams translate{};
	RotateParams rotate{};
	int duplicateCount = 1;
	int mirrorAxis = 0;
	ResampleParams resample{};
	PathOffsetParams pathOffset{};
	WeaveParams weave{};
	AssignMotionParams assignMotion{};
	ApproachParams approach{};
	RetractParams retract{};
};

} // namespace RobotInstruction
