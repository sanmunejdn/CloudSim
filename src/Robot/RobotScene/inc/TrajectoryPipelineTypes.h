#ifndef ROBOTSCENE_TRAJECTORYPIPELINETYPES_H
#define ROBOTSCENE_TRAJECTORYPIPELINETYPES_H

/// @file TrajectoryPipelineTypes.h
/// @brief 轨迹增量参考系（相对路点当前 T_base_target）

#include "robot_scene_global.h"

#include <string>
#include <vector>

#include <json.hpp>

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
	ExternalAxisSearch,
	ProjectToGeometry,
	NonRigidRegistration,
	ToWorkpieceInHand
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

/// 方向模式：路径切向 / 法向 / 工具Z / 自定义向量
enum class ROBOT_SCENE_API ApproachDirectionMode : int
{
	PathTangent = 0,
	SurfaceNormal = 1,
	ToolZ = 2,
	Custom = 3
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
	TransformReferenceFrame directionFrame = TransformReferenceFrame::World;
	double customDirectionX = 0.0;
	double customDirectionY = 0.0;
	double customDirectionZ = -1.0;
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
	TransformReferenceFrame directionFrame = TransformReferenceFrame::World;
	double customDirectionX = 0.0;
	double customDirectionY = 0.0;
	double customDirectionZ = 1.0;
	InsertMode insertMode = InsertMode::Trajectory;
	SegmentSelectMode segmentSelectMode = SegmentSelectMode::AllSegments;
	int segmentFrom = 1;
	int segmentTo = 1;
	bool overrideSpeedEnabled = false;
	double speedMmPerSec = 100.0;
};

struct ROBOT_SCENE_API ProjectToGeometryParams
{
	std::string targetBackendId;
	TransformReferenceFrame directionFrame = TransformReferenceFrame::World;
	double directionX = 0.0;
	double directionY = 0.0;
	double directionZ = -1.0;
	double maxDistanceMm = 5000.0;
	double pointCloudHitRadiusMm = 2.0;
};

struct ROBOT_SCENE_API NonRigidRegistrationParams
{
	std::string sourceBackendId;
	std::string targetBackendId;
	double maxBindDistanceMm = 30.0;
	double sampleRadiusRatio = 0.0;
	int maxOuterIters = 30;
	bool rigidPreAlign = false;
	double voxelPrefilterMm = 0.0;
};

/// 工具型→工件型：外部 TCP 参数；参考位姿由 ExecutionContext 注入
struct ROBOT_SCENE_API ToWorkpieceInHandParams
{
	std::string externalTcpBackendId;
	double externalTcpXMm = 0.0;
	double externalTcpYMm = 0.0;
	double externalTcpZMm = 0.0;
	double externalTcpRxDeg = 0.0;
	double externalTcpRyDeg = 0.0;
	double externalTcpRzDeg = 0.0;
	bool enableSpeedTransform = false;
};

struct ROBOT_SCENE_API TrajectoryOpDescriptor
{
	std::string opId;
	TrajectoryOpKind kind = TrajectoryOpKind::Translate;
	OpScope scope{};
	nlohmann::json params = nlohmann::json::object();
	// 未启用时引擎跳过；新建块默认 false，旧 JSON 缺字段按 true
	bool enabled = false;
};

} // namespace RobotInstruction

#endif // ROBOTSCENE_TRAJECTORYPIPELINETYPES_H
