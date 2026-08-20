#ifndef ROBOTSCENE_RAWTRAJECTORY_H
#define ROBOTSCENE_RAWTRAJECTORY_H

/// @file RawTrajectory.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RawTrajectory 接口

#include "robot_scene_global.h"

#include "RobotInstructionModel.h"

#include <string>
#include <vector>

namespace geoalgo
{
struct RawPath;
}

namespace RobotInstruction
{
struct ROBOT_SCENE_API ExternalAxisSnapshot
{
	std::string jointName;
	double positionMmOrRad = 0.0;
	bool isPrismatic = false;
};

struct ROBOT_SCENE_API TrajectoryContext
{
	std::string workpieceFrameId = "workpiece";
	std::string toolFrameId = "tool0";
	std::vector<ExternalAxisSnapshot> externalAxes;
};

enum class ROBOT_SCENE_API FrameStrategy
{
	SurfaceNormalZ = 0,
	FixedZ,
	TangentX
};

struct ROBOT_SCENE_API TrajectoryPoint
{
	Vec3 poseMm;
	Vec3 eulerDeg;
	/// 与 pose 对应的四元数（xyzw）；有则写入时优先于欧拉，避免往返丢姿态
	double quatXyzw[4]{0.0, 0.0, 0.0, 1.0};
	bool hasQuat = false;
	/// 运动规划得到的关节角；有则落盘为 context.currentJointRadCsv，回放免重 IK
	std::vector<double> jointRad;
	double blendRadiusMm = 0.0;
	double speedMmPerSec = 0.0;
	bool reachable = true;
};

struct ROBOT_SCENE_API RawTrajectory
{
	std::vector<TrajectoryPoint> points;
	/// 各子折线在 points 中的结束下标（不含）；空表示整条为一段
	std::vector<std::size_t> segmentEndExclusive;
	TrajectoryContext ctx;
	/// FeatureSpec JSON（与 geometry_backend_ops::featureSpecToJson 契约一致）
	std::string sourceFeatureJson;
};

enum class ROBOT_SCENE_API RawTrajectoryOpKind
{
	FrameFromPath = 0,
	Resample,
	OffsetAlongNormal,
	OffsetLateral,
	SmoothPose,
	AssignBlend,
	AssignSpeedZone,
	Weave,
	InsertApproachRetract,
	ReachabilityFilter,
	ExternalAxisSearch,
	EmitToProgram
};

struct ROBOT_SCENE_API RawTrajectoryOpDescriptor
{
	RawTrajectoryOpKind kind = RawTrajectoryOpKind::FrameFromPath;
	FrameStrategy frameStrategy = FrameStrategy::SurfaceNormalZ;
	double stepMm = 2.0;
	double offsetMm = 0.0;
	double lateralMm = 0.0;
	double blendRadiusMm = 2.0;
	double speedMmPerSec = 100.0;
	double weaveAmplitudeMm = 2.0;
	double weavePeriodMm = 10.0;
	double approachMm = 20.0;
	double retractMm = 20.0;
	bool useLineMotion = true;
};

ROBOT_SCENE_API bool importRawPathToTrajectory(const geoalgo::RawPath& path, FrameStrategy strategy, RawTrajectory& out,
											   std::string* errMsg = nullptr);

ROBOT_SCENE_API bool applyRawTrajectoryOp(const RawTrajectoryOpDescriptor& op, RawTrajectory& trajectory,
										  std::string* errMsg = nullptr);

ROBOT_SCENE_API bool applyRawTrajectoryPipeline(const std::vector<RawTrajectoryOpDescriptor>& ops,
												RawTrajectory& trajectory, std::string* errMsg = nullptr);

ROBOT_SCENE_API std::string rawTrajectoryToPreviewPolylineXyz(const RawTrajectory& trajectory);
ROBOT_SCENE_API std::string rawTrajectoryReachabilityColorsJson(const RawTrajectory& trajectory);

ROBOT_SCENE_API std::string rawTrajectoryWorkpieceBackendId(const RawTrajectory& trajectory);
ROBOT_SCENE_API std::string rawTrajectoryFeatureId(const RawTrajectory& trajectory);

} // namespace RobotInstruction

#endif // ROBOTSCENE_RAWTRAJECTORY_H
