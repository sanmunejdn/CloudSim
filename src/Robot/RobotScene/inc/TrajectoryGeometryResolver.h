#ifndef ROBOTSCENE_TRAJECTORYGEOMETRYRESOLVER_H
#define ROBOTSCENE_TRAJECTORYGEOMETRYRESOLVER_H

/// @file TrajectoryGeometryResolver.h
/// @brief TrajectoryGeometryResolver 接口

#include "robot_scene_global.h"

#include "TrajectoryPipelineTypes.h"
#include "UnifiedTrajectory.h"

#include <functional>
#include <string>
#include <vector>

#include <ShapeHandle.h>

namespace RobotInstruction
{
enum class TrajectoryGeometryKind
{
	PointCloud = 0,
	TriangleMesh = 1,
	Brep = 2
};

struct ROBOT_SCENE_API TrajectoryGeometrySnapshot
{
	TrajectoryGeometryKind kind = TrajectoryGeometryKind::PointCloud;
	/// 后端模型坐标（未乘场景位姿；非刚性绑定用）
	std::vector<float> positionsModelMm;
	std::vector<float> triangleSoupModelMm;
	/// 当前场景世界 mm（投影等用）
	std::vector<float> positionsWorldMm;
	std::vector<float> triangleSoupWorldMm;
	geoalgo::ShapeHandle brepShape;
	/// 列主序 4×4：模型 mm → 世界 mm
	double modelToWorldColMajor16[16]{};
	bool hasModelToWorld = false;
};

using TrajectoryGeometryResolveFn =
	std::function<bool(const std::string& backendId, TrajectoryGeometrySnapshot& out, std::string* errMsg)>;

ROBOT_SCENE_API void setTrajectoryGeometryResolver(TrajectoryGeometryResolveFn fn);
ROBOT_SCENE_API void clearTrajectoryGeometryResolver();
/// 丢弃几何快照与 SPARE 结果缓存（物体被平移/旋转后必须调用）
ROBOT_SCENE_API void invalidateTrajectoryGeometryCache();
ROBOT_SCENE_API bool resolveTrajectoryGeometry(const std::string& backendId, TrajectoryGeometrySnapshot& out,
											   std::string* errMsg);

ROBOT_SCENE_API bool projectUnifiedToGeometry(UnifiedTrajectory& traj, const ProjectToGeometryParams& params,
											  const OpScope& scope, const RobotProgram* program,
											  std::size_t* outMissCount, std::string* errMsg);

ROBOT_SCENE_API std::size_t trajectoryProjectionMissCount();
ROBOT_SCENE_API void resetTrajectoryProjectionMissCount();

struct ROBOT_SCENE_API NonRigidWarpLastStats
{
	bool valid = false;
	std::size_t bindOk = 0;
	std::size_t bindFail = 0;
	double bindDistMinMm = 0.0;
	double bindDistMeanMm = 0.0;
	double bindDistMaxMm = 0.0;
	/// 0=世界系绑源World 1=工件模型系绑源Model（BREP 轨迹 + mesh 源）
	int bindMode = 0;
	double meanErrorMm = 0.0;
	int deformationNodeCount = 0;
	bool spareFromCache = false;
};

ROBOT_SCENE_API NonRigidWarpLastStats trajectoryNonRigidLastStats();
ROBOT_SCENE_API void resetTrajectoryNonRigidLastStats();

} // namespace RobotInstruction

#endif // ROBOTSCENE_TRAJECTORYGEOMETRYRESOLVER_H
