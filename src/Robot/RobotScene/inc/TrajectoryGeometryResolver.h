#pragma once

#include "TrajectoryPipelineTypes.h"
#include "UnifiedTrajectory.h"
#include "robot_scene_global.h"

#include <ShapeHandle.h>

#include <functional>
#include <string>
#include <vector>

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
	std::vector<float> positionsWorldMm;
	std::vector<float> triangleSoupWorldMm;
	geoalgo::ShapeHandle brepShape;
	/// 列主序 4×4：模型 mm → 世界 mm（BREP 射线求交用）
	double modelToWorldColMajor16[16]{};
	bool hasModelToWorld = false;
};

using TrajectoryGeometryResolveFn = std::function<bool(
	const std::string& backendId,
	TrajectoryGeometrySnapshot& out,
	std::string* errMsg)>;

ROBOT_SCENE_API void setTrajectoryGeometryResolver(TrajectoryGeometryResolveFn fn);
ROBOT_SCENE_API void clearTrajectoryGeometryResolver();
ROBOT_SCENE_API bool resolveTrajectoryGeometry(
	const std::string& backendId,
	TrajectoryGeometrySnapshot& out,
	std::string* errMsg);

ROBOT_SCENE_API bool projectUnifiedToGeometry(
	UnifiedTrajectory& traj,
	const ProjectToGeometryParams& params,
	const OpScope& scope,
	const RobotProgram* program,
	std::size_t* outMissCount,
	std::string* errMsg);

ROBOT_SCENE_API std::size_t trajectoryProjectionMissCount();
ROBOT_SCENE_API void resetTrajectoryProjectionMissCount();

} // namespace RobotInstruction
