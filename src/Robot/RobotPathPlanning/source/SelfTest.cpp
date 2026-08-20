/// @file SelfTest.cpp
/// @brief 无 URDF 场景的 API 与 RRT 烟雾测试

#include "RobotPathPlanning.h"

#include "CollisionWorld.h"

#include <cmath>

namespace robot_path
{
namespace
{

bool testEmptyRequest(PathResult& scratch)
{
	PlanRequest req;
	return !planToTcpPose(req, scratch) && !scratch.errMsg.empty();
}

bool testCollisionWorldBox()
{
	collision::CollisionWorld world;
	std::vector<float> box = {
		0, 0, 0, 100, 0, 0, 0, 100, 0, // tri 1
	};
	collision::CollisionBodyId scene;
	scene.kind = "scene";
	scene.backendId = "test_box";
	collision::Mat4 W{};
	for (int i = 0; i < 16; ++i)
		W[static_cast<size_t>(i)] = (i % 5 == 0) ? 1.0 : 0.0;
	world.upsertMeshBody(scene, box.data(), box.size(), W);
	return world.bodyCount() == 1 && !world.checkAll().inCollision;
}

} // namespace

bool runSelfTest(std::vector<std::string>& failures)
{
	failures.clear();
	PathResult scratch;
	if (!testEmptyRequest(scratch))
		failures.push_back("empty request should fail");
	if (!testCollisionWorldBox())
		failures.push_back("collision world box smoke");
	return failures.empty();
}

} // namespace robot_path
