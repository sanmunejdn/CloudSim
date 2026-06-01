#include "TrajectoryOpRegistry.h"

#include "DeleteOp.h"
#include "DuplicateOp.h"
#include "MirrorOp.h"
#include "RecipeGlueOp.h"
#include "RecipeGrindOp.h"
#include "RecipeWeldOp.h"
#include "ReorderOp.h"
#include "RetractOp.h"
#include "RotateOp.h"
#include "TranslateOp.h"
#include "ApproachOp.h"

namespace trajectory_algo
{
namespace
{
bool g_builtinsRegistered = false;
}

void registerTrajectoryOpBuiltins(TrajectoryOpRegistry& registry)
{
	registry.registerOp(std::make_unique<TranslateOp>());
	registry.registerOp(std::make_unique<RotateOp>());
	registry.registerOp(std::make_unique<DeleteOp>());
	registry.registerOp(std::make_unique<DuplicateOp>());
	registry.registerOp(std::make_unique<MirrorOp>());
	registry.registerOp(std::make_unique<ReorderOp>());
	registry.registerOp(std::make_unique<RecipeWeldOp>());
	registry.registerOp(std::make_unique<RecipeGlueOp>());
	registry.registerOp(std::make_unique<RecipeGrindOp>());
	registry.registerOp(std::make_unique<ApproachOp>());
	registry.registerOp(std::make_unique<RetractOp>());
}

void ensureTrajectoryOpBuiltinsRegistered()
{
	if (g_builtinsRegistered)
	{
		return;
	}
	registerTrajectoryOpBuiltins(TrajectoryOpRegistry::instance());
	g_builtinsRegistered = true;
}

} // namespace trajectory_algo
