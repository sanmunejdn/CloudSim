#include "TrajectoryOpRegistry.h"

#include "DeleteOp.h"
#include "DuplicateOp.h"
#include "MirrorOp.h"
#include "ReorderOp.h"
#include "RotateOp.h"
#include "TranslateOp.h"

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
