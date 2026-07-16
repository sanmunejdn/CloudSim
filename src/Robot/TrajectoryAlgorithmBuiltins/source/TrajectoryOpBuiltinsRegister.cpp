// 启动时注册全部内置原子块到 OpRegistry / ConfigRegistry
#include "TrajectoryOpRegistry.h"

#include "ApproachOp.h"
#include "ApproachOpConfig.h"
#include "AssignBlendOp.h"
#include "AssignBlendOpConfig.h"
#include "AssignSpeedZoneOp.h"
#include "AssignSpeedZoneOpConfig.h"
#include "DeleteOp.h"
#include "DeleteOpConfig.h"
#include "DuplicateOp.h"
#include "DuplicateOpConfig.h"
#include "ExternalAxisSearchOp.h"
#include "ExternalAxisSearchOpConfig.h"
#include "MirrorOp.h"
#include "MirrorOpConfig.h"
#include "OffsetAlongNormalOp.h"
#include "OffsetAlongNormalOpConfig.h"
#include "OffsetLateralOp.h"
#include "OffsetLateralOpConfig.h"
#include "NonRigidRegistrationOp.h"
#include "NonRigidRegistrationOpConfig.h"
#include "ProjectToGeometryOp.h"
#include "ProjectToGeometryOpConfig.h"
#include "ReachabilityFilterOp.h"
#include "ReachabilityFilterOpConfig.h"
#include "ReorderOp.h"
#include "ReorderOpConfig.h"
#include "ResampleOp.h"
#include "ResampleOpConfig.h"
#include "RetractOp.h"
#include "RetractOpConfig.h"
#include "RotateOp.h"
#include "RotateOpConfig.h"
#include "SmoothPoseOp.h"
#include "SmoothPoseOpConfig.h"
#include "TranslateOp.h"
#include "TranslateOpConfig.h"
#include "WeaveOp.h"
#include "WeaveOpConfig.h"

#include "TrajectoryOpConfigRegistry.h"

namespace trajectory_algo
{
namespace
{
bool g_builtinsRegistered = false;

void registerOpConfigs()
{
	auto& registry = TrajectoryOpConfigRegistry::instance();
	registry.registerOpConfig(makeTranslateOpConfig());
	registry.registerOpConfig(makeRotateOpConfig());
	registry.registerOpConfig(makeMirrorOpConfig());
	registry.registerOpConfig(makeDeleteOpConfig());
	registry.registerOpConfig(makeDuplicateOpConfig());
	registry.registerOpConfig(makeReorderOpConfig());
	registry.registerOpConfig(makeResampleOpConfig());
	registry.registerOpConfig(makeOffsetAlongNormalOpConfig());
	registry.registerOpConfig(makeOffsetLateralOpConfig());
	registry.registerOpConfig(makeSmoothPoseOpConfig());
	registry.registerOpConfig(makeAssignBlendOpConfig());
	registry.registerOpConfig(makeAssignSpeedZoneOpConfig());
	registry.registerOpConfig(makeWeaveOpConfig());
	registry.registerOpConfig(makeReachabilityFilterOpConfig());
	registry.registerOpConfig(makeExternalAxisSearchOpConfig());
	registry.registerOpConfig(makeApproachOpConfig());
	registry.registerOpConfig(makeRetractOpConfig());
	registry.registerOpConfig(makeProjectToGeometryOpConfig());
	registry.registerOpConfig(makeNonRigidRegistrationOpConfig());
}
} // namespace

void registerTrajectoryOpBuiltins(TrajectoryOpRegistry& registry)
{
	registry.registerOp(std::make_unique<TranslateOp>());
	registry.registerOp(std::make_unique<RotateOp>());
	registry.registerOp(std::make_unique<DeleteOp>());
	registry.registerOp(std::make_unique<DuplicateOp>());
	registry.registerOp(std::make_unique<MirrorOp>());
	registry.registerOp(std::make_unique<ReorderOp>());
	registry.registerOp(std::make_unique<ResampleOp>());
	registry.registerOp(std::make_unique<OffsetAlongNormalOp>());
	registry.registerOp(std::make_unique<OffsetLateralOp>());
	registry.registerOp(std::make_unique<SmoothPoseOp>());
	registry.registerOp(std::make_unique<AssignBlendOp>());
	registry.registerOp(std::make_unique<AssignSpeedZoneOp>());
	registry.registerOp(std::make_unique<WeaveOp>());
	registry.registerOp(std::make_unique<ReachabilityFilterOp>());
	registry.registerOp(std::make_unique<ExternalAxisSearchOp>());
	registry.registerOp(std::make_unique<ApproachOp>());
	registry.registerOp(std::make_unique<RetractOp>());
	registry.registerOp(std::make_unique<ProjectToGeometryOp>());
	registry.registerOp(std::make_unique<NonRigidRegistrationOp>());
}

void ensureTrajectoryOpBuiltinsRegistered()
{
	if (g_builtinsRegistered)
	{
		return;
	}
	registerTrajectoryOpBuiltins(TrajectoryOpRegistry::instance());
	registerOpConfigs();
	g_builtinsRegistered = true;
}

} // namespace trajectory_algo
