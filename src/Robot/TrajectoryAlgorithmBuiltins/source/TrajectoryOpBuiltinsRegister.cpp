/// @file TrajectoryOpBuiltinsRegister.cpp
/// @brief 内置算子注册

// 启动时注册全部内置原子块到 OpRegistry / ConfigRegistry
#include "ApproachOp.h"
#include "ApproachOpConfig.h"
#include "AssignBlendOp.h"
#include "AssignBlendOpConfig.h"
#include "AssignSpeedZoneOp.h"
#include "AssignSpeedZoneOpConfig.h"
#include "ClampToPlaneOp.h"
#include "ClampToPlaneOpConfig.h"
#include "DeleteOp.h"
#include "DeleteOpConfig.h"
#include "DuplicateOp.h"
#include "DuplicateOpConfig.h"
#include "ExternalAxisSearchOp.h"
#include "ExternalAxisSearchOpConfig.h"
#include "InsertMidOp.h"
#include "InsertMidOpConfig.h"
#include "MirrorOp.h"
#include "MirrorOpConfig.h"
#include "NonRigidRegistrationOp.h"
#include "NonRigidRegistrationOpConfig.h"
#include "OffsetLateralOp.h"
#include "OffsetLateralOpConfig.h"
#include "OrientToPathOp.h"
#include "OrientToPathOpConfig.h"
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
#include "ReversePathOp.h"
#include "ReversePathOpConfig.h"
#include "RotateOp.h"
#include "RotateOpConfig.h"
#include "ScalePathOp.h"
#include "ScalePathOpConfig.h"
#include "ShiftStartOp.h"
#include "ShiftStartOpConfig.h"
#include "SmoothPoseOp.h"
#include "SmoothPoseOpConfig.h"
#include "SpeedRampOp.h"
#include "SpeedRampOpConfig.h"
#include "SpikeFilterOp.h"
#include "SpikeFilterOpConfig.h"
#include "ToWorkpieceInHandOp.h"
#include "ToWorkpieceInHandOpConfig.h"
#include "TrajectoryOpConfigRegistry.h"
#include "TrajectoryOpRegistry.h"
#include "TranslateOp.h"
#include "TranslateOpConfig.h"
#include "TrimPathOp.h"
#include "TrimPathOpConfig.h"
#include "WeaveOp.h"
#include "WeaveOpConfig.h"

namespace trajectory_algo
{
namespace
{
bool g_builtinsRegistered = false;

void registerOpConfigs(TrajectoryOpConfigRegistry& registry)
{
	registry.registerOpConfig(makeTranslateOpConfig());
	registry.registerOpConfig(makeRotateOpConfig());
	registry.registerOpConfig(makeMirrorOpConfig());
	registry.registerOpConfig(makeDeleteOpConfig());
	registry.registerOpConfig(makeDuplicateOpConfig());
	registry.registerOpConfig(makeReorderOpConfig());
	registry.registerOpConfig(makeResampleOpConfig());
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
	registry.registerOpConfig(makeToWorkpieceInHandOpConfig());
	registry.registerOpConfig(makeReversePathOpConfig());
	registry.registerOpConfig(makeOrientToPathOpConfig());
	registry.registerOpConfig(makeTrimPathOpConfig());
	registry.registerOpConfig(makeShiftStartOpConfig());
	registry.registerOpConfig(makeInsertMidOpConfig());
	registry.registerOpConfig(makeClampToPlaneOpConfig());
	registry.registerOpConfig(makeScalePathOpConfig());
	registry.registerOpConfig(makeSpeedRampOpConfig());
	registry.registerOpConfig(makeSpikeFilterOpConfig());
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
	registry.registerOp(std::make_unique<ToWorkpieceInHandOp>());
	registry.registerOp(std::make_unique<ReversePathOp>());
	registry.registerOp(std::make_unique<OrientToPathOp>());
	registry.registerOp(std::make_unique<TrimPathOp>());
	registry.registerOp(std::make_unique<ShiftStartOp>());
	registry.registerOp(std::make_unique<InsertMidOp>());
	registry.registerOp(std::make_unique<ClampToPlaneOp>());
	registry.registerOp(std::make_unique<ScalePathOp>());
	registry.registerOp(std::make_unique<SpeedRampOp>());
	registry.registerOp(std::make_unique<SpikeFilterOp>());
}

void ensureTrajectoryOpBuiltinsRegistered()
{
	if (g_builtinsRegistered)
	{
		return;
	}
	TrajectoryOpRegistry& registry = TrajectoryOpRegistry::instance();
	registerTrajectoryOpBuiltins(registry);
	registerOpConfigs(TrajectoryOpConfigRegistry::instance());
	g_builtinsRegistered = true;
}

} // namespace trajectory_algo
