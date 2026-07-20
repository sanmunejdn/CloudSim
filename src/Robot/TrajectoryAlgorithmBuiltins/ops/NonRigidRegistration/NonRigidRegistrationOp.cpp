/// @file NonRigidRegistrationOp.cpp
/// @brief NonRigidRegistrationOp 实现

// 非刚性配准轨迹纠正：绑定源几何 + SPARE 变形写回
#include "NonRigidRegistrationOp.h"

#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryOpParamsParse.h"

namespace trajectory_algo
{
namespace
{
TrajectoryOpParamField boolParamField(const std::string& key, const std::string& labelEn, const std::string& labelZh,
									  const bool defaultValue, const int order, const std::string& group)
{
	TrajectoryOpParamField field{};
	field.key = key;
	field.type = TrajectoryParamType::Bool;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.defaultBool = defaultValue;
	field.order = order;
	field.group = group;
	return field;
}

} // namespace

RobotInstruction::TrajectoryOpKind NonRigidRegistrationOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::NonRigidRegistration;
}

const char* NonRigidRegistrationOp::displayName(const bool chinese) const
{
	return chinese ? "非刚性配准纠正" : "NonRigidRegistration";
}

TrajectoryOpCapability NonRigidRegistrationOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
NonRigidRegistrationOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::NonRigidRegistration;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	return op;
}

std::vector<TrajectoryOpParamField> NonRigidRegistrationOp::paramFields() const
{
	return {
		messageParamField("nrr.sourceBackendId", "Source geometry backend (point cloud or mesh).",
						  "源几何 backend（点云或 mesh）。", 0),
		messageParamField("nrr.targetBackendId", "Target geometry backend (point cloud or mesh).",
						  "目标几何 backend（点云或 mesh）。", 1),
		doubleParamField("nrr.maxBindDistanceMm", "Max Bind Distance", "绑定最大距离", "mm", 0.1, 10000.0, 0.1, 30.0, 2,
						 "nrr"),
		intParamField("nrr.maxOuterIters", "SPARE Outer Iters", "SPARE 外轮数", 1, 200, 30, 3, "nrr"),
		boolParamField("nrr.rigidPreAlign", "Rigid Pre-Align", "刚性预对齐", false, 4, "nrr"),
		doubleParamField("nrr.voxelPrefilterMm", "Voxel Prefilter", "体素预滤波", "mm", 0.0, 1000.0, 0.1, 0.0, 5,
						 "nrr"),
		doubleParamField("nrr.sampleRadiusRatio", "Sample Radius Ratio", "采样半径比", "", 0.0, 100.0, 0.01, 0.0, 6,
						 "nrr"),
	};
}

bool NonRigidRegistrationOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	const RobotInstruction::NonRigidRegistrationParams nrr = parseNonRigidRegistrationParams(op.params);
	if (nrr.sourceBackendId.empty())
	{
		if (errMsg)
		{
			*errMsg = "non-rigid source backend is required";
		}
		return false;
	}
	if (nrr.targetBackendId.empty())
	{
		if (errMsg)
		{
			*errMsg = "non-rigid target backend is required";
		}
		return false;
	}
	if (nrr.maxBindDistanceMm <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "max bind distance must be > 0";
		}
		return false;
	}
	return true;
}

std::string NonRigidRegistrationOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op,
												  const bool chinese) const
{
	(void)chinese;
	const RobotInstruction::NonRigidRegistrationParams nrr = parseNonRigidRegistrationParams(op.params);
	return std::string("NonRigidRegistration ") + nrr.sourceBackendId + " -> " + nrr.targetBackendId;
}

bool NonRigidRegistrationOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
										 RobotInstruction::UnifiedTrajectory& traj,
										 const TrajectoryOpExecutionContext& ctx, std::string* errMsg) const
{
	if (!ctx.nonRigidTrajectoryWarp)
	{
		if (errMsg)
		{
			*errMsg = "non-rigid trajectory warp not available";
		}
		return false;
	}
	std::size_t missCount = 0;
	return ctx.nonRigidTrajectoryWarp->warp(traj, parseNonRigidRegistrationParams(op.params), op.scope, ctx.program,
											&missCount, errMsg);
}

} // namespace trajectory_algo
