/// @file NonRigidRegistrationOp.cpp
/// @brief NonRigidRegistration 轨迹算子

// 非刚性配准轨迹纠正：绑定源几何 + SPARE/SDF 变形写回
#include "NonRigidRegistrationOp.h"

#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryOpParamsParse.h"

#include <utility>

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
	std::vector<TrajectoryOpParamField> fields;
	fields.push_back(messageParamField("nrr.sourceBackendId", "Source geometry backend (point cloud or mesh).",
									   "源几何 backend（点云或 mesh）。", 0));
	fields.push_back(messageParamField("nrr.targetBackendId", "Target geometry backend (point cloud or mesh).",
									   "目标几何 backend（点云或 mesh）。", 1));
	fields.push_back(doubleParamField("nrr.maxBindDistanceMm", "Max Bind Distance", "绑定最大距离", "mm", 0.1, 10000.0,
									  0.1, 30.0, 2, "nrr"));
	fields.push_back(enumParamField("nrr.solver", "Solver", "算法", {"0", "1"}, {"SPARE", "SDF/DDF"}, {"SPARE", "SDF/DDF"},
									0, 3, "nrr"));
	fields.push_back(intParamField("nrr.maxOuterIters", "Outer Iters", "外轮数", 1, 200, 30, 4, "nrr"));
	fields.push_back(boolParamField("nrr.rigidPreAlign", "Rigid Pre-Align", "刚性预对齐", false, 5, "nrr"));

	TrajectoryOpParamField coarse = boolParamField("nrr.coarseGlobalAlign", "Global Coarse Align (RANSAC)",
												   "全局粗对齐 (特征 RANSAC)", false, 6, "nrr");
	coarse.visibleWhenFieldKey = "nrr.solver";
	coarse.visibleWhenIntValue = 0;
	fields.push_back(std::move(coarse));

	fields.push_back(doubleParamField("nrr.voxelPrefilterMm", "Voxel Prefilter", "体素预滤波", "mm", 0.0, 1000.0, 0.1,
									  0.0, 7, "nrr"));
	fields.push_back(doubleParamField("nrr.sampleRadiusRatio", "Sample Radius Ratio", "采样半径比", "", 0.0, 100.0, 0.01,
									  0.0, 8, "nrr"));

	TrajectoryOpParamField fieldMode =
		enumParamField("nrr.sdfFieldMode", "Field Mode", "场模式", {"0", "1"}, {"DDF 有向距离", "有符号 SDF"},
					   {"DDF vector", "Signed SDF"}, 1, 9, "nrr");
	fieldMode.visibleWhenFieldKey = "nrr.solver";
	fieldMode.visibleWhenIntValue = 1;
	fields.push_back(std::move(fieldMode));

	TrajectoryOpParamField fieldVoxel =
		doubleParamField("nrr.sdfFieldVoxelMm", "Field Voxel", "场体素", "mm", 0.0, 50.0, 0.1, 0.0, 10, "nrr");
	fieldVoxel.visibleWhenFieldKey = "nrr.solver";
	fieldVoxel.visibleWhenIntValue = 1;
	fields.push_back(std::move(fieldVoxel));

	TrajectoryOpParamField fineTerm =
		enumParamField("nrr.sdfFineDataTerm", "Fine Data Term", "细阶段数据项", {"0", "1", "2"},
					   {"点-面", "DDF", "SDF"}, {"Point-to-plane", "DDF", "SDF"}, 0, 11, "nrr");
	fineTerm.visibleWhenFieldKey = "nrr.solver";
	fineTerm.visibleWhenIntValue = 1;
	fields.push_back(std::move(fineTerm));
	return fields;
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
	const char* solver = nrr.solver == RobotInstruction::NonRigidRegistrationSolver::Sdf ? "SDF" : "SPARE";
	return std::string("NonRigidRegistration ") + solver + " " + nrr.sourceBackendId + " -> " + nrr.targetBackendId;
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
