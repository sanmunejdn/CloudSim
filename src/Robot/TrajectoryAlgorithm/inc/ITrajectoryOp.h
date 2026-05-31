#pragma once

#include "TrajectoryApplyAction.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "RobotProgramCatalog.h"

namespace trajectory_algo
{

enum class TrajectoryOpCapability : uint32_t
{
	None = 0,
	PreviewPoseTransform = 1u << 0,
	ApplyPoseTransform = 1u << 1,
	ApplyStructuralEdit = 1u << 2
};

inline TrajectoryOpCapability operator|(TrajectoryOpCapability a, TrajectoryOpCapability b)
{
	return static_cast<TrajectoryOpCapability>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline bool hasCapability(TrajectoryOpCapability caps, TrajectoryOpCapability flag)
{
	return (static_cast<uint32_t>(caps) & static_cast<uint32_t>(flag)) != 0;
}

struct TrajectoryOpContext
{
	const RobotInstruction::RobotProgram* program = nullptr;
};

struct TRAJECTORY_ALGORITHM_API PreviewTransformStep
{
	enum class Kind
	{
		TranslateOnly = 0,
		RotateOnly,
		TranslateAndRotate,
		AxisReverse,
		FixedOrientationToFirst
	};

	Kind kind = Kind::TranslateOnly;
	std::unordered_set<std::string> targetIds;
	RobotInstruction::TranslateParams translate{};
	RobotInstruction::RotateParams rotate{};
	int mirrorAxis = 0;
	std::string referenceId;
};

class TRAJECTORY_ALGORITHM_API ITrajectoryOp
{
public:
	virtual ~ITrajectoryOp() = default;

	virtual RobotInstruction::TrajectoryOpKind kind() const = 0;
	virtual const char* displayName(bool chinese) const = 0;
	virtual TrajectoryOpCapability capabilities() const = 0;

	virtual RobotInstruction::TrajectoryOpDescriptor makeDefaultDescriptor(
		const RobotInstruction::OpScope& defaultScope) const = 0;

	virtual std::vector<TrajectoryOpParamField> paramFields() const = 0;
	virtual bool validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const = 0;
	virtual std::string formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, bool chinese) const = 0;

	virtual bool contributePreviewTransform(
		const RobotInstruction::TrajectoryOpDescriptor& op,
		const std::vector<std::string>& targetIds,
		PreviewTransformStep& out) const = 0;

	virtual std::vector<TrajectoryApplyAction> buildApplyActions(
		const TrajectoryOpContext& ctx,
		const RobotInstruction::TrajectoryOpDescriptor& op) const = 0;
};

} // namespace trajectory_algo
