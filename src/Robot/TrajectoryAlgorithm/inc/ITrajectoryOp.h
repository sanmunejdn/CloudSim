#pragma once

#include "TrajectoryOpExecutionContext.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryPipelineTypes.h"
#include "trajectory_algorithm_global.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace RobotInstruction
{
struct UnifiedTrajectory;
}

namespace trajectory_algo
{

enum class TrajectoryOpCapability : uint32_t
{
	None = 0,
	/// 位姿类块：参与 collectPreviewWaypointIds 的 scope 路点收集
	PreviewPoseTransform = 1u << 0,
};

inline TrajectoryOpCapability operator|(TrajectoryOpCapability a, TrajectoryOpCapability b)
{
	return static_cast<TrajectoryOpCapability>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline bool hasCapability(TrajectoryOpCapability caps, TrajectoryOpCapability flag)
{
	return (static_cast<uint32_t>(caps) & static_cast<uint32_t>(flag)) != 0;
}

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

	/// Unified IR 路径处理；几何块在 Builtins 中 override，默认 false 表示未实现
	virtual bool processPath(
		const RobotInstruction::TrajectoryOpDescriptor& op,
		RobotInstruction::UnifiedTrajectory& traj,
		const TrajectoryOpExecutionContext& ctx,
		std::string* errMsg) const;
};

} // namespace trajectory_algo

