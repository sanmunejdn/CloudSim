/// @file TrajectoryPathAdapters.cpp
/// @brief TrajectoryPathAdapters 实现

#include "TrajectoryPathAdapters.h"

namespace RobotInstruction
{
bool ingressUnifiedFromProgram(const RobotProgram& program, UnifiedTrajectory& out, std::string* errMsg)
{
	return unifiedTrajectoryFromProgram(program, out, errMsg);
}

bool ingressUnifiedForEdit(const RobotProgram& program, const std::string& pathPlanInstructionId,
						   UnifiedTrajectory& out, std::string* errMsg)
{
	if (!pathPlanInstructionId.empty())
	{
		return unifiedTrajectoryFromPathPlanOutput(program, pathPlanInstructionId, out, errMsg);
	}
	return unifiedTrajectoryFromProgram(program, out, errMsg);
}

bool ingressUnifiedFromRaw(const RawTrajectory& sourceRaw, const RawToUnifiedRebuildFn& rebuild, UnifiedTrajectory& out,
						   std::string* errMsg)
{
	if (!rebuild)
	{
		if (errMsg)
		{
			*errMsg = "raw rebuild callback missing";
		}
		return false;
	}
	return rebuild(sourceRaw, out, errMsg);
}

bool egressUnifiedToProgram(const UnifiedTrajectory& traj, RobotProgram& program, std::string* errMsg)
{
	return unifiedTrajectoryToProgram(traj, program, errMsg);
}

bool egressUnifiedToRaw(const UnifiedTrajectory& traj, RawTrajectory& raw, std::string* errMsg)
{
	return unifiedTrajectoryToRaw(traj, raw, errMsg);
}

bool egressUnifiedMergeIntoProgram(const UnifiedTrajectory& traj, RobotProgram& program,
								   const std::string& pathPlanInstructionId, std::string* errMsg,
								   std::string* outOutputGroupId)
{
	return unifiedTrajectoryMergeIntoProgram(traj, program, pathPlanInstructionId, errMsg, outOutputGroupId);
}

} // namespace RobotInstruction
