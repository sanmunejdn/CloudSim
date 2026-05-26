#pragma once

#include "ProgramEditCommand.h"
#include "RobotInstructionModel.h"
#include "TrajectoryPipelineTypes.h"
#include "robot_scene_global.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace RobotInstruction
{

/// 预览链：按 id 叠加运动点位姿
class ROBOT_SCENE_API IMotionPoseQuery
{
public:
	virtual ~IMotionPoseQuery() = default;
	virtual bool queryMotionPose(const Base& ins, Vec3& poseMm, Vec3& eulerDeg) const = 0;
};

class ROBOT_SCENE_API DefaultMotionPoseQuery final : public IMotionPoseQuery
{
public:
	bool queryMotionPose(const Base& ins, Vec3& poseMm, Vec3& eulerDeg) const override;
};

class ROBOT_SCENE_API TransformMotionPoseQuery final : public IMotionPoseQuery
{
public:
	TransformMotionPoseQuery(
		std::unique_ptr<IMotionPoseQuery> inner,
		std::unordered_set<std::string> targetIds,
		TranslateParams translate,
		RotateParams rotate);

	bool queryMotionPose(const Base& ins, Vec3& poseMm, Vec3& eulerDeg) const override;

private:
	std::unique_ptr<IMotionPoseQuery> m_inner;
	std::unordered_set<std::string> m_targetIds;
	TranslateParams m_translate{};
	RotateParams m_rotate{};
};

ROBOT_SCENE_API std::unique_ptr<IMotionPoseQuery> buildPreviewPoseQueryChain(
	const std::vector<std::shared_ptr<Base>>& rootSteps,
	const RobotProgram* program,
	const std::vector<TrajectoryOpDescriptor>& ops);

class ROBOT_SCENE_API TrajectoryPipelineBuilder
{
public:
	void setProgramContext(const RobotProgram* program);
	void setOps(std::vector<TrajectoryOpDescriptor> ops);

	std::unique_ptr<IMotionPoseQuery> buildPreviewPoseQuery(
		const std::vector<std::shared_ptr<Base>>& rootSteps) const;

	std::vector<ProgramEditStack::CommandPtr> buildApplyCommands(
		InstructionProgramDocument& doc,
		std::string* errMsg) const;

private:
	const RobotProgram* m_program = nullptr;
	std::vector<TrajectoryOpDescriptor> m_ops;
};

} // namespace RobotInstruction
