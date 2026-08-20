#ifndef ROBOTSCENE_PROGRAMEDITCOMMAND_H
#define ROBOTSCENE_PROGRAMEDITCOMMAND_H

/// @file ProgramEditCommand.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief ProgramEditCommand 接口

#include "robot_scene_global.h"

#include "InstructionProgramDocument.h"
#include "RobotProgramCatalog.h"
#include "TrajectoryPipelineTypes.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace RobotInstruction
{
class ROBOT_SCENE_API ProgramEditCommand
{
public:
	virtual ~ProgramEditCommand() = default;

	virtual bool execute(InstructionProgramDocument& doc, std::string* errMsg) = 0;
	virtual bool undo(InstructionProgramDocument& doc, std::string* errMsg) = 0;
	virtual const char* commandName() const = 0;
};

class ROBOT_SCENE_API ProgramEditStack
{
public:
	using CommandPtr = std::shared_ptr<ProgramEditCommand>;

	bool execute(CommandPtr cmd, InstructionProgramDocument& doc, std::string* errMsg);
	bool undo(InstructionProgramDocument& doc, std::string* errMsg);
	bool redo(InstructionProgramDocument& doc, std::string* errMsg);
	bool canUndo() const { return !m_undoStack.empty(); }
	bool canRedo() const { return !m_redoStack.empty(); }
	void clear();

private:
	std::vector<CommandPtr> m_undoStack;
	std::vector<CommandPtr> m_redoStack;
};

class ROBOT_SCENE_API InsertInstructionCommand final : public ProgramEditCommand
{
public:
	InsertInstructionCommand(std::shared_ptr<Base> instruction, size_t rootIndex);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "InsertInstruction"; }

private:
	std::shared_ptr<Base> m_instruction;
	size_t m_rootIndex = 0;
	bool m_executed = false;
};

class ROBOT_SCENE_API RemoveInstructionCommand final : public ProgramEditCommand
{
public:
	explicit RemoveInstructionCommand(std::string instructionId);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "RemoveInstruction"; }

private:
	std::string m_instructionId;
	std::shared_ptr<Base> m_removedSnapshot;
	size_t m_rootIndex = 0;
};

class ROBOT_SCENE_API DuplicateInstructionCommand final : public ProgramEditCommand
{
public:
	DuplicateInstructionCommand(std::string sourceInstructionId, size_t insertAfterRootIndex);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "DuplicateInstruction"; }

private:
	std::string m_sourceId;
	size_t m_insertIndex = 0;
	std::shared_ptr<Base> m_duplicate;
};

class ROBOT_SCENE_API TransformMotionSegmentCommand final : public ProgramEditCommand
{
public:
	TransformMotionSegmentCommand(std::vector<std::string> targetIds, std::vector<TrajectoryOpDescriptor> transformOps);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "TransformMotionSegment"; }

private:
	struct SnapshotEntry
	{
		std::string id;
		Vec3 pose{};
		Vec3 euler{};
		std::unordered_map<std::string, std::string> extensions;
	};

	std::vector<std::string> m_targetIds;
	std::vector<TrajectoryOpDescriptor> m_transformOps;
	std::vector<SnapshotEntry> m_before;
};

class ROBOT_SCENE_API CompositeProgramEditCommand final : public ProgramEditCommand
{
public:
	explicit CompositeProgramEditCommand(std::vector<ProgramEditStack::CommandPtr> commands);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "CompositeProgramEdit"; }

private:
	std::vector<ProgramEditStack::CommandPtr> m_commands;
	size_t m_executedCount = 0;
};

class ROBOT_SCENE_API ReplaceProgramContentCommand final : public ProgramEditCommand
{
public:
	ReplaceProgramContentCommand(RobotProgram* program, RobotProgram replacement);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "ReplaceProgramContent"; }

private:
	RobotProgram* m_program = nullptr;
	RobotProgram m_before{};
	RobotProgram m_after{};
	bool m_executed = false;
};

class ROBOT_SCENE_API CreateInstructionGroupCommand final : public ProgramEditCommand
{
public:
	CreateInstructionGroupCommand(RobotProgram* program, std::string groupName, std::vector<std::string> memberIds);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "CreateInstructionGroup"; }

private:
	RobotProgram* m_program = nullptr;
	std::string m_groupName;
	std::vector<std::string> m_memberIds;
	InstructionGroup m_created{};
	bool m_createdGroup = false;
};

class ROBOT_SCENE_API RemoveInstructionGroupCommand final : public ProgramEditCommand
{
public:
	RemoveInstructionGroupCommand(RobotProgram* program, std::string groupId);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "RemoveInstructionGroup"; }

private:
	RobotProgram* m_program = nullptr;
	std::string m_groupId;
	InstructionGroup m_removedGroup{};
	bool m_removedFlag = false;
};

class ROBOT_SCENE_API RenameInstructionGroupCommand final : public ProgramEditCommand
{
public:
	RenameInstructionGroupCommand(RobotProgram* program, std::string groupId, std::string newName);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "RenameInstructionGroup"; }

private:
	RobotProgram* m_program = nullptr;
	std::string m_groupId;
	std::string m_newName;
	std::string m_oldName;
};

class ROBOT_SCENE_API InsertPathPlanCommand final : public ProgramEditCommand
{
public:
	InsertPathPlanCommand(std::shared_ptr<PathPlanInstruction> pathPlan, size_t rootIndex);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "InsertPathPlan"; }

private:
	std::shared_ptr<PathPlanInstruction> m_pathPlan;
	size_t m_rootIndex = 0;
	bool m_executed = false;
};

class ROBOT_SCENE_API UpdatePathPlanPipelineCommand final : public ProgramEditCommand
{
public:
	UpdatePathPlanPipelineCommand(std::string pathPlanId, std::vector<TrajectoryOpDescriptor> pipeline,
								  std::vector<TrajectoryOpDescriptor> appliedHistory);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "UpdatePathPlanPipeline"; }

private:
	std::string m_pathPlanId;
	std::vector<TrajectoryOpDescriptor> m_pipeline;
	std::vector<TrajectoryOpDescriptor> m_appliedHistory;
	std::vector<TrajectoryOpDescriptor> m_pipelineBefore;
	std::vector<TrajectoryOpDescriptor> m_appliedBefore;
};

class ROBOT_SCENE_API UpdatePathPlanRawCommand final : public ProgramEditCommand
{
public:
	UpdatePathPlanRawCommand(RobotProgramCatalog* catalog, std::string pathPlanId, RawTrajectory raw,
							 PathPlanPhase newPhase);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "UpdatePathPlanRaw"; }

private:
	RobotProgramCatalog* m_catalog = nullptr;
	std::string m_pathPlanId;
	RawTrajectory m_raw;
	PathPlanPhase m_newPhase = PathPlanPhase::RawReady;
	RawTrajectory m_rawBefore{};
	PathPlanPhase m_phaseBefore = PathPlanPhase::Draft;
	bool m_hadRawBefore = false;
};

class ROBOT_SCENE_API UpdatePathPlanApplyStateCommand final : public ProgramEditCommand
{
public:
	UpdatePathPlanApplyStateCommand(std::string pathPlanId, PathPlanPhase phase, std::string outputGroupId);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "UpdatePathPlanApplyState"; }

private:
	std::string m_pathPlanId;
	PathPlanPhase m_phase = PathPlanPhase::Applied;
	std::string m_outputGroupId;
	PathPlanPhase m_phaseBefore = PathPlanPhase::Draft;
	std::string m_outputGroupIdBefore;
};

class ROBOT_SCENE_API RemovePathPlanCommand final : public ProgramEditCommand
{
public:
	RemovePathPlanCommand(RobotProgramCatalog* catalog, std::string programId, std::string pathPlanId);

	bool execute(InstructionProgramDocument& doc, std::string* errMsg) override;
	bool undo(InstructionProgramDocument& doc, std::string* errMsg) override;
	const char* commandName() const override { return "RemovePathPlan"; }

private:
	RobotProgramCatalog* m_catalog = nullptr;
	std::string m_programId;
	std::string m_pathPlanId;
	std::shared_ptr<PathPlanInstruction> m_snapshot;
	size_t m_rootIndex = 0;
	std::vector<InstructionGroup> m_removedOutputGroups;
	RawTrajectory m_removedRaw{};
	bool m_hadRaw = false;
	bool m_executed = false;
};

} // namespace RobotInstruction

#endif // ROBOTSCENE_PROGRAMEDITCOMMAND_H
