#pragma once

#include "InstructionProgramDocument.h"
#include "RobotProgramCatalog.h"
#include "TrajectoryPipelineTypes.h"
#include "robot_scene_global.h"

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
	TransformMotionSegmentCommand(
		std::vector<std::string> targetIds,
		std::vector<TrajectoryOpDescriptor> transformOps);

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

class ROBOT_SCENE_API CreateInstructionGroupCommand final : public ProgramEditCommand
{
public:
	CreateInstructionGroupCommand(
		RobotProgram* program,
		std::string groupName,
		std::vector<std::string> memberIds);

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

} // namespace RobotInstruction
