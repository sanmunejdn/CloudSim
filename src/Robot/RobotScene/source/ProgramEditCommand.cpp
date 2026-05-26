#include "ProgramEditCommand.h"

#include "RobotInstructionFactory.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"

#include <RigidTransform.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace RobotInstruction
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

size_t rootIndexOf(const std::vector<std::shared_ptr<Base>>& root, const std::string& id)
{
	for (size_t i = 0; i < root.size(); ++i)
	{
		if (root[i] && root[i]->id() == id)
		{
			return i;
		}
	}
	return root.size();
}

engine::RigidTransform deltaFromOp(const TrajectoryOpDescriptor& op)
{
	if (op.kind == TrajectoryOpKind::Translate)
	{
		return engine::RigidTransform::fromTranslationQuat(
			Eigen::Vector3d(op.translate.dxMm, op.translate.dyMm, op.translate.dzMm),
			Eigen::Quaterniond::Identity());
	}
	if (op.kind == TrajectoryOpKind::Rotate)
	{
		Eigen::Vector3d axis(op.rotate.axisX, op.rotate.axisY, op.rotate.axisZ);
		if (axis.norm() < 1e-9)
		{
			axis = Eigen::Vector3d::UnitZ();
		}
		axis.normalize();
		const double rad = op.rotate.angleDeg * kPi / 180.0;
		const Eigen::Quaterniond q(Eigen::AngleAxisd(rad, axis));
		return engine::RigidTransform::fromTranslationQuat(Eigen::Vector3d::Zero(), q);
	}
	return engine::RigidTransform::identity();
}

void applyDeltaToInstruction(Base& ins, const engine::RigidTransform& delta)
{
	engine::RigidTransform target = engine::RigidTransform::identity();
	if (!readTargetTransformFromInstruction(ins, target))
	{
		return;
	}
	const engine::RigidTransform updated = delta.composeColumn(target);
	writeTargetTransformToInstruction(ins, updated);
	ins.eraseExtensionProperty("context.currentJointRadCsv");
}
} // namespace

bool ProgramEditStack::execute(
	CommandPtr cmd,
	InstructionProgramDocument& doc,
	std::string* errMsg)
{
	if (!cmd)
	{
		return false;
	}
	if (!cmd->execute(doc, errMsg))
	{
		return false;
	}
	m_undoStack.push_back(cmd);
	m_redoStack.clear();
	return true;
}

bool ProgramEditStack::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	if (m_undoStack.empty())
	{
		return false;
	}
	CommandPtr cmd = m_undoStack.back();
	m_undoStack.pop_back();
	if (!cmd->undo(doc, errMsg))
	{
		return false;
	}
	m_redoStack.push_back(cmd);
	return true;
}

bool ProgramEditStack::redo(InstructionProgramDocument& doc, std::string* errMsg)
{
	if (m_redoStack.empty())
	{
		return false;
	}
	CommandPtr cmd = m_redoStack.back();
	m_redoStack.pop_back();
	if (!cmd->execute(doc, errMsg))
	{
		return false;
	}
	m_undoStack.push_back(cmd);
	return true;
}

void ProgramEditStack::clear()
{
	m_undoStack.clear();
	m_redoStack.clear();
}

InsertInstructionCommand::InsertInstructionCommand(std::shared_ptr<Base> instruction, const size_t rootIndex)
	: m_instruction(std::move(instruction))
	, m_rootIndex(rootIndex)
{
}

bool InsertInstructionCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	if (!m_instruction)
	{
		return false;
	}
	m_executed = doc.insertAtRoot(m_rootIndex, m_instruction);
	return m_executed;
}

bool InsertInstructionCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	if (!m_executed || !m_instruction)
	{
		return false;
	}
	return doc.removeById(m_instruction->id());
}

RemoveInstructionCommand::RemoveInstructionCommand(std::string instructionId)
	: m_instructionId(std::move(instructionId))
{
}

bool RemoveInstructionCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	Base* raw = doc.findById(m_instructionId);
	if (!raw)
	{
		return false;
	}
	m_removedSnapshot = cloneInstruction(*raw);
	if (!m_removedSnapshot)
	{
		return false;
	}
	if (doc.rootSteps())
	{
		m_rootIndex = rootIndexOf(*doc.rootSteps(), m_instructionId);
	}
	return doc.removeById(m_instructionId);
}

bool RemoveInstructionCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	if (!m_removedSnapshot)
	{
		return false;
	}
	return doc.insertAtRoot(m_rootIndex, m_removedSnapshot);
}

DuplicateInstructionCommand::DuplicateInstructionCommand(
	std::string sourceInstructionId,
	const size_t insertAfterRootIndex)
	: m_sourceId(std::move(sourceInstructionId))
	, m_insertIndex(insertAfterRootIndex + 1)
{
}

bool DuplicateInstructionCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	Base* raw = doc.findById(m_sourceId);
	if (!raw)
	{
		return false;
	}
	m_duplicate = cloneInstruction(*raw);
	if (!m_duplicate)
	{
		return false;
	}
	if (doc.rootSteps())
	{
		const size_t idx = rootIndexOf(*doc.rootSteps(), m_sourceId);
		if (idx < doc.rootSteps()->size())
		{
			m_insertIndex = idx + 1;
		}
	}
	return doc.insertAtRoot(m_insertIndex, m_duplicate);
}

bool DuplicateInstructionCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	if (!m_duplicate)
	{
		return false;
	}
	return doc.removeById(m_duplicate->id());
}

TransformMotionSegmentCommand::TransformMotionSegmentCommand(
	std::vector<std::string> targetIds,
	std::vector<TrajectoryOpDescriptor> transformOps)
	: m_targetIds(std::move(targetIds))
	, m_transformOps(std::move(transformOps))
{
}

bool TransformMotionSegmentCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	m_before.clear();
	engine::RigidTransform combined = engine::RigidTransform::identity();
	for (const TrajectoryOpDescriptor& op : m_transformOps)
	{
		if (op.kind == TrajectoryOpKind::Translate || op.kind == TrajectoryOpKind::Rotate)
		{
			combined = deltaFromOp(op).composeColumn(combined);
		}
	}
	for (const std::string& id : m_targetIds)
	{
		Base* raw = doc.findById(id);
		if (!raw || !isMotionWaypointType(raw->type()))
		{
			continue;
		}
		SnapshotEntry snap;
		snap.id = raw->id();
		snap.pose = raw->pose();
		snap.euler = raw->eulerDeg();
		snap.extensions = raw->extensionProperties();
		m_before.push_back(std::move(snap));
		applyDeltaToInstruction(*raw, combined);
	}
	return !m_before.empty();
}

bool TransformMotionSegmentCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	for (const SnapshotEntry& snap : m_before)
	{
		Base* raw = doc.findById(snap.id);
		if (raw)
		{
			raw->setPose(snap.pose);
			if (raw->hasEulerProperty())
			{
				raw->setEulerDeg(snap.euler);
			}
			for (const auto& kv : snap.extensions)
			{
				raw->setExtensionProperty(kv.first, kv.second);
			}
		}
	}
	return true;
}

CreateInstructionGroupCommand::CreateInstructionGroupCommand(
	RobotProgram* program,
	std::string groupName,
	std::vector<std::string> memberIds)
	: m_program(program)
	, m_groupName(std::move(groupName))
	, m_memberIds(std::move(memberIds))
{
}

bool CreateInstructionGroupCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	if (!m_program || m_groupName.empty() || m_memberIds.empty())
	{
		if (errMsg)
		{
			*errMsg = "invalid group parameters";
		}
		return false;
	}
	m_created.id = makeGroupId();
	m_created.name = m_groupName;
	m_created.memberInstructionIds = m_memberIds;
	m_program->groups.push_back(m_created);
	m_createdGroup = true;
	return true;
}

bool CreateInstructionGroupCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	if (!m_program || !m_createdGroup)
	{
		return false;
	}
	const auto it = std::remove_if(
		m_program->groups.begin(),
		m_program->groups.end(),
		[this](const InstructionGroup& g) { return g.id == m_created.id; });
	if (it == m_program->groups.end())
	{
		if (errMsg)
		{
			*errMsg = "group not found for undo";
		}
		return false;
	}
	m_program->groups.erase(it, m_program->groups.end());
	m_createdGroup = false;
	return true;
}

RemoveInstructionGroupCommand::RemoveInstructionGroupCommand(
	RobotProgram* program,
	std::string groupId)
	: m_program(program)
	, m_groupId(std::move(groupId))
{
}

bool RemoveInstructionGroupCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	if (!m_program)
	{
		return false;
	}
	const auto it = std::find_if(
		m_program->groups.begin(),
		m_program->groups.end(),
		[this](const InstructionGroup& g) { return g.id == m_groupId; });
	if (it == m_program->groups.end())
	{
		if (errMsg)
		{
			*errMsg = "group not found";
		}
		return false;
	}
	m_removedGroup = *it;
	m_program->groups.erase(it);
	m_removedFlag = true;
	return true;
}

bool RemoveInstructionGroupCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	(void)errMsg;
	if (!m_program || !m_removedFlag)
	{
		return false;
	}
	m_program->groups.push_back(m_removedGroup);
	m_removedFlag = false;
	return true;
}

RenameInstructionGroupCommand::RenameInstructionGroupCommand(
	RobotProgram* program,
	std::string groupId,
	std::string newName)
	: m_program(program)
	, m_groupId(std::move(groupId))
	, m_newName(std::move(newName))
{
}

bool RenameInstructionGroupCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	if (!m_program || m_newName.empty())
	{
		if (errMsg)
		{
			*errMsg = "invalid rename parameters";
		}
		return false;
	}
	InstructionGroup* group = nullptr;
	for (InstructionGroup& g : m_program->groups)
	{
		if (g.id == m_groupId)
		{
			group = &g;
			break;
		}
	}
	if (!group)
	{
		if (errMsg)
		{
			*errMsg = "group not found";
		}
		return false;
	}
	m_oldName = group->name;
	group->name = m_newName;
	return true;
}

bool RenameInstructionGroupCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	(void)errMsg;
	if (!m_program || m_oldName.empty())
	{
		return false;
	}
	for (InstructionGroup& g : m_program->groups)
	{
		if (g.id == m_groupId)
		{
			g.name = m_oldName;
			return true;
		}
	}
	return false;
}

} // namespace RobotInstruction
