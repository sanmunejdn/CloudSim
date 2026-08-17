/// @file ProgramEditCommand.cpp
/// @brief 程序编辑命令

#include "ProgramEditCommand.h"

#include "RobotInstructionFactory.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include <RigidTransform.h>
#include <TrajectoryOpParamsParse.h>
#include <TrajectoryTransformMath.h>

namespace RobotInstruction
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

using IdMap = std::unordered_map<std::string, std::shared_ptr<Base>>;

IdMap collectIdMap(InstructionProgramDocument& doc)
{
	IdMap idMap;
	doc.collectIdMap(idMap);
	return idMap;
}

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
		return trajectory_algo::rigidDeltaFromTranslate(trajectory_algo::parseTranslateParams(op.params));
	}
	if (op.kind == TrajectoryOpKind::Rotate)
	{
		return trajectory_algo::rigidDeltaFromRotate(trajectory_algo::parseRotateParams(op.params));
	}
	return engine::RigidTransform::identity();
}

TransformReferenceFrame frameForOp(const TrajectoryOpDescriptor& op)
{
	if (op.kind == TrajectoryOpKind::Translate)
	{
		return trajectory_algo::parseTranslateParams(op.params).frame;
	}
	if (op.kind == TrajectoryOpKind::Rotate)
	{
		return trajectory_algo::parseRotateParams(op.params).frame;
	}
	return TransformReferenceFrame::World;
}

RobotProgram cloneProgramSnapshot(const RobotProgram& program)
{
	RobotProgram snapshot;
	snapshot.id = program.id;
	snapshot.name = program.name;
	snapshot.isMain = program.isMain;
	snapshot.groups = program.groups;
	snapshot.steps.reserve(program.steps.size());
	for (const std::shared_ptr<Base>& step : program.steps)
	{
		if (!step)
		{
			continue;
		}
		std::shared_ptr<Base> cloned = cloneInstructionPreservingId(*step);
		if (cloned)
		{
			snapshot.steps.push_back(cloned);
		}
	}
	return snapshot;
}

void overwriteProgram(RobotProgram& target, const RobotProgram& source)
{
	target.id = source.id;
	target.name = source.name;
	target.isMain = source.isMain;
	target.groups = source.groups;
	target.steps.clear();
	target.steps.reserve(source.steps.size());
	for (const std::shared_ptr<Base>& step : source.steps)
	{
		if (!step)
		{
			continue;
		}
		std::shared_ptr<Base> cloned = cloneInstructionPreservingId(*step);
		if (cloned)
		{
			target.steps.push_back(cloned);
		}
	}
}

void applyDeltaToInstruction(Base& ins, const engine::RigidTransform& delta, const TransformReferenceFrame frame)
{
	engine::RigidTransform target = engine::RigidTransform::identity();
	if (!readTargetTransformFromInstruction(ins, target))
	{
		return;
	}
	const engine::RigidTransform updated = trajectory_algo::applyTransformDelta(target, delta, frame);
	writeTargetTransformToInstruction(ins, updated);
	ins.eraseExtensionProperty("context.currentJointRadCsv");
	// 位姿已变，旧渲染缓存会导致 OSG 仍按 render.tcpWorldMat4 显示错误位置
	ins.eraseExtensionProperty("render.tcpWorldMat4");
	ins.eraseExtensionProperty("render.tcpLocalMat4");
}

TrajectoryOpDescriptor interpolatedDescriptor(const TrajectoryOpDescriptor& op, const double t)
{
	TrajectoryOpDescriptor out = op;
	trajectory_algo::interpolateTransformParamsInPlace(out, t);
	return out;
}

void applyAxisReverseToInstruction(Base& ins, const int mirrorAxis)
{
	engine::RigidTransform target = engine::RigidTransform::identity();
	if (!readTargetTransformFromInstruction(ins, target))
	{
		return;
	}
	Eigen::Matrix3d rot = target.rotation().toRotationMatrix();
	Eigen::Vector3d axes[3] = {rot.col(0), rot.col(1), rot.col(2)};
	const int reversedAxis = std::max(0, std::min(2, mirrorAxis));
	const int keptAxis = (reversedAxis + 1) % 3;
	const int rebuiltAxis = 3 - reversedAxis - keptAxis;
	axes[reversedAxis] = -axes[reversedAxis];
	axes[rebuiltAxis] = axes[reversedAxis].cross(axes[keptAxis]);
	if (axes[rebuiltAxis].norm() < 1e-9)
	{
		return;
	}
	axes[rebuiltAxis].normalize();
	axes[keptAxis] = axes[rebuiltAxis].cross(axes[reversedAxis]);
	if (axes[keptAxis].norm() < 1e-9)
	{
		return;
	}
	axes[reversedAxis].normalize();
	axes[keptAxis].normalize();
	Eigen::Matrix3d updatedRot;
	updatedRot.col(0) = axes[0];
	updatedRot.col(1) = axes[1];
	updatedRot.col(2) = axes[2];
	writeTargetTransformToInstruction(
		ins, engine::RigidTransform::fromTranslationQuat(target.translationMm(), Eigen::Quaterniond(updatedRot)));
	ins.eraseExtensionProperty("context.currentJointRadCsv");
	ins.eraseExtensionProperty("render.tcpWorldMat4");
	ins.eraseExtensionProperty("render.tcpLocalMat4");
}

void applyFixedOrientationToInstruction(Base& ins, const engine::RigidTransform& refTarget)
{
	engine::RigidTransform target = engine::RigidTransform::identity();
	if (!readTargetTransformFromInstruction(ins, target))
	{
		return;
	}
	writeTargetTransformToInstruction(
		ins, engine::RigidTransform::fromTranslationQuat(target.translationMm(), refTarget.rotation()));
	ins.eraseExtensionProperty("context.currentJointRadCsv");
	ins.eraseExtensionProperty("render.tcpWorldMat4");
	ins.eraseExtensionProperty("render.tcpLocalMat4");
}
} // namespace

bool ProgramEditStack::execute(CommandPtr cmd, InstructionProgramDocument& doc, std::string* errMsg)
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
	: m_instruction(std::move(instruction)), m_rootIndex(rootIndex)
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
	const IdMap idMap = collectIdMap(doc);
	const auto it = idMap.find(m_instructionId);
	if (it == idMap.end() || !it->second)
	{
		return false;
	}
	Base* raw = it->second.get();
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

DuplicateInstructionCommand::DuplicateInstructionCommand(std::string sourceInstructionId,
														 const size_t insertAfterRootIndex)
	: m_sourceId(std::move(sourceInstructionId)), m_insertIndex(insertAfterRootIndex + 1)
{
}

bool DuplicateInstructionCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	const IdMap idMap = collectIdMap(doc);
	const auto it = idMap.find(m_sourceId);
	if (it == idMap.end() || !it->second)
	{
		return false;
	}
	Base* raw = it->second.get();
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

TransformMotionSegmentCommand::TransformMotionSegmentCommand(std::vector<std::string> targetIds,
															 std::vector<TrajectoryOpDescriptor> transformOps)
	: m_targetIds(std::move(targetIds)), m_transformOps(std::move(transformOps))
{
}

bool TransformMotionSegmentCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	m_before.clear();
	const IdMap idMap = collectIdMap(doc);
	std::vector<std::string> validTargetIds;
	validTargetIds.reserve(m_targetIds.size());
	for (const std::string& id : m_targetIds)
	{
		const auto it = idMap.find(id);
		if (it != idMap.end() && it->second && isMotionWaypointType(it->second->type()))
		{
			validTargetIds.push_back(id);
		}
	}
	std::unordered_map<size_t, engine::RigidTransform> reorderRefByOpIndex;
	for (size_t opIdx = 0; opIdx < m_transformOps.size(); ++opIdx)
	{
		const TrajectoryOpDescriptor& op = m_transformOps[opIdx];
		if (op.kind != TrajectoryOpKind::Reorder || validTargetIds.empty())
		{
			continue;
		}
		const auto refIt = idMap.find(validTargetIds.front());
		if (refIt == idMap.end() || !refIt->second)
		{
			continue;
		}
		engine::RigidTransform refTarget = engine::RigidTransform::identity();
		if (readTargetTransformFromInstruction(*refIt->second, refTarget))
		{
			reorderRefByOpIndex.emplace(opIdx, refTarget);
		}
	}
	for (size_t targetIdx = 0; targetIdx < validTargetIds.size(); ++targetIdx)
	{
		const std::string& id = validTargetIds[targetIdx];
		const auto it = idMap.find(id);
		if (it == idMap.end() || !it->second || !isMotionWaypointType(it->second->type()))
		{
			continue;
		}
		Base* raw = it->second.get();
		SnapshotEntry snap;
		snap.id = raw->id();
		snap.pose = raw->pose();
		snap.euler = raw->eulerDeg();
		snap.extensions = raw->extensionProperties();
		m_before.push_back(std::move(snap));
		for (size_t opIdx = 0; opIdx < m_transformOps.size(); ++opIdx)
		{
			TrajectoryOpDescriptor op = m_transformOps[opIdx];
			if (op.kind != TrajectoryOpKind::Translate && op.kind != TrajectoryOpKind::Rotate)
			{
				if (op.kind == TrajectoryOpKind::Mirror)
				{
					applyAxisReverseToInstruction(*raw, trajectory_algo::parseMirrorAxis(op.params));
				}
				else if (op.kind == TrajectoryOpKind::Reorder)
				{
					const auto refIt = reorderRefByOpIndex.find(opIdx);
					if (refIt != reorderRefByOpIndex.end())
					{
						applyFixedOrientationToInstruction(*raw, refIt->second);
					}
				}
				continue;
			}
			const double t = validTargetIds.size() <= 1
								 ? 0.0
								 : static_cast<double>(targetIdx) / static_cast<double>(validTargetIds.size() - 1);
			op = interpolatedDescriptor(op, t);
			applyDeltaToInstruction(*raw, deltaFromOp(op), frameForOp(op));
		}
	}
	return !m_before.empty();
}

bool TransformMotionSegmentCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	const IdMap idMap = collectIdMap(doc);
	for (const SnapshotEntry& snap : m_before)
	{
		const auto it = idMap.find(snap.id);
		if (it != idMap.end() && it->second)
		{
			Base* raw = it->second.get();
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

CompositeProgramEditCommand::CompositeProgramEditCommand(std::vector<ProgramEditStack::CommandPtr> commands)
	: m_commands(std::move(commands))
{
}

bool CompositeProgramEditCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	m_executedCount = 0;
	for (const auto& cmd : m_commands)
	{
		if (!cmd || !cmd->execute(doc, errMsg))
		{
			for (size_t i = m_executedCount; i > 0; --i)
			{
				std::string rollbackErr;
				m_commands[i - 1]->undo(doc, &rollbackErr);
			}
			m_executedCount = 0;
			return false;
		}
		++m_executedCount;
	}
	return m_executedCount > 0;
}

bool CompositeProgramEditCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	if (m_executedCount == 0)
	{
		return false;
	}
	for (size_t i = m_executedCount; i > 0; --i)
	{
		std::string undoErr;
		if (!m_commands[i - 1] || !m_commands[i - 1]->undo(doc, &undoErr))
		{
			return false;
		}
	}
	m_executedCount = 0;
	return true;
}

ReplaceProgramContentCommand::ReplaceProgramContentCommand(RobotProgram* program, RobotProgram replacement)
	: m_program(program), m_after(std::move(replacement))
{
}

bool ReplaceProgramContentCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	if (!m_program)
	{
		if (errMsg)
		{
			*errMsg = "program is null";
		}
		return false;
	}
	if (!m_executed)
	{
		m_before = cloneProgramSnapshot(*m_program);
	}
	overwriteProgram(*m_program, m_after);
	m_executed = true;
	return true;
}

bool ReplaceProgramContentCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	(void)errMsg;
	if (!m_program || !m_executed)
	{
		return false;
	}
	overwriteProgram(*m_program, m_before);
	m_executed = false;
	return true;
}

CreateInstructionGroupCommand::CreateInstructionGroupCommand(RobotProgram* program, std::string groupName,
															 std::vector<std::string> memberIds)
	: m_program(program), m_groupName(std::move(groupName)), m_memberIds(std::move(memberIds))
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
	const auto it = std::remove_if(m_program->groups.begin(), m_program->groups.end(),
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

RemoveInstructionGroupCommand::RemoveInstructionGroupCommand(RobotProgram* program, std::string groupId)
	: m_program(program), m_groupId(std::move(groupId))
{
}

bool RemoveInstructionGroupCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	if (!m_program)
	{
		return false;
	}
	const auto it = std::find_if(m_program->groups.begin(), m_program->groups.end(),
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

RenameInstructionGroupCommand::RenameInstructionGroupCommand(RobotProgram* program, std::string groupId,
															 std::string newName)
	: m_program(program), m_groupId(std::move(groupId)), m_newName(std::move(newName))
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

InsertPathPlanCommand::InsertPathPlanCommand(std::shared_ptr<PathPlanInstruction> pathPlan, const size_t rootIndex)
	: m_pathPlan(std::move(pathPlan)), m_rootIndex(rootIndex)
{
}

bool InsertPathPlanCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	if (!m_pathPlan)
	{
		if (errMsg)
		{
			*errMsg = "path plan is null";
		}
		return false;
	}
	if (!doc.insertAtRoot(m_rootIndex, m_pathPlan))
	{
		if (errMsg)
		{
			*errMsg = "insert path plan failed";
		}
		return false;
	}
	m_executed = true;
	doc.renumberAndNotify();
	return true;
}

bool InsertPathPlanCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	if (!m_executed || !m_pathPlan)
	{
		return false;
	}
	doc.removeById(m_pathPlan->id());
	doc.renumberAndNotify();
	m_executed = false;
	return true;
}

UpdatePathPlanPipelineCommand::UpdatePathPlanPipelineCommand(std::string pathPlanId,
															 std::vector<TrajectoryOpDescriptor> pipeline,
															 std::vector<TrajectoryOpDescriptor> appliedHistory)
	: m_pathPlanId(std::move(pathPlanId)), m_pipeline(std::move(pipeline)), m_appliedHistory(std::move(appliedHistory))
{
}

bool UpdatePathPlanPipelineCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	Base* base = doc.findById(m_pathPlanId);
	PathPlanInstruction* pp = base ? asPathPlan(*base) : nullptr;
	if (!pp)
	{
		if (errMsg)
		{
			*errMsg = "path plan not found";
		}
		return false;
	}
	m_pipelineBefore = pp->pipeline();
	m_appliedBefore = pp->appliedHistory();
	pp->setPipeline(m_pipeline);
	pp->appliedHistoryMut() = m_appliedHistory;
	return true;
}

bool UpdatePathPlanPipelineCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	Base* base = doc.findById(m_pathPlanId);
	PathPlanInstruction* pp = base ? asPathPlan(*base) : nullptr;
	if (!pp)
	{
		if (errMsg)
		{
			*errMsg = "path plan not found";
		}
		return false;
	}
	pp->setPipeline(m_pipelineBefore);
	pp->appliedHistoryMut() = m_appliedBefore;
	return true;
}

UpdatePathPlanRawCommand::UpdatePathPlanRawCommand(RobotProgramCatalog* catalog, std::string pathPlanId,
												   RawTrajectory raw, const PathPlanPhase newPhase)
	: m_catalog(catalog), m_pathPlanId(std::move(pathPlanId)), m_raw(std::move(raw)), m_newPhase(newPhase)
{
}

bool UpdatePathPlanRawCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	if (!m_catalog)
	{
		if (errMsg)
		{
			*errMsg = "catalog is null";
		}
		return false;
	}
	PathPlanInstruction* pp = m_catalog->findPathPlan(m_catalog->activeProgramId(), m_pathPlanId);
	if (!pp)
	{
		if (errMsg)
		{
			*errMsg = "path plan not found";
		}
		return false;
	}
	m_phaseBefore = pp->phase();
	m_hadRawBefore = m_catalog->pathPlanRaws().load(m_pathPlanId, m_rawBefore);
	m_catalog->pathPlanRaws().save(m_pathPlanId, m_raw);
	pp->setPhase(m_newPhase);
	pp->bumpRawRevision();
	if (pp->rawTrajectoryKey().empty())
	{
		pp->setRawTrajectoryKey(m_pathPlanId);
	}
	return true;
}

bool UpdatePathPlanRawCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	if (!m_catalog)
	{
		return false;
	}
	PathPlanInstruction* pp = m_catalog->findPathPlan(m_catalog->activeProgramId(), m_pathPlanId);
	if (!pp)
	{
		if (errMsg)
		{
			*errMsg = "path plan not found";
		}
		return false;
	}
	pp->setPhase(m_phaseBefore);
	if (m_hadRawBefore)
	{
		m_catalog->pathPlanRaws().save(m_pathPlanId, m_rawBefore);
	}
	else
	{
		m_catalog->pathPlanRaws().remove(m_pathPlanId);
	}
	return true;
}

UpdatePathPlanApplyStateCommand::UpdatePathPlanApplyStateCommand(std::string pathPlanId, const PathPlanPhase phase,
																 std::string outputGroupId)
	: m_pathPlanId(std::move(pathPlanId)), m_phase(phase), m_outputGroupId(std::move(outputGroupId))
{
}

bool UpdatePathPlanApplyStateCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)doc;
	Base* base = doc.findById(m_pathPlanId);
	PathPlanInstruction* pp = base ? asPathPlan(*base) : nullptr;
	if (!pp)
	{
		if (errMsg)
		{
			*errMsg = "path plan not found";
		}
		return false;
	}
	m_phaseBefore = pp->phase();
	m_outputGroupIdBefore = pp->outputGroupId();
	pp->setPhase(m_phase);
	pp->setOutputGroupId(m_outputGroupId);
	return true;
}

bool UpdatePathPlanApplyStateCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	Base* base = doc.findById(m_pathPlanId);
	PathPlanInstruction* pp = base ? asPathPlan(*base) : nullptr;
	if (!pp)
	{
		return false;
	}
	pp->setPhase(m_phaseBefore);
	pp->setOutputGroupId(m_outputGroupIdBefore);
	return true;
}

RemovePathPlanCommand::RemovePathPlanCommand(RobotProgramCatalog* catalog, std::string programId,
											 std::string pathPlanId)
	: m_catalog(catalog), m_programId(std::move(programId)), m_pathPlanId(std::move(pathPlanId))
{
}

bool RemovePathPlanCommand::execute(InstructionProgramDocument& doc, std::string* errMsg)
{
	if (!m_catalog)
	{
		if (errMsg)
		{
			*errMsg = "catalog is null";
		}
		return false;
	}
	Base* base = doc.findById(m_pathPlanId);
	PathPlanInstruction* pp = base ? asPathPlan(*base) : nullptr;
	if (!pp)
	{
		if (errMsg)
		{
			*errMsg = "path plan not found";
		}
		return false;
	}
	m_snapshot = std::dynamic_pointer_cast<PathPlanInstruction>(
		std::static_pointer_cast<Base>(cloneInstructionPreservingId(*pp)));
	if (!m_snapshot)
	{
		if (errMsg)
		{
			*errMsg = "clone path plan failed";
		}
		return false;
	}
	if (doc.rootSteps())
	{
		m_rootIndex = rootIndexOf(*doc.rootSteps(), m_pathPlanId);
	}
	RobotProgram* prog = m_catalog->findProgram(m_programId);
	if (prog)
	{
		for (const InstructionGroup& group : prog->groups)
		{
			if (group.role == InstructionGroupRole::PathPlanOutput && group.pathPlanInstructionId == m_pathPlanId)
			{
				m_removedOutputGroups.push_back(group);
			}
		}
	}
	m_hadRaw = m_catalog->pathPlanRaws().load(m_pathPlanId, m_removedRaw);
	if (!doc.removeById(m_pathPlanId))
	{
		if (errMsg)
		{
			*errMsg = "remove path plan failed";
		}
		return false;
	}
	const std::unordered_set<std::string> removedIds{m_pathPlanId};
	m_catalog->prunePathPlanReferences(m_programId, removedIds);
	m_executed = true;
	return true;
}

bool RemovePathPlanCommand::undo(InstructionProgramDocument& doc, std::string* errMsg)
{
	(void)errMsg;
	if (!m_executed || !m_snapshot || !m_catalog)
	{
		return false;
	}
	if (!doc.insertAtRoot(m_rootIndex, m_snapshot))
	{
		return false;
	}
	RobotProgram* prog = m_catalog->findProgram(m_programId);
	if (prog)
	{
		for (const InstructionGroup& group : m_removedOutputGroups)
		{
			prog->groups.push_back(group);
		}
	}
	if (m_hadRaw)
	{
		m_catalog->pathPlanRaws().save(m_pathPlanId, m_removedRaw);
	}
	m_executed = false;
	return true;
}

} // namespace RobotInstruction
