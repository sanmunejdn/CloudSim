#include "TrajectoryEditSession.h"

#include "InstructionProgramDocument.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"
#include "RobotSimulationController.h"

#include <RigidTransform.h>

TrajectoryEditSession::TrajectoryEditSession(QObject* parent)
	: QObject(parent)
{
}

void TrajectoryEditSession::bindStore(RobotProgramStore* store)
{
	m_store = store;
}

void TrajectoryEditSession::bindEditService(ProgramEditService* service)
{
	m_editService = service;
}

void TrajectoryEditSession::bindSimulationController(RobotSimulationController* controller)
{
	m_simController = controller;
}

void TrajectoryEditSession::updatePipelineOps(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops)
{
	m_ops = std::move(ops);
	m_builder.setOps(m_ops);
	if (m_previewActive)
	{
		(void)reapplyPreview(nullptr);
	}
}

void TrajectoryEditSession::setPipeline(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops)
{
	reset();
	m_ops = std::move(ops);
	m_builder.setOps(m_ops);
}

void TrajectoryEditSession::setContextProgramId(const std::string& programId)
{
	m_contextProgramId = programId;
	if (m_store)
	{
		m_store->setActiveProgramIdUtf8(programId);
		const RobotInstruction::RobotProgram* prog = m_store->activeCatalog().findProgram(programId);
		m_builder.setProgramContext(prog);
	}
}

void TrajectoryEditSession::setDefaultGroupId(const std::string& groupId)
{
	m_defaultGroupId = groupId;
}

std::vector<std::string> TrajectoryEditSession::collectPreviewWaypointIds() const
{
	std::vector<std::string> out;
	if (!m_store)
	{
		return out;
	}
	const std::string programId = m_contextProgramId.empty() ? m_store->activeProgramIdUtf8() : m_contextProgramId;
	const RobotInstruction::RobotProgram* prog = m_store->activeCatalog().findProgram(programId);
	if (!prog)
	{
		return out;
	}
	RobotInstruction::RobotProgramCatalog catalog;
	std::unordered_map<std::string, bool> seen;
	for (const RobotInstruction::TrajectoryOpDescriptor& op : m_ops)
	{
		if (op.kind != RobotInstruction::TrajectoryOpKind::Translate
			&& op.kind != RobotInstruction::TrajectoryOpKind::Rotate)
		{
			continue;
		}
		const std::vector<std::string> ids = catalog.resolveOpScopeInstructionIds(op.scope, *prog);
		std::vector<std::string> waypointIds = ids;
		if (op.scope.kind == RobotInstruction::OpScope::Kind::Group)
		{
			waypointIds = catalog.expandToMotionWaypointIds(*prog, ids);
		}
		for (const std::string& id : waypointIds)
		{
			if (seen.count(id) == 0)
			{
				seen[id] = true;
				out.push_back(id);
			}
		}
	}
	return out;
}

void TrajectoryEditSession::refreshPreviewVisuals()
{
	if (!m_simController)
	{
		return;
	}
	m_simController->refreshInstructionPoseAxes();
	if (IRobotMainWindowHost* host = m_simController->host())
	{
		if (IRobotOsgViewHost* osg = host->osgView())
		{
			osg->requestRedraw();
		}
	}
}

bool TrajectoryEditSession::reapplyPreview(QString* outError)
{
	if (!m_previewActive || m_previewSnapshots.empty())
	{
		return false;
	}
	restorePreviewSnapshots();
	if (!applyPreviewTransforms(outError))
	{
		return false;
	}
	refreshPreviewVisuals();
	return true;
}

bool TrajectoryEditSession::preview(QString* outError)
{
	if (!m_store)
	{
		if (outError)
		{
			*outError = QStringLiteral("no store");
		}
		return false;
	}
	if (m_ops.empty())
	{
		if (outError)
		{
			*outError = QStringLiteral("流水线为空，请先添加平移或旋转块");
		}
		return false;
	}
	bool hasPreviewOp = false;
	for (const RobotInstruction::TrajectoryOpDescriptor& op : m_ops)
	{
		if (op.kind == RobotInstruction::TrajectoryOpKind::Translate
			|| op.kind == RobotInstruction::TrajectoryOpKind::Rotate)
		{
			hasPreviewOp = true;
			break;
		}
	}
	if (!hasPreviewOp)
	{
		if (outError)
		{
			*outError = QStringLiteral("当前流水线无可预览的平移/旋转块");
		}
		return false;
	}
	const std::vector<std::string> waypointIds = collectPreviewWaypointIds();
	if (waypointIds.empty())
	{
		if (outError)
		{
			*outError = QStringLiteral("作用域内无运动路点");
		}
		return false;
	}
	if (m_previewActive)
	{
		restorePreviewSnapshots();
	}
	else if (!capturePreviewSnapshots(outError))
	{
		return false;
	}
	if (m_previewSnapshots.empty())
	{
		if (outError)
		{
			*outError = QStringLiteral("作用域内无运动路点");
		}
		return false;
	}
	if (!applyPreviewTransforms(outError))
	{
		return false;
	}
	m_previewActive = true;
	emit previewStateChanged(true);
	refreshPreviewVisuals();
	return true;
}

bool TrajectoryEditSession::apply(QString* outError)
{
	if (!m_editService || !m_store)
	{
		if (outError)
		{
			*outError = QStringLiteral("not ready");
		}
		return false;
	}
	reset();
	RobotInstruction::InstructionProgramDocument doc(&m_store->activeProgram());
	std::string err;
	const std::vector<RobotInstruction::ProgramEditStack::CommandPtr> cmds =
		m_builder.buildApplyCommands(doc, &err);
	if (cmds.empty())
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("无可执行操作") : QString::fromStdString(err);
		}
		return false;
	}
	for (const RobotInstruction::ProgramEditStack::CommandPtr& cmd : cmds)
	{
		if (!m_editService->execute(cmd, outError))
		{
			return false;
		}
	}
	refreshPreviewVisuals();
	return true;
}

void TrajectoryEditSession::reset()
{
	if (m_previewActive)
	{
		restorePreviewSnapshots();
	}
	clearPreviewSnapshots();
	m_previewActive = false;
	emit previewStateChanged(false);
	refreshPreviewVisuals();
}

void TrajectoryEditSession::abandonPreview()
{
	clearPreviewSnapshots();
	const bool wasActive = m_previewActive;
	m_previewActive = false;
	if (wasActive)
	{
		emit previewStateChanged(false);
	}
	refreshPreviewVisuals();
}

bool TrajectoryEditSession::canApply() const
{
	return m_store && !m_ops.empty();
}

void TrajectoryEditSession::clearPreviewSnapshots()
{
	m_previewSnapshots.clear();
}

bool TrajectoryEditSession::capturePreviewSnapshots(QString* outError)
{
	m_previewSnapshots.clear();
	if (!m_store)
	{
		return false;
	}
	const std::string programId = m_contextProgramId.empty() ? m_store->activeProgramIdUtf8() : m_contextProgramId;
	const RobotInstruction::RobotProgram* prog = m_store->activeCatalog().findProgram(programId);
	if (!prog)
	{
		if (outError)
		{
			*outError = QStringLiteral("program not found");
		}
		return false;
	}
	RobotInstruction::InstructionProgramDocument doc(&m_store->activeProgram());
	for (const std::string& id : collectPreviewWaypointIds())
	{
		RobotInstruction::Base* raw = doc.findById(id);
		if (!raw)
		{
			continue;
		}
		PreviewSnapshot snap;
		snap.id = raw->id();
		snap.pose = raw->pose();
		snap.euler = raw->eulerDeg();
		snap.extensions = raw->extensionProperties();
		m_previewSnapshots.push_back(std::move(snap));
	}
	return true;
}

bool TrajectoryEditSession::applyPreviewTransforms(QString* outError)
{
	if (!m_store)
	{
		return false;
	}
	const std::string programId = m_contextProgramId.empty() ? m_store->activeProgramIdUtf8() : m_contextProgramId;
	const RobotInstruction::RobotProgram* prog = m_store->activeCatalog().findProgram(programId);
	if (!prog)
	{
		if (outError)
		{
			*outError = QStringLiteral("program not found");
		}
		return false;
	}
	m_builder.setProgramContext(prog);
	auto query = m_builder.buildPreviewPoseQuery(prog->steps);
	if (!query)
	{
		return false;
	}
	RobotInstruction::InstructionProgramDocument doc(&m_store->activeProgram());
	for (const PreviewSnapshot& snap : m_previewSnapshots)
	{
		RobotInstruction::Base* raw = doc.findById(snap.id);
		if (!raw)
		{
			continue;
		}
		RobotInstruction::Vec3 pose{};
		RobotInstruction::Vec3 euler{};
		if (!query->queryMotionPose(*raw, pose, euler))
		{
			continue;
		}
		const engine::RigidTransform target = engine::RigidTransform::fromTranslationEulerDeg(
			pose.x,
			pose.y,
			pose.z,
			euler.x,
			euler.y,
			euler.z);
		RobotInstruction::writeTargetTransformToInstruction(*raw, target);
		raw->eraseExtensionProperty("context.currentJointRadCsv");
	}
	syncPreviewRenderMatrices();
	return true;
}

void TrajectoryEditSession::syncPreviewRenderMatrices()
{
	if (!m_simController || !m_store || m_previewSnapshots.empty())
	{
		return;
	}
	std::vector<std::shared_ptr<RobotInstruction::Base>> flat;
	RobotInstruction::flattenInstructionsRecursive(m_store->activeProgram(), flat);
	for (const PreviewSnapshot& snap : m_previewSnapshots)
	{
		for (const std::shared_ptr<RobotInstruction::Base>& ins : flat)
		{
			if (ins && ins->id() == snap.id)
			{
				m_simController->syncInstructionRenderMatricesFromPose(ins);
				break;
			}
		}
	}
}

void TrajectoryEditSession::restorePreviewSnapshots()
{
	if (!m_store)
	{
		return;
	}
	RobotInstruction::InstructionProgramDocument doc(&m_store->activeProgram());
	for (const PreviewSnapshot& snap : m_previewSnapshots)
	{
		RobotInstruction::Base* raw = doc.findById(snap.id);
		if (!raw)
		{
			continue;
		}
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
