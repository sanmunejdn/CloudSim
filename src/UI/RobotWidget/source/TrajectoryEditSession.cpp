#include "TrajectoryEditSession.h"

#include "InstructionProgramDocument.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"
#include "RobotSimulationController.h"
#include "RobotSimulationMath.h"

#include <ITrajectoryOp.h>
#include "TrajectoryOpBridge.h"

#include <Adapters.h>
#include <RigidTransform.h>

#include <cmath>
#include <osg/Matrixd>

namespace
{

bool rigidBaseWorldFromSnapshot(
	const osg::Matrixd& snapWorld,
	const osg::Matrixd& snapLocal,
	osg::Matrixd& outBaseWorld)
{
	osg::Matrixd invLocal;
	if (!invLocal.invert(snapLocal))
	{
		return false;
	}
	outBaseWorld = invLocal * snapWorld;
	const osg::Vec3d t = outBaseWorld.getTrans();
	return std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z());
}

bool previewPoseNearlyUnchanged(
	const RobotInstruction::Vec3& snapPose,
	const RobotInstruction::Vec3& snapEuler,
	const RobotInstruction::Vec3& newPose,
	const RobotInstruction::Vec3& newEuler)
{
	const auto near = [](const double a, const double b) { return std::abs(a - b) <= 0.001; };
	return near(snapPose.x, newPose.x) && near(snapPose.y, newPose.y) && near(snapPose.z, newPose.z)
		&& near(snapEuler.x, newEuler.x) && near(snapEuler.y, newEuler.y) && near(snapEuler.z, newEuler.z);
}

void restoreRenderExtensionsFromSnapshot(
	RobotInstruction::Base& raw,
	const std::unordered_map<std::string, std::string>& extensions)
{
	for (const auto& kv : extensions)
	{
		if (kv.first.rfind("render.", 0) == 0)
		{
			raw.setExtensionProperty(kv.first, kv.second);
		}
	}
}

} // namespace

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
		const trajectory_algo::ITrajectoryOp* algo =
			RobotInstruction::trajectoryOpGet(op.kind);
		if (!algo
			|| !trajectory_algo::hasCapability(
				algo->capabilities(),
				trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform))
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
		const trajectory_algo::ITrajectoryOp* algo =
			RobotInstruction::trajectoryOpGet(op.kind);
		if (algo
			&& trajectory_algo::hasCapability(
				algo->capabilities(),
				trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform))
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
	const std::vector<std::string> affectedIds = collectPreviewWaypointIds();
	std::unordered_map<std::string, std::string> frozenBaseWorldCsvById;
	for (const PreviewSnapshot& snap : m_previewSnapshots)
	{
		osg::Matrixd snapWorld;
		snapWorld.makeIdentity();
		const auto itW = snap.extensions.find("render.tcpWorldMat4");
		if (itW == snap.extensions.end() || itW->second.empty()
			|| !RobotSimulationMath::decodeMatrix4Csv(itW->second, snapWorld))
		{
			continue;
		}
		osg::Matrixd snapLocal;
		snapLocal.makeIdentity();
		const auto itL = snap.extensions.find("render.tcpLocalMat4");
		if (itL != snap.extensions.end() && !itL->second.empty()
			&& RobotSimulationMath::decodeMatrix4Csv(itL->second, snapLocal))
		{
			// ok
		}
		else
		{
			const engine::RigidTransform t = engine::RigidTransform::fromTranslationEulerDeg(
				snap.pose.x,
				snap.pose.y,
				snap.pose.z,
				snap.euler.x,
				snap.euler.y,
				snap.euler.z);
			snapLocal = engine::osgMatrixFromRigidTransform(t);
		}
		osg::Matrixd baseWorld;
		if (rigidBaseWorldFromSnapshot(snapWorld, snapLocal, baseWorld))
		{
			frozenBaseWorldCsvById[snap.id] = RobotSimulationMath::encodeMatrix4Csv(baseWorld);
		}
	}
	// Apply 需从快照基线重算，但不要在落盘前 refresh（会短暂用旧 render.tcpWorldMat4 画轴）
	if (m_previewActive && !m_previewSnapshots.empty())
	{
		restorePreviewSnapshots();
	}
	clearPreviewStateWithoutRestore();
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
	m_applying = true;
	for (const RobotInstruction::ProgramEditStack::CommandPtr& cmd : cmds)
	{
		if (!m_editService->execute(cmd, outError))
		{
			m_applying = false;
			return false;
		}
	}
	m_applying = false;
	if (!frozenBaseWorldCsvById.empty())
	{
		syncRenderMatricesFromFrozenBase(affectedIds, frozenBaseWorldCsvById);
	}
	else
	{
		syncRenderMatricesForInstructionIds(affectedIds);
	}
	refreshPreviewVisuals();
	return true;
}

void TrajectoryEditSession::clearPipelineAfterCommit()
{
	m_ops.clear();
	m_builder.setOps(m_ops);
}

void TrajectoryEditSession::clearPreviewStateWithoutRestore()
{
	clearPreviewSnapshots();
	const bool wasActive = m_previewActive;
	m_previewActive = false;
	if (wasActive)
	{
		emit previewStateChanged(false);
	}
}

void TrajectoryEditSession::reset()
{
	if (m_previewActive && !m_previewSnapshots.empty())
	{
		restorePreviewSnapshots();
	}
	clearPreviewStateWithoutRestore();
	refreshPreviewVisuals();
}

void TrajectoryEditSession::abandonPreview()
{
	if (m_previewActive && !m_previewSnapshots.empty())
	{
		restorePreviewSnapshots();
	}
	clearPreviewStateWithoutRestore();
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
		if (previewPoseNearlyUnchanged(snap.pose, snap.euler, pose, euler))
		{
			restoreRenderExtensionsFromSnapshot(*raw, snap.extensions);
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
		raw->eraseExtensionProperty("render.tcpWorldMat4");
		raw->eraseExtensionProperty("render.tcpLocalMat4");
	}
	syncPreviewRenderMatrices();
	return true;
}

void TrajectoryEditSession::syncRenderMatricesForInstructionIds(const std::vector<std::string>& ids)
{
	if (!m_simController || !m_store || ids.empty())
	{
		return;
	}
	std::vector<std::shared_ptr<RobotInstruction::Base>> flat;
	RobotInstruction::flattenInstructionsRecursive(m_store->activeProgram(), flat);
	for (const std::string& id : ids)
	{
		for (const std::shared_ptr<RobotInstruction::Base>& ins : flat)
		{
			if (ins && ins->id() == id)
			{
				m_simController->syncInstructionRenderMatricesFromPose(ins);
				break;
			}
		}
	}
}

bool TrajectoryEditSession::writeRenderMatricesFromSnapshotBase(
	const PreviewSnapshot& snap,
	RobotInstruction::Base& raw,
	const std::string* frozenBaseWorldCsv,
	double* outWorldDeltaMm) const
{
	osg::Matrixd snapWorld;
	snapWorld.makeIdentity();
	const auto itW = snap.extensions.find("render.tcpWorldMat4");
	const bool hasSnapWorld = itW != snap.extensions.end() && !itW->second.empty()
		&& RobotSimulationMath::decodeMatrix4Csv(itW->second, snapWorld);
	osg::Matrixd snapLocal;
	snapLocal.makeIdentity();
	const auto itL = snap.extensions.find("render.tcpLocalMat4");
	if (itL != snap.extensions.end() && !itL->second.empty()
		&& RobotSimulationMath::decodeMatrix4Csv(itL->second, snapLocal))
	{
		// ok
	}
	else
	{
		const engine::RigidTransform t = engine::RigidTransform::fromTranslationEulerDeg(
			snap.pose.x,
			snap.pose.y,
			snap.pose.z,
			snap.euler.x,
			snap.euler.y,
			snap.euler.z);
		snapLocal = engine::osgMatrixFromRigidTransform(t);
	}
	if (!hasSnapWorld)
	{
		return false;
	}
	osg::Matrixd baseWorld;
	if (frozenBaseWorldCsv && !frozenBaseWorldCsv->empty()
		&& RobotSimulationMath::decodeMatrix4Csv(*frozenBaseWorldCsv, baseWorld))
	{
		// ok
	}
	else if (!rigidBaseWorldFromSnapshot(snapWorld, snapLocal, baseWorld))
	{
		return false;
	}
	engine::RigidTransform newTarget{};
	if (!RobotInstruction::readTargetTransformFromInstruction(raw, newTarget))
	{
		return false;
	}
	const osg::Matrixd newLocal = engine::osgMatrixFromRigidTransform(newTarget);
	const osg::Matrixd newWorld = newLocal * baseWorld;
	if (outWorldDeltaMm)
	{
		const osg::Vec3d delta = newWorld.getTrans() - snapWorld.getTrans();
		*outWorldDeltaMm = delta.length();
	}
	raw.setExtensionProperty("render.tcpLocalMat4", RobotSimulationMath::encodeMatrix4Csv(newLocal));
	raw.setExtensionProperty("render.tcpWorldMat4", RobotSimulationMath::encodeMatrix4Csv(newWorld));
	return true;
}

void TrajectoryEditSession::syncPreviewRenderMatrices()
{
	if (m_previewSnapshots.empty() || !m_store)
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
		if (!writeRenderMatricesFromSnapshotBase(snap, *raw, nullptr, nullptr))
		{
			restoreRenderExtensionsFromSnapshot(*raw, snap.extensions);
		}
	}
}

void TrajectoryEditSession::syncRenderMatricesFromFrozenBase(
	const std::vector<std::string>& ids,
	const std::unordered_map<std::string, std::string>& frozenBaseWorldCsvById)
{
	if (!m_store || ids.empty())
	{
		return;
	}
	RobotInstruction::InstructionProgramDocument doc(&m_store->activeProgram());
	for (const std::string& id : ids)
	{
		const auto itBase = frozenBaseWorldCsvById.find(id);
		if (itBase == frozenBaseWorldCsvById.end())
		{
			continue;
		}
		RobotInstruction::Base* raw = doc.findById(id);
		if (!raw)
		{
			continue;
		}
		osg::Matrixd baseWorld;
		if (!RobotSimulationMath::decodeMatrix4Csv(itBase->second, baseWorld))
		{
			continue;
		}
		engine::RigidTransform newTarget{};
		if (!RobotInstruction::readTargetTransformFromInstruction(*raw, newTarget))
		{
			continue;
		}
		const osg::Matrixd newLocal = engine::osgMatrixFromRigidTransform(newTarget);
		const osg::Matrixd newWorld = newLocal * baseWorld;
		raw->setExtensionProperty("render.tcpLocalMat4", RobotSimulationMath::encodeMatrix4Csv(newLocal));
		raw->setExtensionProperty("render.tcpWorldMat4", RobotSimulationMath::encodeMatrix4Csv(newWorld));
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
