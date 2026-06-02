#include "TrajectoryEditSession.h"

#include "InstructionProgramDocument.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"
#include "RecipeBlueprint.h"
#include "RobotSimulationController.h"
#include "RobotSimulationMath.h"
#include "FeaturePickTransform.h"
#include "UnifiedTrajectory.h"

#include <ITrajectoryOp.h>
#include "TrajectoryOpBridge.h"

#include <Adapters.h>
#include <RigidTransform.h>
#include "RawTrajectory.h"

#include <algorithm>
#include <cmath>
#include <osg/Matrixd>
#include <unordered_set>

namespace
{
using InstructionIndex = std::unordered_map<std::string, std::shared_ptr<RobotInstruction::Base>>;
constexpr size_t kLightweightPreviewGuardThreshold = 2000;
constexpr size_t kLightweightPreviewMaxWaypoints = 1200;
constexpr const char* kPreviewProbeId = "__preview_probe__";

std::string scopeCacheKey(const RobotInstruction::OpScope& scope)
{
	std::string key = std::to_string(static_cast<int>(scope.kind));
	key.push_back('|');
	key += scope.groupId;
	key.push_back('|');
	key += std::to_string(scope.pointFrom);
	key.push_back('|');
	key += std::to_string(scope.pointTo);
	if (scope.kind == RobotInstruction::OpScope::Kind::InstructionIds)
	{
		for (const std::string& id : scope.instructionIds)
		{
			key.push_back('|');
			key += id;
		}
	}
	return key;
}

InstructionIndex buildInstructionIndex(std::vector<std::shared_ptr<RobotInstruction::Base>>& steps)
{
	std::vector<std::shared_ptr<RobotInstruction::Base>> flat;
	RobotInstruction::flattenInstructionsRecursive(steps, flat);
	InstructionIndex index;
	index.reserve(flat.size());
	for (const std::shared_ptr<RobotInstruction::Base>& ins : flat)
	{
		if (ins)
		{
			index.emplace(ins->id(), ins);
		}
	}
	return index;
}

bool canContributePreview(
	const trajectory_algo::ITrajectoryOp& algo,
	const RobotInstruction::TrajectoryOpDescriptor& op)
{
	if (!trajectory_algo::hasCapability(
			algo.capabilities(),
			trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform))
	{
		return false;
	}
	trajectory_algo::PreviewTransformStep step{};
	const std::vector<std::string> probeIds = { kPreviewProbeId };
	if (!algo.contributePreviewTransform(op, probeIds, step))
	{
		return false;
	}
	return !step.targetIds.empty();
}

std::vector<std::string> downsampleWaypointIds(const std::vector<std::string>& ids)
{
	if (ids.size() <= kLightweightPreviewMaxWaypoints)
	{
		return ids;
	}
	std::vector<std::string> out;
	out.reserve(kLightweightPreviewMaxWaypoints);
	const size_t stride = std::max<size_t>(1, ids.size() / kLightweightPreviewMaxWaypoints);
	for (size_t i = 0; i < ids.size(); i += stride)
	{
		out.push_back(ids[i]);
		if (out.size() >= kLightweightPreviewMaxWaypoints)
		{
			break;
		}
	}
	return out;
}

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

bool isUnifiedOnlyKind(const RobotInstruction::TrajectoryOpKind kind)
{
	return RobotInstruction::isRecipeOpKind(kind)
		|| kind == RobotInstruction::TrajectoryOpKind::Approach
		|| kind == RobotInstruction::TrajectoryOpKind::Retract;
}

void appendGeometryOpsHistory(
	std::vector<RobotInstruction::TrajectoryOpDescriptor>& history,
	const std::vector<RobotInstruction::TrajectoryOpDescriptor>& geometryOps)
{
	for (const RobotInstruction::TrajectoryOpDescriptor& op : geometryOps)
	{
		if (!RobotInstruction::isRecipeOpKind(op.kind))
		{
			history.push_back(op);
		}
	}
}

void partitionPipelineOps(
	const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops,
	std::vector<RobotInstruction::TrajectoryOpDescriptor>& recipeOps,
	std::vector<RobotInstruction::TrajectoryOpDescriptor>& geometryOps)
{
	recipeOps.clear();
	geometryOps.clear();
	recipeOps.reserve(ops.size());
	geometryOps.reserve(ops.size());
	for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		if (RobotInstruction::isRecipeOpKind(op.kind))
		{
			recipeOps.push_back(op);
		}
		else
		{
			geometryOps.push_back(op);
		}
	}
}

bool requiresUnifiedApply(
	const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops,
	const bool hasRawTrajectory)
{
	if (hasRawTrajectory)
	{
		return true;
	}
	for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		if (isUnifiedOnlyKind(op.kind))
		{
			return true;
		}
	}
	return false;
}

std::vector<std::string> collectMotionIds(const RobotInstruction::RobotProgram& program)
{
	std::vector<std::string> ids;
	std::vector<std::shared_ptr<RobotInstruction::Base>> flat;
	RobotInstruction::flattenInstructionsRecursive(program.steps, flat);
	ids.reserve(flat.size());
	for (const std::shared_ptr<RobotInstruction::Base>& base : flat)
	{
		if (base && RobotInstruction::isMotionWaypointType(base->type()))
		{
			ids.push_back(base->id());
		}
	}
	return ids;
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
	invalidatePreviewScopeCache();
}

void TrajectoryEditSession::bindEditService(ProgramEditService* service)
{
	if (m_editService)
	{
		disconnect(m_editService, nullptr, this, nullptr);
	}
	m_editService = service;
	if (m_editService)
	{
		m_programRevision = m_editService->revision();
		connect(m_editService, &ProgramEditService::revisionChanged, this, [this](const int revision) {
			m_programRevision = revision;
			invalidatePreviewScopeCache();
		});
	}
}

void TrajectoryEditSession::bindSimulationController(RobotSimulationController* controller)
{
	m_simController = controller;
}

void TrajectoryEditSession::updatePipelineOps(
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops,
	const bool /*allowPreviewReapply*/)
{
	m_ops = std::move(ops);
	m_builder.setOps(m_ops);
	invalidatePreviewScopeCache();
	syncPipelineToBoundPathPlan();
}

void TrajectoryEditSession::setPipeline(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops)
{
	reset();
	m_ops = std::move(ops);
	m_builder.setOps(m_ops);
	invalidatePreviewScopeCache();
	syncPipelineToBoundPathPlan();
}

void TrajectoryEditSession::setContextProgramId(const std::string& programId)
{
	m_contextProgramId = programId;
	invalidatePreviewScopeCache();
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
	if (!m_store)
	{
		return {};
	}
	const std::string programId = m_contextProgramId.empty() ? m_store->activeProgramIdUtf8() : m_contextProgramId;
	if (m_previewWaypointCacheValid && m_previewWaypointCacheProgramId == programId
		&& m_previewWaypointCacheRevision == m_programRevision)
	{
		return m_previewWaypointCache;
	}
	const RobotInstruction::RobotProgram* prog = m_store->activeCatalog().findProgram(programId);
	if (!prog)
	{
		return {};
	}
	std::vector<std::string> out;
	RobotInstruction::RobotProgramCatalog catalog;
	std::unordered_set<std::string> seen;
	std::unordered_map<std::string, std::vector<std::string>> scopeCache;
	for (const RobotInstruction::TrajectoryOpDescriptor& op : m_ops)
	{
		const trajectory_algo::ITrajectoryOp* algo =
			RobotInstruction::trajectoryOpGet(op.kind);
		if (!algo || !canContributePreview(*algo, op))
		{
			continue;
		}
		const std::string key = scopeCacheKey(op.scope);
		auto it = scopeCache.find(key);
		if (it == scopeCache.end())
		{
			std::vector<std::string> ids = catalog.resolveOpScopeInstructionIds(op.scope, *prog);
			if (op.scope.kind == RobotInstruction::OpScope::Kind::Group)
			{
				ids = catalog.expandToMotionWaypointIds(*prog, ids);
			}
			it = scopeCache.emplace(key, std::move(ids)).first;
		}
		for (const std::string& id : it->second)
		{
			if (seen.insert(id).second)
			{
				out.push_back(id);
			}
		}
	}
	m_previewWaypointCache = out;
	m_previewWaypointCacheProgramId = programId;
	m_previewWaypointCacheRevision = m_programRevision;
	m_previewWaypointCacheValid = true;
	return out;
}

void TrajectoryEditSession::invalidatePreviewScopeCache()
{
	m_previewWaypointCacheValid = false;
	m_previewWaypointCacheProgramId.clear();
	m_previewWaypointCacheRevision = -1;
	m_previewWaypointCache.clear();
}

bool TrajectoryEditSession::rebuildUnifiedFromSourceRaw(
	const RobotInstruction::RawTrajectory& sourceRaw,
	RobotInstruction::UnifiedTrajectory& unified,
	QString* outError) const
{
	RobotInstruction::RawTrajectory rawForUnified = sourceRaw;
	const std::string backendId = RobotInstruction::rawTrajectoryWorkpieceBackendId(sourceRaw);
	if (!backendId.empty() && m_simController && m_simController->host()
		&& m_simController->host()->osgView())
	{
		RobotInstruction::RawTrajectory worldRaw;
		std::string worldErr;
		if (feature_pick_transform::transformRawTrajectoryToWorld(
				m_simController->host()->osgView(),
				backendId,
				sourceRaw,
				worldRaw,
				&worldErr))
		{
			rawForUnified = std::move(worldRaw);
		}
	}
	std::string convErr;
	if (!RobotInstruction::unifiedTrajectoryFromRaw(rawForUnified, unified, &convErr))
	{
		if (outError)
		{
			*outError = convErr.empty()
				? QStringLiteral("raw trajectory convert failed")
				: QString::fromStdString(convErr);
		}
		return false;
	}
	return true;
}

void TrajectoryEditSession::refreshPreviewVisuals()
{
	if (!m_simController)
	{
		return;
	}
	m_simController->refreshInstructionPoseAxes(false);
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
	const bool hasRaw = m_rawTrajectory.has_value() && !m_rawTrajectory->points.empty();
	if (hasRaw)
	{
		return false;
	}
	if (requiresUnifiedApply(m_ops, false))
	{
		return previewUnifiedFromProgramPipeline(outError);
	}
	restorePreviewSnapshots();
	if (!applyPreviewTransforms(outError))
	{
		return false;
	}
	refreshPreviewVisuals();
	return true;
}

bool TrajectoryEditSession::previewPipeline(
	const std::vector<RobotInstruction::TrajectoryOpDescriptor>& pipelineOps,
	QString* outError)
{
	m_ops = pipelineOps;
	m_builder.setOps(m_ops);
	invalidatePreviewScopeCache();
	if (m_simController)
	{
		m_simController->setRawTrajectoryPreviewActive(false);
		if (IRobotMainWindowHost* host = m_simController->host())
		{
			if (IRobotOsgViewHost* osg = host->osgView())
			{
				osg->clearRawTrajectoryOverlay();
				osg->clearRawTrajectoryOverlayFrames();
			}
		}
	}
	if (m_rawTrajectory.has_value() && !m_rawTrajectory->points.empty())
	{
		if (outError)
		{
			*outError = QStringLiteral("请使用 buildRawPreviewWithPipeline");
		}
		return false;
	}
	if (requiresUnifiedApply(pipelineOps, false))
	{
		return previewUnifiedFromProgramPipeline(outError);
	}
	return preview(outError);
}

bool TrajectoryEditSession::previewUnifiedFromProgramPipeline(QString* outError)
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
			*outError = QStringLiteral("流水线为空");
		}
		return false;
	}
	std::vector<RobotInstruction::TrajectoryOpDescriptor> recipeOps;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> geometryOps;
	partitionPipelineOps(m_ops, recipeOps, geometryOps);
	bool hasGeometryPreview = false;
	for (const RobotInstruction::TrajectoryOpDescriptor& op : geometryOps)
	{
		const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(op.kind);
		if (algo
			&& trajectory_algo::hasCapability(
				algo->capabilities(),
				trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform))
		{
			hasGeometryPreview = true;
			break;
		}
	}
	if (!hasGeometryPreview && recipeOps.empty())
	{
		if (outError)
		{
			*outError = QStringLiteral("当前流水线无可预览块");
		}
		return false;
	}
	const std::vector<std::string> waypointIds = collectPreviewWaypointIds();
	if (waypointIds.empty())
	{
		if (outError)
		{
			*outError = QStringLiteral("作用域内无运动路点，请先在程序中创建路点或完成轨迹离散");
		}
		return false;
	}
	m_effectivePreviewWaypointIds = waypointIds;
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
	if (m_previewActive)
	{
		restorePreviewSnapshots();
	}
	else if (!capturePreviewSnapshots(outError))
	{
		return false;
	}
	RobotInstruction::UnifiedTrajectory unified{};
	std::string err;
	if (!RobotInstruction::unifiedTrajectoryFromProgram(*prog, unified, &err))
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("program convert failed") : QString::fromStdString(err);
		}
		return false;
	}
	RobotInstruction::RawTrajectory rawWorking{};
	if (!RobotInstruction::unifiedTrajectoryToRaw(unified, rawWorking, &err))
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("unified->raw failed") : QString::fromStdString(err);
		}
		return false;
	}
	for (const RobotInstruction::TrajectoryOpDescriptor& op : recipeOps)
	{
		if (!RobotInstruction::applyRecipeDescriptorToRawTrajectory(op, rawWorking, &err))
		{
			if (outError)
			{
				*outError = err.empty() ? QStringLiteral("recipe preview failed") : QString::fromStdString(err);
			}
			return false;
		}
		if (!rebuildUnifiedFromSourceRaw(rawWorking, unified, outError))
		{
			return false;
		}
	}
	auto applyUnifiedOps = [&](const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops) -> bool {
		for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
		{
			if (!RobotInstruction::applyUnifiedTrajectoryOp(op, unified, &err))
			{
				if (outError)
				{
					*outError = err.empty() ? QStringLiteral("unified preview failed") : QString::fromStdString(err);
				}
				return false;
			}
		}
		return true;
	};
	// 程序 Apply 已落盘的几何在路点里，pending 留给首次 raw Apply，预览不再叠一次
	if (!applyUnifiedOps(geometryOps))
	{
		return false;
	}
	const std::vector<std::string> motionIds = collectMotionIds(*prog);
	std::vector<std::shared_ptr<RobotInstruction::Base>>& activeSteps = m_store->activeProgram();
	InstructionIndex index = buildInstructionIndex(activeSteps);
	std::vector<std::string> changedIds;
	changedIds.reserve(motionIds.size());
	for (size_t i = 0; i < unified.points.size(); ++i)
	{
		std::string id;
		if (!unified.points[i].sourceInstructionId.empty())
		{
			id = unified.points[i].sourceInstructionId;
		}
		else if (i < motionIds.size())
		{
			id = motionIds[i];
		}
		else
		{
			continue;
		}
		const auto it = index.find(id);
		if (it == index.end() || !it->second)
		{
			continue;
		}
		const engine::RigidTransform target = engine::RigidTransform::fromTranslationEulerDeg(
			unified.points[i].poseMm.x,
			unified.points[i].poseMm.y,
			unified.points[i].poseMm.z,
			unified.points[i].eulerDeg.x,
			unified.points[i].eulerDeg.y,
			unified.points[i].eulerDeg.z);
		RobotInstruction::writeTargetTransformToInstruction(*it->second, target);
		it->second->eraseExtensionProperty("context.currentJointRadCsv");
		it->second->eraseExtensionProperty("render.tcpWorldMat4");
		it->second->eraseExtensionProperty("render.tcpLocalMat4");
		changedIds.push_back(id);
	}
	if (!changedIds.empty())
	{
		syncRenderMatricesForInstructionIds(changedIds, true);
	}
	m_previewActive = true;
	emit previewStateChanged(true);
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
			*outError = QStringLiteral("流水线为空，请先添加算法块");
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
			*outError = QStringLiteral("当前流水线无可预览块（Recipe/进退刀仅在应用时生效）");
		}
		return false;
	}
	const std::vector<std::string> waypointIds = collectPreviewWaypointIds();
	if (waypointIds.empty())
	{
		if (outError)
		{
			*outError = QStringLiteral("当前参数无有效预览变更");
		}
		updateLightweightPreviewState(false);
		return false;
	}
	const bool useLightweight = waypointIds.size() > kLightweightPreviewGuardThreshold;
	m_effectivePreviewWaypointIds = useLightweight ? downsampleWaypointIds(waypointIds) : waypointIds;
	updateLightweightPreviewState(useLightweight);
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
	const bool hasRawContext = m_rawTrajectory.has_value() || m_bakedWorldRaw.has_value();
	const bool useUnifiedApply = requiresUnifiedApply(m_ops, hasRawContext);

	std::vector<RobotInstruction::TrajectoryOpDescriptor> recipeOps;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> geometryOps;
	recipeOps.reserve(m_ops.size());
	geometryOps.reserve(m_ops.size());
	for (const RobotInstruction::TrajectoryOpDescriptor& op : m_ops)
	{
		if (RobotInstruction::isRecipeOpKind(op.kind))
		{
			recipeOps.push_back(op);
		}
		else
		{
			geometryOps.push_back(op);
		}
	}

	if (useUnifiedApply)
	{
		RobotInstruction::RobotProgram* activeProgram = m_store->activeCatalog().mainProgram();
		if (!activeProgram)
		{
			if (outError)
			{
				*outError = QStringLiteral("active program is null");
			}
			return false;
		}
		RobotInstruction::UnifiedTrajectory unified{};
		RobotInstruction::RawTrajectory rawWorking{};
		bool usingRaw = false;
		auto rebuildUnifiedFromRawForApply = [this, &unified](
												 const RobotInstruction::RawTrajectory& sourceRaw,
												 const char* phase,
												 QString* outErrorPtr) -> bool
		{
			if (rebuildUnifiedFromSourceRaw(sourceRaw, unified, outErrorPtr))
			{
				return true;
			}
			if (outErrorPtr && outErrorPtr->isEmpty())
			{
				*outErrorPtr = QStringLiteral("raw trajectory convert failed (%1)").arg(QString::fromUtf8(phase));
			}
			return false;
		};
		auto applyOpListToUnified = [&](const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops,
										 QString* errOut) -> bool {
			for (const RobotInstruction::TrajectoryOpDescriptor& priorOp : ops)
			{
				std::string priorErr;
				if (!RobotInstruction::applyUnifiedTrajectoryOp(priorOp, unified, &priorErr))
				{
					if (errOut)
					{
						*errOut = priorErr.empty()
							? QStringLiteral("累积几何变换应用失败")
							: QString::fromStdString(priorErr);
					}
					return false;
				}
			}
			return true;
		};
		auto applyGeometryOpsToUnified = [&](const std::vector<RobotInstruction::TrajectoryOpDescriptor>& geometryOps,
											   QString* errOut) -> bool {
			for (const RobotInstruction::TrajectoryOpDescriptor& op : geometryOps)
			{
				std::string opErr;
				if (!RobotInstruction::applyUnifiedTrajectoryOp(op, unified, &opErr))
				{
					if (errOut)
					{
						*errOut = opErr.empty() ? QStringLiteral("unified op apply failed") : QString::fromStdString(opErr);
					}
					return false;
				}
			}
			return true;
		};

		bool unifiedFromProgram = false;
		if (m_rawTrajectory.has_value())
		{
			usingRaw = true;
			rawWorking = *m_rawTrajectory;
			if (!rebuildUnifiedFromRawForApply(rawWorking, "init", outError))
			{
				return false;
			}
			if (!applyOpListToUnified(m_pendingPreRawGeometryOps, outError))
			{
				return false;
			}
			m_pendingPreRawGeometryOps.clear();
		}
		else
		{
			std::vector<std::shared_ptr<RobotInstruction::Base>> flat;
			RobotInstruction::flattenInstructionsRecursive(activeProgram->steps, flat);
			int motionCount = 0;
			for (const std::shared_ptr<RobotInstruction::Base>& base : flat)
			{
				if (base && RobotInstruction::isMotionWaypointType(base->type()))
				{
					++motionCount;
				}
			}
			if (motionCount > 0)
			{
				std::string convErr;
				if (RobotInstruction::unifiedTrajectoryFromProgram(*activeProgram, unified, &convErr))
				{
					unifiedFromProgram = true;
				}
				else if (outError)
				{
					*outError = convErr.empty() ? QStringLiteral("program convert failed")
												: QString::fromStdString(convErr);
					return false;
				}
			}
			if (!unifiedFromProgram)
			{
				if (outError)
				{
					*outError = QStringLiteral("无原始轨迹且程序中无路点");
				}
				return false;
			}
		}
		if (!recipeOps.empty())
		{
			if (!usingRaw)
			{
				if (outError)
				{
					*outError = QStringLiteral("配方块需要原始轨迹输入");
				}
				return false;
			}
			if (rawWorking.points.empty() && m_rawTrajectory.has_value())
			{
				rawWorking = *m_rawTrajectory;
			}
			for (const RobotInstruction::TrajectoryOpDescriptor& op : recipeOps)
			{
				std::string recipeErr;
				if (!RobotInstruction::applyRecipeDescriptorToRawTrajectory(op, rawWorking, &recipeErr))
				{
					if (outError)
					{
						*outError = recipeErr.empty() ? QStringLiteral("recipe apply failed") : QString::fromStdString(recipeErr);
					}
					return false;
				}
			}
			if (!rebuildUnifiedFromRawForApply(rawWorking, "recipe", outError))
			{
				return false;
			}
			m_bakedWorldRaw.reset();
		}
		if (!applyOpListToUnified(m_accumulatedGeometryOps, outError))
		{
			return false;
		}
		if (!applyGeometryOpsToUnified(geometryOps, outError))
		{
			return false;
		}
		RobotInstruction::RobotProgram replacement = *activeProgram;
		std::string emitErr;
		std::string outputGroupId;
		const bool boundPathPlan = !m_boundPathPlanId.empty();
		if (boundPathPlan)
		{
			if (!RobotInstruction::unifiedTrajectoryMergeIntoProgram(
					unified,
					replacement,
					m_boundPathPlanId,
					&emitErr,
					&outputGroupId))
			{
				if (outError)
				{
					*outError = emitErr.empty() ? QStringLiteral("materialize program failed")
												: QString::fromStdString(emitErr);
				}
				return false;
			}
		}
		else if (!RobotInstruction::unifiedTrajectoryToProgram(unified, replacement, &emitErr))
		{
			if (outError)
			{
				*outError = emitErr.empty() ? QStringLiteral("materialize program failed") : QString::fromStdString(emitErr);
			}
			return false;
		}
		appendGeometryOpsHistory(m_accumulatedGeometryOps, geometryOps);
		if (usingRaw)
		{
			if (!rawWorking.points.empty())
			{
				m_rawTrajectory = rawWorking;
			}
			RobotInstruction::RawTrajectory baked{};
			if (RobotInstruction::unifiedTrajectoryToRaw(unified, baked, nullptr))
			{
				m_bakedWorldRaw = std::move(baked);
			}
		}
		std::vector<RobotInstruction::ProgramEditStack::CommandPtr> cmds;
		if (boundPathPlan)
		{
			RobotInstruction::RobotProgramCatalog& catalog = m_store->activeCatalog();
			cmds.push_back(std::make_shared<RobotInstruction::UpdatePathPlanPipelineCommand>(
				m_boundPathPlanId,
				m_ops,
				m_accumulatedGeometryOps));
			if (usingRaw && m_rawTrajectory.has_value())
			{
				cmds.push_back(std::make_shared<RobotInstruction::UpdatePathPlanRawCommand>(
					&catalog,
					m_boundPathPlanId,
					rawWorking,
					RobotInstruction::PathPlanPhase::Applied));
			}
			cmds.push_back(std::make_shared<RobotInstruction::UpdatePathPlanApplyStateCommand>(
				m_boundPathPlanId,
				RobotInstruction::PathPlanPhase::Applied,
				outputGroupId));
			if (!outputGroupId.empty())
			{
				m_defaultGroupId = outputGroupId;
			}
		}
		cmds.push_back(std::make_shared<RobotInstruction::ReplaceProgramContentCommand>(
			activeProgram,
			std::move(replacement)));
		std::vector<RobotInstruction::ProgramEditStack::CommandPtr> batch;
		if (cmds.size() > 1)
		{
			batch.push_back(std::make_shared<RobotInstruction::CompositeProgramEditCommand>(std::move(cmds)));
		}
		else
		{
			batch = std::move(cmds);
		}
		if (!m_editService->executeBatch(batch, outError))
		{
			return false;
		}
		const std::vector<std::string> affectedIds = collectMotionIds(*activeProgram);
		syncRenderMatricesForInstructionIds(affectedIds, true);
		refreshPreviewVisuals();
		return true;
	}

	const std::vector<std::string> affectedIds = !m_effectivePreviewWaypointIds.empty()
		? m_effectivePreviewWaypointIds
		: collectPreviewWaypointIds();
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
	if (!m_editService->executeBatch(cmds, outError))
	{
		m_applying = false;
		return false;
	}
	m_applying = false;
	if (!m_rawTrajectory.has_value())
	{
		appendGeometryOpsHistory(m_pendingPreRawGeometryOps, geometryOps);
	}
	else
	{
		appendGeometryOpsHistory(m_accumulatedGeometryOps, geometryOps);
	}
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
	m_effectivePreviewWaypointIds.clear();
	updateLightweightPreviewState(false);
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

void TrajectoryEditSession::clearTrajectoryGeometryHistory()
{
	m_pendingPreRawGeometryOps.clear();
	m_accumulatedGeometryOps.clear();
	m_bakedWorldRaw.reset();
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

void TrajectoryEditSession::bindPathPlan(const std::string& pathPlanInstructionId)
{
	m_boundPathPlanId = pathPlanInstructionId;
	if (!m_store || pathPlanInstructionId.empty())
	{
		return;
	}
	RobotInstruction::RobotProgramCatalog& catalog = m_store->activeCatalog();
	RobotInstruction::PathPlanInstruction* pp = catalog.findPathPlan(
		catalog.activeProgramId(),
		pathPlanInstructionId);
	if (!pp)
	{
		return;
	}
	m_ops = pp->pipeline();
	m_accumulatedGeometryOps = pp->appliedHistory();
	m_builder.setOps(m_ops);
	if (!pp->outputGroupId().empty())
	{
		m_defaultGroupId = pp->outputGroupId();
	}
	loadRawFromBoundPathPlan();
}

void TrajectoryEditSession::clearPathPlanBinding()
{
	m_boundPathPlanId.clear();
}

bool TrajectoryEditSession::persistBoundPathPlanPipeline(QString* outError)
{
	if (!m_editService || !m_store || m_boundPathPlanId.empty())
	{
		return true;
	}
	auto cmd = std::make_shared<RobotInstruction::UpdatePathPlanPipelineCommand>(
		m_boundPathPlanId,
		m_ops,
		m_accumulatedGeometryOps);
	return m_editService->execute(cmd, outError);
}

bool TrajectoryEditSession::syncPipelineToBoundPathPlan()
{
	if (!m_store || m_boundPathPlanId.empty())
	{
		return false;
	}
	RobotInstruction::PathPlanInstruction* pp = m_store->activeCatalog().findPathPlan(
		m_store->activeCatalog().activeProgramId(),
		m_boundPathPlanId);
	if (!pp)
	{
		return false;
	}
	pp->setPipeline(m_ops);
	pp->appliedHistoryMut() = m_accumulatedGeometryOps;
	if (m_rawTrajectory.has_value())
	{
		m_store->activeCatalog().pathPlanRaws().save(m_boundPathPlanId, *m_rawTrajectory);
	}
	return true;
}

bool TrajectoryEditSession::loadRawFromBoundPathPlan()
{
	if (!m_store || m_boundPathPlanId.empty())
	{
		return false;
	}
	RobotInstruction::RawTrajectory raw;
	if (!m_store->activeCatalog().pathPlanRaws().load(m_boundPathPlanId, raw))
	{
		return false;
	}
	m_rawTrajectory = std::move(raw);
	m_bakedWorldRaw.reset();
	emit rawTrajectoryChanged();
	return true;
}

namespace
{
std::string makeUniquePathPlanName(
	const RobotInstruction::RawTrajectory& traj,
	const RobotInstruction::RobotProgramCatalog& catalog,
	const std::string& programId)
{
	std::string base = RobotInstruction::rawTrajectoryFeatureId(traj);
	if (base.empty())
	{
		base = "path_plan";
	}
	std::unordered_set<std::string> used;
	if (const RobotInstruction::RobotProgram* prog = catalog.findProgram(programId))
	{
		for (const std::shared_ptr<RobotInstruction::Base>& ins : prog->steps)
		{
			if (!ins || ins->type() != RobotInstruction::Type::PathPlan)
			{
				continue;
			}
			if (const RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(*ins))
			{
				used.insert(pp->name());
			}
		}
	}
	if (used.count(base) == 0)
	{
		return base;
	}
	for (int suffix = 2; suffix < 1000; ++suffix)
	{
		const std::string candidate = base + "_" + std::to_string(suffix);
		if (used.count(candidate) == 0)
		{
			return candidate;
		}
	}
	return base + "_" + std::to_string(used.size() + 1);
}
} // namespace

void TrajectoryEditSession::setRawTrajectory(RobotInstruction::RawTrajectory traj)
{
	clearTrajectoryGeometryHistory();
	m_rawTrajectory = std::move(traj);
	m_bakedWorldRaw.reset();
	if (m_store && m_boundPathPlanId.empty())
	{
		auto pathPlan = std::make_shared<RobotInstruction::PathPlanInstruction>();
		pathPlan->setSourceFeatureJson(m_rawTrajectory->sourceFeatureJson);
		pathPlan->setRawTrajectoryKey(pathPlan->id());
		pathPlan->setName(
			makeUniquePathPlanName(*m_rawTrajectory, m_store->activeCatalog(), m_store->activeProgramIdUtf8()));
		m_boundPathPlanId = pathPlan->id();
		size_t insertIdx = 0;
		for (const std::shared_ptr<RobotInstruction::Base>& step : m_store->activeProgram())
		{
			if (step && step->type() == RobotInstruction::Type::PathPlan)
			{
				++insertIdx;
			}
		}
		if (m_editService)
		{
			std::vector<RobotInstruction::ProgramEditStack::CommandPtr> cmds;
			cmds.push_back(std::make_shared<RobotInstruction::InsertPathPlanCommand>(pathPlan, insertIdx));
			cmds.push_back(std::make_shared<RobotInstruction::UpdatePathPlanRawCommand>(
				&m_store->activeCatalog(),
				m_boundPathPlanId,
				*m_rawTrajectory,
				RobotInstruction::PathPlanPhase::RawReady));
			QString err;
			(void)m_editService->executeBatch(cmds, &err);
		}
		else
		{
			m_store->activeProgram().insert(m_store->activeProgram().begin(), pathPlan);
			m_store->activeCatalog().pathPlanRaws().save(m_boundPathPlanId, *m_rawTrajectory);
		}
	}
	else if (m_store && !m_boundPathPlanId.empty())
	{
		RobotInstruction::RobotProgramCatalog& catalog = m_store->activeCatalog();
		if (m_editService)
		{
			auto cmd = std::make_shared<RobotInstruction::UpdatePathPlanRawCommand>(
				&catalog,
				m_boundPathPlanId,
				*m_rawTrajectory,
				RobotInstruction::PathPlanPhase::RawReady);
			QString err;
			(void)m_editService->execute(cmd, &err);
		}
		else
		{
			catalog.pathPlanRaws().save(m_boundPathPlanId, *m_rawTrajectory);
		}
		if (RobotInstruction::PathPlanInstruction* pp = catalog.findPathPlan(
				catalog.activeProgramId(),
				m_boundPathPlanId))
		{
			pp->appliedHistoryMut().clear();
			if (!m_editService)
			{
				pp->setPhase(RobotInstruction::PathPlanPhase::RawReady);
				pp->bumpRawRevision();
			}
			pp->setSourceFeatureJson(m_rawTrajectory->sourceFeatureJson);
			if (pp->name().empty())
			{
				pp->setName(makeUniquePathPlanName(
					*m_rawTrajectory,
					catalog,
					catalog.activeProgramId()));
			}
			if (pp->rawTrajectoryKey().empty())
			{
				pp->setRawTrajectoryKey(m_boundPathPlanId);
			}
		}
	}
	emit rawTrajectoryChanged();
}

const RobotInstruction::RawTrajectory* TrajectoryEditSession::rawTrajectory() const
{
	return m_rawTrajectory ? &*m_rawTrajectory : nullptr;
}

bool TrajectoryEditSession::hasRawTrajectory() const
{
	return m_rawTrajectory.has_value() && !m_rawTrajectory->points.empty();
}

void TrajectoryEditSession::clearRawTrajectory()
{
	if (!m_rawTrajectory.has_value() && !m_bakedWorldRaw.has_value())
	{
		return;
	}
	m_rawTrajectory.reset();
	m_bakedWorldRaw.reset();
	emit rawTrajectoryChanged();
}

bool TrajectoryEditSession::buildRawPreviewWithPipeline(
	const std::vector<RobotInstruction::TrajectoryOpDescriptor>& pipelineOps,
	RobotInstruction::RawTrajectory& outPreviewRaw,
	QString* outError) const
{
	if (!m_rawTrajectory.has_value() || m_rawTrajectory->points.empty())
	{
		if (outError)
		{
			*outError = QStringLiteral("无原始轨迹");
		}
		return false;
	}
	std::vector<RobotInstruction::TrajectoryOpDescriptor> recipeOps;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> geometryOps;
	recipeOps.reserve(pipelineOps.size());
	geometryOps.reserve(pipelineOps.size());
	for (const RobotInstruction::TrajectoryOpDescriptor& op : pipelineOps)
	{
		if (RobotInstruction::isRecipeOpKind(op.kind))
		{
			recipeOps.push_back(op);
		}
		else
		{
			geometryOps.push_back(op);
		}
	}
	RobotInstruction::RawTrajectory rawWorking = *m_rawTrajectory;
	RobotInstruction::UnifiedTrajectory unified{};
	if (!rebuildUnifiedFromSourceRaw(rawWorking, unified, outError))
	{
		return false;
	}
	std::string err;
	auto applyUnifiedOps = [&](const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops) -> bool {
		for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
		{
			if (!RobotInstruction::applyUnifiedTrajectoryOp(op, unified, &err))
			{
				if (outError)
				{
					*outError = err.empty() ? QStringLiteral("unified preview failed") : QString::fromStdString(err);
				}
				return false;
			}
		}
		return true;
	};
	if (!applyUnifiedOps(m_pendingPreRawGeometryOps))
	{
		return false;
	}
	for (const RobotInstruction::TrajectoryOpDescriptor& op : recipeOps)
	{
		if (!RobotInstruction::applyRecipeDescriptorToRawTrajectory(op, rawWorking, &err))
		{
			if (outError)
			{
				*outError = err.empty() ? QStringLiteral("recipe preview failed") : QString::fromStdString(err);
			}
			return false;
		}
		if (!rebuildUnifiedFromSourceRaw(rawWorking, unified, outError))
		{
			return false;
		}
	}
	if (!applyUnifiedOps(m_accumulatedGeometryOps))
	{
		return false;
	}
	if (!applyUnifiedOps(geometryOps))
	{
		return false;
	}
	RobotInstruction::RawTrajectory worldPreview{};
	if (!RobotInstruction::unifiedTrajectoryToRaw(unified, worldPreview, &err))
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("unified->raw failed") : QString::fromStdString(err);
		}
		return false;
	}
	outPreviewRaw = std::move(worldPreview);
	return true;
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
	std::vector<std::shared_ptr<RobotInstruction::Base>>& activeSteps = m_store->activeProgram();
	InstructionIndex index = buildInstructionIndex(activeSteps);
	for (const std::string& id : m_effectivePreviewWaypointIds)
	{
		const auto it = index.find(id);
		if (it == index.end() || !it->second)
		{
			continue;
		}
		RobotInstruction::Base* raw = it->second.get();
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
	std::vector<std::shared_ptr<RobotInstruction::Base>>& activeSteps = m_store->activeProgram();
	InstructionIndex index = buildInstructionIndex(activeSteps);
	std::vector<std::string> changedIds;
	changedIds.reserve(m_previewSnapshots.size());
	for (const PreviewSnapshot& snap : m_previewSnapshots)
	{
		const auto it = index.find(snap.id);
		if (it == index.end() || !it->second)
		{
			continue;
		}
		RobotInstruction::Base* raw = it->second.get();
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
		changedIds.push_back(snap.id);
	}
	if (!changedIds.empty())
	{
		syncPreviewRenderMatrices(&changedIds);
	}
	return true;
}

void TrajectoryEditSession::syncRenderMatricesForInstructionIds(
	const std::vector<std::string>& ids,
	const bool worldFrameTcp)
{
	if (!m_simController || !m_store || ids.empty())
	{
		return;
	}
	InstructionIndex index = buildInstructionIndex(m_store->activeProgram());
	for (const std::string& id : ids)
	{
		const auto it = index.find(id);
		if (it != index.end() && it->second)
		{
			if (worldFrameTcp)
			{
				m_simController->syncInstructionRenderMatricesFromWorldPose(it->second);
			}
			else
			{
				m_simController->syncInstructionRenderMatricesFromPose(it->second);
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

void TrajectoryEditSession::syncPreviewRenderMatrices(const std::vector<std::string>* updatedIds)
{
	if (m_previewSnapshots.empty() || !m_store)
	{
		return;
	}
	std::unordered_set<std::string> filterIds;
	if (updatedIds && !updatedIds->empty())
	{
		filterIds.insert(updatedIds->begin(), updatedIds->end());
	}
	InstructionIndex index = buildInstructionIndex(m_store->activeProgram());
	for (const PreviewSnapshot& snap : m_previewSnapshots)
	{
		if (!filterIds.empty() && filterIds.count(snap.id) == 0)
		{
			continue;
		}
		const auto it = index.find(snap.id);
		if (it == index.end() || !it->second)
		{
			continue;
		}
		RobotInstruction::Base* raw = it->second.get();
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
	InstructionIndex index = buildInstructionIndex(m_store->activeProgram());
	for (const std::string& id : ids)
	{
		const auto itBase = frozenBaseWorldCsvById.find(id);
		if (itBase == frozenBaseWorldCsvById.end())
		{
			continue;
		}
		const auto itRaw = index.find(id);
		if (itRaw == index.end() || !itRaw->second)
		{
			continue;
		}
		RobotInstruction::Base* raw = itRaw->second.get();
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
	InstructionIndex index = buildInstructionIndex(m_store->activeProgram());
	for (const PreviewSnapshot& snap : m_previewSnapshots)
	{
		const auto it = index.find(snap.id);
		if (it == index.end() || !it->second)
		{
			continue;
		}
		RobotInstruction::Base* raw = it->second.get();
		raw->setPose(snap.pose);
		if (raw->hasEulerProperty())
		{
			raw->setEulerDeg(snap.euler);
		}
		// 预览会写入 context.targetTransform*；快照里若不存在，必须显式清掉，避免 pose 与 target 真值不一致。
		const auto itTargetQ = snap.extensions.find(RobotInstruction::kExtContextTargetTransformQuatCsv);
		if (itTargetQ == snap.extensions.end())
		{
			raw->eraseExtensionProperty(RobotInstruction::kExtContextTargetTransformQuatCsv);
		}
		const auto itTargetT = snap.extensions.find(RobotInstruction::kExtContextTargetTransformTransMmCsv);
		if (itTargetT == snap.extensions.end())
		{
			raw->eraseExtensionProperty(RobotInstruction::kExtContextTargetTransformTransMmCsv);
		}
		for (const auto& kv : snap.extensions)
		{
			raw->setExtensionProperty(kv.first, kv.second);
		}
	}
}

void TrajectoryEditSession::updateLightweightPreviewState(const bool active)
{
	if (m_lightweightPreviewActive == active)
	{
		return;
	}
	m_lightweightPreviewActive = active;
	if (!m_simController)
	{
		return;
	}
	IRobotMainWindowHost* host = m_simController->host();
	if (!host)
	{
		return;
	}
	if (active)
	{
		host->appendRunInfo(QStringLiteral("轨迹点过多，当前使用轻预览模式"));
	}
	else
	{
		host->appendRunInfo(QStringLiteral("已恢复完整预览模式"));
	}
}
