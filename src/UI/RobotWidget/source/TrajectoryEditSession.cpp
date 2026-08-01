/// @file TrajectoryEditSession.cpp
/// @brief TrajectoryEditSession 实现

#include "TrajectoryEditSession.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "FeaturePickTransform.h"
#include "FrameBackendData.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "InstructionProgramDocument.h"
#include "ProgramEditCommand.h"
#include "RawTrajectory.h"
#include "RecipeBlueprint.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"
#include "RobotExternalAxes.h"
#include "RobotSimulationController.h"
#include "RobotSimulationMath.h"
#include "RunLogger.h"
#include "SimulationCommandWidget.h"
#include "TrajectoryGeometryResolver.h"
#include "TrajectoryGeometryResolverHost.h"
#include "TrajectoryOpBridge.h"
#include "TrajectoryPathAdapters.h"
#include "UnifiedTrajectory.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <Adapters.h>
#include <ITrajectoryOp.h>
#include <RigidTransform.h>
#include <osg/Matrixd>

namespace
{
using InstructionIndex = std::unordered_map<std::string, std::shared_ptr<RobotInstruction::Base>>;
constexpr size_t kLightweightPreviewGuardThreshold = 2000;
constexpr size_t kLightweightPreviewMaxWaypoints = 1200;

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

bool opUsesPoseScopePreview(const trajectory_algo::ITrajectoryOp& algo)
{
	return trajectory_algo::hasCapability(algo.capabilities(),
										  trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform);
}

bool rigidBaseWorldFromSnapshot(const osg::Matrixd& snapWorld, const osg::Matrixd& snapLocal,
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

bool isUnifiedPathOpKind(const RobotInstruction::TrajectoryOpKind kind)
{
	switch (kind)
	{
	case RobotInstruction::TrajectoryOpKind::Approach:
	case RobotInstruction::TrajectoryOpKind::Retract:
	case RobotInstruction::TrajectoryOpKind::Resample:
	case RobotInstruction::TrajectoryOpKind::OffsetAlongNormal:
	case RobotInstruction::TrajectoryOpKind::OffsetLateral:
	case RobotInstruction::TrajectoryOpKind::SmoothPose:
	case RobotInstruction::TrajectoryOpKind::AssignBlend:
	case RobotInstruction::TrajectoryOpKind::AssignSpeedZone:
	case RobotInstruction::TrajectoryOpKind::Weave:
	case RobotInstruction::TrajectoryOpKind::ReachabilityFilter:
	case RobotInstruction::TrajectoryOpKind::ExternalAxisSearch:
	case RobotInstruction::TrajectoryOpKind::Delete:
	case RobotInstruction::TrajectoryOpKind::Duplicate:
	case RobotInstruction::TrajectoryOpKind::ProjectToGeometry:
	case RobotInstruction::TrajectoryOpKind::NonRigidRegistration:
		return true;
	default:
		return false;
	}
}

void mergeOpsInto(std::vector<RobotInstruction::TrajectoryOpDescriptor>& target,
				  const std::vector<RobotInstruction::TrajectoryOpDescriptor>& source)
{
	for (const auto& srcOp : source)
	{
		if (srcOp.opId.empty())
		{
			target.push_back(srcOp);
			continue;
		}
		auto it = std::find_if(target.begin(), target.end(), [&](const auto& t) { return t.opId == srcOp.opId; });
		if (it != target.end())
		{
			*it = srcOp;
		}
		else
		{
			target.push_back(srcOp);
		}
	}
}

bool pipelineNeedsOverlayPreview(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		if (!op.enabled)
		{
			continue;
		}
		switch (op.kind)
		{
		case RobotInstruction::TrajectoryOpKind::Resample:
		case RobotInstruction::TrajectoryOpKind::Approach:
		case RobotInstruction::TrajectoryOpKind::Retract:
		case RobotInstruction::TrajectoryOpKind::Delete:
		case RobotInstruction::TrajectoryOpKind::Duplicate:
		case RobotInstruction::TrajectoryOpKind::OffsetAlongNormal:
		case RobotInstruction::TrajectoryOpKind::OffsetLateral:
		case RobotInstruction::TrajectoryOpKind::SmoothPose:
		case RobotInstruction::TrajectoryOpKind::Weave:
		case RobotInstruction::TrajectoryOpKind::ProjectToGeometry:
			return true;
		default:
			break;
		}
	}
	return false;
}

bool pipelineHasProjectToGeometry(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		if (op.enabled && op.kind == RobotInstruction::TrajectoryOpKind::ProjectToGeometry)
		{
			return true;
		}
	}
	return false;
}

bool pipelineHasNonRigidRegistration(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		if (op.enabled && op.kind == RobotInstruction::TrajectoryOpKind::NonRigidRegistration)
		{
			return true;
		}
	}
	return false;
}

bool pipelineNeedsPoseScopePreview(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		if (!op.enabled)
		{
			continue;
		}
		const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(op.kind);
		if (algo && trajectory_algo::hasCapability(algo->capabilities(),
												   trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform))
		{
			return true;
		}
	}
	return false;
}

bool pipelineNeedsPropertyWritebackPreview(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	for (const RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		if (!op.enabled)
		{
			continue;
		}
		if (op.kind == RobotInstruction::TrajectoryOpKind::AssignBlend ||
			op.kind == RobotInstruction::TrajectoryOpKind::AssignSpeedZone)
		{
			return true;
		}
	}
	return false;
}

RobotInstruction::UnifiedTrajectory downsampleUnifiedForOverlay(const RobotInstruction::UnifiedTrajectory& unified)
{
	if (unified.points.size() <= kLightweightPreviewMaxWaypoints)
	{
		return unified;
	}
	RobotInstruction::UnifiedTrajectory out = unified;
	out.points.clear();
	out.points.reserve(kLightweightPreviewMaxWaypoints);
	const size_t stride = std::max<size_t>(1, unified.points.size() / kLightweightPreviewMaxWaypoints);
	for (size_t i = 0; i < unified.points.size(); i += stride)
	{
		out.points.push_back(unified.points[i]);
		if (out.points.size() >= kLightweightPreviewMaxWaypoints)
		{
			break;
		}
	}
	return out;
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

void restoreRenderExtensionsFromSnapshot(RobotInstruction::Base& raw,
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

TrajectoryEditSession::TrajectoryEditSession(QObject* parent) : QObject(parent) {}

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
		connect(m_editService, &ProgramEditService::revisionChanged, this,
				[this](const int revision)
				{
					m_programRevision = revision;
					invalidatePreviewScopeCache();
				});
	}
}

void TrajectoryEditSession::bindSimulationController(RobotSimulationController* controller)
{
	m_simController = controller;
	ensureGeometryResolverBound();
}

void TrajectoryEditSession::ensureGeometryResolverBound() const
{
	if (!m_simController)
	{
		trajectory_geometry_host::clearTrajectoryGeometryResolverBinding();
		return;
	}
	IRobotMainWindowHost* host = m_simController->host();
	if (!host)
	{
		trajectory_geometry_host::clearTrajectoryGeometryResolverBinding();
		return;
	}
	trajectory_geometry_host::bindTrajectoryGeometryResolver(host->document(), host->osgView());
}

void TrajectoryEditSession::injectWorkpieceReferenceOnEngine() const
{
	if (!m_simController)
	{
		m_pipelineEngine.setWorkpieceReferenceInBase(nullptr);
		m_pipelineEngine.setExternalTcpFrameResolver(nullptr);
		m_pipelineEngine.setExternalAxisSearchService(nullptr);
		m_pipelineEngine.setExternalAxisConfigs({});
		return;
	}
	RobotInstruction::Vec3 poseMm{};
	RobotInstruction::Vec3 eulerDeg{};
	if (!m_simController->tryCaptureCurrentRobotTcpPose(poseMm, eulerDeg, nullptr, nullptr, nullptr, nullptr))
	{
		m_pipelineEngine.setWorkpieceReferenceInBase(nullptr);
	}
	else
	{
		const engine::RigidTransform ref = engine::RigidTransform::fromTranslationEulerDeg(
			poseMm.x, poseMm.y, poseMm.z, eulerDeg.x, eulerDeg.y, eulerDeg.z);
		m_pipelineEngine.setWorkpieceReferenceInBase(&ref);
	}

	IRobotMainWindowHost* host = m_simController->host();
	if (!host || !host->document())
	{
		m_pipelineEngine.setExternalTcpFrameResolver(nullptr);
		return;
	}
	BackendDataManager* mgr = &host->document()->backend();
	m_pipelineEngine.setExternalTcpFrameResolver(
		[mgr](const std::string& backendId, engine::RigidTransform& out, std::string* errMsg) -> bool
		{
			const std::shared_ptr<BackendDataBase> data = mgr->getData(backendId);
			if (!data || !std::dynamic_pointer_cast<FrameBackendData>(data))
			{
				if (errMsg)
				{
					*errMsg = "external TCP frame backend not found: " + backendId;
				}
				return false;
			}
			const BackendMat4 world = data->worldMatrix(mgr);
			engine::ColMajorMat4 cm{};
			for (int i = 0; i < 16; ++i)
			{
				cm[static_cast<size_t>(i)] = world.v[i];
			}
			out = engine::rigidTransformFromColMajor(cm);
			return true;
		});
	injectExternalAxisSearchOnEngine();
}

void TrajectoryEditSession::injectExternalAxisSearchOnEngine() const
{
	if (!m_simController || !m_simController->host())
	{
		m_pipelineEngine.setExternalAxisSearchService(nullptr);
		m_pipelineEngine.setExternalAxisConfigs({});
		return;
	}
	IRobotDocumentHost* doc = m_simController->host()->document();
	SimulationCommandWidget* cmdPage = m_simController->host()->simulationCommandPage();
	if (!doc || !cmdPage)
	{
		m_pipelineEngine.setExternalAxisSearchService(nullptr);
		m_pipelineEngine.setExternalAxisConfigs({});
		return;
	}
	const int instIdx = cmdPage->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		m_pipelineEngine.setExternalAxisSearchService(nullptr);
		m_pipelineEngine.setExternalAxisConfigs({});
		return;
	}
	const RobotExternal::RobotExternalAxisConfigSet& set = doc->robotExternalAxesForInstance(instIdx);
	std::vector<trajectory_algo::ExternalAxisSearchConfigDto> dtos;
	for (const RobotExternal::RobotExternalAxisConfig& a : set.axes)
	{
		if (!a.enabled)
		{
			continue;
		}
		trajectory_algo::ExternalAxisSearchConfigDto d;
		d.enabled = true;
		d.jointName = a.jointName;
		d.isPrismatic = a.isPrismatic;
		d.lower = a.lower;
		d.upper = a.upper;
		d.home = a.home;
		d.axis[0] = a.axis[0];
		d.axis[1] = a.axis[1];
		d.axis[2] = a.axis[2];
		dtos.push_back(d);
	}
	m_pipelineEngine.setExternalAxisConfigs(std::move(dtos));
	if (!RobotExternal::hasEnabledExternalAxes(set))
	{
		m_pipelineEngine.setExternalAxisSearchService(nullptr);
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	const QString comboTcp = cmdPage->selectedTcpLink();
	const QString tcp = RobotSimulationMath::defaultTcpLinkNameForUrdf(urdfPath, comboTcp);
	const QVector<double> agg = m_simController->aggregatedJointAnglesRad();
	const int offset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	std::vector<double> seed;
	seed.reserve(static_cast<size_t>(nj));
	for (int j = 0; j < nj; ++j)
	{
		const int idx = offset + j;
		seed.push_back((idx >= 0 && idx < agg.size()) ? agg[idx] : 0.0);
	}
	m_externalAxisSearchService.setRobotContext(urdfPath, tcp, seed);
	m_pipelineEngine.setExternalAxisSearchService(&m_externalAxisSearchService);
}

void TrajectoryEditSession::reportProjectionMissesIfAny() const
{
	if (!pipelineHasProjectToGeometry(m_ops) || !m_simController)
	{
		return;
	}
	IRobotMainWindowHost* host = m_simController->host();
	if (!host)
	{
		return;
	}
	const std::size_t misses = RobotInstruction::trajectoryProjectionMissCount();
	if (misses == 0)
	{
		return;
	}
	host->appendRunWarning(
		QStringLiteral("轨迹投影：%1 个点未命中几何，已保留原位置").arg(static_cast<qulonglong>(misses)));
}

void TrajectoryEditSession::reportNonRigidStatsIfAny() const
{
	if (!pipelineHasNonRigidRegistration(m_ops) || !m_simController)
	{
		return;
	}
	IRobotMainWindowHost* host = m_simController->host();
	if (!host)
	{
		return;
	}
	const RobotInstruction::NonRigidWarpLastStats stats = RobotInstruction::trajectoryNonRigidLastStats();
	if (!stats.valid)
	{
		return;
	}
	QString msg =
		QStringLiteral("非刚性配准：绑定成功 %1，失败 %2；模式=%3；绑定距离 min/mean/max=%4/%5/%6 mm；"
					   "SPARE 均值误差 %7 mm，变形节点 %8")
			.arg(static_cast<qulonglong>(stats.bindOk))
			.arg(static_cast<qulonglong>(stats.bindFail))
			.arg(stats.bindMode == 1 ? QStringLiteral("工件模型系") : QStringLiteral("世界系"))
			.arg(stats.bindDistMinMm, 0, 'f', 3)
			.arg(stats.bindDistMeanMm, 0, 'f', 3)
			.arg(stats.bindDistMaxMm, 0, 'f', 3)
			.arg(stats.meanErrorMm, 0, 'f', 3)
			.arg(stats.deformationNodeCount);
	if (stats.spareFromCache)
	{
		msg += QStringLiteral("（缓存）");
	}
	host->appendRunInfo(msg);
}

void TrajectoryEditSession::updatePipelineOps(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops,
											  const bool /*allowPreviewReapply*/)
{
	// 草稿期只改 session；PathPlan.pipeline 由 Apply Command 提交，避免 before==after
	m_ops = std::move(ops);
	invalidatePreviewScopeCache();
}

void TrajectoryEditSession::setPipeline(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops)
{
	reset();
	m_ops = std::move(ops);
	invalidatePreviewScopeCache();
}

void TrajectoryEditSession::replacePipelineOpsFromStore(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops)
{
	m_ops = std::move(ops);
	invalidatePreviewScopeCache();
}

void TrajectoryEditSession::setContextProgramId(const std::string& programId)
{
	m_contextProgramId = programId;
	invalidatePreviewScopeCache();
	if (m_store)
	{
		m_store->setActiveProgramIdUtf8(programId);
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
	if (m_previewWaypointCacheValid && m_previewWaypointCacheProgramId == programId &&
		m_previewWaypointCacheRevision == m_programRevision)
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
		const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(op.kind);
		if (!algo || !opUsesPoseScopePreview(*algo))
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

bool TrajectoryEditSession::ingressProgramUnified(const RobotInstruction::RobotProgram& program,
												  RobotInstruction::UnifiedTrajectory& unified,
												  std::string* errMsg) const
{
	if (!m_boundPathPlanId.empty() && !hasRawTrajectory())
	{
		return RobotInstruction::ingressUnifiedForEdit(program, m_boundPathPlanId, unified, errMsg);
	}
	return RobotInstruction::ingressUnifiedFromProgram(program, unified, errMsg);
}

void TrajectoryEditSession::invalidatePreviewScopeCache()
{
	m_previewWaypointCacheValid = false;
	m_previewWaypointCacheProgramId.clear();
	m_previewWaypointCacheRevision = -1;
	m_previewWaypointCache.clear();
}

bool TrajectoryEditSession::rebuildUnifiedFromSourceRaw(const RobotInstruction::RawTrajectory& sourceRaw,
														RobotInstruction::UnifiedTrajectory& unified,
														QString* outError) const
{
	RobotInstruction::RawTrajectory rawForUnified = sourceRaw;
	// 有 Feature/Mesh spec 时必须 file→world，否则非刚性等算子会拿模型坐标去绑世界几何
	if (!sourceRaw.sourceFeatureJson.empty())
	{
		const std::string backendId = RobotInstruction::rawTrajectoryWorkpieceBackendId(sourceRaw);
		if (backendId.empty())
		{
			if (outError)
			{
				*outError = QStringLiteral("轨迹缺少工件 backendId，无法变换到世界坐标");
			}
			return false;
		}
		if (!m_simController || !m_simController->host() || !m_simController->host()->osgView())
		{
			if (outError)
			{
				*outError = QStringLiteral("无 OSG 视图，无法将轨迹变换到世界坐标");
			}
			return false;
		}
		RobotInstruction::RawTrajectory worldRaw;
		std::string worldErr;
		if (!feature_pick_transform::transformRawTrajectoryToWorld(m_simController->host()->osgView(), backendId,
																  sourceRaw, worldRaw, &worldErr))
		{
			if (outError)
			{
				*outError = worldErr.empty() ? QStringLiteral("轨迹变换到世界坐标失败")
											: QString::fromStdString(worldErr);
			}
			return false;
		}
		rawForUnified = std::move(worldRaw);
		RunLogger::info(std::string("[轨迹入管] file→world 完成 backend=") + backendId +
						" points=" + std::to_string(rawForUnified.points.size()));
	}
	std::string convErr;
	if (!RobotInstruction::unifiedTrajectoryFromRaw(rawForUnified, unified, &convErr))
	{
		if (outError)
		{
			*outError =
				convErr.empty() ? QStringLiteral("raw trajectory convert failed") : QString::fromStdString(convErr);
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
	if (!m_overlayPreviewActive)
	{
		m_simController->refreshInstructionPoseAxes(false);
	}
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
	if (!m_previewActive)
	{
		return false;
	}
	if (m_rawTrajectory.has_value() && !m_rawTrajectory->points.empty())
	{
		return false;
	}
	return previewUnifiedFromProgramPipeline(outError);
}

bool TrajectoryEditSession::previewPipeline(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& pipelineOps,
											QString* outError)
{
	m_ops = pipelineOps;
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
	return previewUnifiedFromProgramPipeline(outError);
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
	bool hasGeometryPreview = false;
	for (const RobotInstruction::TrajectoryOpDescriptor& op : m_ops)
	{
		if (!op.enabled)
		{
			continue;
		}
		const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(op.kind);
		if (algo && (trajectory_algo::hasCapability(algo->capabilities(),
													trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform) ||
					 isUnifiedPathOpKind(op.kind)))
		{
			hasGeometryPreview = true;
			break;
		}
	}
	if (!hasGeometryPreview)
	{
		if (outError)
		{
			*outError = QStringLiteral("当前流水线无可预览块");
		}
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
	std::vector<std::string> waypointIds;
	if (pipelineNeedsPoseScopePreview(m_ops))
	{
		waypointIds = collectPreviewWaypointIds();
		if (waypointIds.empty())
		{
			if (outError)
			{
				*outError = QStringLiteral("作用域内无运动路点，请先在程序中创建路点或完成轨迹离散");
			}
			return false;
		}
	}
	else
	{
		RobotInstruction::UnifiedTrajectory ingressProbe{};
		std::string ingressErr;
		if (!ingressProgramUnified(*prog, ingressProbe, &ingressErr))
		{
			if (outError)
			{
				*outError = ingressErr.empty()
								? QStringLiteral("作用域内无运动路点，请先在程序中创建路点或完成轨迹离散")
								: QString::fromStdString(ingressErr);
			}
			return false;
		}
		waypointIds.reserve(ingressProbe.points.size());
		for (const RobotInstruction::UnifiedTrajectoryPoint& point : ingressProbe.points)
		{
			if (!point.sourceInstructionId.empty())
			{
				waypointIds.push_back(point.sourceInstructionId);
			}
		}
		if (waypointIds.empty())
		{
			waypointIds = collectMotionIds(*prog);
		}
		if (waypointIds.empty())
		{
			if (outError)
			{
				*outError = QStringLiteral("作用域内无运动路点，请先在程序中创建路点或完成轨迹离散");
			}
			return false;
		}
	}
	m_effectivePreviewWaypointIds = waypointIds;
	if (m_previewActive)
	{
		if ((!m_overlayPreviewActive || m_overlayStoreWritebackActive) && !m_previewSnapshots.empty())
		{
			restorePreviewSnapshots();
		}
		clearOverlayPreview();
		m_overlayStoreWritebackActive = false;
	}
	RobotInstruction::UnifiedTrajectory unified{};
	std::string err;
	if (!ingressProgramUnified(*prog, unified, &err))
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("program convert failed") : QString::fromStdString(err);
		}
		return false;
	}
	m_pipelineEngine.clear();
	m_pipelineEngine.setProgramContext(prog);
	m_pipelineEngine.setUsingRaw(false);
	m_pipelineEngine.setUnifiedBaseline(unified);
	m_pipelineEngine.setOps(m_ops);
	ensureGeometryResolverBound();
	injectWorkpieceReferenceOnEngine();
	if (!m_pipelineEngine.executeFull(&err))
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("unified preview failed") : QString::fromStdString(err);
		}
		return false;
	}
	reportProjectionMissesIfAny();
	reportNonRigidStatsIfAny();
	unified = m_pipelineEngine.result();
	const bool lightweight = unified.points.size() > kLightweightPreviewGuardThreshold;
	updateLightweightPreviewState(lightweight);
	const bool writePose = pipelineNeedsPoseScopePreview(m_ops);
	const bool writeBlendSpeed = pipelineNeedsPropertyWritebackPreview(m_ops);
	if (pipelineNeedsOverlayPreview(m_ops))
	{
		m_overlayPreviewActive = true;
		const RobotInstruction::UnifiedTrajectory overlayUnified =
			lightweight ? downsampleUnifiedForOverlay(unified) : unified;
		if (!showUnifiedOverlayPreview(overlayUnified, outError))
		{
			m_overlayPreviewActive = false;
			return false;
		}
		// 几何走 overlay，位姿/工艺属性仍写回 store，与 Apply 字段一致
		if (writePose || writeBlendSpeed)
		{
			if (!capturePreviewSnapshots(outError))
			{
				clearOverlayPreview();
				return false;
			}
			std::vector<std::string> changedIds;
			if (!applyUnifiedPreviewWriteback(unified, writePose, writeBlendSpeed, changedIds))
			{
				restorePreviewSnapshots();
				clearOverlayPreview();
				clearPreviewSnapshots();
				if (outError)
				{
					*outError = QStringLiteral("混合预览写回失败");
				}
				return false;
			}
			if (!changedIds.empty())
			{
				syncRenderMatricesForInstructionIds(changedIds, true);
			}
			m_overlayStoreWritebackActive = true;
		}
		m_previewActive = true;
		emit previewStateChanged(true);
		refreshPreviewVisuals();
		return true;
	}
	m_overlayPreviewActive = false;
	m_overlayStoreWritebackActive = false;
	if (!capturePreviewSnapshots(outError))
	{
		return false;
	}
	std::vector<std::string> changedIds;
	if (!applyUnifiedPreviewWriteback(unified, true, true, changedIds))
	{
		restorePreviewSnapshots();
		clearPreviewSnapshots();
		if (outError)
		{
			*outError = QStringLiteral("预览写回失败");
		}
		return false;
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

void TrajectoryEditSession::clearOverlayPreview()
{
	if (!m_overlayPreviewActive)
	{
		return;
	}
	if (m_simController)
	{
		m_simController->setRawTrajectoryPreviewActive(false);
		if (IRobotMainWindowHost* host = m_simController->host())
		{
			if (IRobotOsgViewHost* osg = host->osgView())
			{
				osg->clearRawTrajectoryOverlay();
				osg->clearRawTrajectoryOverlayFrames();
				osg->requestRedraw();
			}
		}
	}
	m_overlayPreviewActive = false;
}

bool TrajectoryEditSession::applyUnifiedPreviewWriteback(const RobotInstruction::UnifiedTrajectory& unified,
														 const bool writePose, const bool writeBlendSpeed,
														 std::vector<std::string>& outChangedIds)
{
	outChangedIds.clear();
	if (!m_store || (!writePose && !writeBlendSpeed))
	{
		return true;
	}
	std::vector<std::shared_ptr<RobotInstruction::Base>>& activeSteps = m_store->activeProgram();
	InstructionIndex index = buildInstructionIndex(activeSteps);
	outChangedIds.reserve(unified.points.size());
	for (const RobotInstruction::UnifiedTrajectoryPoint& point : unified.points)
	{
		if (point.sourceInstructionId.empty())
		{
			continue;
		}
		const auto it = index.find(point.sourceInstructionId);
		if (it == index.end() || !it->second)
		{
			continue;
		}
		RobotInstruction::Base& ins = *it->second;
		if (writePose)
		{
			const engine::RigidTransform target = engine::RigidTransform::fromTranslationEulerDeg(
				point.poseMm.x, point.poseMm.y, point.poseMm.z, point.eulerDeg.x, point.eulerDeg.y, point.eulerDeg.z);
			RobotInstruction::writeTargetTransformToInstruction(ins, target);
			ins.eraseExtensionProperty("context.currentJointRadCsv");
			ins.eraseExtensionProperty("render.tcpWorldMat4");
			ins.eraseExtensionProperty("render.tcpLocalMat4");
		}
		if (writeBlendSpeed)
		{
			ins.setBlendRadius(point.blendRadiusMm);
			if (point.speedMmPerSec > 0.0)
			{
				ins.setSpeed(point.speedMmPerSec);
			}
		}
		outChangedIds.push_back(point.sourceInstructionId);
	}
	return true;
}

bool TrajectoryEditSession::showUnifiedOverlayPreview(const RobotInstruction::UnifiedTrajectory& unified,
													  QString* outError)
{
	RobotInstruction::RawTrajectory worldPreview{};
	std::string err;
	if (!RobotInstruction::unifiedTrajectoryToRaw(unified, worldPreview, &err))
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("unified->raw failed") : QString::fromStdString(err);
		}
		return false;
	}
	if (!m_simController)
	{
		if (outError)
		{
			*outError = QStringLiteral("simulation controller not bound");
		}
		return false;
	}
	IRobotMainWindowHost* host = m_simController->host();
	IRobotOsgViewHost* osg = host ? host->osgView() : nullptr;
	if (!osg)
	{
		if (outError)
		{
			*outError = QStringLiteral("osg view not available");
		}
		return false;
	}
	osg->clearInstructionPoseAxes();
	RobotOsgUi::RawTrajectoryPreviewOptions options;
	options.showAxisX = true;
	options.showAxisY = true;
	options.showAxisZ = true;
	options.showAxes = true;
	options.axisInterval = 0;
	feature_pick_transform::applyWorldRawTrajectoryPreviewToOsg(osg, worldPreview, options, &err);
	if (!err.empty())
	{
		if (outError)
		{
			*outError = QString::fromStdString(err);
		}
		m_simController->setRawTrajectoryPreviewActive(false);
		return false;
	}
	m_simController->setRawTrajectoryPreviewActive(true);
	osg->requestRedraw();
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
	std::string validateErr;
	if (!RobotInstruction::validateTrajectoryPipeline(m_ops, &validateErr))
	{
		if (outError)
		{
			*outError = validateErr.empty() ? QStringLiteral("invalid trajectory pipeline")
											: QString::fromStdString(validateErr);
		}
		return false;
	}

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
	std::string pipelineErr;
	if (m_rawTrajectory.has_value())
	{
		usingRaw = true;
		if (!configurePipelineEngineForRaw(m_ops))
		{
			if (outError)
			{
				*outError = QStringLiteral("无原始轨迹");
			}
			return false;
		}
		ensureGeometryResolverBound();
		injectWorkpieceReferenceOnEngine();
		if (!m_pipelineEngine.executeFull(&pipelineErr))
		{
			if (outError)
			{
				*outError =
					pipelineErr.empty() ? QStringLiteral("pipeline apply failed") : QString::fromStdString(pipelineErr);
			}
			return false;
		}
		reportProjectionMissesIfAny();
		reportNonRigidStatsIfAny();
		unified = m_pipelineEngine.result();
		rawWorking = m_pipelineEngine.rawWorking();
		m_bakedWorldRaw.reset();
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
		if (motionCount <= 0)
		{
			if (outError)
			{
				*outError = QStringLiteral("无原始轨迹且程序中无路点");
			}
			return false;
		}
		std::string convErr;
		if (!ingressProgramUnified(*activeProgram, unified, &convErr))
		{
			if (outError)
			{
				*outError =
					convErr.empty() ? QStringLiteral("program convert failed") : QString::fromStdString(convErr);
			}
			return false;
		}
		m_pipelineEngine.clear();
		m_pipelineEngine.setProgramContext(activeProgram);
		m_pipelineEngine.setUsingRaw(false);
		m_pipelineEngine.setUnifiedBaseline(unified);
		m_pipelineEngine.setOps(m_ops);
		ensureGeometryResolverBound();
		injectWorkpieceReferenceOnEngine();
		if (!m_pipelineEngine.executeFull(&pipelineErr))
		{
			if (outError)
			{
				*outError =
					pipelineErr.empty() ? QStringLiteral("pipeline apply failed") : QString::fromStdString(pipelineErr);
			}
			return false;
		}
		reportProjectionMissesIfAny();
		reportNonRigidStatsIfAny();
		unified = m_pipelineEngine.result();
	}
	RobotInstruction::RobotProgram replacement = *activeProgram;
	std::string emitErr;
	std::string outputGroupId;
	const bool boundPathPlan = !m_boundPathPlanId.empty();
	if (boundPathPlan)
	{
		if (!RobotInstruction::unifiedTrajectoryMergeIntoProgram(unified, replacement, m_boundPathPlanId, &emitErr,
																 &outputGroupId))
		{
			if (outError)
			{
				*outError =
					emitErr.empty() ? QStringLiteral("materialize program failed") : QString::fromStdString(emitErr);
			}
			return false;
		}
	}
	else if (!RobotInstruction::unifiedTrajectoryToProgram(unified, replacement, &emitErr))
	{
		if (outError)
		{
			*outError =
				emitErr.empty() ? QStringLiteral("materialize program failed") : QString::fromStdString(emitErr);
		}
		return false;
	}
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
		// appliedHistory 记录本次提交的草稿；pipeline 同步为提交后状态（undo 靠 Command 内 before）
		cmds.push_back(
			std::make_shared<RobotInstruction::UpdatePathPlanPipelineCommand>(m_boundPathPlanId, m_ops, m_ops));
		if (usingRaw && m_rawTrajectory.has_value())
		{
			// 持久化 CAD raw（可重放流水线）；显示真值在指令路点，Applied 阶段不再用此 raw 画 OSG
			cmds.push_back(std::make_shared<RobotInstruction::UpdatePathPlanRawCommand>(
				&catalog, m_boundPathPlanId, rawWorking, RobotInstruction::PathPlanPhase::Applied));
		}
		cmds.push_back(std::make_shared<RobotInstruction::UpdatePathPlanApplyStateCommand>(
			m_boundPathPlanId, RobotInstruction::PathPlanPhase::Applied, outputGroupId));
		if (!outputGroupId.empty())
		{
			m_defaultGroupId = outputGroupId;
		}
	}
	cmds.push_back(
		std::make_shared<RobotInstruction::ReplaceProgramContentCommand>(activeProgram, std::move(replacement)));
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
	clearPreviewStateWithoutRestore();
	refreshPreviewVisuals();
	return true;
}

void TrajectoryEditSession::clearPipelineAfterCommit()
{
	m_ops.clear();
}

void TrajectoryEditSession::clearPreviewStateWithoutRestore()
{
	clearOverlayPreview();
	m_overlayStoreWritebackActive = false;
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
	if (m_previewActive && (!m_overlayPreviewActive || m_overlayStoreWritebackActive) && !m_previewSnapshots.empty())
	{
		restorePreviewSnapshots();
	}
	clearPreviewStateWithoutRestore();
	refreshPreviewVisuals();
}

void TrajectoryEditSession::clearTrajectoryGeometryHistory()
{
	m_bakedWorldRaw.reset();
}

void TrajectoryEditSession::abandonPreview()
{
	clearPreviewStateWithoutRestore();
	refreshPreviewVisuals();
}

bool TrajectoryEditSession::canApply() const
{
	if (!m_store || m_ops.empty())
	{
		return false;
	}
	std::string validateErr;
	if (!RobotInstruction::validateTrajectoryPipeline(m_ops, &validateErr))
	{
		return false;
	}
	if (hasRawTrajectory())
	{
		return true;
	}
	const RobotInstruction::RobotProgram* prog = m_store->activeCatalog().findProgram(
		m_contextProgramId.empty() ? m_store->activeProgramIdUtf8() : m_contextProgramId);
	if (!prog)
	{
		return false;
	}
	RobotInstruction::UnifiedTrajectory probe{};
	std::string ingressErr;
	if (!m_boundPathPlanId.empty())
	{
		return RobotInstruction::ingressUnifiedForEdit(*prog, m_boundPathPlanId, probe, &ingressErr);
	}
	return RobotInstruction::ingressUnifiedFromProgram(*prog, probe, &ingressErr);
}

void TrajectoryEditSession::bindPathPlan(const std::string& pathPlanInstructionId)
{
	m_boundPathPlanId = pathPlanInstructionId;
	if (!m_store || pathPlanInstructionId.empty())
	{
		return;
	}
	RobotInstruction::RobotProgramCatalog& catalog = m_store->activeCatalog();
	RobotInstruction::PathPlanInstruction* pp = catalog.findPathPlan(catalog.activeProgramId(), pathPlanInstructionId);
	if (!pp)
	{
		return;
	}
	m_ops = pp->pipeline();
	if (!pp->outputGroupId().empty())
	{
		m_defaultGroupId = pp->outputGroupId();
	}
	(void)loadRawFromBoundPathPlan();
	emit pathPlanBound(pathPlanInstructionId);
}

std::string TrajectoryEditSession::boundSourceFeatureJson() const
{
	if (m_rawTrajectory.has_value() && !m_rawTrajectory->sourceFeatureJson.empty())
	{
		return m_rawTrajectory->sourceFeatureJson;
	}
	if (!m_store || m_boundPathPlanId.empty())
	{
		return {};
	}
	if (const RobotInstruction::PathPlanInstruction* pp =
			m_store->activeCatalog().findPathPlan(m_store->activeCatalog().activeProgramId(), m_boundPathPlanId))
	{
		return pp->sourceFeatureJson();
	}
	return {};
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
	auto cmd = std::make_shared<RobotInstruction::UpdatePathPlanPipelineCommand>(m_boundPathPlanId, m_ops, m_ops);
	return m_editService->execute(cmd, outError);
}

bool TrajectoryEditSession::syncPipelineToBoundPathPlan()
{
	if (!m_store || m_boundPathPlanId.empty())
	{
		return false;
	}
	RobotInstruction::PathPlanInstruction* pp =
		m_store->activeCatalog().findPathPlan(m_store->activeCatalog().activeProgramId(), m_boundPathPlanId);
	if (!pp)
	{
		return false;
	}
	pp->setPipeline(m_ops);
	if (m_rawTrajectory.has_value())
	{
		m_store->activeCatalog().pathPlanRaws().save(m_boundPathPlanId, *m_rawTrajectory);
	}
	return true;
}

bool TrajectoryEditSession::reloadBoundPathPlanFromStore()
{
	if (!m_store || m_boundPathPlanId.empty())
	{
		return false;
	}
	RobotInstruction::PathPlanInstruction* pp =
		m_store->activeCatalog().findPathPlan(m_store->activeCatalog().activeProgramId(), m_boundPathPlanId);
	if (!pp)
	{
		return false;
	}
	m_ops = pp->pipeline();
	if (!pp->outputGroupId().empty())
	{
		m_defaultGroupId = pp->outputGroupId();
	}
	(void)loadRawFromBoundPathPlan();
	invalidatePreviewScopeCache();
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
		m_rawTrajectory.reset();
		m_bakedWorldRaw.reset();
		emit rawTrajectoryChanged();
		return false;
	}
	m_rawTrajectory = std::move(raw);
	m_bakedWorldRaw.reset();
	emit rawTrajectoryChanged();
	return true;
}

namespace
{
std::string makeUniquePathPlanName(const RobotInstruction::RawTrajectory& traj,
								   const RobotInstruction::RobotProgramCatalog& catalog, const std::string& programId)
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
	++m_deferProgramRevisionUiSync;
	struct DeferGuard
	{
		int* counter;
		~DeferGuard()
		{
			if (counter)
			{
				--(*counter);
			}
		}
	} defer{&m_deferProgramRevisionUiSync};
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
				&m_store->activeCatalog(), m_boundPathPlanId, *m_rawTrajectory,
				RobotInstruction::PathPlanPhase::RawReady));
			std::vector<RobotInstruction::ProgramEditStack::CommandPtr> batch;
			batch.push_back(std::make_shared<RobotInstruction::CompositeProgramEditCommand>(std::move(cmds)));
			QString err;
			(void)m_editService->executeBatch(batch, &err);
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
		RobotInstruction::PathPlanInstruction* pp =
			catalog.findPathPlan(catalog.activeProgramId(), m_boundPathPlanId);
		if (pp)
		{
			pp->setSourceFeatureJson(m_rawTrajectory->sourceFeatureJson);
			if (pp->name().empty())
			{
				pp->setName(makeUniquePathPlanName(*m_rawTrajectory, catalog, catalog.activeProgramId()));
			}
			if (pp->rawTrajectoryKey().empty())
			{
				pp->setRawTrajectoryKey(m_boundPathPlanId);
			}
		}
		if (m_editService)
		{
			std::vector<RobotInstruction::ProgramEditStack::CommandPtr> cmds;
			if (pp)
			{
				// 重离散清空 appliedHistory，纳入同一撤销单元
				cmds.push_back(std::make_shared<RobotInstruction::UpdatePathPlanPipelineCommand>(
					m_boundPathPlanId, pp->pipeline(), std::vector<RobotInstruction::TrajectoryOpDescriptor>{}));
			}
			cmds.push_back(std::make_shared<RobotInstruction::UpdatePathPlanRawCommand>(
				&catalog, m_boundPathPlanId, *m_rawTrajectory, RobotInstruction::PathPlanPhase::RawReady));
			std::vector<RobotInstruction::ProgramEditStack::CommandPtr> batch;
			if (cmds.size() > 1)
			{
				batch.push_back(std::make_shared<RobotInstruction::CompositeProgramEditCommand>(std::move(cmds)));
			}
			else
			{
				batch = std::move(cmds);
			}
			QString err;
			(void)m_editService->executeBatch(batch, &err);
		}
		else if (pp)
		{
			catalog.pathPlanRaws().save(m_boundPathPlanId, *m_rawTrajectory);
			pp->appliedHistoryMut().clear();
			pp->setPhase(RobotInstruction::PathPlanPhase::RawReady);
			pp->bumpRawRevision();
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
	bool hadStoreRaw = false;
	if (m_store && !m_boundPathPlanId.empty())
	{
		RobotInstruction::RawTrajectory stored;
		hadStoreRaw = m_store->activeCatalog().pathPlanRaws().load(m_boundPathPlanId, stored) && !stored.points.empty();
	}
	if (!m_rawTrajectory.has_value() && !m_bakedWorldRaw.has_value() && !hadStoreRaw)
	{
		return;
	}
	m_rawTrajectory.reset();
	m_bakedWorldRaw.reset();
	if (m_store && !m_boundPathPlanId.empty())
	{
		m_store->activeCatalog().pathPlanRaws().remove(m_boundPathPlanId);
		if (RobotInstruction::PathPlanInstruction* pp =
				m_store->activeCatalog().findPathPlan(m_store->activeCatalog().activeProgramId(), m_boundPathPlanId))
		{
			pp->setSourceFeatureJson(std::string{});
			if (pp->phase() == RobotInstruction::PathPlanPhase::RawReady ||
				pp->phase() == RobotInstruction::PathPlanPhase::Applied)
			{
				pp->setPhase(RobotInstruction::PathPlanPhase::Draft);
			}
		}
	}
	emit rawTrajectoryChanged();
}

bool TrajectoryEditSession::configurePipelineEngineForRaw(
	const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops) const
{
	if (!m_rawTrajectory.has_value() || m_rawTrajectory->points.empty())
	{
		return false;
	}
	m_pipelineEngine.clear();
	m_pipelineEngine.setUsingRaw(true);
	m_pipelineEngine.setSourceRaw(*m_rawTrajectory);
	m_pipelineEngine.setRawRebuildFn(
		[this](const RobotInstruction::RawTrajectory& sourceRaw, RobotInstruction::UnifiedTrajectory& outUnified,
			   std::string* errMsg) -> bool
		{
			QString qErr;
			const bool ok = rebuildUnifiedFromSourceRaw(sourceRaw, outUnified, &qErr);
			if (!ok && errMsg && !qErr.isEmpty())
			{
				*errMsg = qErr.toStdString();
			}
			return ok;
		});
	const std::string programId = m_contextProgramId.empty() ? m_store->activeProgramIdUtf8() : m_contextProgramId;
	if (const RobotInstruction::RobotProgram* prog = m_store->activeCatalog().findProgram(programId))
	{
		m_pipelineEngine.setProgramContext(prog);
	}
	m_pipelineEngine.setOps(ops);
	return true;
}

bool TrajectoryEditSession::syncPipelineEngine(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	if (!configurePipelineEngineForRaw(ops))
	{
		return false;
	}
	return true;
}

bool TrajectoryEditSession::runPipelineEngineFull(QString* outError)
{
	ensureGeometryResolverBound();
	injectWorkpieceReferenceOnEngine();
	std::string err;
	if (!m_pipelineEngine.executeFull(&err))
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("pipeline execute failed") : QString::fromStdString(err);
		}
		return false;
	}
	reportProjectionMissesIfAny();
	reportNonRigidStatsIfAny();
	return true;
}

bool TrajectoryEditSession::runPipelineEngineFrom(const std::size_t nodeIndex, QString* outError)
{
	// 与 Full 一致：重绑以拿到源/目标当前世界矩阵（改参预览也会走此路径）
	ensureGeometryResolverBound();
	injectWorkpieceReferenceOnEngine();
	std::string err;
	if (!m_pipelineEngine.executeFrom(nodeIndex, &err))
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("pipeline partial execute failed") : QString::fromStdString(err);
		}
		return false;
	}
	return true;
}

bool TrajectoryEditSession::buildRawPreviewWithPipeline(
	const std::vector<RobotInstruction::TrajectoryOpDescriptor>& pipelineOps,
	RobotInstruction::RawTrajectory& outPreviewRaw, QString* outError) const
{
	if (!configurePipelineEngineForRaw(pipelineOps))
	{
		if (outError)
		{
			*outError = QStringLiteral("无原始轨迹");
		}
		return false;
	}
	ensureGeometryResolverBound();
	injectWorkpieceReferenceOnEngine();
	std::string err;
	if (!m_pipelineEngine.executeFull(&err))
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("unified preview failed") : QString::fromStdString(err);
		}
		return false;
	}
	reportProjectionMissesIfAny();
	reportNonRigidStatsIfAny();
	RobotInstruction::RawTrajectory worldPreview{};
	if (!RobotInstruction::unifiedTrajectoryToRaw(m_pipelineEngine.result(), worldPreview, &err))
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
		snap.blendRadius = raw->blendRadius();
		snap.speed = raw->speed();
		snap.extensions = raw->extensionProperties();
		m_previewSnapshots.push_back(std::move(snap));
	}
	return true;
}

void TrajectoryEditSession::syncRenderMatricesForInstructionIds(const std::vector<std::string>& ids,
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

bool TrajectoryEditSession::writeRenderMatricesFromSnapshotBase(const PreviewSnapshot& snap,
																RobotInstruction::Base& raw,
																const std::string* frozenBaseWorldCsv,
																double* outWorldDeltaMm) const
{
	osg::Matrixd snapWorld;
	snapWorld.makeIdentity();
	const auto itW = snap.extensions.find("render.tcpWorldMat4");
	const bool hasSnapWorld = itW != snap.extensions.end() && !itW->second.empty() &&
							  RobotSimulationMath::decodeMatrix4Csv(itW->second, snapWorld);
	osg::Matrixd snapLocal;
	snapLocal.makeIdentity();
	const auto itL = snap.extensions.find("render.tcpLocalMat4");
	if (itL != snap.extensions.end() && !itL->second.empty() &&
		RobotSimulationMath::decodeMatrix4Csv(itL->second, snapLocal))
	{
		// ok
	}
	else
	{
		const engine::RigidTransform t = engine::RigidTransform::fromTranslationEulerDeg(
			snap.pose.x, snap.pose.y, snap.pose.z, snap.euler.x, snap.euler.y, snap.euler.z);
		snapLocal = engine::osgMatrixFromRigidTransform(t);
	}
	if (!hasSnapWorld)
	{
		return false;
	}
	osg::Matrixd baseWorld;
	if (frozenBaseWorldCsv && !frozenBaseWorldCsv->empty() &&
		RobotSimulationMath::decodeMatrix4Csv(*frozenBaseWorldCsv, baseWorld))
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
		raw->setBlendRadius(snap.blendRadius);
		raw->setSpeed(snap.speed);
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
