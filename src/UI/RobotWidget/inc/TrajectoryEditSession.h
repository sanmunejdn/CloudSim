#ifndef ROBOTWIDGET_TRAJECTORYEDITSESSION_H
#define ROBOTWIDGET_TRAJECTORYEDITSESSION_H

/// @file TrajectoryEditSession.h
/// @brief 轨迹编辑流水线编排（Preview 三分支 + Apply 走 Command）

#include "robotwidget_global.h"

#include "ProgramEditService.h"
#include "RawTrajectory.h"
#include "ExternalAxisSearchService.h"
#include "RobotProgramStore.h"
#include "TrajectoryPipelineEngine.h"
#include "TrajectoryPipelineTypes.h"
#include "UnifiedTrajectory.h"

#include <QObject>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

class RobotSimulationController;

/// 轨迹编辑流水线编排（Preview 三分支 + Apply 走 Command）
class ROBOTWIDGET_EXPORT TrajectoryEditSession : public QObject
{
	Q_OBJECT

public:
	explicit TrajectoryEditSession(QObject* parent = nullptr);

	void bindStore(RobotProgramStore* store);
	void bindEditService(ProgramEditService* service);
	void bindSimulationController(RobotSimulationController* controller);

	void setPipeline(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops);
	void updatePipelineOps(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops, bool allowPreviewReapply = true);

	void setContextProgramId(const std::string& programId);
	void setDefaultGroupId(const std::string& groupId);

	void bindPathPlan(const std::string& pathPlanInstructionId);
	void clearPathPlanBinding();
	const std::string& boundPathPlanId() const { return m_boundPathPlanId; }
	std::size_t boundPipelineOpCount() const { return m_ops.size(); }
	bool syncPipelineToBoundPathPlan();
	bool persistBoundPathPlanPipeline(QString* outError = nullptr);
	bool loadRawFromBoundPathPlan();
	/// 从已绑定 PathPlan 重载算子表与 raw，不触发 pathPlanBound
	bool reloadBoundPathPlanFromStore();
	/// 当前绑定 PathPlan 的 sourceFeatureJson：优先 raw，回退 PathPlan 指令字段
	std::string boundSourceFeatureJson() const;

	/// 尚无 raw 时按流水线预览（Unified 引擎，与 Apply 一致）
	bool previewPipeline(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& pipelineOps,
						 QString* outError = nullptr);
	bool apply(QString* outError = nullptr);
	void reset();
	/// 清空累积几何变换与 baked raw（轨迹编辑页「重置」按钮）
	void clearTrajectoryGeometryHistory();
	void abandonPreview();
	void clearPipelineAfterCommit();
	bool isApplying() const { return m_applying; }
	bool defersProgramRevisionUiSync() const { return m_deferProgramRevisionUiSync > 0; }
	bool isPreviewActive() const { return m_previewActive; }
	bool canApply() const;

	void setRawTrajectory(RobotInstruction::RawTrajectory traj);
	const RobotInstruction::RawTrajectory* rawTrajectory() const;
	bool hasRawTrajectory() const;
	void clearRawTrajectory();
	/// 与 Apply 相同顺序：pending → recipe(流水线) → accumulated → geometry(流水线)
	bool buildRawPreviewWithPipeline(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& pipelineOps,
									 RobotInstruction::RawTrajectory& outPreviewRaw, QString* outError = nullptr) const;

	RobotInstruction::TrajectoryPipelineEngine& pipelineEngine() { return m_pipelineEngine; }
	const RobotInstruction::TrajectoryPipelineEngine& pipelineEngine() const { return m_pipelineEngine; }
	bool syncPipelineEngine(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops);
	bool runPipelineEngineFull(QString* outError = nullptr);
	bool runPipelineEngineFrom(std::size_t nodeIndex, QString* outError = nullptr);

signals:
	void previewStateChanged(bool active);
	void rawTrajectoryChanged();
	void pathPlanBound(const std::string& pathPlanInstructionId);

private:
	struct PreviewSnapshot
	{
		std::string id;
		RobotInstruction::Vec3 pose{};
		RobotInstruction::Vec3 euler{};
		double blendRadius = 0.0;
		double speed = 0.0;
		std::unordered_map<std::string, std::string> extensions;
	};

	void clearPreviewSnapshots();
	bool capturePreviewSnapshots(QString* outError);
	void restorePreviewSnapshots();
	void clearPreviewStateWithoutRestore();
	void syncPreviewRenderMatrices(const std::vector<std::string>* updatedIds = nullptr);
	void syncRenderMatricesForInstructionIds(const std::vector<std::string>& ids, bool worldFrameTcp = false);
	bool writeRenderMatricesFromSnapshotBase(const PreviewSnapshot& snap, RobotInstruction::Base& raw,
											 const std::string* frozenBaseWorldCsv, double* outWorldDeltaMm) const;
	bool reapplyPreview(QString* outError = nullptr);
	void refreshPreviewVisuals();
	bool rebuildUnifiedFromSourceRaw(const RobotInstruction::RawTrajectory& sourceRaw,
									 RobotInstruction::UnifiedTrajectory& unified, QString* outError = nullptr) const;
	bool configurePipelineEngineForRaw(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops) const;
	bool previewUnifiedFromProgramPipeline(QString* outError);
	void clearOverlayPreview();
	bool showUnifiedOverlayPreview(const RobotInstruction::UnifiedTrajectory& unified, QString* outError);
	bool applyUnifiedPreviewWriteback(const RobotInstruction::UnifiedTrajectory& unified, bool writePose,
									  bool writeBlendSpeed, std::vector<std::string>& outChangedIds);
	std::vector<std::string> collectPreviewWaypointIds() const;
	bool ingressProgramUnified(const RobotInstruction::RobotProgram& program,
							   RobotInstruction::UnifiedTrajectory& unified, std::string* errMsg) const;
	void invalidatePreviewScopeCache();
	void updateLightweightPreviewState(bool active);
	void ensureGeometryResolverBound() const;
	void reportProjectionMissesIfAny() const;
	/// 捕获当前 TCP 并注入管道（失败则清除，算子侧报错）
	void injectWorkpieceReferenceOnEngine() const;
	void injectExternalAxisSearchOnEngine() const;

	RobotProgramStore* m_store = nullptr;
	ProgramEditService* m_editService = nullptr;
	RobotSimulationController* m_simController = nullptr;
	mutable RobotInstruction::TrajectoryPipelineEngine m_pipelineEngine;
	mutable RobotInstruction::ExternalAxisSearchService m_externalAxisSearchService;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> m_ops;
	std::string m_contextProgramId;
	std::string m_defaultGroupId;
	std::string m_boundPathPlanId;
	int m_programRevision = 0;
	mutable bool m_previewWaypointCacheValid = false;
	mutable std::string m_previewWaypointCacheProgramId;
	mutable int m_previewWaypointCacheRevision = -1;
	mutable std::vector<std::string> m_previewWaypointCache;
	std::vector<std::string> m_effectivePreviewWaypointIds;
	std::vector<PreviewSnapshot> m_previewSnapshots;
	bool m_lightweightPreviewActive = false;
	bool m_previewActive = false;
	bool m_overlayPreviewActive = false;
	/// overlay 预览时是否已写回 store（混合预览，reset 需恢复快照）
	bool m_overlayStoreWritebackActive = false;
	bool m_applying = false;
	int m_deferProgramRevisionUiSync = 0;
	std::optional<RobotInstruction::RawTrajectory> m_rawTrajectory;
	std::optional<RobotInstruction::RawTrajectory> m_bakedWorldRaw;
};

#endif // ROBOTWIDGET_TRAJECTORYEDITSESSION_H
