#pragma once

#include "ProgramEditService.h"
#include "RobotProgramStore.h"
#include "RawTrajectory.h"
#include "UnifiedTrajectory.h"
#include "TrajectoryPipelineBuilder.h"
#include "TrajectoryPipelineTypes.h"
#include "robotwidget_global.h"

#include <QObject>

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

class RobotSimulationController;

/// 轨迹编辑流水线编排（Preview 临时改 pose，Apply 走 Command）
class ROBOTWIDGET_EXPORT TrajectoryEditSession : public QObject
{
	Q_OBJECT

public:
	explicit TrajectoryEditSession(QObject* parent = nullptr);

	void bindStore(RobotProgramStore* store);
	void bindEditService(ProgramEditService* service);
	void bindSimulationController(RobotSimulationController* controller);

	void setPipeline(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops);
	void updatePipelineOps(
		std::vector<RobotInstruction::TrajectoryOpDescriptor> ops,
		bool allowPreviewReapply = true);

	void setContextProgramId(const std::string& programId);
	void setDefaultGroupId(const std::string& groupId);

	void bindPathPlan(const std::string& pathPlanInstructionId);
	void clearPathPlanBinding();
	const std::string& boundPathPlanId() const { return m_boundPathPlanId; }
	bool syncPipelineToBoundPathPlan();
	bool persistBoundPathPlanPipeline(QString* outError = nullptr);
	bool loadRawFromBoundPathPlan();

	bool preview(QString* outError = nullptr);
	/// 尚无 raw 时按流水线预览（含配方时走 Unified，与 Apply 一致）
	bool previewPipeline(
		const std::vector<RobotInstruction::TrajectoryOpDescriptor>& pipelineOps,
		QString* outError = nullptr);
	bool apply(QString* outError = nullptr);
	void reset();
	/// 清空累积几何变换与 baked raw（轨迹编辑页「重置」按钮）
	void clearTrajectoryGeometryHistory();
	void abandonPreview();
	void clearPipelineAfterCommit();
	bool isApplying() const { return m_applying; }
	bool isPreviewActive() const { return m_previewActive; }
	bool canApply() const;

	void setRawTrajectory(RobotInstruction::RawTrajectory traj);
	const RobotInstruction::RawTrajectory* rawTrajectory() const;
	bool hasRawTrajectory() const;
	void clearRawTrajectory();
	/// 与 Apply 相同顺序：pending → recipe(流水线) → accumulated → geometry(流水线)
	bool buildRawPreviewWithPipeline(
		const std::vector<RobotInstruction::TrajectoryOpDescriptor>& pipelineOps,
		RobotInstruction::RawTrajectory& outPreviewRaw,
		QString* outError = nullptr) const;

signals:
	void previewStateChanged(bool active);
	void rawTrajectoryChanged();

private:
	struct PreviewSnapshot
	{
		std::string id;
		RobotInstruction::Vec3 pose{};
		RobotInstruction::Vec3 euler{};
		std::unordered_map<std::string, std::string> extensions;
	};

	void clearPreviewSnapshots();
	bool capturePreviewSnapshots(QString* outError);
	bool applyPreviewTransforms(QString* outError);
	void restorePreviewSnapshots();
	void clearPreviewStateWithoutRestore();
	void syncPreviewRenderMatrices(const std::vector<std::string>* updatedIds = nullptr);
	void syncRenderMatricesForInstructionIds(const std::vector<std::string>& ids, bool worldFrameTcp = false);
	void syncRenderMatricesFromFrozenBase(
		const std::vector<std::string>& ids,
		const std::unordered_map<std::string, std::string>& frozenBaseWorldCsvById);
	bool writeRenderMatricesFromSnapshotBase(
		const PreviewSnapshot& snap,
		RobotInstruction::Base& raw,
		const std::string* frozenBaseWorldCsv,
		double* outWorldDeltaMm) const;
	bool reapplyPreview(QString* outError = nullptr);
	void refreshPreviewVisuals();
	bool rebuildUnifiedFromSourceRaw(
		const RobotInstruction::RawTrajectory& sourceRaw,
		RobotInstruction::UnifiedTrajectory& unified,
		QString* outError = nullptr) const;
	bool previewUnifiedFromProgramPipeline(QString* outError);
	std::vector<std::string> collectPreviewWaypointIds() const;
	void invalidatePreviewScopeCache();
	void updateLightweightPreviewState(bool active);

	RobotProgramStore* m_store = nullptr;
	ProgramEditService* m_editService = nullptr;
	RobotSimulationController* m_simController = nullptr;
	RobotInstruction::TrajectoryPipelineBuilder m_builder;
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
	bool m_applying = false;
	std::optional<RobotInstruction::RawTrajectory> m_rawTrajectory;
	/// 轨迹离散前（session 尚无 raw）Apply 的几何块，首次 raw Apply 时先叠加
	std::vector<RobotInstruction::TrajectoryOpDescriptor> m_pendingPreRawGeometryOps;
	/// 历次 raw Apply 的几何块（含进退刀等），下次从 CAD raw 重放
	std::vector<RobotInstruction::TrajectoryOpDescriptor> m_accumulatedGeometryOps;
	std::optional<RobotInstruction::RawTrajectory> m_bakedWorldRaw;
};
