#pragma once

#include "ProgramEditService.h"
#include "RobotProgramStore.h"
#include "RawTrajectory.h"
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
	void updatePipelineOps(std::vector<RobotInstruction::TrajectoryOpDescriptor> ops);

	void setContextProgramId(const std::string& programId);
	void setDefaultGroupId(const std::string& groupId);

	bool preview(QString* outError = nullptr);
	bool apply(QString* outError = nullptr);
	void reset();
	void abandonPreview();
	void clearPipelineAfterCommit();
	bool isApplying() const { return m_applying; }
	bool isPreviewActive() const { return m_previewActive; }
	bool canApply() const;

	void setRawTrajectory(RobotInstruction::RawTrajectory traj);
	const RobotInstruction::RawTrajectory* rawTrajectory() const;
	bool hasRawTrajectory() const;
	void clearRawTrajectory();

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
	void syncPreviewRenderMatrices();
	void syncRenderMatricesForInstructionIds(const std::vector<std::string>& ids);
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
	std::vector<std::string> collectPreviewWaypointIds() const;

	RobotProgramStore* m_store = nullptr;
	ProgramEditService* m_editService = nullptr;
	RobotSimulationController* m_simController = nullptr;
	RobotInstruction::TrajectoryPipelineBuilder m_builder;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> m_ops;
	std::string m_contextProgramId;
	std::string m_defaultGroupId;
	std::vector<PreviewSnapshot> m_previewSnapshots;
	bool m_previewActive = false;
	bool m_applying = false;
	std::optional<RobotInstruction::RawTrajectory> m_rawTrajectory;
};
