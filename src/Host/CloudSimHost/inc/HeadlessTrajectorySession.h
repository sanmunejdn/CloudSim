#ifndef CLOUDSIMHOST_HEADLESSTRAJECTORYSESSION_H
#define CLOUDSIMHOST_HEADLESSTRAJECTORYSESSION_H

/// @file HeadlessTrajectorySession.h
/// @brief Web/Headless 轨迹会话：PathPlan 绑定、特征离散、管线预览/应用（无 OSG）

#include "cloudsim_host_global.h"

#include "RawTrajectory.h"
#include "TrajectoryPipelineTypes.h"

#include <QHash>
#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace RobotInstruction
{
class RobotProgramCatalog;
class PathPlanInstruction;
}

namespace cloudsim::host
{
class DocumentHost;

/// Web 轨迹编排；编辑门闩对齐桌面「开始修改」
class CLOUDSIM_HOST_EXPORT HeadlessTrajectorySession
{
public:
	explicit HeadlessTrajectorySession(DocumentHost& host);
	~HeadlessTrajectorySession();

	bool featureEditActive() const { return m_featureEditActive; }
	bool beginEdit(QString* err = nullptr);
	void cancelEdit();

	bool createPathPlan(const QString& sceneBackendId, QString* outPathPlanId, QString* err = nullptr);
	bool bindPathPlan(const QString& pathPlanId, QString* err = nullptr, const QString& sceneBackendId = QString());
	void clearBinding();
	QString boundPathPlanId() const { return QString::fromStdString(m_boundPathPlanId); }

	QJsonObject sessionSummaryJson() const;
	QJsonArray listPathPlansJson(const QString& sceneBackendId) const;

	bool pickMeshElement(const QByteArray& body, QJsonObject* out, QString* err);
	/// 拾取悬停：命中索引 + 世界系折线（供前端高亮，不改特征表）
	bool pickHover(const QByteArray& body, QJsonObject* out, QString* err);
	bool featureCatalogJson(const QString& workpieceBackendId, QByteArray* out, QString* err);
	/// 离散策略 schema（fields + defaults）；strategyId 空则返回策略目录
	bool featureSchemaJson(const QString& strategyId, QJsonObject* out, QString* err);
	bool setFeaturesAndDiscretize(const QByteArray& featureListJson, QString* err);
	bool discretizeMeshSpec(const QByteArray& meshSpecJson, QString* err);

	bool setPipelineJson(const QByteArray& pipelineJson, QString* err);
	bool fillRecipe(const QString& recipeKind, QString* err);
	QByteArray pipelineJson() const;
	/// 算子参数 schema + 当前值（对接 TrajectoryOpParamSchema）
	bool opSchemaJson(const QString& kind, int opIndex, QJsonObject* out, QString* err);

	bool preview(QJsonObject* outPolylineWorld, QString* err);
	bool apply(QString* err);
	/// 不经流水线：Raw → LINE（对齐桌面「生成」）
	bool emitRawProgram(QString* err);
	bool resetPipeline(QString* err);
	/// 调色板：kind + 中文名（与桌面 trajectoryOpPaletteKinds 一致）
	bool opPaletteJson(QJsonObject* out, QString* err);

	bool undoDraft(QString* err);
	bool redoDraft(QString* err);

	bool saveTemplate(const QString& kind, const QString& name, const QByteArray& payload, QString* err);
	bool loadTemplate(const QString& kind, const QString& name, QByteArray* out, QString* err);
	bool deleteTemplate(const QString& kind, const QString& name, QString* err);
	QJsonArray listTemplatesJson(const QString& kind) const;

	bool hasRaw() const { return m_raw.has_value() && !m_raw->points.empty(); }
	const RobotInstruction::RawTrajectory* raw() const;

private:
	struct DraftSnap
	{
		std::optional<RobotInstruction::RawTrajectory> raw;
		std::vector<RobotInstruction::TrajectoryOpDescriptor> ops;
	};

	DocumentHost& m_host;
	std::string m_boundPathPlanId;
	std::string m_sceneBackendId;
	bool m_featureEditActive = false;
	/// 应用/生成落地后禁止再次「生成」，避免覆盖
	bool m_emitDisabledAfterApply = false;
	std::optional<RobotInstruction::RawTrajectory> m_raw;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> m_ops;
	std::vector<DraftSnap> m_undo;
	std::vector<DraftSnap> m_redo;
	struct EngineHolder;
	std::unique_ptr<EngineHolder> m_engine;
	/// workpiece#faceIndex → 世界系面三角 soup，悬停复用
	QHash<QString, QJsonArray> m_faceHighlightSoup;

	RobotInstruction::RobotProgramCatalog* catalog();
	RobotInstruction::PathPlanInstruction* boundPathPlan();
	void pushUndo();
	bool requireEdit(QString* err) const;
	bool requireBound(QString* err) const;
	bool persistRaw(QString* err);
	bool persistPipeline(QString* err);
	bool worldFromModelPoint(const std::string& backendId, double mx, double my, double mz, double& wx, double& wy,
							 double& wz) const;
	bool modelFromWorldPoint(const std::string& backendId, double wx, double wy, double wz, double& mx, double& my,
							 double& mz) const;
	bool modelFromWorldDir(const std::string& backendId, double wx, double wy, double wz, double& mx, double& my,
						   double& mz) const;
	bool transformRawToWorld(const RobotInstruction::RawTrajectory& modelRaw,
							 RobotInstruction::RawTrajectory& worldRaw, QString* err) const;
	bool runPipelineOnWorldRaw(RobotInstruction::RawTrajectory& worldRawInOut, QString* err);
	QString templatesDir(const QString& kind) const;
	/// 共用射线命中；includeHighlight 时附加 polylinesWorld
	bool pickShapeRay(const QByteArray& body, bool requireEditGate, bool includeHighlight, QJsonObject* out,
					  QString* err);
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_HEADLESSTRAJECTORYSESSION_H
