#pragma once



#include "robotwidget_global.h"



#include "RobotOsgUiTypes.h"

#include <FeatureListDocument.h>

#include <QWidget>



#include <functional>

#include <string>



namespace RobotInstruction

{

struct RawTrajectory;

}



class QCheckBox;

class QComboBox;

class QGroupBox;

class QLabel;

class QPushButton;

class QSpinBox;

class QTableView;

class QTimer;

class FeatureDiscretizerParamPanel;

class FeatureTableModel;

class IRobotMainWindowHost;

class TrajectoryEditSession;

class RobotSimulationController;

namespace geoalgo
{
class ShapeHandle;
struct FeatureCatalog;
struct WorkpieceRef;
}

class ROBOTWIDGET_EXPORT FeatureTrajectoryPageWidget : public QWidget

{

	Q_OBJECT



public:

	explicit FeatureTrajectoryPageWidget(QWidget* parent = nullptr);



	void setUseChinese(bool chinese);

	void bindHost(IRobotMainWindowHost* host);

	void bindSession(TrajectoryEditSession* session);

	void bindSimulationController(RobotSimulationController* controller);

	void setStepPathResolver(std::function<QString(const QString& backendId)> resolver);

	/// AI / 宿主：轨迹页 combo 当前 STEP 工件
	bool currentWorkpiece(QString& backendId, QString& stepPath) const;

	/// 确保当前工件特征目录已枚举（AI 解析前调用）
	bool ensureFeatureCatalogEnumerated(QString* err = nullptr);

	/// 由 catalog 切片 JSON 构建 3D 编号 overlay
	bool buildAndShowCandidatePreview(const QByteArray& catalogSliceUtf8);

	void clearCandidatePreview();

	/// 确认离散：features[] → RawTrajectory + 默认工艺流水线
	bool commitFeaturePlanFromAi(const QByteArray& planJsonUtf8, QString* summary, QString* err);

	RobotOsgUi::RawTrajectoryPreviewOptions previewOptions() const;

	/// 轨迹编辑 Apply/生成程序后清空本页，便于下一条 PathPlan
	void resetAfterTrajectoryCommit();

	/// 显式加载当前绑定 PathPlan 的特征表并进入可编辑（自动重离散）模式
	bool beginEditBoundPathPlan(QString* err = nullptr);

	bool isFeatureEditActive() const { return m_featureEditActive; }

signals:
	void workpieceComboChanged();

private slots:

	void onDiscretize();

	void onPickEdge();

	void onPickFace();

	void onCancelPick();

	void onTableSelectionChanged();

	void onDeleteSelectedRows();

	void onDeleteAllRows();

	void onStrategyComboChanged();

	void onPathPlanBound(const std::string& pathPlanId);



private:

	enum class PickSessionKind

	{

		None,

		Edge,

		Face

	};



	void refreshBackendCombo();

	void refreshStrategyCombo(geoalgo::GeometryAffinity filterAffinity = geoalgo::GeometryAffinity::Any);

	void updateUiLabels();

	void updatePickUiState();

	void showTrajectoryPreview(const RobotInstruction::RawTrajectory& traj);

	std::string resolvePreviewBackendId(const RobotInstruction::RawTrajectory& traj) const;

	RobotOsgUi::RawTrajectoryPreviewOptions currentPreviewOptions() const;

	void setStatus(const QString& text);

	void exitPickMode();

	void onMeshPickCommitted(const struct PickResult& pick, int pickKindInt);

	bool buildFeatureEntryFromPick(
		bool pickFace,
		const geoalgo::Point3d& modelA,
		const geoalgo::Point3d& modelB,
		int knownFaceIndex,
		int knownEdgeIndex,
		geoalgo::FeatureEntry& out,
		QString* err) const;

	bool buildFeatureListDocument(geoalgo::FeatureListDocument& out, QString* err) const;

	bool discretizeFromTable(bool quiet = false);

	void loadParamsForSelectedRow();

	void syncStrategyComboToEntry(const geoalgo::FeatureEntry& entry);

	std::string resolveStrategyIdForPick(bool pickFace) const;

	std::string defaultStrategyIdForGeometry(geoalgo::GeometryAffinity required) const;

	void normalizeEntryStrategyForGeometry(geoalgo::FeatureEntry& entry) const;

	void applyParamsToSelectedRow();

	void scheduleParameterRediscretize();

	void onParameterRediscretize();

	void refreshPreviewFromSession();

	bool buildPreviewOverlayJson(const QByteArray& catalogSliceUtf8, QByteArray& outPreviewJson, QString* err) const;

	bool enumerateCatalogForBackend(const QString& backendId, geoalgo::FeatureCatalog& out, QString* err) const;

	bool resolveWorkpieceShapeForBackend(const QString& backendId, geoalgo::ShapeHandle& outShape,
		geoalgo::WorkpieceRef& outRef, QString* err) const;

	bool autoEnumerateCatalogForCurrentWorkpiece(bool quiet, QString* err);

	QString strategyDisplayName(const std::string& strategyId) const;

	void ensureDiscretizerRuntimeLoaded() const;

	bool applyFeatureListDocument(const geoalgo::FeatureListDocument& doc, bool restoreWorkpiece);

	bool loadFeatureListFromJson(const std::string& jsonUtf8, QString* err = nullptr);

	bool selectBackendComboById(const QString& backendId);



	IRobotMainWindowHost* m_host = nullptr;

	TrajectoryEditSession* m_session = nullptr;

	RobotSimulationController* m_simController = nullptr;

	std::function<QString(const QString&)> m_stepPathResolver;

	bool m_chinese = true;

	bool m_suppressParamRediscretize = false;

	int m_strategyRowSyncDepth = 0;

	bool m_featureEditActive = false;

	mutable bool m_runtimeLoaded = false;

	PickSessionKind m_pickSession = PickSessionKind::None;

	geoalgo::GeometryAffinity m_lastPickAffinity = geoalgo::GeometryAffinity::Any;

	QString m_cachedCatalogBackendId;

	std::string m_cachedCatalogJsonUtf8;

	std::string m_lastLoadedPathPlanId;

	std::string m_lastLoadedSourceJson;



	QComboBox* m_backendCombo = nullptr;

	QTableView* m_featureTable = nullptr;

	FeatureTableModel* m_featureModel = nullptr;

	QComboBox* m_strategyCombo = nullptr;

	FeatureDiscretizerParamPanel* m_paramPanel = nullptr;

	QLabel* m_pickStatusLabel = nullptr;

	QPushButton* m_pickEdgeBtn = nullptr;

	QPushButton* m_pickFaceBtn = nullptr;

	QPushButton* m_cancelPickBtn = nullptr;

	QPushButton* m_discretizeBtn = nullptr;

	QGroupBox* m_previewGroup = nullptr;

	QCheckBox* m_showAxisXCheck = nullptr;

	QCheckBox* m_showAxisYCheck = nullptr;

	QCheckBox* m_showAxisZCheck = nullptr;

	QSpinBox* m_axisIntervalSpin = nullptr;

	QTimer* m_rediscretizeTimer = nullptr;

};

