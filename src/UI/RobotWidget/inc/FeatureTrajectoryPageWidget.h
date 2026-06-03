#pragma once



#include "robotwidget_global.h"



#include "RobotOsgUiTypes.h"



#include <QWidget>



#include <functional>

#include <string>



namespace RobotInstruction

{

struct RawTrajectory;

}



class QCheckBox;

class QComboBox;

class QDoubleSpinBox;

class QGroupBox;

class QLabel;

class QPlainTextEdit;

class QPushButton;

class QSpinBox;

class QStackedWidget;

class QTimer;

class IRobotMainWindowHost;

class TrajectoryEditSession;

class RobotSimulationController;

namespace geoalgo
{
struct FeatureCatalog;
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

signals:
	void workpieceComboChanged();

private slots:

	void onLoadCatalog();

	void onDiscretize();

	void onPickEdge();

	void onPickFace();

	void onCancelPick();



private:

	enum class PickSessionKind

	{

		None,

		Edge,

		Face

	};



	void refreshBackendCombo();

	void updateUiLabels();

	void updatePickUiState();

	void updateDiscretizeParamMode();

	void showTrajectoryPreview(const RobotInstruction::RawTrajectory& traj);

	std::string resolvePreviewBackendId(const RobotInstruction::RawTrajectory& traj) const;

	RobotOsgUi::RawTrajectoryPreviewOptions currentPreviewOptions() const;

	void setStatus(const QString& text);

	void exitPickMode();

	void onMeshPickCommitted(const struct PickResult& pick, int pickKindInt);

	bool prepareSpecForDiscretize(std::string& jsonText, std::string* errMsg);

	bool applyDiscretizeUiToJson(std::string& jsonText);

	void syncDiscretizeUiFromSpecJson();

	bool discretizeFromEditor();

	bool isFaceUvGridKind() const;

	bool editorHasValidFeatureSpec() const;

	bool resolveDiscretizeBaseJson(std::string& outJson) const;

	void commitLastFeatureSpec(const std::string& jsonText);

	void updateActiveFeatureLabel();

	void scheduleParameterRediscretize();

	void onParameterRediscretize();

	void refreshPreviewFromSession();

	bool buildPreviewOverlayJson(const QByteArray& catalogSliceUtf8, QByteArray& outPreviewJson, QString* err) const;

	bool enumerateCatalogForBackend(const QString& backendId, geoalgo::FeatureCatalog& out, QString* err) const;

	bool autoEnumerateCatalogForCurrentWorkpiece(bool updateEditor, bool quiet, QString* err);

	bool shouldReplaceEditorWithCatalog() const;



	IRobotMainWindowHost* m_host = nullptr;

	TrajectoryEditSession* m_session = nullptr;

	RobotSimulationController* m_simController = nullptr;

	std::function<QString(const QString&)> m_stepPathResolver;

	bool m_chinese = true;

	bool m_hasLastFeatureSpec = false;

	bool m_suppressParamRediscretize = false;

	PickSessionKind m_pickSession = PickSessionKind::None;

	std::string m_lastFeatureSpecJson;

	QString m_cachedCatalogBackendId;

	std::string m_cachedCatalogJsonUtf8;



	QComboBox* m_backendCombo = nullptr;

	QComboBox* m_faceKindCombo = nullptr;

	QPlainTextEdit* m_specEditor = nullptr;

	QLabel* m_pickStatusLabel = nullptr;

	QLabel* m_activeFeatureLabel = nullptr;

	QPushButton* m_pickEdgeBtn = nullptr;

	QPushButton* m_pickFaceBtn = nullptr;

	QPushButton* m_cancelPickBtn = nullptr;

	QPushButton* m_catalogBtn = nullptr;

	QPushButton* m_discretizeBtn = nullptr;



	QGroupBox* m_discretizeGroup = nullptr;

	QStackedWidget* m_discretizeStack = nullptr;

	QDoubleSpinBox* m_stepMmSpin = nullptr;

	QDoubleSpinBox* m_linearDeflectionSpin = nullptr;

	QSpinBox* m_uvCountUSpin = nullptr;

	QSpinBox* m_uvCountVSpin = nullptr;

	QDoubleSpinBox* m_gridAngleSpin = nullptr;

	QCheckBox* m_showAxesCheck = nullptr;

	QSpinBox* m_axisIntervalSpin = nullptr;

	QTimer* m_rediscretizeTimer = nullptr;

};


