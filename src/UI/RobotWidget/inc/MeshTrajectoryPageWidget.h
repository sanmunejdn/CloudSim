#ifndef ROBOTWIDGET_MESHTRAJECTORYPAGEWIDGET_H
#define ROBOTWIDGET_MESHTRAJECTORYPAGEWIDGET_H

/// @file MeshTrajectoryPageWidget.h
/// @brief MeshTrajectoryPageWidget 接口

#include "robotwidget_global.h"

#include <QWidget>
#include <memory>

#include <MeshTrajectorySession.h>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPushButton;
class QSpinBox;
class QStackedWidget;

class IRobotMainWindowHost;
class TrajectoryEditSession;
class RobotSimulationController;

class ROBOTWIDGET_EXPORT MeshTrajectoryPageWidget : public QWidget
{
	Q_OBJECT

public:
	explicit MeshTrajectoryPageWidget(QWidget* parent = nullptr);
	~MeshTrajectoryPageWidget() override;

	void setUseChinese(bool chinese);
	void bindHost(IRobotMainWindowHost* host);
	void bindSession(TrajectoryEditSession* session);
	void bindSimulationController(RobotSimulationController* controller);

	/// 轨迹编辑 Apply/生成程序后清空 mesh 选择与预览
	void resetAfterTrajectoryCommit();

private slots:
	void onGenerateClicked();
	void onClearSelectionClicked();
	void onInvertSelectionClicked();
	void onPickClickClicked();
	void onPickBrushClicked();
	void onPickPolylineClicked();
	void onCancelPickClicked();
	void onMethodChanged(int index);
	void onBackendChanged(int index);
	void onEditSectionClicked();
	void onPlaneSpinChanged();
	void onFromCameraNormalClicked();
	void onBsplineParamChanged();
	void onShowSectionToggled(bool checked);

private:
	void updateUiLabels();
	void applyMethodVisibility();
	void refreshBackendCombo();
	void reloadMeshSession();
	void syncSelectionHighlight();
	void syncMethodPreview();
	void syncSectionPlanePreview();
	void syncBsplineSurfacePreview();
	void clearMethodPreview();
	void cancelActivePick();
	void endSectionPlaneEdit();
	void hideSectionPlanePreview();
	void startSectionPlaneEdit();
	void syncPlaneSpinboxesFromModel(const double origin[3], const double normal[3]);
	void pushPlaneSpinboxesToOsg();
	void applySelectionIndices(const std::vector<int>& indices, MeshTrajectorySelectionMode mode);
	void wirePickHandlers();
	void fillBsplineParamsFromUi(geoalgo::MeshTrajectoryBsplineParams& out) const;

	bool m_chinese = true;
	bool m_sectionEditActive = false;
	bool m_shuttingDown = false;
	bool m_syncingPlaneSpinboxes = false;
	IRobotMainWindowHost* m_host = nullptr;
	TrajectoryEditSession* m_session = nullptr;
	RobotSimulationController* m_simController = nullptr;

	std::unique_ptr<MeshTrajectorySession> m_meshSession;

	QComboBox* m_backendCombo = nullptr;
	QGroupBox* m_selGroup = nullptr;
	QLabel* m_selectionLabel = nullptr;
	QComboBox* m_methodCombo = nullptr;
	QStackedWidget* m_methodStack = nullptr;
	QWidget* m_crossPage = nullptr;
	QWidget* m_bsplinePage = nullptr;
	QGroupBox* m_crossDiscGroup = nullptr;
	QGroupBox* m_bsplineDiscGroup = nullptr;
	QDoubleSpinBox* m_planeOx = nullptr;
	QDoubleSpinBox* m_planeOy = nullptr;
	QDoubleSpinBox* m_planeOz = nullptr;
	QDoubleSpinBox* m_planeNx = nullptr;
	QDoubleSpinBox* m_planeNy = nullptr;
	QDoubleSpinBox* m_planeNz = nullptr;
	QPushButton* m_fromCameraNormalBtn = nullptr;
	QCheckBox* m_showSectionCheck = nullptr;
	QPushButton* m_editSectionBtn = nullptr;
	QSpinBox* m_uvCountU = nullptr;
	QSpinBox* m_uvCountV = nullptr;
	QDoubleSpinBox* m_gridAngleDeg = nullptr;
	QDoubleSpinBox* m_fitUvSpacingMm = nullptr;
	QComboBox* m_nurbsFitModeCombo = nullptr;
	QComboBox* m_traceModeCombo = nullptr;
	QDoubleSpinBox* m_stepMm = nullptr;
	QCheckBox* m_outputNormalCheck = nullptr;
	QCheckBox* m_outputTangentCheck = nullptr;
	QCheckBox* m_bsplineOutputNormalCheck = nullptr;
	QCheckBox* m_bsplineOutputTangentCheck = nullptr;
	QPushButton* m_pickClickBtn = nullptr;
	QPushButton* m_pickBrushBtn = nullptr;
	QPushButton* m_pickPolylineBtn = nullptr;
	QPushButton* m_cancelPickBtn = nullptr;
	QPushButton* m_clearSelBtn = nullptr;
	QPushButton* m_invertSelBtn = nullptr;
	QPushButton* m_generateBtn = nullptr;
	QLabel* m_statusLabel = nullptr;
};

#endif // ROBOTWIDGET_MESHTRAJECTORYPAGEWIDGET_H
