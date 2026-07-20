#ifndef ROBOTWIDGET_TRAJECTORYGENERATIONPAGEWIDGET_H
#define ROBOTWIDGET_TRAJECTORYGENERATIONPAGEWIDGET_H

/// @file TrajectoryGenerationPageWidget.h
/// @brief 轨迹生成容器：路径规划顶栏 + CAD/BREP | Mesh

#include "robotwidget_global.h"

#include <QWidget>
#include <functional>

class QComboBox;

class QHBoxLayout;

class QLabel;

class QPushButton;

class QTabWidget;

class FeatureTrajectoryPageWidget;

class MeshTrajectoryPageWidget;

class IRobotMainWindowHost;

class TrajectoryEditSession;

class RobotSimulationController;

class RobotProgramStore;

class ProgramEditService;

class SimulationCommandWidget;

/// 轨迹生成容器：路径规划顶栏 + CAD/BREP | Mesh

class ROBOTWIDGET_EXPORT TrajectoryGenerationPageWidget : public QWidget

{
	Q_OBJECT

public:
	explicit TrajectoryGenerationPageWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);

	void bindHost(IRobotMainWindowHost* host);

	void bindSession(TrajectoryEditSession* session);

	void bindSimulationController(RobotSimulationController* controller);

	void bindStore(RobotProgramStore* store);

	void bindEditService(ProgramEditService* service);

	void bindCommandPage(SimulationCommandWidget* commandPage);

	void setStepPathResolver(std::function<QString(const QString& backendId)> resolver);

	FeatureTrajectoryPageWidget* brepPage() const { return m_brepPage; }

	MeshTrajectoryPageWidget* meshPage() const { return m_meshPage; }

	void resetAfterTrajectoryCommit();

private slots:

	void onPathPlanComboChanged(int index);

	void onNewPathPlanClicked();

	void onBeginEditClicked();

	void onCancelEditClicked();

	void onPathPlanBound(const std::string& pathPlanId);

private:
	void refreshPathPlanCombo();

	void updatePathPlanBarLabels();

	void updateEditButtonsState();

	QHBoxLayout* m_pathPlanBar = nullptr;

	QLabel* m_pathPlanLabel = nullptr;

	QComboBox* m_pathPlanCombo = nullptr;

	QPushButton* m_newPathPlanBtn = nullptr;

	QPushButton* m_beginEditBtn = nullptr;

	QPushButton* m_cancelEditBtn = nullptr;

	QTabWidget* m_tabs = nullptr;

	FeatureTrajectoryPageWidget* m_brepPage = nullptr;

	MeshTrajectoryPageWidget* m_meshPage = nullptr;

	RobotProgramStore* m_store = nullptr;

	ProgramEditService* m_editService = nullptr;

	TrajectoryEditSession* m_session = nullptr;

	SimulationCommandWidget* m_commandPage = nullptr;

	bool m_chinese = true;

	bool m_readOnly = false;
};

#endif // ROBOTWIDGET_TRAJECTORYGENERATIONPAGEWIDGET_H
