#pragma once

#include "robotwidget_global.h"

#include <QWidget>

#include <functional>

class QTabWidget;
class FeatureTrajectoryPageWidget;
class MeshTrajectoryPageWidget;

class IRobotMainWindowHost;
class TrajectoryEditSession;
class RobotSimulationController;

/// 轨迹生成容器：CAD/BREP | Mesh
class ROBOTWIDGET_EXPORT TrajectoryGenerationPageWidget : public QWidget
{
	Q_OBJECT

public:
	explicit TrajectoryGenerationPageWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void bindHost(IRobotMainWindowHost* host);
	void bindSession(TrajectoryEditSession* session);
	void bindSimulationController(RobotSimulationController* controller);
	void setStepPathResolver(std::function<QString(const QString& backendId)> resolver);

	FeatureTrajectoryPageWidget* brepPage() const { return m_brepPage; }
	MeshTrajectoryPageWidget* meshPage() const { return m_meshPage; }

private:
	QTabWidget* m_tabs = nullptr;
	FeatureTrajectoryPageWidget* m_brepPage = nullptr;
	MeshTrajectoryPageWidget* m_meshPage = nullptr;
};
