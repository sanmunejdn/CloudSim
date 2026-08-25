/// @file MainWindow.cpp
/// @brief 主窗口编排

#include "MainWindow.h"

#include "../RobotWidget/inc/DeviceCommandPageWidget.h"
#include "../RobotWidget/inc/FeatureTrajectoryPageWidget.h"
#include "../RobotWidget/inc/IRobotOsgViewHost.h"
#include "../RobotWidget/inc/RobotAxisControlWidget.h"
#include "../RobotWidget/inc/RobotExternalAxisSettingsWidget.h"
#include "../RobotWidget/inc/RobotCollisionSettingsWidget.h"
#include "../RobotWidget/inc/RobotCommPageWidget.h"
#include "../RobotWidget/inc/RobotFrameSettingsWidget.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"
#include "../RobotWidget/inc/IoSignalNetworkService.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"
#include "../RobotWidget/inc/TrajectoryEditPageWidget.h"
#include "../RobotWidget/inc/TrajectoryGenerationPageWidget.h"
#include "AiAssistantDockWidget.h"
#include "ApplicationStyle.h"
#include "ApplicationSettings.h"
#include "AssemblyMatePanel.h"
#include "BackendFollowSolve.h"
#include "BackendHierarchyFollow.h"
#include "BackendSceneDocumentFacade.h"
#include "BackendVisualSync.h"
#include "CoreTypes.h"
#include "DevicePageWidget.h"
#include "DocumentHostEvents.h"
#include "DocumentPage.h"
#include "io/CustomDeviceRobotMountOps.h"
#include "IoSignalPageWidget.h"
#include "IDataService.h"
#include "IRenderView.h"
#include "IRobotBackendPoseSink.h"
#include "JobSystem.h"
#include "MainWindowRobotHost.h"
#include "MainWindowSelectionService.h"
#include "MainWindow_p.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionTransform.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotProgramExport.h"
#include "RobotTeachIk.h"
#include "RunInfoPage.h"
#include "RunLogger.h"
#include "StyledDockTitleBar.h"
#include "WidgetRenderAccess.h"
#include "PluginHostContext.h"
#include "PluginManager.h"
#include "qteditorfactory.h"
#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

#include <QAbstractItemView>
#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHeaderView>
#include <QJsonObject>
#include <QList>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QRegularExpression>
#include <QSet>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QToolBar>
#include <QMetaObject>
#include <QStatusBar>
#include <QStringList>
#include <QTabBar>
#include <QTabWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QVector>
#include <QWidget>
#include <QXmlStreamReader>
#include <algorithm>
#include <cmath>
#include <functional>
#include <locale>
#include <memory>
#include <sstream>
#include <unordered_set>

#include <Adapters.h>
#include <RigidTransform.h>
#include <ToolKinematics.h>

using namespace mainwindow_detail;
using namespace RobotSimulation;

QString MainWindow::i18n(const QString& en, const QString& zh) const
{
	return m_useChinese ? zh : en;
}

bool MainWindow::useChinese() const
{
	return m_useChinese;
}

void MainWindow::loadUiPreferencesFromStorage()
{
	m_uiPreferences = ApplicationSettings::load();
	m_useChinese = !m_uiPreferences.language.startsWith(QStringLiteral("en"), Qt::CaseInsensitive);
}

bool MainWindow::sidePanelTabSavedVisible(const QString& key, const bool defaultVisible) const
{
	return m_uiPreferences.sidePanelTabs.contains(key) ? m_uiPreferences.sidePanelTabs.value(key) : defaultVisible;
}

void MainWindow::applySavedViewLayout()
{
	m_restoringUiPreferences = true;
	setLeftSidePanelVisible(m_uiPreferences.leftPanelVisible);
	setRightSidePanelVisible(m_uiPreferences.rightPanelVisible);
	if (m_uiPreferences.leftDockWidth >= 160 && m_propertyDock)
	{
		m_leftDockSavedWidth = m_uiPreferences.leftDockWidth;
		resizeDocks({m_propertyDock}, {m_leftDockSavedWidth}, Qt::Horizontal);
	}
	if (m_uiPreferences.rightDockWidth >= 160 && m_unitDock)
	{
		m_rightDockSavedWidth = m_uiPreferences.rightDockWidth;
		resizeDocks({m_unitDock}, {m_rightDockSavedWidth}, Qt::Horizontal);
	}

	for (auto it = m_sidePanelTabToggles.constBegin(); it != m_sidePanelTabToggles.constEnd(); ++it)
	{
		QWidget* widget = it.key();
		if (!widget)
		{
			continue;
		}
		const QString key = ApplicationSettings::sidePanelTabKey(widget);
		const bool visible = sidePanelTabSavedVisible(key, true);
		if (it.value().viewAction)
		{
			const QSignalBlocker blocker(it.value().viewAction);
			it.value().viewAction->setChecked(visible);
		}
		applySidePanelTabToggleVisibility(widget, visible);
	}
	syncSidePanelToggleUi();
	m_restoringUiPreferences = false;
}

void MainWindow::applyLanguage()
{
	setWindowTitle(i18n(QStringLiteral("CloudSim - MainWindow"), QStringLiteral("CloudSim - 主窗口")));
	if (m_fileMenu)
		m_fileMenu->setTitle(i18n(QStringLiteral("File"), QStringLiteral("文件")));
	if (m_newDocumentAction)
	{
		m_newDocumentAction->setText(i18n(QStringLiteral("New"), QStringLiteral("新建")));
	}
	if (m_openModelAction)
		m_openModelAction->setText(i18n(QStringLiteral("Open Model..."), QStringLiteral("打开模型...")));
	if (m_openPointCloudAction)
		m_openPointCloudAction->setText(i18n(QStringLiteral("Open Point Cloud..."), QStringLiteral("打开点云...")));
	if (m_openProjectAction)
		m_openProjectAction->setText(i18n(QStringLiteral("Open Project..."), QStringLiteral("打开工程...")));
	if (m_saveAction)
		m_saveAction->setText(i18n(QStringLiteral("Save Project..."), QStringLiteral("保存工程...")));
	if (m_exitAction)
		m_exitAction->setText(i18n(QStringLiteral("Exit"), QStringLiteral("退出")));
	if (m_viewMenu)
		m_viewMenu->setTitle(i18n(QStringLiteral("View"), QStringLiteral("视图")));
	if (m_insertMenu)
		m_insertMenu->setTitle(i18n(QStringLiteral("Insert"), QStringLiteral("插入")));
	if (m_createCoordinateFrameAction)
	{
		m_createCoordinateFrameAction->setText(
			i18n(QStringLiteral("Coordinate Frame..."), QStringLiteral("坐标系...")));
	}
	if (m_assemblyMateAction)
	{
		m_assemblyMateAction->setText(i18n(QStringLiteral("Mate..."), QStringLiteral("配合...")));
	}
	if (m_assemblyMateDock)
	{
		m_assemblyMateDock->setWindowTitle(i18n(QStringLiteral("Mate"), QStringLiteral("配合")));
	}
	if (m_assemblyMatePanel)
	{
		m_assemblyMatePanel->applyLanguage();
	}
	if (m_resetLayoutAction)
	{
		m_resetLayoutAction->setText(i18n(QStringLiteral("Reset Layout"), QStringLiteral("重置布局")));
	}
	if (m_toggleLeftPanelAction)
	{
		m_toggleLeftPanelAction->setText(i18n(QStringLiteral("Left Panel"), QStringLiteral("左侧面板")));
	}
	if (m_toggleRightPanelAction)
	{
		m_toggleRightPanelAction->setText(i18n(QStringLiteral("Right Panel"), QStringLiteral("右侧面板")));
	}
	if (m_settingsMenu)
		m_settingsMenu->setTitle(i18n(QStringLiteral("Settings"), QStringLiteral("设置")));
	if (m_helpMenu)
		m_helpMenu->setTitle(i18n(QStringLiteral("Help"), QStringLiteral("帮助")));
	if (m_helpDocumentationAction)
	{
		m_helpDocumentationAction->setText(
			i18n(QStringLiteral("Documentation"), QStringLiteral("帮助文档")));
	}
	if (m_aboutAction)
	{
		m_aboutAction->setText(i18n(QStringLiteral("About CloudSim"), QStringLiteral("关于 CloudSim")));
	}
	if (m_workspaceModeMenu)
		m_workspaceModeMenu->setTitle(i18n(QStringLiteral("Mode Switch"), QStringLiteral("模式切换")));
	rebuildWorkspaceModeSwitcher();
	if (m_appearanceMenu)
	{
		m_appearanceMenu->setTitle(i18n(QStringLiteral("Theme"), QStringLiteral("风格")));
	}
	if (m_lightThemeAction)
	{
		m_lightThemeAction->setText(i18n(QStringLiteral("Light"), QStringLiteral("浅色")));
	}
	if (m_darkThemeAction)
	{
		m_darkThemeAction->setText(i18n(QStringLiteral("Dark"), QStringLiteral("深色")));
	}
	if (m_languageMenu)
		m_languageMenu->setTitle(i18n(QStringLiteral("Language"), QStringLiteral("语言")));
	if (m_languageEnglishAction)
		m_languageEnglishAction->setText(QStringLiteral("English"));
	if (m_languageChineseAction)
		m_languageChineseAction->setText(QStringLiteral("中文"));
	if (m_languageEnglishAction)
		m_languageEnglishAction->setChecked(!m_useChinese);
	if (m_languageChineseAction)
		m_languageChineseAction->setChecked(m_useChinese);

	if (m_viewModeAction)
		m_viewModeAction->setText(i18n(QStringLiteral("View Mode"), QStringLiteral("视图模式")));
	if (m_objectModeAction)
		m_objectModeAction->setText(i18n(QStringLiteral("Object Select"), QStringLiteral("对象选择")));
	if (m_pointPickModeAction)
		m_pointPickModeAction->setText(i18n(QStringLiteral("Point Pick"), QStringLiteral("点选模式")));
	if (m_meshLinePickModeAction)
		m_meshLinePickModeAction->setText(i18n(QStringLiteral("Line Pick"), QStringLiteral("线选择模式")));
	if (m_meshFacePickModeAction)
		m_meshFacePickModeAction->setText(i18n(QStringLiteral("Face Pick"), QStringLiteral("面选择模式")));
	if (m_gizmoLocalFrameAction)
	{
		m_gizmoLocalFrameAction->setText(
			i18n(QStringLiteral("Transform: Local (object axes)"), QStringLiteral("变换：物体系（罗盘轴）")));
	}
	if (m_gizmoWorldFrameAction)
	{
		m_gizmoWorldFrameAction->setText(i18n(QStringLiteral("Transform: World"), QStringLiteral("变换：世界系")));
	}

	if (m_propertyDock)
	{
		m_propertyDock->setWindowTitle(i18n(QStringLiteral("Property"), QStringLiteral("属性")));
	}
	if (m_propertyDockTabs && m_propertyDockTabs->count() >= 2)
	{
		m_propertyDockTabs->setTabText(0, i18n(QStringLiteral("Property"), QStringLiteral("属性")));
		m_propertyDockTabs->setTabText(1, i18n(QStringLiteral("Devices"), QStringLiteral("设备")));
		if (m_propertyDockTabs->count() >= 3)
		{
			m_propertyDockTabs->setTabText(2, i18n(QStringLiteral("Signals"), QStringLiteral("信号")));
		}
	}
	if (m_devicePage)
	{
		m_devicePage->setUseChinese(m_useChinese);
	}
	if (m_ioSignalPage)
	{
		m_ioSignalPage->setUseChinese(m_useChinese);
	}
	if (m_unitDock)
	{
		m_unitDock->setWindowTitle(m_processFlowSideUiActive
									   ? i18n(QStringLiteral("AI Assistant"), QStringLiteral("AI 助手"))
									   : i18n(QStringLiteral("Workspace"), QStringLiteral("工作区")));
	}
	if (m_processFlowLeftDock)
	{
		// 几何建模等会设 panel windowTitle；无则仍是工艺流程节点库
		QWidget* leftW = m_processFlowLeftDock->widget();
		m_processFlowLeftDock->setWindowTitle(
			(leftW && !leftW->windowTitle().isEmpty())
				? leftW->windowTitle()
				: i18n(QStringLiteral("Node Library"), QStringLiteral("节点库")));
	}
	if (m_processFlowRightDock)
	{
		QWidget* rightW = m_processFlowRightDock->widget();
		m_processFlowRightDock->setWindowTitle(
			(rightW && !rightW->windowTitle().isEmpty())
				? rightW->windowTitle()
				: i18n(QStringLiteral("Simulation"), QStringLiteral("仿真面板")));
	}
	if (m_rightPanelTabs && m_rightPanelTabs->count() >= 1 && !m_processFlowSideUiActive && m_unitDockTabs)
	{
		const int wsIdx = m_rightPanelTabs->indexOf(m_unitDockTabs);
		if (wsIdx >= 0)
		{
			m_rightPanelTabs->setTabText(wsIdx, i18n(QStringLiteral("Workspace"), QStringLiteral("工作区")));
		}
	}
	if (m_aiAssistantPage)
	{
		setPluginSidePanelTabTitle(m_aiAssistantPage, i18n(QStringLiteral("AI Assistant"), QStringLiteral("AI 助手")));
	}
	if (m_unitDockTabs && m_unitDockTabs->count() >= 3)
	{
		m_unitDockTabs->setTabText(0, i18n(QStringLiteral("Units"), QStringLiteral("单元部件")));
		m_unitDockTabs->setTabText(1, i18n(QStringLiteral("Devices"), QStringLiteral("设备")));
		m_unitDockTabs->setTabText(2, i18n(QStringLiteral("Scene graph"), QStringLiteral("场景层级")));
	}
	if (simulationCommandPage())
	{
		simulationCommandPage()->setUseChinese(m_useChinese);
	}
	if (RobotSimulationDockWidget* simDock = m_robotSimulation ? m_robotSimulation->simulationDock() : nullptr)
	{
		simDock->setUseChinese(m_useChinese);
		if (RobotAxisControlWidget* axis = simDock->axisPage())
		{
			axis->setUseChinese(m_useChinese);
		}
		if (DeviceCommandPageWidget* deviceCmd = simDock->deviceCommandPage())
		{
			deviceCmd->setUseChinese(m_useChinese);
		}
		if (RobotFrameSettingsWidget* frame = simDock->framePage())
		{
			frame->setUseChinese(m_useChinese);
		}
		if (RobotExternalAxisSettingsWidget* ext = simDock->externalAxisPage())
		{
			ext->setUseChinese(m_useChinese);
		}
		if (RobotCollisionSettingsWidget* col = simDock->collisionPage())
		{
			col->setUseChinese(m_useChinese);
		}
		if (TrajectoryEditPageWidget* traj = simDock->trajectoryEditPage())
		{
			traj->setUseChinese(m_useChinese);
		}
		if (TrajectoryGenerationPageWidget* gen = simDock->trajectoryGenerationPage())
		{
			gen->setUseChinese(m_useChinese);
		}
		if (RobotCommPageWidget* comm = simDock->robotCommPage())
		{
			comm->setUseChinese(m_useChinese);
		}
	}
	refreshSimulationJointListFromCurrentDoc();
	if (m_runDock)
	{
		m_runDock->setWindowTitle(i18n(QStringLiteral("Runtime Output"), QStringLiteral("运行信息")));
	}
	if (m_aiAssistantPage)
	{
		m_aiAssistantPage->setUseChinese(m_useChinese);
	}
	if (m_runInfoPage)
	{
		m_runInfoPage->setUiLanguage(m_useChinese);
	}

	if (m_propertyBrowser)
	{
		if (QTreeWidget* tw = m_propertyBrowser->findChild<QTreeWidget*>())
		{
			tw->setHeaderLabels(QStringList() << i18n(QStringLiteral("Property"), QStringLiteral("属性"))
											  << i18n(QStringLiteral("Value"), QStringLiteral("值")));
			tw->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked |
								QAbstractItemView::EditKeyPressed | QAbstractItemView::AnyKeyPressed);
		}
	}
	if (m_osgSceneTree)
	{
		m_osgSceneTree->setHeaderLabels(QStringList()
										<< i18n(QStringLiteral("Node"), QStringLiteral("节点"))
										<< i18n(QStringLiteral("Local transform"), QStringLiteral("本地变换矩阵")));
	}
	if (m_unitsTreeBinder)
	{
		m_unitsTreeBinder->setAnnotationGroupLabel(i18n(QStringLiteral("Annotations"), QStringLiteral("注释")));
	}
	refreshBackendTree();
	notifyPluginsLanguageChanged();
	if (m_documentTabs)
	{
		for (int i = 0; i < m_documentTabs->count(); ++i)
		{
			if (auto* page = qobject_cast<DocumentPage*>(m_documentTabs->widget(i)))
			{
				page->setViewportToolBarUseChinese(m_useChinese);
			}
		}
	}
}

namespace
{
bool robotSelectionUsesGizmoAnchor(const DocumentPage* doc, const QString& selectionId)
{
	return doc && !selectionId.isEmpty() && doc->robotGizmoAnchorBackendId(selectionId) != selectionId;
}

bool robotPerLinkAnchorGizmoOwnsPoseSync(const DocumentPage* doc, const QString& backendId)
{
	if (!doc || backendId.isEmpty())
	{
		return false;
	}
	bool isSceneRoot = false;
	const int instIdx = doc->robotInstanceIndexForPerLinkBackend(backendId, &isSceneRoot);
	if (instIdx < 0 || isSceneRoot || !doc->robotUsesPerLinkBackendsForInstance(instIdx))
	{
		return false;
	}
	const QString anchorId = doc->robotGizmoAnchorBackendId(doc->robotSceneBackendIdForInstance(instIdx));
	return !anchorId.isEmpty() && backendId == anchorId;
}

} // namespace

void MainWindow::onSelectedObjectPoseChanged(float x, float y, float z)
{
	if (sender() != renderWidgetFromPage(currentPage()))
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const MainWindowSelectionService::SelectionSnapshot snapshot = MainWindowSelectionService::currentSelection(*this);
	if (!snapshot.valid())
	{
		return;
	}
	DocumentPage* doc = currentPage();
	if (!doc || !doc->data().isValid(snapshot.backendId))
	{
		return;
	}
	if (robotSelectionUsesGizmoAnchor(doc, snapshot.backendId))
	{
		return;
	}
	// 根连杆 gizmo 由 FK 钩子写 Data + Follow；此处 applyWorldPoseMm 会冲掉 FK 结果
	if (robotPerLinkAnchorGizmoOwnsPoseSync(doc, snapshot.backendId))
	{
		if (doc->render().isTransformGizmoDragging())
		{
			syncPropertyPanelGizmoLiveValues(snapshot.backendId);
		}
		return;
	}
	cloudsim::core::PoseDto pose = doc->data().worldPoseMm(snapshot.backendId);
	pose.positionMm.x = static_cast<double>(x);
	pose.positionMm.y = static_cast<double>(y);
	pose.positionMm.z = static_cast<double>(z);
	float rx = 0, ry = 0, rz = 0;
	if (doc->render().selectedRotationEulerDeg(rx, ry, rz))
	{
		pose.eulerDeg.x = static_cast<double>(rx);
		pose.eulerDeg.y = static_cast<double>(ry);
		pose.eulerDeg.z = static_cast<double>(rz);
	}
	QString err;
	if (!doc->data().applyWorldPoseMm(snapshot.backendId, pose, &err))
	{
		return;
	}
	refreshFollowSolveAndPropertyPanelFromOsgWrite(snapshot.backendId);
}

void MainWindow::onSelectedObjectRotationChanged(float rx, float ry, float rz)
{
	if (sender() != renderWidgetFromPage(currentPage()))
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const MainWindowSelectionService::SelectionSnapshot snapshot = MainWindowSelectionService::currentSelection(*this);
	if (!snapshot.valid())
	{
		return;
	}
	DocumentPage* doc = currentPage();
	if (!doc || !doc->data().isValid(snapshot.backendId))
	{
		return;
	}
	if (robotSelectionUsesGizmoAnchor(doc, snapshot.backendId))
	{
		return;
	}
	if (robotPerLinkAnchorGizmoOwnsPoseSync(doc, snapshot.backendId))
	{
		if (doc->render().isTransformGizmoDragging())
		{
			syncPropertyPanelGizmoLiveValues(snapshot.backendId);
		}
		return;
	}
	cloudsim::core::PoseDto pose = doc->data().worldPoseMm(snapshot.backendId);
	float px = 0, py = 0, pz = 0;
	if (doc->render().selectedPosition(px, py, pz))
	{
		pose.positionMm.x = static_cast<double>(px);
		pose.positionMm.y = static_cast<double>(py);
		pose.positionMm.z = static_cast<double>(pz);
	}
	pose.eulerDeg.x = static_cast<double>(rx);
	pose.eulerDeg.y = static_cast<double>(ry);
	pose.eulerDeg.z = static_cast<double>(rz);
	QString err;
	if (!doc->data().applyWorldPoseMm(snapshot.backendId, pose, &err))
	{
		return;
	}
	refreshFollowSolveAndPropertyPanelFromOsgWrite(snapshot.backendId);
}

void MainWindow::onSelectedObjectColorChanged(float r, float g, float b, float a)
{
	if (sender() != renderWidgetFromPage(currentPage()))
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const MainWindowSelectionService::SelectionSnapshot snapshot = MainWindowSelectionService::currentSelection(*this);
	if (!snapshot.valid())
	{
		return;
	}
	DocumentPage* doc = currentPage();
	if (!doc || !doc->data().isValid(snapshot.backendId))
	{
		return;
	}
	cloudsim::core::ColorDto color;
	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;
	QString err;
	if (!doc->data().applyColor(snapshot.backendId, color, &err))
	{
		return;
	}
	refreshFollowSolveAndPropertyPanelFromOsgWrite(snapshot.backendId);
}

void MainWindow::onTransformGizmoCommitted()
{
	if (sender() != renderWidgetFromPage(currentPage()))
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const QString backendId = m_selectionState.selectedBackendId();
	if (backendId.isEmpty())
	{
		return;
	}
	DocumentPage* doc = currentPage();
	if (!doc)
	{
		return;
	}
	bool isSceneRoot = false;
	const int instIdx = doc->robotInstanceIndexForPerLinkBackend(backendId, &isSceneRoot);
	if (instIdx >= 0 && isSceneRoot && doc->robotUsesPerLinkBackendsForInstance(instIdx))
	{
		// 松手再 FK 一次：避免拖动中钩子未触发时只拧了单连杆
		refreshPerLinkRobotObjectGizmoFk(*doc);
		doc->data().markFollowDirtyFromMove(backendId);
		runFollowSolveAndSyncForPage(*doc);
		updatePropertyPanel(backendId);
		cloudsim::host::publishPoseCommittedFromBackendId(*doc, backendId);
		return;
	}
	if (instIdx >= 0 && !isSceneRoot && doc->robotUsesPerLinkBackendsForInstance(instIdx) &&
		backendId == doc->robotGizmoAnchorBackendId(doc->robotSceneBackendIdForInstance(instIdx)))
	{
		refreshPerLinkRobotObjectGizmoFk(*doc);
		syncRobotKinematicsAfterPoseEdit(backendId);
		runFollowSolveAndSyncForPage(*doc);
		updatePropertyPanel(backendId);
		cloudsim::host::publishPoseCommittedFromBackendId(*doc, backendId);
		return;
	}
	(void)doc->render().commitGizmoPoseToBackend(backendId);
	syncRobotKinematicsAfterPoseEdit(backendId);
	// follower 手动拖完要烘焙 local，否则松手 Follow 会用旧偏移写回
	cloudsim::host::bakeFollowLocalAfterManualPoseEdit(*doc, backendId.toStdString());
	if (cloudsim::host::rebakeMountedDeviceFromInstallFramePose(*doc, backendId.toStdString()))
	{
		cloudsim::core::FollowSolveContextDto ctx;
		(void)doc->data().runFollowSolveAndSync(ctx, nullptr);
	}
	doc->data().markFollowDirtyFromMove(backendId);
	updatePropertyPanel(backendId);
	cloudsim::host::publishPoseCommittedFromBackendId(*doc, backendId);
}

void MainWindow::refreshFollowSolveAndPropertyPanelFromOsgWrite(const QString& backendId)
{
	if (backendId.isEmpty())
	{
		return;
	}
	DocumentPage* doc = currentPage();
	if (!doc)
	{
		return;
	}
	doc->data().markFollowDirtyFromMove(backendId);
	cloudsim::core::IRenderView* rv = &doc->render();
	const bool dragging = rv->isTransformGizmoDragging();
	// 拖动中也要求解跟随：机器人 FK / 目标件位移时 follower 须实时跟；手动拖 follower 由 gizmoSelectedBackendId 排除
	(void)doc->data().runFollowSolveAndSync(makeFollowSolveContextDto(*doc), nullptr);
	cloudsim::host::refreshCustomDevicesFollowingKinematicsTargets(*doc);
	if (dragging || shouldDeferPropertyPanelRebuild(backendId))
	{
		if (dragging && m_propertyKeyToVariant.isEmpty())
		{
			updatePropertyPanel(backendId);
		}
		else if (dragging)
		{
			syncPropertyPanelGizmoLiveValues(backendId);
		}
		else
		{
			syncPropertyPanelRowValues(backendId);
		}
	}
	else if (isInlineTextPropertyEditActive(backendId))
	{
		return;
	}
	else
	{
		updatePropertyPanel(backendId);
	}
}

void MainWindow::schedulePropertyPanelCommitRefresh(const QString& backendId)
{
	if (backendId.isEmpty())
	{
		return;
	}
	m_propertyPanelCommitPendingBackendId = backendId;
	m_propertyPanelCommitTimer.start(220);
}

void MainWindow::onPropertyPanelCommitTimer()
{
	const QString want = m_propertyPanelCommitPendingBackendId;
	m_propertyPanelCommitPendingBackendId.clear();
	if (want.isEmpty())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection() || m_selectionState.selectedBackendId() != want)
	{
		return;
	}
	if (shouldDeferPropertyPanelRebuild(want) || isInlineTextPropertyEditActive(want))
	{
		m_propertyPanelCommitPendingBackendId = want;
		m_propertyPanelCommitTimer.start(220);
		return;
	}
	updatePropertyPanel(want);
}

void MainWindow::onActiveAxisChanged(const QString& axisName)
{
	if (sender() != renderWidgetFromPage(currentPage()))
	{
		return;
	}
	m_activeAxisName = axisName;
	// 换选会 emit None；不可清掉换选重建的 update 守卫
	if (!m_variantManager)
	{
		return;
	}
	QtProperty* prop = m_propertyKeyToVariant.value(QStringLiteral("ui.active_axis"));
	if (!prop)
	{
		return;
	}
	const QSignalBlocker blocker(m_variantManager);
	const bool wasUpdating = m_updatingPropertyBrowser;
	m_updatingPropertyBrowser = true;
	m_variantManager->setValue(prop, axisName);
	m_updatingPropertyBrowser = wasUpdating;
}

namespace
{
void resetInteractionPickModes(IRobotOsgViewHost& view)
{
	view.setObjectSelectionMode(false);
	view.setPointPickMode(false);
	view.setMeshLinePickMode(false);
	view.setMeshFacePickMode(false);
}
} // namespace

void MainWindow::onViewModeTriggered()
{
	IRobotOsgViewHost* view = activeOsgViewHost();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction ||
		!m_meshFacePickModeAction || !view)
	{
		return;
	}
	m_viewModeAction->setChecked(true);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	if (DocumentPage* page = currentPage())
	{
		page->setViewportObjectSelectionChecked(false);
	}
	if (m_assemblyMatePanel)
	{
		m_assemblyMatePanel->interruptPicking();
	}
	resetInteractionPickModes(*view);
}

void MainWindow::onObjectModeTriggered()
{
	IRobotOsgViewHost* view = activeOsgViewHost();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction ||
		!m_meshFacePickModeAction || !view)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(true);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	if (DocumentPage* page = currentPage())
	{
		page->setViewportObjectSelectionChecked(true);
	}
	if (m_assemblyMatePanel)
	{
		m_assemblyMatePanel->interruptPicking();
	}
	view->setObjectSelectionMode(true);
	view->setPointPickMode(false);
	view->setMeshLinePickMode(false);
	view->setMeshFacePickMode(false);

	// 导入后 activeBackend 常停在末根连杆；仅 setSelectionActive 会把罗盘挂在该轴上
	DocumentPage* doc = currentPage();
	if (doc && doc->render().hasImportedContent())
	{
		QString seed = m_selectionState.selectedBackendId();
		if (seed.isEmpty())
		{
			const std::string active = doc->render().activeBackendId();
			if (!active.empty())
			{
				seed = QString::fromStdString(active);
			}
		}
		if (!seed.isEmpty())
		{
			MainWindowSelectionService::handleOsgBackendObjectPicked(*this, seed);
		}
		else
		{
			view->setSelectionActive(true);
		}
	}
}

void MainWindow::onPointPickModeTriggered()
{
	IRobotOsgViewHost* view = activeOsgViewHost();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction ||
		!m_meshFacePickModeAction || !view)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(true);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	if (DocumentPage* page = currentPage())
	{
		page->setViewportObjectSelectionChecked(false);
	}
	if (m_assemblyMatePanel)
	{
		m_assemblyMatePanel->interruptPicking();
	}
	view->setObjectSelectionMode(false);
	view->setPointPickMode(true);
	view->setMeshLinePickMode(false);
	view->setMeshFacePickMode(false);
	MainWindowSelectionService::ensureBackendForPickMode(*this,
														 MainWindowSelectionService::SelectedBackendKind::PointCloud);
}

void MainWindow::onMeshLinePickModeTriggered()
{
	IRobotOsgViewHost* view = activeOsgViewHost();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction ||
		!m_meshFacePickModeAction || !view)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(true);
	m_meshFacePickModeAction->setChecked(false);
	if (DocumentPage* page = currentPage())
	{
		page->setViewportObjectSelectionChecked(false);
	}
	if (m_assemblyMatePanel)
	{
		m_assemblyMatePanel->interruptPicking();
	}
	view->setObjectSelectionMode(false);
	view->setPointPickMode(false);
	view->setMeshLinePickMode(true);
	view->setMeshFacePickMode(false);
	MainWindowSelectionService::ensureBackendForPickMode(*this, MainWindowSelectionService::SelectedBackendKind::Mesh);
}

void MainWindow::onMeshFacePickModeTriggered()
{
	IRobotOsgViewHost* view = activeOsgViewHost();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction ||
		!m_meshFacePickModeAction || !view)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(true);
	if (DocumentPage* page = currentPage())
	{
		page->setViewportObjectSelectionChecked(false);
	}
	if (m_assemblyMatePanel)
	{
		m_assemblyMatePanel->interruptPicking();
	}
	view->setObjectSelectionMode(false);
	view->setPointPickMode(false);
	view->setMeshLinePickMode(false);
	view->setMeshFacePickMode(true);
	MainWindowSelectionService::ensureBackendForPickMode(*this, MainWindowSelectionService::SelectedBackendKind::Mesh);
}

void MainWindow::onSelectionCanceledByEsc()
{
	if (sender() != renderWidgetFromPage(currentPage()))
	{
		return;
	}
	IRobotOsgViewHost* view = activeOsgViewHost();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction ||
		!m_meshFacePickModeAction || !view)
	{
		return;
	}
	m_viewModeAction->setChecked(true);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	if (DocumentPage* page = currentPage())
	{
		page->setViewportObjectSelectionChecked(false);
	}
	resetInteractionPickModes(*view);
	MainWindowSelectionService::clearSelection(*this, true);
}

void MainWindow::onLanguageEnglishTriggered()
{
	m_useChinese = false;
	applyLanguage();
	persistUiPreferencesToStorage();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("UI language switched to English."));
	}
}

void MainWindow::onLanguageChineseTriggered()
{
	m_useChinese = true;
	applyLanguage();
	persistUiPreferencesToStorage();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("\u754c\u9762\u8bed\u8a00\u5df2\u5207\u6362\u4e3a\u4e2d\u6587\u3002"));
	}
}

void MainWindow::onThemeActionGroupTriggered(QAction* action)
{
	if (!action)
	{
		return;
	}
	if (action == m_lightThemeAction)
	{
		ApplicationStyle::applyTheme(qApp, ApplicationStyle::Theme::Light);
		ApplicationStyle::saveTheme(ApplicationStyle::Theme::Light);
		if (m_lightThemeAction && m_darkThemeAction)
		{
			QSignalBlocker blockLight(m_lightThemeAction);
			QSignalBlocker blockDark(m_darkThemeAction);
			m_lightThemeAction->setChecked(true);
			m_darkThemeAction->setChecked(false);
		}
		setAllDocumentViewerDarkBackground(false);
		refreshModeToolBarTheme();
	}
	else if (action == m_darkThemeAction)
	{
		ApplicationStyle::applyTheme(qApp, ApplicationStyle::Theme::Dark);
		ApplicationStyle::saveTheme(ApplicationStyle::Theme::Dark);
		if (m_lightThemeAction && m_darkThemeAction)
		{
			QSignalBlocker blockLight(m_lightThemeAction);
			QSignalBlocker blockDark(m_darkThemeAction);
			m_lightThemeAction->setChecked(false);
			m_darkThemeAction->setChecked(true);
		}
		setAllDocumentViewerDarkBackground(true);
		refreshModeToolBarTheme();
	}
}

void MainWindow::setAllDocumentViewerDarkBackground(bool dark)
{
	if (!m_documentTabs)
	{
		return;
	}
	for (int i = 0; i < m_documentTabs->count(); ++i)
	{
		auto* p = qobject_cast<DocumentPage*>(m_documentTabs->widget(i));
		if (p)
		{
			p->render().setViewerBackgroundForDarkUi(dark);
			p->setViewportToolBarDarkTheme(dark);
		}
	}
}

namespace
{
constexpr int kDefaultSideDockWidth = 240;
constexpr int kDefaultRightDockWidth = 360;
constexpr int kMinRestorableDockWidth = 160;

bool sideDockShown(const QDockWidget* dock)
{
	// 主窗未 show 时 isVisible() 恒 false；用 isHidden 表示用户/逻辑是否收起过
	return dock && !dock->isHidden();
}

void showSideDock(QDockWidget* dock, int& savedWidth, const int fallbackWidth)
{
	if (!dock || sideDockShown(dock))
	{
		return;
	}
	dock->show();
	const int w = savedWidth >= kMinRestorableDockWidth ? savedWidth : fallbackWidth;
	savedWidth = w;
}

void hideSideDock(QDockWidget* dock, int& savedWidth)
{
	if (!sideDockShown(dock))
	{
		return;
	}
	const int w = dock->width();
	if (w >= kMinRestorableDockWidth)
	{
		savedWidth = w;
	}
	dock->hide();
}
} // namespace

void MainWindow::persistUiPreferencesToStorage()
{
	m_uiPreferences.language = m_useChinese ? QStringLiteral("zh") : QStringLiteral("en");
	m_uiPreferences.theme = ApplicationStyle::loadSavedTheme();
	m_uiPreferences.leftPanelVisible =
		m_processFlowSideUiActive ? sideDockShown(m_processFlowLeftDock) : sideDockShown(m_propertyDock);
	m_uiPreferences.rightPanelVisible =
		m_processFlowSideUiActive
			? (sideDockShown(m_unitDock)
			   || (m_processFlowUsesRightDock && sideDockShown(m_processFlowRightDock)))
			: sideDockShown(m_unitDock);
	if (m_propertyDock && m_propertyDock->width() >= kMinRestorableDockWidth)
	{
		m_uiPreferences.leftDockWidth = m_propertyDock->width();
	}
	if (m_unitDock && m_unitDock->width() >= kMinRestorableDockWidth)
	{
		m_uiPreferences.rightDockWidth = m_unitDock->width();
	}

	m_uiPreferences.sidePanelTabs.clear();
	for (auto it = m_sidePanelTabToggles.constBegin(); it != m_sidePanelTabToggles.constEnd(); ++it)
	{
		const QWidget* widget = it.key();
		if (!widget)
		{
			continue;
		}
		const QString key = ApplicationSettings::sidePanelTabKey(widget);
		const bool visible =
			it.value().viewAction ? it.value().viewAction->isChecked()
								  : (m_rightPanelTabs && m_rightPanelTabs->indexOf(const_cast<QWidget*>(widget)) >= 0);
		m_uiPreferences.sidePanelTabs.insert(key, visible);
	}

	if (m_pluginManager && m_pluginManager->hostContext())
	{
		m_uiPreferences.workspaceModeId = m_pluginManager->hostContext()->currentWorkspaceMode();
	}

	ApplicationSettings::save(m_uiPreferences);
}

void MainWindow::setLeftSidePanelVisible(const bool visible)
{
	if (m_processFlowSideUiActive)
	{
		if (!m_processFlowLeftDock)
		{
			return;
		}
		if (visible)
		{
			showSideDock(m_processFlowLeftDock, m_processFlowLeftSavedWidth, 260);
			resizeDocks({m_processFlowLeftDock}, {m_processFlowLeftSavedWidth}, Qt::Horizontal);
		}
		else
		{
			hideSideDock(m_processFlowLeftDock, m_processFlowLeftSavedWidth);
		}
		syncSidePanelToggleUi();
		if (!m_restoringUiPreferences)
		{
			persistUiPreferencesToStorage();
		}
		return;
	}

	if (!m_propertyDock)
	{
		return;
	}
	if (visible)
	{
		showSideDock(m_propertyDock, m_leftDockSavedWidth, kDefaultSideDockWidth);
		resizeDocks({m_propertyDock}, {m_leftDockSavedWidth}, Qt::Horizontal);
	}
	else
	{
		hideSideDock(m_propertyDock, m_leftDockSavedWidth);
	}
	syncSidePanelToggleUi();
	if (!m_restoringUiPreferences)
	{
		persistUiPreferencesToStorage();
	}
}

void MainWindow::setRightSidePanelVisible(const bool visible)
{
	if (m_processFlowSideUiActive)
	{
		if (!m_processFlowRightDock && !m_unitDock)
		{
			return;
		}
		if (visible)
		{
			if (m_processFlowUsesRightDock && m_processFlowRightDock)
			{
				showSideDock(m_processFlowRightDock, m_processFlowRightSavedWidth, 320);
				resizeDocks({m_processFlowRightDock}, {m_processFlowRightSavedWidth}, Qt::Horizontal);
			}
			if (m_unitDock)
			{
				showSideDock(m_unitDock, m_rightDockSavedWidth, kDefaultRightDockWidth);
			}
		}
		else
		{
			if (m_processFlowUsesRightDock)
			{
				hideSideDock(m_processFlowRightDock, m_processFlowRightSavedWidth);
			}
			hideSideDock(m_unitDock, m_rightDockSavedWidth);
		}
		syncSidePanelToggleUi();
		if (!m_restoringUiPreferences)
		{
			persistUiPreferencesToStorage();
		}
		return;
	}

	if (!m_unitDock)
	{
		return;
	}
	if (visible)
	{
		showSideDock(m_unitDock, m_rightDockSavedWidth, kDefaultRightDockWidth);
		resizeDocks({m_unitDock}, {m_rightDockSavedWidth}, Qt::Horizontal);
	}
	else
	{
		hideSideDock(m_unitDock, m_rightDockSavedWidth);
	}
	syncSidePanelToggleUi();
	if (!m_restoringUiPreferences)
	{
		persistUiPreferencesToStorage();
	}
}

void MainWindow::syncSidePanelToggleUi()
{
	const bool leftVisible =
		m_processFlowSideUiActive ? sideDockShown(m_processFlowLeftDock) : sideDockShown(m_propertyDock);
	const bool rightVisible =
		m_processFlowSideUiActive
			? (sideDockShown(m_unitDock)
			   || (m_processFlowUsesRightDock && sideDockShown(m_processFlowRightDock)))
			: sideDockShown(m_unitDock);

	if (m_toggleLeftPanelAction)
	{
		const QSignalBlocker blocker(m_toggleLeftPanelAction);
		m_toggleLeftPanelAction->setChecked(leftVisible);
	}
	if (m_toggleRightPanelAction)
	{
		const QSignalBlocker blocker(m_toggleRightPanelAction);
		m_toggleRightPanelAction->setChecked(rightVisible);
	}
	if (!m_documentTabs)
	{
		return;
	}
	for (int i = 0; i < m_documentTabs->count(); ++i)
	{
		auto* page = qobject_cast<DocumentPage*>(m_documentTabs->widget(i));
		if (page)
		{
			page->syncViewportSidePanelToggleState(leftVisible, rightVisible);
		}
	}
}

bool MainWindow::viewerUsesDarkBackground() const
{
	// 与 QSettings 一致；菜单勾选在导入机器人等长流程后可能与已保存主题不同步
	return ApplicationStyle::usesDarkTheme();
}

DocumentPage* MainWindow::currentPage()
{
	if (!m_documentTabs)
	{
		return nullptr;
	}
	return qobject_cast<DocumentPage*>(m_documentTabs->currentWidget());
}

DocumentPage* MainWindow::currentPage() const
{
	return const_cast<MainWindow*>(this)->currentPage();
}

DocumentPage* MainWindow::pageByDocumentId(const QString& documentId) const
{
	if (!m_documentTabs || documentId.isEmpty())
	{
		return nullptr;
	}
	for (int i = 0; i < m_documentTabs->count(); ++i)
	{
		auto* page = qobject_cast<DocumentPage*>(m_documentTabs->widget(i));
		if (page && page->documentId() == documentId)
		{
			return page;
		}
	}
	return nullptr;
}

bool MainWindow::activateDocumentById(const QString& documentId)
{
	DocumentPage* page = pageByDocumentId(documentId);
	if (!page || !m_documentTabs)
	{
		return false;
	}
	if (m_documentTabs->currentWidget() == page)
	{
		return true;
	}
	// 树选中驱动切页时勿 clearSelection，否则选中态被冲掉；并阻断 currentChanged 以免重入 rebuild
	const QSignalBlocker guard(m_documentTabs);
	m_documentTabs->setCurrentWidget(page);
	stopRobotSimulation();
	rebuildUnitsDocument(documentId);
	refreshOsgSceneTree();
	syncViewModeActionsFromCurrentOsg();
	refreshSimulationJointListFromCurrentDoc();
	return true;
}

cloudsim::host::DocumentHost* MainWindow::currentDocumentHost()
{
	return currentPage();
}

cloudsim::host::DocumentHost* MainWindow::documentHostAt(int tabIndex)
{
	if (!m_documentTabs || tabIndex < 0 || tabIndex >= m_documentTabs->count())
	{
		return nullptr;
	}
	return qobject_cast<DocumentPage*>(m_documentTabs->widget(tabIndex));
}

void MainWindow::appendRunInfo(const QString& message)
{
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(message);
	}
}

void MainWindow::enqueueBackgroundJob(const QString& title,
									  std::function<void(const PluginJobProgressFn& progress)> work,
									  std::function<void(bool threw, const QString& message)> onFinished)
{
	if (!m_jobSystem)
	{
		if (onFinished)
		{
			onFinished(true, QStringLiteral("JobSystem not available"));
		}
		return;
	}
	m_jobSystem->enqueue(
		title,
		[work = std::move(work)](const JobProgressSink& sink)
		{
			if (work)
			{
				PluginJobProgressFn pluginSink = [&sink](double fraction, const QString& msg)
				{
					if (sink)
					{
						sink(fraction, msg);
					}
				};
				work(pluginSink);
			}
		},
		std::move(onFinished));
}

quint64 MainWindow::enqueueCancellableBackgroundJob(
	const QString& title,
	std::function<void(const PluginJobProgressFn& progress, const PluginJobCanceledFn& canceled)> work,
	std::function<void(bool threw, const QString& message)> onFinished)
{
	if (!m_jobSystem)
	{
		if (onFinished)
		{
			onFinished(true, QStringLiteral("JobSystem not available"));
		}
		return 0;
	}
	return m_jobSystem->enqueueCancellable(
		title,
		[work = std::move(work)](const JobProgressSink& sink, const JobCancelToken& token)
		{
			if (!work)
			{
				return;
			}
			PluginJobProgressFn pluginSink = [&sink](double fraction, const QString& msg)
			{
				if (sink)
				{
					sink(fraction, msg);
				}
			};
			work(pluginSink, [&token]() { return token.canceled(); });
		},
		std::move(onFinished));
}

bool MainWindow::cancelBackgroundJob(quint64 jobId)
{
	return m_jobSystem ? m_jobSystem->cancel(jobId) : false;
}

QDockWidget* MainWindow::addPluginDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area)
{
	auto* dock = new QDockWidget(title, this);
	static int s_anonDockSerial = 0;
	QString objectName = widget ? widget->objectName() : QString();
	if (objectName.isEmpty())
	{
		objectName = QStringLiteral("PluginDock_anon_%1").arg(++s_anonDockSerial);
	}
	else if (!objectName.startsWith(QStringLiteral("PluginDock_")))
	{
		objectName = QStringLiteral("PluginDock_") + objectName;
	}
	dock->setObjectName(objectName);
	dock->setWidget(widget);
	addDockWidget(area, dock);
	return dock;
}

void MainWindow::setCentralAlternateWidget(QWidget* widget)
{
	if (cloudsim::host::DocumentHost* doc = currentDocumentHost())
	{
		doc->setCentralAlternateWidget(widget);
	}
}

void MainWindow::showCentralScene3D()
{
	if (cloudsim::host::DocumentHost* doc = currentDocumentHost())
	{
		doc->showCentralScene3D();
	}
}

void MainWindow::showCentralAlternate()
{
	if (cloudsim::host::DocumentHost* doc = currentDocumentHost())
	{
		doc->showCentralAlternate();
	}
}

bool MainWindow::isShowingCentralAlternate() const
{
	const cloudsim::host::DocumentHost* doc =
		const_cast<MainWindow*>(this)->currentDocumentHost();
	return doc && doc->isShowingCentralAlternate();
}

bool MainWindow::embedActiveRenderWidget(QWidget* slot, QString* outError)
{
	cloudsim::host::DocumentHost* doc = currentDocumentHost();
	if (!doc)
	{
		if (outError)
			*outError = QStringLiteral("No active document.");
		return false;
	}
	return doc->embedRenderWidget(slot, outError);
}

void MainWindow::restoreActiveRenderWidget()
{
	if (cloudsim::host::DocumentHost* doc = currentDocumentHost())
	{
		doc->restoreRenderWidget();
	}
}

void MainWindow::setModeToolBar(QWidget* toolBar)
{
	if (!m_modeToolBar)
	{
		m_modeToolBar = addToolBar(QStringLiteral("Mode"));
		m_modeToolBar->setObjectName(QStringLiteral("ModeToolBar"));
		m_modeToolBar->setMovable(false);
		m_modeToolBar->setFloatable(false);
		m_modeToolBar->setAllowedAreas(Qt::TopToolBarArea);
		m_modeToolBar->setContextMenuPolicy(Qt::PreventContextMenu);
		m_modeToolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
	}

	// addWidget 走 QWidgetAction：只改子控件 visible 无效，会两套 Ribbon 并排
	auto setExclusive = [this](QWidget* active)
	{
		for (QAction* a : m_modeToolBar->actions())
		{
			if (QWidget* w = m_modeToolBar->widgetForAction(a))
			{
				const bool on = (active != nullptr && w == active);
				a->setVisible(on);
				w->setVisible(on);
			}
		}
	};

	if (!toolBar)
	{
		setExclusive(nullptr);
		m_modeToolBar->hide();
		return;
	}

	bool found = false;
	for (QAction* a : m_modeToolBar->actions())
	{
		if (m_modeToolBar->widgetForAction(a) == toolBar)
		{
			found = true;
			break;
		}
	}
	if (!found)
	{
		toolBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
		m_modeToolBar->addWidget(toolBar);
	}
	setExclusive(toolBar);
	m_modeToolBar->show();
	refreshModeToolBarTheme();
}

void MainWindow::refreshModeToolBarTheme()
{
	if (!m_modeToolBar)
	{
		return;
	}
	const bool dark = ApplicationStyle::usesDarkTheme();
	for (QAction* a : m_modeToolBar->actions())
	{
		if (QWidget* w = m_modeToolBar->widgetForAction(a))
		{
			QMetaObject::invokeMethod(w, "applyTheme", Qt::DirectConnection, Q_ARG(bool, dark));
		}
	}
}

void MainWindow::notifyWorkspaceModesChanged()
{
	rebuildWorkspaceModeSwitcher();
}

void MainWindow::markActiveDocumentModified()
{
	DocumentPage* doc = currentPage();
	if (!doc || !m_documentTabs)
		return;
	const QString id = doc->documentId();
	if (id.isEmpty())
		return;
	m_modifiedDocumentIds.insert(id);
	const int idx = m_documentTabs->indexOf(doc);
	if (idx < 0)
		return;
	QString text = m_documentTabs->tabText(idx);
	if (!text.endsWith(QLatin1Char('*')))
		m_documentTabs->setTabText(idx, text + QLatin1Char('*'));
}

void MainWindow::clearActiveDocumentModified()
{
	DocumentPage* doc = currentPage();
	if (!doc || !m_documentTabs)
		return;
	const QString id = doc->documentId();
	m_modifiedDocumentIds.remove(id);
	const int idx = m_documentTabs->indexOf(doc);
	if (idx < 0)
		return;
	QString text = m_documentTabs->tabText(idx);
	while (text.endsWith(QLatin1Char('*')))
		text.chop(1);
	m_documentTabs->setTabText(idx, text);
}

bool MainWindow::isActiveDocumentModified() const
{
	const DocumentPage* doc = currentPage();
	if (!doc)
		return false;
	return m_modifiedDocumentIds.contains(doc->documentId());
}

void MainWindow::rebuildWorkspaceModeSwitcher()
{
	struct ModeItem
	{
		QString modeId;
		QString titleZh;
		QString titleEn;
	};
	std::vector<ModeItem> items;
	if (m_pluginManager && m_pluginManager->hostContext())
	{
		for (const auto& reg : m_pluginManager->hostContext()->workspaceModes())
		{
			items.push_back({reg.modeId, reg.titleZh, reg.titleEn});
		}
	}
	if (items.empty())
		items.push_back({QString(), QStringLiteral("主程序"), QStringLiteral("Main")});
	const QString cur =
		(m_pluginManager && m_pluginManager->hostContext()) ? m_pluginManager->hostContext()->currentWorkspaceMode()
															: QString();
	const bool zh = useChinese();

	if (!m_workspaceModeMenu || !m_workspaceModeActionGroup)
		return;

	const QList<QAction*> old = m_workspaceModeMenu->actions();
	for (QAction* a : old)
	{
		m_workspaceModeActionGroup->removeAction(a);
		m_workspaceModeMenu->removeAction(a);
		delete a;
	}
	for (const auto& item : items)
	{
		QAction* act = m_workspaceModeMenu->addAction(zh ? item.titleZh : item.titleEn);
		act->setCheckable(true);
		act->setData(item.modeId);
		act->setChecked(item.modeId == cur);
		m_workspaceModeActionGroup->addAction(act);
	}
}

void MainWindow::onWorkspaceModeMenuTriggered(QAction* action)
{
	if (!action)
		return;
	onWorkspaceModeRequested(action->data().toString());
}

void MainWindow::enterProcessFlowSideUi(QWidget* leftPanel, QWidget* rightPanel)
{
	m_processFlowSideUiActive = true;
	m_processFlowUsesRightDock = (rightPanel != nullptr);
	m_unitDockVisibleBeforeProcessFlow = sideDockShown(m_unitDock);
	m_propertyDockVisibleBeforeProcessFlow = sideDockShown(m_propertyDock);
	hideSideDock(m_propertyDock, m_leftDockSavedWidth);

	if (!m_processFlowLeftDock)
	{
		m_processFlowLeftDock =
			new QDockWidget(i18n(QStringLiteral("Node Library"), QStringLiteral("节点库")), this);
		m_processFlowLeftDock->setObjectName(QStringLiteral("ProcessFlowLeftDock"));
		m_processFlowLeftDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
		applyStyledDockTitleBar(m_processFlowLeftDock);
		addDockWidget(Qt::LeftDockWidgetArea, m_processFlowLeftDock);
	}
	else if (!qobject_cast<StyledDockTitleBar*>(m_processFlowLeftDock->titleBarWidget()))
	{
		applyStyledDockTitleBar(m_processFlowLeftDock);
	}

	{
		const QString leftTitle =
			(leftPanel && !leftPanel->windowTitle().isEmpty())
				? leftPanel->windowTitle()
				: i18n(QStringLiteral("Node Library"), QStringLiteral("节点库"));
		m_processFlowLeftDock->setWindowTitle(leftTitle);
	}

	if (rightPanel)
	{
		if (!m_processFlowRightDock)
		{
			m_processFlowRightDock =
				new QDockWidget(i18n(QStringLiteral("Simulation"), QStringLiteral("仿真面板")), this);
			m_processFlowRightDock->setObjectName(QStringLiteral("ProcessFlowRightDock"));
			m_processFlowRightDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
			applyStyledDockTitleBar(m_processFlowRightDock);
			addDockWidget(Qt::RightDockWidgetArea, m_processFlowRightDock);
		}
		{
			const QString rightTitle =
				!rightPanel->windowTitle().isEmpty()
					? rightPanel->windowTitle()
					: i18n(QStringLiteral("Simulation"), QStringLiteral("仿真面板"));
			m_processFlowRightDock->setWindowTitle(rightTitle);
		}
		if (!qobject_cast<StyledDockTitleBar*>(m_processFlowRightDock->titleBarWidget()))
		{
			applyStyledDockTitleBar(m_processFlowRightDock);
		}
		if (m_processFlowRightDock->widget() != rightPanel)
		{
			m_processFlowRightDock->setWidget(rightPanel);
		}
	}

	if (leftPanel && m_processFlowLeftDock->widget() != leftPanel)
	{
		m_processFlowLeftDock->setWidget(leftPanel);
	}

	if (leftPanel)
	{
		showSideDock(m_processFlowLeftDock, m_processFlowLeftSavedWidth, 260);
		resizeDocks({m_processFlowLeftDock}, {m_processFlowLeftSavedWidth}, Qt::Horizontal);
	}
	else
	{
		hideSideDock(m_processFlowLeftDock, m_processFlowLeftSavedWidth);
	}

	// 无右栏时拆掉与 AI 的 tab 组，避免底栏残留「仿真面板」
	if (rightPanel)
	{
		showSideDock(m_processFlowRightDock, m_processFlowRightSavedWidth, 320);
		resizeDocks({m_processFlowRightDock}, {m_processFlowRightSavedWidth}, Qt::Horizontal);
	}
	else if (m_processFlowRightDock)
	{
		hideSideDock(m_processFlowRightDock, m_processFlowRightSavedWidth);
		removeDockWidget(m_processFlowRightDock);
		addDockWidget(Qt::RightDockWidgetArea, m_processFlowRightDock);
		m_processFlowRightDock->hide();
	}

	// 仅保留 AI：去掉工作区及插件页签，Dock 标题改为 AI 助手
	detachNonAiRightTabsForProcessFlow();
	if (m_unitDock)
	{
		addDockWidget(Qt::RightDockWidgetArea, m_unitDock);
		showSideDock(m_unitDock, m_rightDockSavedWidth, kDefaultRightDockWidth);
		if (m_processFlowRightDock && rightPanel)
		{
			tabifyDockWidget(m_processFlowRightDock, m_unitDock);
			m_unitDock->raise();
		}
	}

	syncSidePanelToggleUi();
}

void MainWindow::detachNonAiRightTabsForProcessFlow()
{
	if (!m_rightPanelTabs || !m_aiAssistantPage)
	{
		return;
	}
	if (!m_processFlowDetachedRightTabs.isEmpty())
	{
		return;
	}

	applySidePanelTabToggleVisibility(m_aiAssistantPage, true);

	for (int i = 0; i < m_rightPanelTabs->count(); ++i)
	{
		QWidget* w = m_rightPanelTabs->widget(i);
		if (!w || w == m_aiAssistantPage)
		{
			continue;
		}
		ProcessFlowDetachedRightTab d;
		d.widget = w;
		d.title = m_rightPanelTabs->tabText(i);
		d.index = i;
		m_processFlowDetachedRightTabs.append(d);
	}
	for (const ProcessFlowDetachedRightTab& d : m_processFlowDetachedRightTabs)
	{
		const int idx = m_rightPanelTabs->indexOf(d.widget);
		if (idx >= 0)
		{
			m_rightPanelTabs->removeTab(idx);
		}
	}

	m_rightPanelTabs->setCurrentWidget(m_aiAssistantPage);
	if (m_rightPanelTabs->tabBar())
	{
		m_rightPanelTabs->tabBar()->setVisible(false);
	}
	if (m_unitDock)
	{
		m_unitDock->setWindowTitle(i18n(QStringLiteral("AI Assistant"), QStringLiteral("AI 助手")));
	}
}

void MainWindow::restoreRightTabsAfterProcessFlow()
{
	if (m_unitDock)
	{
		m_unitDock->setWindowTitle(i18n(QStringLiteral("Workspace"), QStringLiteral("工作区")));
	}
	if (m_rightPanelTabs && m_rightPanelTabs->tabBar())
	{
		m_rightPanelTabs->tabBar()->setVisible(true);
	}
	if (!m_rightPanelTabs || m_processFlowDetachedRightTabs.isEmpty())
	{
		m_processFlowDetachedRightTabs.clear();
		return;
	}

	std::sort(m_processFlowDetachedRightTabs.begin(), m_processFlowDetachedRightTabs.end(),
			  [](const ProcessFlowDetachedRightTab& a, const ProcessFlowDetachedRightTab& b) { return a.index < b.index; });
	for (const ProcessFlowDetachedRightTab& d : m_processFlowDetachedRightTabs)
	{
		QWidget* w = d.widget.data();
		if (!w)
		{
			continue;
		}
		if (m_rightPanelTabs->indexOf(w) >= 0)
		{
			continue;
		}
		const int insertAt = qBound(0, d.index, m_rightPanelTabs->count());
		m_rightPanelTabs->insertTab(insertAt, w, d.title);
	}
	m_processFlowDetachedRightTabs.clear();
}

void MainWindow::discardProcessFlowRightTabsForShutdown()
{
	m_processFlowSideUiActive = false;
	m_processFlowUsesRightDock = false;
	// Host 自有「工作区」页签 removeTab 后无父对象，需挂回以免泄漏；插件页交给各自 shutdown delete
	if (m_rightPanelTabs && m_unitDockTabs && m_rightPanelTabs->indexOf(m_unitDockTabs) < 0)
	{
		m_rightPanelTabs->insertTab(0, m_unitDockTabs, i18n(QStringLiteral("Workspace"), QStringLiteral("工作区")));
	}
	if (m_rightPanelTabs && m_rightPanelTabs->tabBar())
	{
		m_rightPanelTabs->tabBar()->setVisible(true);
	}
	if (m_unitDock)
	{
		m_unitDock->setWindowTitle(i18n(QStringLiteral("Workspace"), QStringLiteral("工作区")));
	}
	m_processFlowDetachedRightTabs.clear();
}

void MainWindow::exitProcessFlowSideUi()
{
	if (!m_processFlowSideUiActive)
	{
		return;
	}
	m_processFlowSideUiActive = false;
	m_processFlowUsesRightDock = false;
	hideSideDock(m_processFlowLeftDock, m_processFlowLeftSavedWidth);
	hideSideDock(m_processFlowRightDock, m_processFlowRightSavedWidth);
	restoreRightTabsAfterProcessFlow();
	if (m_unitDock)
	{
		addDockWidget(Qt::RightDockWidgetArea, m_unitDock);
	}
	for (int i = 0; i < documentTabCount(); ++i)
	{
		if (cloudsim::host::DocumentHost* doc = documentHostAt(i))
		{
			doc->showCentralScene3D();
		}
	}
	if (m_propertyDockVisibleBeforeProcessFlow)
	{
		showSideDock(m_propertyDock, m_leftDockSavedWidth, kDefaultSideDockWidth);
		resizeDocks({m_propertyDock}, {m_leftDockSavedWidth}, Qt::Horizontal);
	}
	if (m_unitDockVisibleBeforeProcessFlow)
	{
		showSideDock(m_unitDock, m_rightDockSavedWidth, kDefaultRightDockWidth);
		resizeDocks({m_unitDock}, {m_rightDockSavedWidth}, Qt::Horizontal);
	}
	else
	{
		hideSideDock(m_unitDock, m_rightDockSavedWidth);
	}
	syncSidePanelToggleUi();
}

void MainWindow::enterAlternateSideUi(QWidget* leftPanel, QWidget* rightPanel)
{
	enterProcessFlowSideUi(leftPanel, rightPanel);
}

void MainWindow::exitAlternateSideUi()
{
	exitProcessFlowSideUi();
}

void MainWindow::afterBackendFollowPropertyEdited(const QString& propertyKey, const QString& valueText)
{
	(void)propertyKey;
	(void)valueText;
	DocumentPage* doc = currentPage();
	cloudsim::core::IRenderView* rv = doc ? &doc->render() : nullptr;
	if (!doc || !rv || !m_selectionState.hasBackendSelection())
	{
		return;
	}
	if (!doc->data().isValid(m_selectionState.selectedBackendId()))
	{
		return;
	}
	cloudsim::core::FollowSolveContextDto ctx;
	ctx.skipAll = false;
	if (rv->isTransformGizmoDragging())
	{
		ctx.gizmoSelectedBackendId = m_selectionState.selectedBackendId();
	}
	(void)doc->data().runFollowSolveAndSync(ctx, nullptr);
}

BackendHierarchyModel* MainWindow::activeHierarchyModel()
{
	DocumentPage* p = currentPage();
	return p ? &p->hierarchyModel() : nullptr;
}

const BackendHierarchyModel* MainWindow::activeHierarchyModel() const
{
	DocumentPage* p = currentPage();
	return p ? &p->hierarchyModel() : nullptr;
}

IRobotOsgViewHost* MainWindow::activeOsgViewHost()
{
	return m_robotHost ? m_robotHost->osgView() : nullptr;
}

cloudsim::core::FollowSolveContextDto MainWindow::makeFollowSolveContextDto(DocumentPage& page) const
{
	cloudsim::core::FollowSolveContextDto ctx;
	const cloudsim::core::IRenderView& rv = page.render();
	ctx.skipAll = false;
	// TCP 末端拖动与对象 gizmo 共用 isTransformGizmoDragging，勿把 follower 当手动拖
	if (rv.isTransformGizmoDragging() && !rv.isTcpDragTeachActive() && m_selectionState.hasBackendSelection())
	{
		ctx.gizmoSelectedBackendId = m_selectionState.selectedBackendId();
	}
	return ctx;
}

void MainWindow::runFollowSolveAndSyncForPage(DocumentPage& page, const std::string* manualPoseAuthorityBackendId)
{
	cloudsim::core::FollowSolveContextDto ctx = makeFollowSolveContextDto(page);
	if (manualPoseAuthorityBackendId && !manualPoseAuthorityBackendId->empty())
	{
		ctx.manualPoseAuthorityBackendId = QString::fromStdString(*manualPoseAuthorityBackendId);
	}
	(void)page.data().runFollowSolveAndSync(ctx, nullptr);
	cloudsim::host::refreshCustomDevicesFollowingKinematicsTargets(page);
}

void MainWindow::installBackendFollowFrameHook(DocumentPage* page)
{
	if (!page)
	{
		return;
	}
	page->render().setRobotObjectGizmoSyncHook([this, page]() -> bool
											   { return page && isPerLinkRobotObjectGizmoActive(page); });
	page->render().setRobotObjectGizmoFkRefreshHook(
		[this, page]()
		{
			if (page)
			{
				refreshPerLinkRobotObjectGizmoFk(*page);
			}
		});
	page->render().setPerFrameHook(
		[this, page]()
		{
			if (!page || !m_documentTabs || m_documentTabs->currentWidget() != page)
			{
				return;
			}
			cloudsim::core::IRenderView& rv = page->render();
			(void)rv;
			// 仅脏集 / forced；gizmo 拖动靠 markFollowDirty 填脏，勿对无关 follower 空转
			if (page->followDirtyBackendIds().empty() && !page->followSolveForcedPending())
			{
				return;
			}
			runFollowSolveAndSyncForPage(*page);
		});
}

bool MainWindow::isPerLinkRobotObjectGizmoActive(const DocumentPage* page) const
{
	if (!page)
	{
		return false;
	}
	const std::string activeIdStd = const_cast<DocumentPage*>(page)->render().activeBackendId();
	const QString activeId = QString::fromStdString(activeIdStd);
	if (activeId.isEmpty())
	{
		return false;
	}
	bool isSceneRoot = false;
	const int instIdx = page->robotInstanceIndexForPerLinkBackend(activeId, &isSceneRoot);
	if (instIdx < 0 || !page->robotUsesPerLinkBackendsForInstance(instIdx))
	{
		return false;
	}
	const QString anchorId = page->robotGizmoAnchorBackendId(page->robotSceneBackendIdForInstance(instIdx));
	// 必须挂在根连杆 mesh 上；空壳 scene 根无 outer，不能当拖动主体
	return !anchorId.isEmpty() && activeId == anchorId;
}

void MainWindow::refreshPerLinkRobotObjectGizmoFk(DocumentPage& doc)
{
	if (!m_robotSimulation)
	{
		return;
	}
	const std::string activeIdStd = doc.render().activeBackendId();
	const QString activeId = QString::fromStdString(activeIdStd);
	if (activeId.isEmpty())
	{
		return;
	}
	bool isSceneRoot = false;
	const int instIdx = doc.robotInstanceIndexForPerLinkBackend(activeId, &isSceneRoot);
	if (instIdx < 0)
	{
		return;
	}
	const QString anchorId = doc.robotGizmoAnchorBackendId(doc.robotSceneBackendIdForInstance(instIdx));
	if (anchorId.isEmpty())
	{
		return;
	}
	const QVector<double> all = m_robotSimulation->aggregatedJointAnglesRad();
	const int offset = doc.robotJointOffsetInAggregatedVector(instIdx);
	const int nj = doc.robotRevoluteJointNamesForInstance(instIdx).size();
	QVector<double> q(nj, 0.0);
	if (nj > 0 && all.size() >= offset + nj)
	{
		for (int j = 0; j < nj; ++j)
		{
			q[j] = all[offset + j];
		}
	}
	(void)doc.applyPerLinkRobotFkFromGizmoAnchor(instIdx, anchorId, q);
}

void MainWindow::applyHierarchyFollowBinding(DocumentPage* doc, const std::string& childId, const std::string& parentId)
{
	if (!doc)
	{
		return;
	}
	cloudsim::host::applyHierarchyFollowBinding(*doc, childId, parentId);
	runFollowSolveAndSyncForPage(*doc);
}

QString MainWindow::nextUntitledDocumentTitle() const
{
	const QString base = i18n(QStringLiteral("Untitled"), QStringLiteral("未命名"));
	QSet<int> used;
	if (m_documentTabs)
	{
		for (int i = 0; i < m_documentTabs->count(); ++i)
		{
			const QString text = m_documentTabs->tabText(i);
			if (text == base)
			{
				used.insert(1);
				continue;
			}
			const QString prefix = base + QLatin1Char(' ');
			if (!text.startsWith(prefix))
			{
				continue;
			}
			bool ok = false;
			const int n = text.mid(prefix.size()).toInt(&ok);
			if (ok && n > 0)
			{
				used.insert(n);
			}
		}
	}
	int n = 1;
	while (used.contains(n))
	{
		++n;
	}
	return base + QLatin1Char(' ') + QString::number(n);
}

void MainWindow::onNewDocument()
{
	if (!m_documentTabs)
	{
		return;
	}
	DocumentPage* page = createDocumentPageTab();
	if (!page)
	{
		return;
	}
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("New document."), QStringLiteral("\u65b0\u5efa\u6587\u6863\u3002")));
	}
}

DocumentPage* MainWindow::createDocumentPageTab(const QString& title)
{
	if (!m_documentTabs)
	{
		return nullptr;
	}
	auto* page = new DocumentPage(m_documentTabs, m_appEvents);
	wireDocumentPageSignals(page);
	page->render().setViewerBackgroundForDarkUi(viewerUsesDarkBackground());
	page->setViewportToolBarDarkTheme(viewerUsesDarkBackground());
	const QString tabTitle = title.isEmpty() ? nextUntitledDocumentTitle() : title;
	m_documentTabs->addTab(page, tabTitle);
	m_documentTabs->setCurrentWidget(page);
	return page;
}

bool MainWindow::isReusableBlankDocument(const DocumentPage* page) const
{
	if (!page)
	{
		return false;
	}
	if (!page->projectFilePath().isEmpty())
	{
		return false;
	}
	if (m_modifiedDocumentIds.contains(page->documentId()))
	{
		return false;
	}
	if (page->hasRobotSimulationContext())
	{
		return false;
	}
	return page->documentData().listAll().isEmpty();
}

DocumentPage* MainWindow::findDocumentPageByProjectPath(const QString& projectPath) const
{
	if (!m_documentTabs || projectPath.isEmpty())
	{
		return nullptr;
	}
	const QString want = QFileInfo(projectPath).absoluteFilePath();
	for (int i = 0; i < m_documentTabs->count(); ++i)
	{
		auto* page = qobject_cast<DocumentPage*>(m_documentTabs->widget(i));
		if (!page || page->projectFilePath().isEmpty())
		{
			continue;
		}
		if (QFileInfo(page->projectFilePath()).absoluteFilePath() == want)
		{
			return page;
		}
	}
	return nullptr;
}

void MainWindow::stashIoNetworkToDocument(DocumentPage* page)
{
	if (!page || !m_robotSimulation)
	{
		return;
	}
	page->setIoSignalNetworkCache(m_robotSimulation->ioSignalNetworkToJson());
}

void MainWindow::restoreIoNetworkFromDocument(DocumentPage* page)
{
	if (!m_robotSimulation)
	{
		return;
	}
	m_ioBoundDocumentPage = page;
	if (!page)
	{
		if (IoSignalNetworkService* net = m_robotSimulation->ioSignalNetwork())
		{
			net->clear();
		}
		return;
	}
	const QJsonObject cached = page->ioSignalNetworkCache();
	if (!cached.isEmpty())
	{
		QString err;
		(void)m_robotSimulation->ioSignalNetworkFromJson(cached, &err);
	}
	else if (IoSignalNetworkService* net = m_robotSimulation->ioSignalNetwork())
	{
		net->clear();
		m_robotSimulation->syncIoOwnersFromDocument();
	}
	if (m_ioSignalPage)
	{
		m_ioSignalPage->setNetwork(m_robotSimulation->ioSignalNetwork());
		m_ioSignalPage->refreshFromModel();
		m_robotSimulation->setIoUiOwnerId(m_ioSignalPage->currentOwnerId());
	}
}

void MainWindow::syncCollisionUiFromDocument(DocumentPage* page)
{
	if (!page || !m_robotSimulation || !m_robotSimulation->simulationDock() ||
		!m_robotSimulation->simulationDock()->collisionPage())
	{
		return;
	}
	m_robotSimulation->simulationDock()->collisionPage()->setSettings(page->robotCollisionSettings());
}

void MainWindow::onDocumentTabChanged(int)
{
	DocumentPage* next = currentPage();
	if (m_ioBoundDocumentPage && m_ioBoundDocumentPage != next)
	{
		stashIoNetworkToDocument(m_ioBoundDocumentPage);
	}
	if (m_robotSimulation && m_robotSimulation->programExecutor().isRunning())
	{
		// currentChanged 时 currentPage 已是新页；禁止把运行中关节角 FK 到新文档
		m_robotSimulation->stopRobotSimulation(false);
	}
	// 先换树再清选中，避免对旧文档节点发 selection/itemChanged 写可见性
	if (next)
	{
		rebuildUnitsDocument(next->documentId());
	}
	else if (m_unitsTreeBinder)
	{
		m_unitsTreeBinder->showOnlyDocument(QString());
	}
	MainWindowSelectionService::clearSelection(*this, true);
	restoreIoNetworkFromDocument(next);
	syncCollisionUiFromDocument(next);
	refreshOsgSceneTree();
	syncViewModeActionsFromCurrentOsg();
	refreshSimulationJointListFromCurrentDoc();
}

void MainWindow::closeDocumentTab(int index)
{
	if (!m_documentTabs || index < 0 || index >= m_documentTabs->count())
	{
		return;
	}

	// 如果只剩最后一个标签，不允许关闭（保持至少一个文档）
	if (m_documentTabs->count() <= 1)
	{
		return;
	}

	auto* page = qobject_cast<DocumentPage*>(m_documentTabs->widget(index));
	if (!page)
	{
		return;
	}

	// 检查文档是否有内容需要保存
	const bool hasContent = !page->data().listAll().isEmpty();
	if (hasContent)
	{
		const QString title = m_documentTabs->tabText(index);
		const QString msg = i18n(QStringLiteral("Save changes to '%1' before closing?").arg(title),
								 QStringLiteral("关闭前是否保存对 '%1' 的更改？").arg(title));

		QMessageBox::StandardButton result =
			QMessageBox::question(this, i18n(QStringLiteral("Close Document"), QStringLiteral("关闭文档")), msg,
								  QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);

		if (result == QMessageBox::Cancel)
		{
			return;
		}

		if (result == QMessageBox::Save)
		{
			// 先切换到目标标签，触发保存，再关闭
			m_documentTabs->setCurrentIndex(index);
			onSaveProject();
			// 如果用户取消了保存对话框（文件路径为空），不关闭
			if (page->projectFilePath().isEmpty() && m_documentTabs->count() > 1)
			{
				return;
			}
		}
	}

	// 停止仿真并清理状态（无论关闭的是哪个标签）
	if (index == m_documentTabs->currentIndex())
	{
		stopRobotSimulation();
		MainWindowSelectionService::clearSelection(*this, true);
	}

	const QString closingDocId = page->documentId();
	if (m_robotSimulation)
	{
		m_robotSimulation->forgetDocumentJointUiState(closingDocId);
	}
	if (m_ioBoundDocumentPage == page)
	{
		m_ioBoundDocumentPage = nullptr;
	}
	m_documentTabs->removeTab(index);
	if (m_unitsTreeBinder)
	{
		m_unitsTreeBinder->removeDocument(closingDocId);
	}
	m_unitsTreeDirtyDocumentIds.remove(closingDocId);
	if (m_pluginManager)
	{
		m_pluginManager->invokeDocumentClosed(closingDocId);
	}
	page->deleteLater();

	// 关闭后刷新当前标签的状态
	if (m_documentTabs->count() > 0)
	{
		onDocumentTabChanged(m_documentTabs->currentIndex());
	}
}

void MainWindow::syncViewModeActionsFromCurrentOsg()
{
	IRobotOsgViewHost* view = activeOsgViewHost();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction ||
		!m_meshFacePickModeAction)
	{
		return;
	}
	if (!view)
	{
		m_viewModeAction->setChecked(true);
		m_objectModeAction->setChecked(false);
		m_pointPickModeAction->setChecked(false);
		m_meshLinePickModeAction->setChecked(false);
		m_meshFacePickModeAction->setChecked(false);
		if (DocumentPage* page = currentPage())
		{
			page->setViewportObjectSelectionChecked(false);
		}
		if (m_gizmoFrameGroup && m_gizmoLocalFrameAction && m_gizmoWorldFrameAction)
		{
			const QSignalBlocker bg(m_gizmoFrameGroup);
			const QSignalBlocker b1(m_gizmoLocalFrameAction);
			const QSignalBlocker b2(m_gizmoWorldFrameAction);
			m_gizmoLocalFrameAction->setChecked(true);
			m_gizmoWorldFrameAction->setChecked(false);
		}
		return;
	}
	const bool inViewMode = !view->objectSelectionMode() && !view->pointPickMode() && !view->meshLinePickMode() &&
							!view->meshFacePickMode();
	m_viewModeAction->setChecked(inViewMode);
	m_objectModeAction->setChecked(view->objectSelectionMode());
	m_pointPickModeAction->setChecked(view->pointPickMode());
	m_meshLinePickModeAction->setChecked(view->meshLinePickMode());
	m_meshFacePickModeAction->setChecked(view->meshFacePickMode());
	if (DocumentPage* page = currentPage())
	{
		page->setViewportObjectSelectionChecked(view->objectSelectionMode());
	}
	if (m_gizmoFrameGroup && m_gizmoLocalFrameAction && m_gizmoWorldFrameAction)
	{
		const QSignalBlocker bg(m_gizmoFrameGroup);
		const QSignalBlocker b1(m_gizmoLocalFrameAction);
		const QSignalBlocker b2(m_gizmoWorldFrameAction);
		if (view->transformGizmoFrameIsLocal())
		{
			m_gizmoLocalFrameAction->setChecked(true);
			m_gizmoWorldFrameAction->setChecked(false);
		}
		else
		{
			m_gizmoLocalFrameAction->setChecked(false);
			m_gizmoWorldFrameAction->setChecked(true);
		}
	}
}

void MainWindow::onPointPickFeedback(const QString& text)
{
	if (sender() != renderWidgetFromPage(currentPage()))
	{
		return;
	}
	if (statusBar())
	{
		statusBar()->showMessage(text);
	}
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(text);
	}
}

void MainWindow::onMeshPickFeedback(const QString& text)
{
	onPointPickFeedback(text);
}
