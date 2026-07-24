/// @file MainWindow.cpp
/// @brief MainWindow 实现

#include "MainWindow.h"

#include "../RobotWidget/inc/FeatureTrajectoryPageWidget.h"
#include "../RobotWidget/inc/IRobotOsgViewHost.h"
#include "../RobotWidget/inc/RobotAxisControlWidget.h"
#include "../RobotWidget/inc/RobotExternalAxisSettingsWidget.h"
#include "../RobotWidget/inc/RobotCollisionSettingsWidget.h"
#include "../RobotWidget/inc/RobotFrameSettingsWidget.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"
#include "../RobotWidget/inc/TrajectoryEditPageWidget.h"
#include "../RobotWidget/inc/TrajectoryGenerationPageWidget.h"
#include "AiAssistantDockWidget.h"
#include "ApplicationStyle.h"
#include "BackendFollowSolve.h"
#include "BackendHierarchyFollow.h"
#include "BackendSceneDocumentFacade.h"
#include "BackendVisualSync.h"
#include "CoreTypes.h"
#include "DevicePageWidget.h"
#include "DocumentHostEvents.h"
#include "DocumentPage.h"
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
#include "WidgetRenderAccess.h"
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
#include <QHeaderView>
#include <QList>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QRegularExpression>
#include <QSet>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QStringList>
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
	}
	if (m_devicePage)
	{
		m_devicePage->setUseChinese(m_useChinese);
	}
	if (m_unitDock)
	{
		m_unitDock->setWindowTitle(i18n(QStringLiteral("Workspace"), QStringLiteral("工作区")));
	}
	if (m_rightPanelTabs && m_rightPanelTabs->count() >= 1)
	{
		m_rightPanelTabs->setTabText(0, i18n(QStringLiteral("Workspace"), QStringLiteral("工作区")));
	}
	if (m_aiAssistantPage)
	{
		setPluginSidePanelTabTitle(m_aiAssistantPage, i18n(QStringLiteral("AI Assistant"), QStringLiteral("AI 助手")));
	}
	if (m_unitDockTabs && m_unitDockTabs->count() >= 3)
	{
		m_unitDockTabs->setTabText(0, i18n(QStringLiteral("Units"), QStringLiteral("单元部件")));
		m_unitDockTabs->setTabText(1, i18n(QStringLiteral("Robot"), QStringLiteral("机器人")));
		m_unitDockTabs->setTabText(2, i18n(QStringLiteral("Scene graph"), QStringLiteral("场景层级")));
	}
	if (simulationCommandPage())
	{
		simulationCommandPage()->setUseChinese(m_useChinese);
	}
	if (RobotSimulationDockWidget* simDock = m_robotSimulation ? m_robotSimulation->simulationDock() : nullptr)
	{
		if (RobotAxisControlWidget* axis = simDock->axisPage())
		{
			axis->setUseChinese(m_useChinese);
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
		QTabWidget* tabs = simDock->tabWidget();
		if (tabs && tabs->count() >= 2)
		{
			tabs->setTabText(RobotSimulationDockWidget::kTabIndexInstructions,
							 i18n(QStringLiteral("Instructions"), QStringLiteral("指令")));
			tabs->setTabText(RobotSimulationDockWidget::kTabIndexAxisControl,
							 i18n(QStringLiteral("Axis control"), QStringLiteral("轴控制")));
			if (tabs->count() > RobotSimulationDockWidget::kTabIndexFrames)
			{
				tabs->setTabText(RobotSimulationDockWidget::kTabIndexFrames,
								 i18n(QStringLiteral("Frames"), QStringLiteral("坐标系")));
			}
			if (tabs->count() > RobotSimulationDockWidget::kTabIndexExternalAxes)
			{
				tabs->setTabText(RobotSimulationDockWidget::kTabIndexExternalAxes,
								 i18n(QStringLiteral("External Axes"), QStringLiteral("外部轴")));
			}
			if (tabs->count() > RobotSimulationDockWidget::kTabIndexCollision)
			{
				tabs->setTabText(RobotSimulationDockWidget::kTabIndexCollision,
								 i18n(QStringLiteral("Collision"), QStringLiteral("碰撞检测")));
			}
			if (tabs->count() > RobotSimulationDockWidget::kTabIndexTrajectoryGeneration)
			{
				tabs->setTabText(RobotSimulationDockWidget::kTabIndexTrajectoryGeneration,
								 i18n(QStringLiteral("Trajectory Generation"), QStringLiteral("轨迹生成")));
			}
			if (tabs->count() > RobotSimulationDockWidget::kTabIndexTrajectoryEdit)
			{
				tabs->setTabText(RobotSimulationDockWidget::kTabIndexTrajectoryEdit,
								 i18n(QStringLiteral("Trajectory Edit"), QStringLiteral("轨迹编辑")));
			}
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
		doc->data().markFollowDirtyFromMove(backendId);
		updatePropertyPanel(backendId);
		cloudsim::host::publishPoseCommittedFromBackendId(*doc, backendId);
		return;
	}
	(void)doc->render().commitGizmoPoseToBackend(backendId);
	syncRobotKinematicsAfterPoseEdit(backendId);
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
	if (!dragging)
	{
		cloudsim::core::FollowSolveContextDto ctx;
		// 仅 TCP 示教互斥跟随；仿真 Run 必须解跟随，否则绑定工件/工具静止
		ctx.skipAll = rv->isTcpDragTeachActive();
		(void)doc->data().runFollowSolveAndSync(ctx, nullptr);
	}
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
	if (shouldDeferPropertyPanelRebuild(want))
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
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	updatePropertyPanel(m_selectionState.selectedBackendId());
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
	view->setObjectSelectionMode(true);
	view->setPointPickMode(false);
	view->setMeshLinePickMode(false);
	view->setMeshFacePickMode(false);

	// 场景有内容时允许 gizmo；树刷新可能清空选中但不卸载场景
	DocumentPage* doc = currentPage();
	if (doc && doc->render().hasImportedContent())
	{
		view->setSelectionActive(true);
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
	resetInteractionPickModes(*view);
	MainWindowSelectionService::clearSelection(*this, true);
}

void MainWindow::onLanguageEnglishTriggered()
{
	m_useChinese = false;
	applyLanguage();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("UI language switched to English."));
	}
}

void MainWindow::onLanguageChineseTriggered()
{
	m_useChinese = true;
	applyLanguage();
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

void MainWindow::setLeftSidePanelVisible(const bool visible)
{
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
}

void MainWindow::setRightSidePanelVisible(const bool visible)
{
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
}

void MainWindow::syncSidePanelToggleUi()
{
	const bool leftVisible = sideDockShown(m_propertyDock);
	const bool rightVisible = sideDockShown(m_unitDock);

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

QDockWidget* MainWindow::addPluginDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area)
{
	auto* dock = new QDockWidget(title, this);
	dock->setObjectName(QStringLiteral("PluginDock_") + title);
	dock->setWidget(widget);
	addDockWidget(area, dock);
	return dock;
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
	ctx.skipAll = rv->isTcpDragTeachActive();
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
	ctx.skipAll = rv.isTcpDragTeachActive();
	if (rv.isTransformGizmoDragging() && m_selectionState.hasBackendSelection())
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
			if (rv.isTcpDragTeachActive())
			{
				return;
			}
			// FK 路径已在 notify 内同步求解并清空脏集；此处只处理 gizmo 拖动 / 属性 dirty / forced
			if (page->followDirtyBackendIds().empty() && !page->followSolveForcedPending() &&
				!rv.isTransformGizmoDragging())
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
	auto* page = new DocumentPage(m_documentTabs, m_appEvents);
	wireDocumentPageSignals(page);
	page->render().setViewerBackgroundForDarkUi(viewerUsesDarkBackground());
	page->setViewportToolBarDarkTheme(viewerUsesDarkBackground());
	const QString title = nextUntitledDocumentTitle();
	m_documentTabs->addTab(page, title);
	// setCurrentWidget → currentChanged → onDocumentTabChanged，勿再手动 rebuild/回调
	m_documentTabs->setCurrentWidget(page);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("New document."), QStringLiteral("\u65b0\u5efa\u6587\u6863\u3002")));
	}
}

void MainWindow::onDocumentTabChanged(int)
{
	if (m_robotSimulation && m_robotSimulation->programExecutor().isRunning())
	{
		stopRobotSimulation();
	}
	// 先换树再清选中，避免对旧文档节点发 selection/itemChanged 写可见性
	if (DocumentPage* cur = currentPage())
	{
		rebuildUnitsDocument(cur->documentId());
	}
	else if (m_unitsTreeBinder)
	{
		m_unitsTreeBinder->showOnlyDocument(QString());
	}
	MainWindowSelectionService::clearSelection(*this, true);
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
	m_documentTabs->removeTab(index);
	if (m_unitsTreeBinder)
	{
		m_unitsTreeBinder->removeDocument(closingDocId);
	}
	m_unitsTreeDirtyDocumentIds.remove(closingDocId);
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
