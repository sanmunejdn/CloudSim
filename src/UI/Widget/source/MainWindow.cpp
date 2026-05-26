#include "MainWindow.h"

#include "BackendFollowSolve.h"
#include "BackendHierarchyFollow.h"
#include "DocumentHostEvents.h"
#include "BackendSceneDocumentFacade.h"
#include "RobotInstructionController.h"
#include "RobotInstructionModel.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <locale>
#include <unordered_set>

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
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QRegularExpression>
#include <QSet>
#include <QSignalBlocker>
#include <QStringList>
#include <QStatusBar>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTabWidget>
#include <QVector>
#include <QVBoxLayout>
#include <QWidget>
#include <QXmlStreamReader>

#include <osg/Vec3f>

#include "AiAssistantDockWidget.h"
#include "ApplicationStyle.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendHierarchyModel.h"
#include "BackendFollowMath.h"
#include "BackendFollowTransformSolver.h"
#include "DocumentPage.h"
#include "WidgetDocumentAccess.h"
#include "IRenderView.h"
#include "FollowAttachmentComponent.h"
#include "DevicePageWidget.h"
#include "../RobotWidget/inc/RobotAxisControlWidget.h"
#include "../RobotWidget/inc/RobotFrameSettingsWidget.h"
#include "MainWindow_p.h"
#include "MainWindowSelectionService.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"
#include "IRobotBackendPoseSink.h"
#include "RobotInstructionProgram.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionTransform.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotTeachIk.h"

#include <Adapters.h>
#include <RigidTransform.h>
#include <ToolKinematics.h>
#include "RobotProgramExport.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"
#include "RunInfoPage.h"
#include "RunLogger.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"
#include "../RobotWidget/inc/TrajectoryEditPageWidget.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"

#include "../../OsgWidgetCore/inc/OsgScene.h"
#include "ObjectGizmoFrame.h"
#include "BackendVisualMath.h"

#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/Quat>

#include "qteditorfactory.h"
#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

using namespace mainwindow_detail;
using namespace RobotSimulation;

QString MainWindow::i18n(const QString& en, const QString& zh) const
{
	return m_useChinese ? zh : en;
}

void MainWindow::applyLanguage()
{
	setWindowTitle(i18n(QStringLiteral("CloudSim - MainWindow"), QStringLiteral("CloudSim - 主窗口")));
	if (m_fileMenu) m_fileMenu->setTitle(i18n(QStringLiteral("File"), QStringLiteral("文件")));
	if (m_newDocumentAction)
	{
		m_newDocumentAction->setText(i18n(QStringLiteral("New"), QStringLiteral("新建")));
	}
	if (m_openModelAction) m_openModelAction->setText(i18n(QStringLiteral("Open Model..."), QStringLiteral("打开模型...")));
	if (m_openPointCloudAction) m_openPointCloudAction->setText(i18n(QStringLiteral("Open Point Cloud..."), QStringLiteral("打开点云...")));
	if (m_openProjectAction) m_openProjectAction->setText(i18n(QStringLiteral("Open Project..."), QStringLiteral("打开工程...")));
	if (m_saveAction) m_saveAction->setText(i18n(QStringLiteral("Save Project..."), QStringLiteral("保存工程...")));
	if (m_exitAction) m_exitAction->setText(i18n(QStringLiteral("Exit"), QStringLiteral("退出")));
	if (m_viewMenu) m_viewMenu->setTitle(i18n(QStringLiteral("View"), QStringLiteral("视图")));
	if (m_resetLayoutAction)
	{
		m_resetLayoutAction->setText(i18n(QStringLiteral("Reset Layout"), QStringLiteral("重置布局")));
	}
	if (m_settingsMenu) m_settingsMenu->setTitle(i18n(QStringLiteral("Settings"), QStringLiteral("设置")));
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
	if (m_languageMenu) m_languageMenu->setTitle(i18n(QStringLiteral("Language"), QStringLiteral("语言")));
	if (m_languageEnglishAction) m_languageEnglishAction->setText(QStringLiteral("English"));
	if (m_languageChineseAction) m_languageChineseAction->setText(QStringLiteral("中文"));
	if (m_languageEnglishAction) m_languageEnglishAction->setChecked(!m_useChinese);
	if (m_languageChineseAction) m_languageChineseAction->setChecked(m_useChinese);

	if (m_viewModeAction) m_viewModeAction->setText(i18n(QStringLiteral("View Mode"), QStringLiteral("视图模式")));
	if (m_objectModeAction) m_objectModeAction->setText(i18n(QStringLiteral("Object Select"), QStringLiteral("对象选择")));
	if (m_pointPickModeAction) m_pointPickModeAction->setText(i18n(QStringLiteral("Point Pick"), QStringLiteral("点选模式")));
	if (m_meshLinePickModeAction) m_meshLinePickModeAction->setText(i18n(QStringLiteral("Line Pick"), QStringLiteral("线选择模式")));
	if (m_meshFacePickModeAction) m_meshFacePickModeAction->setText(i18n(QStringLiteral("Face Pick"), QStringLiteral("面选择模式")));
	if (m_gizmoLocalFrameAction)
	{
		m_gizmoLocalFrameAction->setText(i18n(QStringLiteral("Transform: Local (object axes)"),
			QStringLiteral("变换：物体系（罗盘轴）")));
	}
	if (m_gizmoWorldFrameAction)
	{
		m_gizmoWorldFrameAction->setText(i18n(QStringLiteral("Transform: World"), QStringLiteral("变换：世界系")));
	}
	if (m_simulationStartAction)
	{
		m_simulationStartAction->setText(i18n(QStringLiteral("Start Simulation"), QStringLiteral("开始仿真")));
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
	if (m_unitDock)
	{
		m_unitDock->setWindowTitle(i18n(QStringLiteral("Workspace"), QStringLiteral("工作区")));
	}
	if (m_rightPanelTabs && m_rightPanelTabs->count() >= 2)
	{
		m_rightPanelTabs->setTabText(0, i18n(QStringLiteral("Workspace"), QStringLiteral("工作区")));
		m_rightPanelTabs->setTabText(1, i18n(QStringLiteral("AI"), QStringLiteral("AI")));
	}
	if (m_unitDockTabs && m_unitDockTabs->count() >= 3)
	{
		m_unitDockTabs->setTabText(0, i18n(QStringLiteral("Units"), QStringLiteral("单元部件")));
		m_unitDockTabs->setTabText(1, i18n(QStringLiteral("Simulation"), QStringLiteral("指令仿真")));
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
		if (TrajectoryEditPageWidget* traj = simDock->trajectoryEditPage())
		{
			traj->setUseChinese(m_useChinese);
		}
		QTabWidget* tabs = simDock->tabWidget();
		if (tabs && tabs->count() >= 2)
		{
			tabs->setTabText(0, i18n(QStringLiteral("Instructions"), QStringLiteral("指令")));
			tabs->setTabText(1, i18n(QStringLiteral("Axis control"), QStringLiteral("轴控制")));
			if (tabs->count() >= 3)
			{
				tabs->setTabText(2, i18n(QStringLiteral("Frames"), QStringLiteral("坐标系")));
			}
			if (tabs->count() >= 4)
			{
				tabs->setTabText(3, i18n(QStringLiteral("Trajectory Edit"), QStringLiteral("轨迹编辑")));
			}
		}
	}
	refreshSimulationJointListFromCurrentDoc();
	if (m_runDock)
	{
		m_runDock->setWindowTitle(i18n(QStringLiteral("Runtime Output"), QStringLiteral("运行信息")));
	}
	if (m_toggleAiAssistantAction)
	{
		m_toggleAiAssistantAction->setText(i18n(QStringLiteral("AI Assistant"), QStringLiteral("AI 助手")));
	}
	if (m_runInfoPage) m_runInfoPage->setUiLanguage(m_useChinese);
	if (m_aiAssistantPage) m_aiAssistantPage->setUseChinese(m_useChinese);

	if (m_propertyBrowser)
	{
		if (QTreeWidget* tw = m_propertyBrowser->findChild<QTreeWidget*>())
		{
			tw->setHeaderLabels(QStringList()
				<< i18n(QStringLiteral("Property"), QStringLiteral("属性"))
				<< i18n(QStringLiteral("Value"), QStringLiteral("值")));
			tw->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked
				| QAbstractItemView::EditKeyPressed | QAbstractItemView::AnyKeyPressed);
		}
	}
	if (m_osgSceneTree)
	{
		m_osgSceneTree->setHeaderLabels(QStringList()
			<< i18n(QStringLiteral("Node"), QStringLiteral("节点"))
			<< i18n(QStringLiteral("Local transform"), QStringLiteral("本地变换矩阵")));
	}
	if (m_backendRootItem)
	{
		m_backendRootItem->setText(0, i18n(QStringLiteral("BackendDataManager"), QStringLiteral("后端数据管理器")));
	}
	if (m_annotationRootItem)
	{
		m_annotationRootItem->setText(0, i18n(QStringLiteral("Annotations"), QStringLiteral("注释")));
	}
	refreshBackendTree();
	notifyPluginsLanguageChanged();
}

void MainWindow::onSelectedObjectPoseChanged(float x, float y, float z)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const MainWindowSelectionService::SelectionSnapshot snapshot =
		MainWindowSelectionService::currentSelection(*this);
	if (!snapshot.valid())
	{
		return;
	}
	OsgWidget* osgW = currentOsgWidget();
	BackendVec3 euler{};
	if (osgW)
	{
		const osg::Vec3f er = osgW->selectedRotationEulerDeg();
		euler.x = static_cast<double>(er.x());
		euler.y = static_cast<double>(er.y());
		euler.z = static_cast<double>(er.z());
	}
	auto pointCloud = std::dynamic_pointer_cast<PointCloudBackendData>(snapshot.data);
	if (pointCloud)
	{
		BackendVec3 pose;
		pose.x = x;
		pose.y = y;
		pose.z = z;
		if (!osgW)
		{
			euler = pointCloud->rotation();
		}
		if (pointCloud->supportsBackendTransform())
		{
			pointCloud->applyBackendWorldPose(pose, euler);
		}
		else
		{
			pointCloud->setPose(pose);
		}
		refreshFollowSolveAndPropertyPanelFromOsgWrite(pointCloud);
		return;
	}
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(snapshot.data);
	if (mesh)
	{
		BackendVec3 pose;
		pose.x = x;
		pose.y = y;
		pose.z = z;
		if (!osgW)
		{
			euler = mesh->rotation();
		}
		if (mesh->supportsBackendTransform())
		{
			mesh->applyBackendWorldPose(pose, euler);
		}
		else
		{
			mesh->setPose(pose);
		}
		refreshFollowSolveAndPropertyPanelFromOsgWrite(mesh);
	}
}

void MainWindow::onSelectedObjectRotationChanged(float rx, float ry, float rz)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const MainWindowSelectionService::SelectionSnapshot snapshot =
		MainWindowSelectionService::currentSelection(*this);
	if (!snapshot.valid())
	{
		return;
	}
	OsgWidget* osgW = currentOsgWidget();
	BackendVec3 pos{};
	if (osgW)
	{
		const osg::Vec3f p = osgW->selectedPosition();
		pos.x = static_cast<double>(p.x());
		pos.y = static_cast<double>(p.y());
		pos.z = static_cast<double>(p.z());
	}
	auto pointCloud = std::dynamic_pointer_cast<PointCloudBackendData>(snapshot.data);
	if (pointCloud)
	{
		BackendVec3 rot;
		rot.x = rx;
		rot.y = ry;
		rot.z = rz;
		if (!osgW)
		{
			pos = pointCloud->pose();
		}
		if (pointCloud->supportsBackendTransform())
		{
			pointCloud->applyBackendWorldPose(pos, rot);
		}
		else
		{
			pointCloud->setPose(pos);
			pointCloud->setRotation(rot);
		}
		refreshFollowSolveAndPropertyPanelFromOsgWrite(pointCloud);
		return;
	}
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(snapshot.data);
	if (mesh)
	{
		BackendVec3 rot;
		rot.x = rx;
		rot.y = ry;
		rot.z = rz;
		if (!osgW)
		{
			pos = mesh->pose();
		}
		if (mesh->supportsBackendTransform())
		{
			mesh->applyBackendWorldPose(pos, rot);
		}
		else
		{
			mesh->setPose(pos);
			mesh->setRotation(rot);
		}
		refreshFollowSolveAndPropertyPanelFromOsgWrite(mesh);
	}
}

void MainWindow::onSelectedObjectColorChanged(float r, float g, float b, float a)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const MainWindowSelectionService::SelectionSnapshot snapshot =
		MainWindowSelectionService::currentSelection(*this);
	if (!snapshot.valid())
	{
		return;
	}
	if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(snapshot.data))
	{
		BackendColor c;
		c.r = r; c.g = g; c.b = b; c.a = a;
		pc->setColor(c);
		refreshFollowSolveAndPropertyPanelFromOsgWrite(pc);
		return;
	}
	if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(snapshot.data))
	{
		BackendColor c;
		c.r = r; c.g = g; c.b = b; c.a = a;
		mesh->setColor(c);
		refreshFollowSolveAndPropertyPanelFromOsgWrite(mesh);
	}
}

void MainWindow::onTransformGizmoCommitted()
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	const std::shared_ptr<BackendDataBase> data = MainWindowSelectionService::selectedBackendData(*this);
	if (!data)
	{
		return;
	}
	OsgWidget* osgW = currentOsgWidget();
	if (osgW)
	{
		(void)osgW->writeActiveBackendPoseFromOsg(*data);
	}
	syncRobotKinematicsAfterPoseEdit(data);
	DocumentPage* doc = currentPage();
	if (doc)
	{
		doc->markFollowAttachmentDirtyFromBackendMove(doc->backend(), data->id());
	}
	updatePropertyPanel(data);
	if (DocumentPage* doc = currentPage())
	{
		cloudsim::host::publishPoseCommittedFromBackend(*doc, *data);
	}
}

void MainWindow::refreshFollowSolveAndPropertyPanelFromOsgWrite(const std::shared_ptr<BackendDataBase>& data)
{
	if (!data)
	{
		return;
	}
	DocumentPage* doc = currentPage();
	OsgWidget* osg = widgetOsgFromPage(doc);
	if (doc && osg)
	{
		doc->markFollowAttachmentDirtyFromBackendMove(doc->backend(), data->id());
		if (!osg->isTransformGizmoDragging())
		{
			runBackendFollowSolveAndSync(*doc, *osg);
		}
	}
	if (!osg || !osg->isTransformGizmoDragging())
	{
		updatePropertyPanel(data);
	}
}

void MainWindow::schedulePropertyPanelCommitRefresh(const std::shared_ptr<BackendDataBase>& data)
{
	if (!data)
	{
		return;
	}
	m_propertyPanelCommitPendingBackendId = QString::fromStdString(data->id());
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
	const std::shared_ptr<BackendDataBase> d = MainWindowSelectionService::selectedBackendData(*this);
	if (!d || QString::fromStdString(d->id()) != want)
	{
		return;
	}
	updatePropertyPanel(d);
}

void MainWindow::onActiveAxisChanged(const QString& axisName)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	m_activeAxisName = axisName;
	if (!m_selectionState.hasBackendSelection())
	{
		return;
	}
	updatePropertyPanel(MainWindowSelectionService::currentSelection(*this).data);
}

void MainWindow::onViewModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(true);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(false);
}

void MainWindow::onObjectModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(true);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(true);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(false);

	// Allow gizmo / transform whenever the scene has a loaded object; tree refresh (e.g. language)
	// can clear the selection without unloading the scene.
	if (osg->hasImportedContent())
	{
		osg->setSelectionActive(true);
	}
}

void MainWindow::onPointPickModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(true);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(true);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(false);
	MainWindowSelectionService::ensureBackendForPickMode(
		*this, MainWindowSelectionService::SelectedBackendKind::PointCloud);
}

void MainWindow::onMeshLinePickModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(true);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(true);
	osg->setMeshFacePickMode(false);
	MainWindowSelectionService::ensureBackendForPickMode(
		*this, MainWindowSelectionService::SelectedBackendKind::Mesh);
}

void MainWindow::onMeshFacePickModeTriggered()
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	m_viewModeAction->setChecked(false);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(true);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(true);
	MainWindowSelectionService::ensureBackendForPickMode(
		*this, MainWindowSelectionService::SelectedBackendKind::Mesh);
}

void MainWindow::onSelectionCanceledByEsc()
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	OsgWidget* osg = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction || !osg)
	{
		return;
	}
	// OsgWidget emits this from ESC to leave object-select / point-pick; camera stays put (see OsgWidget manipulator attach).
	m_viewModeAction->setChecked(true);
	m_objectModeAction->setChecked(false);
	m_pointPickModeAction->setChecked(false);
	m_meshLinePickModeAction->setChecked(false);
	m_meshFacePickModeAction->setChecked(false);
	osg->setObjectSelectionMode(false);
	osg->setPointPickMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(false);
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
		setAllDocumentViewerDarkBackground(false);
	}
		else if (action == m_darkThemeAction)
	{
		ApplicationStyle::applyTheme(qApp, ApplicationStyle::Theme::Dark);
		ApplicationStyle::saveTheme(ApplicationStyle::Theme::Dark);
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
		if (OsgWidget* osg = widgetOsgFromPage(p))
		{
			osg->setViewerBackgroundForDarkUi(dark);
		}
	}
}

bool MainWindow::viewerUsesDarkBackground() const
{
	if (m_darkThemeAction && m_lightThemeAction)
	{
		return m_darkThemeAction->isChecked();
	}
	return ApplicationStyle::loadSavedTheme() == ApplicationStyle::Theme::Dark;
}

DocumentPage* MainWindow::currentPage() const
{
	if (!m_documentTabs)
	{
		return nullptr;
	}
	return qobject_cast<DocumentPage*>(m_documentTabs->currentWidget());
}

OsgWidget* MainWindow::currentOsgWidget() const
{
	DocumentPage* p = currentPage();
	if (!p)
	{
		return nullptr;
	}
	return widgetOsgFromPage(p);
}

BackendDataManager& MainWindow::activeBackend()
{
	static BackendDataManager s_unused;
	DocumentPage* p = currentPage();
	return p ? p->backend() : s_unused;
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

void MainWindow::wireDocumentPageSignals(DocumentPage* page)
{
	OsgWidget* o = widgetOsgFromPage(page);
	if (!o)
	{
		return;
	}
	connect(o, &OsgWidget::selectedObjectPoseChanged, this, &MainWindow::onSelectedObjectPoseChanged);
	connect(o, &OsgWidget::selectedObjectRotationChanged, this, &MainWindow::onSelectedObjectRotationChanged);
	connect(o, &OsgWidget::selectedObjectColorChanged, this, &MainWindow::onSelectedObjectColorChanged);
	connect(o, &OsgWidget::transformGizmoCommitted, this, &MainWindow::onTransformGizmoCommitted);
	connect(o, &OsgWidget::tcpDragTeachPoseChanged, this, &MainWindow::onTcpDragTeachPoseChanged);
	connect(o, &OsgWidget::tcpDragTeachEnded, this, &MainWindow::onTcpDragTeachEnded);
	connect(o, &OsgWidget::activeAxisChanged, this, &MainWindow::onActiveAxisChanged);
	connect(o, &OsgWidget::selectionCanceledByEsc, this, &MainWindow::onSelectionCanceledByEsc);
	connect(o, &OsgWidget::annotationCreated, this, &MainWindow::onAnnotationCreated);
	connect(o, &OsgWidget::annotationRemoved, this, &MainWindow::onAnnotationRemoved);
	connect(o, &OsgWidget::annotationVisibilityChanged, this, &MainWindow::onAnnotationVisibilityChanged);
	connect(o, &OsgWidget::pointPickFeedback, this, &MainWindow::onPointPickFeedback);
	connect(o, &OsgWidget::meshPickFeedback, this, &MainWindow::onMeshPickFeedback);
	connect(o, &OsgWidget::backendObjectPicked, this, &MainWindow::onOsgBackendObjectPicked);
	installBackendFollowFrameHook(page);
}

void MainWindow::installBackendFollowFrameHook(DocumentPage* page)
{
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		return;
	}
	osg->setPerFrameHook([this, page](OsgWidget* o) {
		if (!page || !o || !m_documentTabs || m_documentTabs->currentWidget() != page)
		{
			return;
		}
		// 末端拖动示教：IK 逐帧写连杆位姿，禁止跟随求解写回以免与 FK 冲突（表现为关节闪回零位）
		if (o->isTcpDragTeachActive())
		{
			return;
		}
		if (page->followDirtyBackendIds().empty() && !page->followSolveForcedPending() && !o->isTransformGizmoDragging())
		{
			return;
		}
		runBackendFollowSolveAndSync(*page, *o);
	});
}

cloudsim::host::FollowSolveContext MainWindow::makeFollowSolveContext(OsgWidget& osg) const
{
	cloudsim::host::FollowSolveContext ctx;
	ctx.skipAll = [this, &osg]() {
		if (osg.isTcpDragTeachActive())
		{
			return true;
		}
		return m_robotSimulation && m_robotSimulation->programExecutor().isRunning();
	};
	ctx.fillGizmoSelectedId = [this, &osg](std::string& outId) -> bool {
		if (!osg.isTransformGizmoDragging() || !m_selectionState.hasBackendSelection())
		{
			return false;
		}
		outId = m_selectionState.selectedBackendId().toStdString();
		return true;
	};
	return ctx;
}

void MainWindow::runBackendFollowSolveAndSync(DocumentPage& page, OsgWidget& osg,
	const std::string* manualPoseAuthorityBackendId)
{
	cloudsim::host::FollowSolveContext ctx = makeFollowSolveContext(osg);
	cloudsim::host::runBackendFollowSolveAndSync(page, osg, &ctx, manualPoseAuthorityBackendId);
}

void MainWindow::applyHierarchyFollowBinding(DocumentPage* doc, const std::string& childId, const std::string& parentId)
{
	if (!doc)
	{
		return;
	}
	cloudsim::host::applyHierarchyFollowBinding(*doc, childId, parentId);
	if (OsgWidget* osg = widgetOsgFromPage(doc))
	{
		runBackendFollowSolveAndSync(*doc, *osg);
	}
}

void MainWindow::afterBackendFollowPropertyEdited(const QString& propertyKey, const QString& valueText)
{
	DocumentPage* doc = currentPage();
	OsgWidget* osg = currentOsgWidget();
	if (!doc || !osg || !m_selectionState.hasBackendSelection())
	{
		return;
	}
	const std::shared_ptr<BackendDataBase> data = MainWindowSelectionService::selectedBackendData(*this);
	if (!data)
	{
		return;
	}
	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [osg](const std::string& bid, BackendMat4& out) -> bool {
		osg::Matrixd om;
		if (!osg->getBackendRootWorldMatrix(bid, om))
		{
			return false;
		}
		for (int c = 0; c < 4; ++c)
		{
			for (int r = 0; r < 4; ++r)
			{
				out.v[c * 4 + r] = om(r, c);
			}
		}
		return true;
	};
	if (propertyKey == QStringLiteral("follow.targetId")
		|| propertyKey == QStringLiteral("follow.targetName")
		|| (propertyKey == QStringLiteral("follow.enabled")
			&& (valueText == QStringLiteral("1") || valueText.compare(QStringLiteral("true"), Qt::CaseInsensitive) == 0)))
	{
		(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(doc->backend(), worldQuery, *data, nullptr);
	}
	doc->markFollowAttachmentDirtyFromBackendMove(doc->backend(), data->id());
	runBackendFollowSolveAndSync(*doc, *osg);
	doc->invalidateFollowReverseIndex();
}

void MainWindow::onNewDocument()
{
	if (!m_documentTabs)
	{
		return;
	}
	auto* page = new DocumentPage(m_documentTabs, m_appEvents);
	wireDocumentPageSignals(page);
	if (OsgWidget* osg = widgetOsgFromPage(page))
	{
		osg->setViewerBackgroundForDarkUi(viewerUsesDarkBackground());
	}
	const QString title = i18n(QStringLiteral("Untitled"), QStringLiteral("\u672a\u547d\u540d"));
	m_documentTabs->addTab(page, title);
	m_documentTabs->setCurrentWidget(page);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("New document."), QStringLiteral("\u65b0\u5efa\u6587\u6863\u3002")));
	}
	onDocumentTabChanged(m_documentTabs->currentIndex());
}

void MainWindow::onDocumentTabChanged(int)
{
	stopRobotSimulation();
	MainWindowSelectionService::clearSelection(*this, true);
	refreshBackendTree();
	syncViewModeActionsFromCurrentOsg();
	refreshSimulationJointListFromCurrentDoc();
}

void MainWindow::syncViewModeActionsFromCurrentOsg()
{
	OsgWidget* o = currentOsgWidget();
	if (!m_viewModeAction || !m_objectModeAction || !m_pointPickModeAction || !m_meshLinePickModeAction || !m_meshFacePickModeAction)
	{
		return;
	}
	if (!o)
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
	const bool view = !o->objectSelectionMode() && !o->pointPickMode() && !o->meshLinePickMode() && !o->meshFacePickMode();
	m_viewModeAction->setChecked(view);
	m_objectModeAction->setChecked(o->objectSelectionMode());
	m_pointPickModeAction->setChecked(o->pointPickMode());
	m_meshLinePickModeAction->setChecked(o->meshLinePickMode());
	m_meshFacePickModeAction->setChecked(o->meshFacePickMode());
	if (m_gizmoFrameGroup && m_gizmoLocalFrameAction && m_gizmoWorldFrameAction)
	{
		const QSignalBlocker bg(m_gizmoFrameGroup);
		const QSignalBlocker b1(m_gizmoLocalFrameAction);
		const QSignalBlocker b2(m_gizmoWorldFrameAction);
		if (o->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::Local)
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
	if (sender() != currentOsgWidget())
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

