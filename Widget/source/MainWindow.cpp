#include "MainWindow.h"

#include <algorithm>
#include <functional>
#include <memory>

#include <QAction>
#include <QActionGroup>
#include <QSignalBlocker>
#include <QApplication>
#include <QDockWidget>
#include <QMessageBox>
#include <QHeaderView>
#include <QList>
#include <QMenu>
#include <QMenuBar>
#include <QStringList>
#include <QStatusBar>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTabWidget>
#include <QVector>
#include <QVBoxLayout>
#include <QWidget>

#include <osg/Vec3f>

#include "ApplicationStyle.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentPage.h"
#include "DevicePageWidget.h"
#include "MainWindow_p.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"
#include "RobotSceneKinematics.h"
#include "RunInfoPage.h"
#include "SimulationCommandWidget.h"

#include <osg/Quat>

#include "qteditorfactory.h"
#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

using namespace mainwindow_detail;
using namespace RobotSimulation;

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
{
	setupMenuBar();

	auto* central = new QWidget(this);
	auto* rootLayout = new QVBoxLayout(central);
	rootLayout->setContentsMargins(8, 8, 8, 8);
	rootLayout->setSpacing(8);

	m_documentTabs = new QTabWidget(central);
	m_documentTabs->setDocumentMode(true);
	m_documentTabs->setTabsClosable(false);
	rootLayout->addWidget(m_documentTabs, 1);
	connect(m_documentTabs, &QTabWidget::currentChanged, this, &MainWindow::onDocumentTabChanged);

	auto* firstPage = new DocumentPage(m_documentTabs);
	wireDocumentPageSignals(firstPage);
	m_documentTabs->addTab(firstPage, QStringLiteral("Untitled"));
	setCentralWidget(central);
	setupDockWidgets();
	m_robotSimTimer.setInterval(kPlaybackTimerIntervalMs);
	connect(&m_robotSimTimer, &QTimer::timeout, this, &MainWindow::onRobotSimulationTick);
	applyLanguage();
	const ApplicationStyle::Theme savedTheme = ApplicationStyle::loadSavedTheme();
	ApplicationStyle::applyTheme(qApp, savedTheme);
	setAllDocumentViewerDarkBackground(savedTheme == ApplicationStyle::Theme::Dark);
	if (m_lightThemeAction && m_darkThemeAction)
	{
		m_lightThemeAction->blockSignals(true);
		m_darkThemeAction->blockSignals(true);
		m_lightThemeAction->setChecked(savedTheme == ApplicationStyle::Theme::Light);
		m_darkThemeAction->setChecked(savedTheme == ApplicationStyle::Theme::Dark);
		m_lightThemeAction->blockSignals(false);
		m_darkThemeAction->blockSignals(false);
	}
	const QString pluginReport = currentOsgWidget() ? currentOsgWidget()->pointCloudPluginReport() : QStringLiteral("Ready");
	statusBar()->showMessage(pluginReport, 12000);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("Application started."),
			QStringLiteral("应用程序已启动。")));
		m_runInfoPage->appendInfo(pluginReport);
	}
	onDocumentTabChanged(m_documentTabs ? m_documentTabs->currentIndex() : -1);

	// Reasonable default geometry so the window does not maximize overly wide on first show.
	resize(1180, 760);
	setMinimumSize(900, 560);
}

void MainWindow::setupMenuBar()
{
	// --- File: document / import / exit ---
	m_fileMenu = menuBar()->addMenu(QStringLiteral("File"));
	m_newDocumentAction = m_fileMenu->addAction(QStringLiteral("New"), this, &MainWindow::onNewDocument);
	m_fileMenu->addSeparator();
	m_openProjectAction = m_fileMenu->addAction(QStringLiteral("Open Project..."), this, &MainWindow::onOpenProjectFile);
	m_saveAction = m_fileMenu->addAction(QStringLiteral("Save Project..."), this, &MainWindow::onSaveProject);
	m_fileMenu->addSeparator();
	m_openModelAction = m_fileMenu->addAction(QStringLiteral("Open Model..."), this, &MainWindow::onOpenModel);
	m_openPointCloudAction = m_fileMenu->addAction(QStringLiteral("Open Point Cloud..."), this, &MainWindow::onOpenPointCloud);
	m_fileMenu->addSeparator();
	m_exitAction = m_fileMenu->addAction(QStringLiteral("Exit"), this, &QWidget::close);

	// --- View: layout + 3D interaction modes ---
	m_viewMenu = menuBar()->addMenu(QStringLiteral("View"));
	m_resetLayoutAction = m_viewMenu->addAction(QStringLiteral("Reset Layout"));
	m_viewMenu->addSeparator();

	m_interactionModeGroup = new QActionGroup(this);
	m_interactionModeGroup->setExclusive(true);
	m_viewModeAction = m_viewMenu->addAction(QStringLiteral("View Mode"));
	m_viewModeAction->setCheckable(true);
	m_interactionModeGroup->addAction(m_viewModeAction);
	m_objectModeAction = m_viewMenu->addAction(QStringLiteral("Object Select"));
	m_objectModeAction->setCheckable(true);
	m_interactionModeGroup->addAction(m_objectModeAction);
	m_pointPickModeAction = m_viewMenu->addAction(QStringLiteral("Point Pick"));
	m_pointPickModeAction->setCheckable(true);
	m_interactionModeGroup->addAction(m_pointPickModeAction);
	m_meshLinePickModeAction = m_viewMenu->addAction(QStringLiteral("Line Pick"));
	m_meshLinePickModeAction->setCheckable(true);
	m_interactionModeGroup->addAction(m_meshLinePickModeAction);
	m_meshFacePickModeAction = m_viewMenu->addAction(QStringLiteral("Face Pick"));
	m_meshFacePickModeAction->setCheckable(true);
	m_interactionModeGroup->addAction(m_meshFacePickModeAction);
	m_viewModeAction->setChecked(true);
	connect(m_viewModeAction, &QAction::triggered, this, &MainWindow::onViewModeTriggered);
	connect(m_objectModeAction, &QAction::triggered, this, &MainWindow::onObjectModeTriggered);
	connect(m_pointPickModeAction, &QAction::triggered, this, &MainWindow::onPointPickModeTriggered);
	connect(m_meshLinePickModeAction, &QAction::triggered, this, &MainWindow::onMeshLinePickModeTriggered);
	connect(m_meshFacePickModeAction, &QAction::triggered, this, &MainWindow::onMeshFacePickModeTriggered);
	m_viewMenu->addSeparator();
	m_gizmoFrameGroup = new QActionGroup(this);
	m_gizmoFrameGroup->setExclusive(true);
	m_gizmoLocalFrameAction = m_viewMenu->addAction(QStringLiteral("Transform: Local (object axes)"));
	m_gizmoLocalFrameAction->setCheckable(true);
	m_gizmoFrameGroup->addAction(m_gizmoLocalFrameAction);
	m_gizmoWorldFrameAction = m_viewMenu->addAction(QStringLiteral("Transform: World"));
	m_gizmoWorldFrameAction->setCheckable(true);
	m_gizmoFrameGroup->addAction(m_gizmoWorldFrameAction);
	m_gizmoLocalFrameAction->setChecked(true);
	// Only react when an action becomes checked; ignore triggered(false) when exclusivity unchecks the other item.
	connect(m_gizmoLocalFrameAction, &QAction::triggered, this, [this](bool checked) {
		if (!checked)
		{
			return;
		}
		if (OsgWidget* osg = currentOsgWidget())
		{
			osg->setTransformGizmoFrame(OsgWidget::TransformGizmoFrame::Local);
		}
	});
	connect(m_gizmoWorldFrameAction, &QAction::triggered, this, [this](bool checked) {
		if (!checked)
		{
			return;
		}
		if (OsgWidget* osg = currentOsgWidget())
		{
			osg->setTransformGizmoFrame(OsgWidget::TransformGizmoFrame::World);
		}
	});
	m_viewMenu->addSeparator();
	m_simulationStartAction = m_viewMenu->addAction(QStringLiteral("Start Simulation"), this, &MainWindow::onSimulationStartTriggered);

	// --- Settings: appearance + language ---
	m_settingsMenu = menuBar()->addMenu(QStringLiteral("Settings"));
	m_appearanceMenu = m_settingsMenu->addMenu(QStringLiteral("Theme"));
	m_themeActionGroup = new QActionGroup(this);
	m_themeActionGroup->setExclusive(true);
	m_lightThemeAction = m_appearanceMenu->addAction(QStringLiteral("Light"));
	m_lightThemeAction->setCheckable(true);
	m_themeActionGroup->addAction(m_lightThemeAction);
	m_darkThemeAction = m_appearanceMenu->addAction(QStringLiteral("Dark"));
	m_darkThemeAction->setCheckable(true);
	m_themeActionGroup->addAction(m_darkThemeAction);
	m_lightThemeAction->setChecked(true);
	connect(m_themeActionGroup, &QActionGroup::triggered, this, &MainWindow::onThemeActionGroupTriggered);

	m_languageMenu = m_settingsMenu->addMenu(QStringLiteral("Language"));
	m_languageActionGroup = new QActionGroup(this);
	m_languageActionGroup->setExclusive(true);
	m_languageEnglishAction = m_languageMenu->addAction(QStringLiteral("English"));
	m_languageEnglishAction->setCheckable(true);
	m_languageActionGroup->addAction(m_languageEnglishAction);
	m_languageChineseAction = m_languageMenu->addAction(QStringLiteral("中文"));
	m_languageChineseAction->setCheckable(true);
	m_languageActionGroup->addAction(m_languageChineseAction);
	m_languageChineseAction->setChecked(true);
	connect(m_languageEnglishAction, &QAction::triggered, this, &MainWindow::onLanguageEnglishTriggered);
	connect(m_languageChineseAction, &QAction::triggered, this, &MainWindow::onLanguageChineseTriggered);
}

void MainWindow::setupDockWidgets()
{
	m_propertyDock = new QDockWidget(QStringLiteral("Property"), this);
	m_propertyDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	m_variantManager = new QtVariantPropertyManager(this);
	m_variantFactory = new QtVariantEditorFactory(this);
	m_propertyBrowser = new QtTreePropertyBrowser();
	m_propertyBrowser->setFactoryForManager(m_variantManager, m_variantFactory);
	m_propertyBrowser->setResizeMode(QtTreePropertyBrowser::ResizeToContents);
	m_propertyBrowser->setAlternatingRowColors(true);
	m_propertyBrowser->setHeaderVisible(true);
	m_propertyBrowser->setRootIsDecorated(false);
	m_propertyBrowser->setSplitterPosition(160);
	connect(m_variantManager, &QtVariantPropertyManager::valueChanged, this, &MainWindow::onVariantPropertyValueChanged);
	m_propertyDockTabs = new QTabWidget(m_propertyDock);
	m_propertyDockTabs->setDocumentMode(true);
	m_propertyDockTabs->addTab(m_propertyBrowser, QStringLiteral("Property"));
	m_devicePage = new DevicePageWidget(m_propertyDockTabs);
	m_propertyDockTabs->addTab(m_devicePage, QStringLiteral("Devices"));
	m_propertyDock->setWidget(m_propertyDockTabs);
	connect(m_devicePage, &DevicePageWidget::urdfImportRequested, this, &MainWindow::onUrdfImportRequested);
	addDockWidget(Qt::LeftDockWidgetArea, m_propertyDock);
	resizeDocks({m_propertyDock}, {340}, Qt::Horizontal);

	m_unitDock = new QDockWidget(QStringLiteral("Unit Widget"), this);
	m_unitDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	m_unitDockTabs = new QTabWidget(m_unitDock);
	m_unitDockTabs->setDocumentMode(true);
	m_backendTree = new QTreeWidget();
	m_backendTree->setHeaderHidden(true);
	m_backendRootItem = new QTreeWidgetItem(QStringList() << QStringLiteral("BackendDataManager"));
	m_backendTree->addTopLevelItem(m_backendRootItem);
	m_annotationRootItem = new QTreeWidgetItem(QStringList() << QStringLiteral("Annotations"));
	m_backendRootItem->addChild(m_annotationRootItem);
	m_backendRootItem->setExpanded(true);
	m_annotationRootItem->setExpanded(true);
	connect(m_backendTree, &QTreeWidget::itemSelectionChanged, this, &MainWindow::onBackendTreeSelectionChanged);
	connect(m_backendTree, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int column) {
		if (!item || column != 0 || !currentOsgWidget())
		{
			return;
		}
		const int itemType = item->data(0, kRoleItemType).toInt();
		if (itemType == kItemTypeBackend)
		{
			const bool visible = item->checkState(0) != Qt::Unchecked;
			const QString backendId = item->data(0, kRoleBackendId).toString();
			currentOsgWidget()->setBackendObjectVisible(backendId.toStdString(), visible);
			// Parent visibility cascades to all backend descendants.
			const QSignalBlocker guard(m_backendTree);
			std::function<void(QTreeWidgetItem*)> cascade = [&](QTreeWidgetItem* node) {
				if (!node)
				{
					return;
				}
				for (int i = 0; i < node->childCount(); ++i)
				{
					QTreeWidgetItem* child = node->child(i);
					if (!child || child == m_annotationRootItem)
					{
						continue;
					}
					if (child->data(0, kRoleItemType).toInt() != kItemTypeBackend)
					{
						continue;
					}
					child->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
					const QString childId = child->data(0, kRoleBackendId).toString();
					currentOsgWidget()->setBackendObjectVisible(childId.toStdString(), visible);
					cascade(child);
				}
			};
			cascade(item);
			if (m_runInfoPage)
			{
				m_runInfoPage->appendInfo(QStringLiteral("Scene content %1.").arg(visible ? QStringLiteral("shown") : QStringLiteral("hidden")));
			}
			return;
		}
		if (itemType != kItemTypeAnnotation)
		{
			return;
		}
		const QString annotationId = item->data(0, kRoleAnnotationId).toString();
		const bool visible = item->checkState(0) == Qt::Checked;
		currentOsgWidget()->setAnnotationVisible(annotationId, visible);
	});
	m_backendTree->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(m_backendTree, &QTreeWidget::customContextMenuRequested, this, &MainWindow::onBackendTreeContextMenu);
	m_simulationDockTabs = new QTabWidget(m_unitDockTabs);
	m_simulationCommandPage = new SimulationCommandWidget(m_simulationDockTabs);
	m_robotAxisControlPage = new RobotAxisControlWidget(m_simulationDockTabs);
	m_simulationDockTabs->addTab(m_simulationCommandPage, QStringLiteral("Instructions"));
	m_simulationDockTabs->addTab(m_robotAxisControlPage, QStringLiteral("Axis control"));
	m_unitDockTabs->addTab(m_backendTree, QStringLiteral("Units"));
	m_unitDockTabs->addTab(m_simulationDockTabs, QStringLiteral("Simulation"));
	m_osgSceneTree = new QTreeWidget();
	m_osgSceneTree->setColumnCount(2);
	m_osgSceneTree->setHeaderHidden(false);
	m_osgSceneTree->header()->setStretchLastSection(true);
	m_osgSceneTree->setColumnWidth(0, 260);
	m_osgSceneTree->setUniformRowHeights(false);
	m_osgSceneTree->setWordWrap(true);
	m_osgSceneTree->setAnimated(false);
	m_osgSceneTree->setHeaderLabels(QStringList() << QStringLiteral("Node") << QStringLiteral("Local transform"));
	m_unitDockTabs->addTab(m_osgSceneTree, QStringLiteral("Scene"));
	m_unitDock->setWidget(m_unitDockTabs);
	connect(m_simulationCommandPage, &SimulationCommandWidget::runRequested, this, &MainWindow::onSimulationRunRequested);
	connect(m_simulationCommandPage, &SimulationCommandWidget::stopRequested, this, &MainWindow::onSimulationStopRequested);
	connect(m_robotAxisControlPage, &RobotAxisControlWidget::jointAnglesChanged, this, &MainWindow::onRobotAxisJointAnglesChanged);
	addDockWidget(Qt::RightDockWidgetArea, m_unitDock);

	m_runDock = new QDockWidget(QStringLiteral("Runtime Output"), this);
	m_runDock->setAllowedAreas(Qt::BottomDockWidgetArea);
	m_runInfoPage = new RunInfoPage(m_runDock);
	m_runDock->setWidget(m_runInfoPage);
	addDockWidget(Qt::BottomDockWidgetArea, m_runDock);
	m_runDock->setMinimumHeight(140);
}

QString MainWindow::i18n(const QString& en, const QString& zh) const
{
	return m_useChinese ? zh : en;
}

void MainWindow::applyLanguage()
{
	setWindowTitle(i18n(QStringLiteral("PointCloudProcess - MainWindow"), QStringLiteral("点云处理 - 主窗口")));
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

	if (m_propertyDock) m_propertyDock->setWindowTitle(i18n(QStringLiteral("Property / Devices"), QStringLiteral("属性 / 设备")));
	if (m_propertyDockTabs && m_propertyDockTabs->count() >= 2)
	{
		m_propertyDockTabs->setTabText(0, i18n(QStringLiteral("Property"), QStringLiteral("属性")));
		m_propertyDockTabs->setTabText(1, i18n(QStringLiteral("Devices"), QStringLiteral("设备")));
	}
	if (m_unitDock)
	{
		m_unitDock->setWindowTitle(i18n(QStringLiteral("Units / Simulation / Scene"),
			QStringLiteral("单元 / 仿真 / 场景")));
	}
	if (m_unitDockTabs && m_unitDockTabs->count() >= 3)
	{
		m_unitDockTabs->setTabText(0, i18n(QStringLiteral("Units"), QStringLiteral("单元部件")));
		m_unitDockTabs->setTabText(1, i18n(QStringLiteral("Simulation"), QStringLiteral("指令仿真")));
		m_unitDockTabs->setTabText(2, i18n(QStringLiteral("Scene graph"), QStringLiteral("场景层级")));
	}
	if (m_simulationCommandPage)
	{
		m_simulationCommandPage->setUseChinese(m_useChinese);
	}
	if (m_robotAxisControlPage)
	{
		m_robotAxisControlPage->setUseChinese(m_useChinese);
	}
	if (m_simulationDockTabs && m_simulationDockTabs->count() >= 2)
	{
		m_simulationDockTabs->setTabText(0, i18n(QStringLiteral("Instructions"), QStringLiteral("指令")));
		m_simulationDockTabs->setTabText(1, i18n(QStringLiteral("Axis control"), QStringLiteral("轴控制")));
	}
	refreshSimulationJointListFromCurrentDoc();
	if (m_runDock) m_runDock->setWindowTitle(i18n(QStringLiteral("Runtime Output"), QStringLiteral("运行信息")));
	if (m_runInfoPage) m_runInfoPage->setUiLanguage(m_useChinese);

	if (m_propertyBrowser)
	{
		if (QTreeWidget* tw = m_propertyBrowser->findChild<QTreeWidget*>())
		{
			tw->setHeaderLabels(QStringList()
				<< i18n(QStringLiteral("Property"), QStringLiteral("属性"))
				<< i18n(QStringLiteral("Value"), QStringLiteral("值")));
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
}

void MainWindow::onSelectedObjectPoseChanged(float x, float y, float z)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_backendTree)
	{
		return;
	}

	const QList<QTreeWidgetItem*> selected = m_backendTree->selectedItems();
	if (selected.isEmpty() || selected.first() == m_backendRootItem)
	{
		return;
	}

	const QString id = selected.first()->data(0, kRoleBackendId).toString();
	const auto data = activeBackend().getData(id.toStdString());
	auto pointCloud = std::dynamic_pointer_cast<PointCloudBackendData>(data);
	if (pointCloud)
	{
		BackendVec3 pose;
		pose.x = x;
		pose.y = y;
		pose.z = z;
		pointCloud->setPose(pose);
		updatePropertyPanel(pointCloud);
		return;
	}
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data);
	if (mesh)
	{
		BackendVec3 pose;
		pose.x = x;
		pose.y = y;
		pose.z = z;
		mesh->setPose(pose);
		updatePropertyPanel(mesh);
	}
}

void MainWindow::onSelectedObjectRotationChanged(float rx, float ry, float rz)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_backendTree)
	{
		return;
	}

	const QList<QTreeWidgetItem*> selected = m_backendTree->selectedItems();
	if (selected.isEmpty() || selected.first() == m_backendRootItem)
	{
		return;
	}

	const QString id = selected.first()->data(0, kRoleBackendId).toString();
	const auto data = activeBackend().getData(id.toStdString());
	auto pointCloud = std::dynamic_pointer_cast<PointCloudBackendData>(data);
	if (pointCloud)
	{
		BackendVec3 rot;
		rot.x = rx;
		rot.y = ry;
		rot.z = rz;
		pointCloud->setRotation(rot);
		updatePropertyPanel(pointCloud);
		return;
	}
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data);
	if (mesh)
	{
		BackendVec3 rot;
		rot.x = rx;
		rot.y = ry;
		rot.z = rz;
		mesh->setRotation(rot);
		updatePropertyPanel(mesh);
	}
}

void MainWindow::onSelectedObjectColorChanged(float r, float g, float b, float a)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	if (!m_backendTree)
	{
		return;
	}
	const QList<QTreeWidgetItem*> selected = m_backendTree->selectedItems();
	if (selected.isEmpty() || selected.first() == m_backendRootItem)
	{
		return;
	}
	const QString id = selected.first()->data(0, kRoleBackendId).toString();
	const auto data = activeBackend().getData(id.toStdString());
	if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(data))
	{
		BackendColor c;
		c.r = r; c.g = g; c.b = b; c.a = a;
		pc->setColor(c);
		updatePropertyPanel(pc);
		return;
	}
	if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
	{
		BackendColor c;
		c.r = r; c.g = g; c.b = b; c.a = a;
		mesh->setColor(c);
		updatePropertyPanel(mesh);
	}
}

void MainWindow::onActiveAxisChanged(const QString& axisName)
{
	if (sender() != currentOsgWidget())
	{
		return;
	}
	m_activeAxisName = axisName;
	if (!m_backendTree)
	{
		return;
	}
	const QList<QTreeWidgetItem*> selected = m_backendTree->selectedItems();
	if (selected.isEmpty() || selected.first() == m_backendRootItem)
	{
		return;
	}
	const QString id = selected.first()->data(0, kRoleBackendId).toString();
	updatePropertyPanel(activeBackend().getData(id.toStdString()));
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
		m_runInfoPage->appendInfo(QStringLiteral("界面语言已切换为中文。"));
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
		if (p && p->osgWidget())
		{
			p->osgWidget()->setViewerBackgroundForDarkUi(dark);
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
	return p ? p->osgWidget() : nullptr;
}

BackendDataManager& MainWindow::activeBackend()
{
	static BackendDataManager s_unused;
	DocumentPage* p = currentPage();
	return p ? p->backend() : s_unused;
}

void MainWindow::wireDocumentPageSignals(DocumentPage* page)
{
	if (!page || !page->osgWidget())
	{
		return;
	}
	OsgWidget* o = page->osgWidget();
	connect(o, &OsgWidget::selectedObjectPoseChanged, this, &MainWindow::onSelectedObjectPoseChanged);
	connect(o, &OsgWidget::selectedObjectRotationChanged, this, &MainWindow::onSelectedObjectRotationChanged);
	connect(o, &OsgWidget::selectedObjectColorChanged, this, &MainWindow::onSelectedObjectColorChanged);
	connect(o, &OsgWidget::activeAxisChanged, this, &MainWindow::onActiveAxisChanged);
	connect(o, &OsgWidget::selectionCanceledByEsc, this, &MainWindow::onSelectionCanceledByEsc);
	connect(o, &OsgWidget::annotationCreated, this, &MainWindow::onAnnotationCreated);
	connect(o, &OsgWidget::annotationRemoved, this, &MainWindow::onAnnotationRemoved);
	connect(o, &OsgWidget::annotationVisibilityChanged, this, &MainWindow::onAnnotationVisibilityChanged);
	connect(o, &OsgWidget::pointPickFeedback, this, &MainWindow::onPointPickFeedback);
}

void MainWindow::onNewDocument()
{
	if (!m_documentTabs)
	{
		return;
	}
	auto* page = new DocumentPage(m_documentTabs);
	wireDocumentPageSignals(page);
	if (OsgWidget* osg = page->osgWidget())
	{
		osg->setViewerBackgroundForDarkUi(viewerUsesDarkBackground());
	}
	const QString title = i18n(QStringLiteral("Untitled"), QStringLiteral("未命名"));
	m_documentTabs->addTab(page, title);
	m_documentTabs->setCurrentWidget(page);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("New document."), QStringLiteral("新建文档。")));
	}
	onDocumentTabChanged(m_documentTabs->currentIndex());
}

void MainWindow::onDocumentTabChanged(int)
{
	stopRobotSimulation();
	refreshBackendTree();
	updatePropertyPanel(nullptr);
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

void MainWindow::stopRobotSimulation()
{
	const QVector<double> lastJointAngles = m_robotInstructionPlayback.jointAnglesRad();
	m_robotInstructionPlayback.stop();
	m_robotSimTimer.stop();
	if (m_simulationCommandPage)
	{
		m_simulationCommandPage->setSimulationRunning(false);
	}
	if (m_robotAxisControlPage)
	{
		m_robotAxisControlPage->setInteractionEnabled(true);
		DocumentPage* doc = currentPage();
		if (doc && doc->hasRobotSimulationContext() && !lastJointAngles.isEmpty()
			&& lastJointAngles.size() == m_robotAxisControlPage->jointCount())
		{
			m_robotAxisControlPage->setJointAnglesRad(lastJointAngles);
		}
	}
}

void MainWindow::refreshSimulationJointListFromCurrentDoc()
{
	if (!m_simulationCommandPage || !m_robotAxisControlPage)
	{
		return;
	}
	DocumentPage* doc = currentPage();
	if (doc && doc->hasRobotSimulationContext())
	{
		m_simulationCommandPage->setRevoluteJointNames(doc->robotRevoluteJointNames());
		const QStringList& jn = doc->robotRevoluteJointNames();
		if (!jn.isEmpty() && doc->robotJointLowerRad().size() == jn.size() && doc->robotJointUpperRad().size() == jn.size())
		{
			m_robotAxisControlPage->setJoints(jn, doc->robotJointLowerRad(), doc->robotJointUpperRad());
		}
		else
		{
			m_robotAxisControlPage->clearJoints();
		}
	}
	else
	{
		m_simulationCommandPage->setRevoluteJointNames(QStringList());
		m_robotAxisControlPage->clearJoints();
	}
}

void MainWindow::onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad)
{
	if (m_robotInstructionPlayback.isRunning())
	{
		return;
	}
	DocumentPage* doc = currentPage();
	OsgWidget* osg = currentOsgWidget();
	(void)RobotSceneKinematics::applyJointAnglesFromDocument(doc, osg, jointAnglesRad);
}

void MainWindow::onSimulationStopRequested()
{
	stopRobotSimulation();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("Simulation stopped."), QStringLiteral("仿真已停止。")));
	}
}

void MainWindow::onSimulationRunRequested()
{
	onSimulationStartTriggered();
}

void MainWindow::onSimulationStartTriggered()
{
	if (m_robotInstructionPlayback.isRunning())
	{
		return;
	}
	DocumentPage* doc = currentPage();
	OsgWidget* osg = currentOsgWidget();
	if (!doc || !osg || !m_simulationCommandPage)
	{
		return;
	}
	if (!doc->hasRobotSimulationContext())
	{
		if (m_runInfoPage)
		{
			m_runInfoPage->appendWarning(i18n(
				QStringLiteral("Import a robot (URDF) first, then add simulation commands."),
				QStringLiteral("请先导入机器人(URDF)，再添加仿真指令。")));
		}
		return;
	}
	if (doc->robotUrdfAbsolutePath().isEmpty())
	{
		return;
	}
	if (doc->robotRevoluteJointNames().isEmpty())
	{
		if (m_runInfoPage)
		{
			m_runInfoPage->appendWarning(i18n(
				QStringLiteral("No revolute joints in URDF (joints need type=\"revolute\" or \"continuous\" and an axis)."),
				QStringLiteral("URDF中无可旋转关节（需 type=“revolute/continuous” 及 axis）。")));
		}
		return;
	}
	const QVector<RobotSimulationCommand> queue = m_simulationCommandPage->commands();
	if (queue.isEmpty())
	{
		if (m_runInfoPage)
		{
			m_runInfoPage->appendWarning(i18n(QStringLiteral("Add at least one instruction row."),
				QStringLiteral("请至少添加一条指令。")));
		}
		return;
	}
	const QStringList jnames = doc->robotRevoluteJointNames();
	QVector<double> initialAngles(jnames.size());
	if (m_robotAxisControlPage && m_robotAxisControlPage->jointCount() == jnames.size())
	{
		initialAngles = m_robotAxisControlPage->jointAnglesRad();
	}
	else
	{
		initialAngles.fill(0.0);
	}
	QString err;
	if (!m_robotInstructionPlayback.tryStart(doc, osg, queue, initialAngles, &err))
	{
		if (m_runInfoPage)
		{
			if (err.contains(QLatin1String("Invalid joint index")))
			{
				m_runInfoPage->appendWarning(i18n(QStringLiteral("Invalid joint index in simulation command."),
					QStringLiteral("仿真指令关节索引无效。")));
			}
			else if (!err.isEmpty())
			{
				m_runInfoPage->appendWarning(err);
			}
		}
		return;
	}
	if (m_robotAxisControlPage)
	{
		m_robotAxisControlPage->setInteractionEnabled(false);
	}
	m_simulationCommandPage->setSimulationRunning(true);
	m_robotSimTimer.start();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("Simulation started."), QStringLiteral("仿真已开始。")));
	}
}

void MainWindow::onRobotSimulationTick()
{
	if (!m_robotInstructionPlayback.isRunning())
	{
		m_robotSimTimer.stop();
		return;
	}
	DocumentPage* doc = currentPage();
	OsgWidget* osg = currentOsgWidget();
	const RobotInstructionPlaybackTickResult r = m_robotInstructionPlayback.tick(doc, osg);
	switch (r)
	{
	case RobotInstructionPlaybackTickResult::Continue:
		break;
	case RobotInstructionPlaybackTickResult::Finished:
		stopRobotSimulation();
		if (m_runInfoPage)
		{
			m_runInfoPage->appendInfo(
				i18n(QStringLiteral("Simulation finished."), QStringLiteral("仿真已结束。")));
		}
		break;
	case RobotInstructionPlaybackTickResult::Aborted:
		stopRobotSimulation();
		break;
	}
}
