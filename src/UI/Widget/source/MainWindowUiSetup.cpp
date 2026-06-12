#include "MainWindow.h"

#include <QAbstractItemView>
#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QDockWidget>
#include <QHeaderView>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

#include "ApplicationStyle.h"
#include "UiIconDecorators.h"
#include "DocumentPage.h"
#include "CoreEvents.h"
#include "EventHub.h"
#include "DevicePageWidget.h"
#include "JobSystem.h"
#include "MainWindowSelectionService.h"
#include "MainWindow_p.h"
#include "WidgetRenderAccess.h"
#include "../RobotWidget/inc/IRobotOsgViewHost.h"
#include "PluginManager.h"
#include "ProgressManager.h"
#include "RunInfoPage.h"
#include "RunLogger.h"
#include "MainWindowRobotHost.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"
#include "AiAssistantDockWidget.h"
#include "AiAssistantCoordinator.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>

#include <mutex>
#include <string>

#include "qteditorfactory.h"
#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

using namespace mainwindow_detail;
using namespace RobotSimulation;

namespace
{
/// 内容区已有 QTabWidget 时隐藏 Dock 自带标题栏，避免与页签重复。
void hideDockTitleBar(QDockWidget* dock)
{
	if (!dock)
	{
		return;
	}
	auto* titleBar = new QWidget(dock);
	titleBar->setFixedHeight(0);
	dock->setTitleBarWidget(titleBar);
}

void setupDockTabWidget(QTabWidget* tabs)
{
	if (!tabs)
	{
		return;
	}
	tabs->setDocumentMode(true);
	tabs->setContentsMargins(0, 2, 0, 0);
}
} // namespace

MainWindow::MainWindow(cloudsim::core::EventHub& appEvents, QWidget* parent)
	: QMainWindow(parent)
	, m_appEvents(appEvents)
	, m_instructionPropertyUiHost(*this)
{
	// RunLogger must live in this DLL (same TU as RunInfoPage::setUiSink). The exe used to link RunLogger.lib
	// separately, which duplicated globals so file logging initialized in main never matched UI / RobotScene.
	static std::once_flag s_runLoggerOnce;
	std::call_once(s_runLoggerOnce, []() {
		const QString logDir =
			QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("logs"));
		(void)RunLogger::initialize(logDir.toStdString(), "CloudSim");
		RunLogger::info("Application bootstrap started.");
		const QByteArray kd = qgetenv("ROBOT_KINEMATICS_DEBUG");
		if (!kd.isEmpty() && kd != QByteArray("0"))
		{
			RunLogger::info(std::string("[RobotKinematicsDBG] ROBOT_KINEMATICS_DEBUG=") + kd.constData()
				+ " 鈥?FK dumps: 1=compact 2|full=4x4; use --robot-kinematics-debug 1");
			RunLogger::flush();
		}
		const QByteArray gpd = qgetenv("POINTCLOUD_GIZMO_PIVOT_DIAG");
		if (!gpd.isEmpty() && gpd != QByteArray("0"))
		{
			RunLogger::info(std::string("[GizmoPivotDiag] POINTCLOUD_GIZMO_PIVOT_DIAG=") + gpd.constData()
				+ " 鈥?pivot vs file-origin dumps on selection sync and gizmo drag release");
			RunLogger::flush();
		}
	});

	m_robotHost = std::make_unique<MainWindowRobotHost>(this);
	m_robotSimulation = std::make_unique<RobotSimulationController>(this);
	m_robotSimulation->setHost(m_robotHost.get());
	m_robotSimulation->initializePlanners();
	setupMenuBar();

	auto* central = new QWidget(this);
	central->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	auto* rootLayout = new QVBoxLayout(central);
	rootLayout->setContentsMargins(8, 8, 8, 8);
	rootLayout->setSpacing(8);

	m_documentTabs = new QTabWidget(central);
	m_documentTabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_documentTabs->setDocumentMode(true);
	m_documentTabs->setTabsClosable(false);
	rootLayout->addWidget(m_documentTabs, 1);

	auto* firstPage = new DocumentPage(m_documentTabs, m_appEvents);
	wireDocumentPageSignals(firstPage);
	m_documentTabs->addTab(firstPage, QStringLiteral("Untitled"));

	m_appEvents.subscribe<cloudsim::core::BackendObjectRegisteredEvent>(
		[this](const cloudsim::core::BackendObjectRegisteredEvent& ev) {
			if (m_backendTreeEventRefreshSuppress > 0)
			{
				return;
			}
			DocumentPage* page = currentPage();
			if (!page || ev.documentId != page->documentId())
			{
				return;
			}
			refreshBackendTree();
		});
	m_appEvents.subscribe<cloudsim::core::BackendObjectRemovedEvent>(
		[this](const cloudsim::core::BackendObjectRemovedEvent& ev) {
			if (m_backendTreeEventRefreshSuppress > 0)
			{
				return;
			}
			DocumentPage* page = currentPage();
			if (!page || ev.documentId != page->documentId())
			{
				return;
			}
			refreshBackendTree();
		});
	m_appEvents.subscribe<cloudsim::core::SelectionChangedEvent>(
		[this](const cloudsim::core::SelectionChangedEvent& ev) {
			DocumentPage* page = currentPage();
			if (!page || ev.documentId != page->documentId())
			{
				return;
			}
			if (ev.primaryId.isEmpty())
			{
				updatePropertyPanel(QString());
				return;
			}
			if (m_selectionState.selectedBackendId() != ev.primaryId)
			{
				m_selectionState.setSelectedBackendId(ev.primaryId);
			}
			updatePropertyPanel(ev.primaryId);
		});
	m_appEvents.subscribe<cloudsim::core::PoseCommittedEvent>([this](const cloudsim::core::PoseCommittedEvent& ev) {
		DocumentPage* page = currentPage();
		if (!page || ev.documentId != page->documentId())
		{
			return;
		}
		if (!m_selectionState.hasBackendSelection() || m_selectionState.selectedBackendId() != ev.objectId)
		{
			return;
		}
		if (shouldDeferPropertyPanelRebuild(ev.objectId))
		{
			syncPropertyPanelRowValues(ev.objectId);
			return;
		}
		schedulePropertyPanelCommitRefresh(ev.objectId);
	});

	setCentralWidget(central);
	setupDockWidgets();
	m_robotSimTimer.setInterval(kPlaybackTimerIntervalMs);
	m_robotSimulation->attachPlaybackTimer(&m_robotSimTimer);
	connect(m_documentTabs, &QTabWidget::currentChanged, this, &MainWindow::onDocumentTabChanged);
	m_followTargetNameDebounceTimer.setSingleShot(true);
	connect(&m_followTargetNameDebounceTimer, &QTimer::timeout, this, &MainWindow::flushFollowTargetNamePropertyEdit);
	m_propertyPanelCommitTimer.setSingleShot(true);
	connect(&m_propertyPanelCommitTimer, &QTimer::timeout, this, &MainWindow::onPropertyPanelCommitTimer);
	m_instructionPropertyRefreshTimer.setSingleShot(true);
	connect(&m_instructionPropertyRefreshTimer, &QTimer::timeout, this, &MainWindow::onInstructionPropertyRefreshTimer);
	m_propertyVisualPreviewTimer.setSingleShot(true);
	connect(&m_propertyVisualPreviewTimer, &QTimer::timeout, this, &MainWindow::onPropertyVisualPreviewTimer);
	m_jobSystem = new JobSystem(this);
	if (m_jobSystem->progressManager())
	{
		connect(m_jobSystem->progressManager(), &ProgressManager::jobProgress, this,
			[this](quint64 /*jobId*/, double /*fraction*/, const QString& message) {
				if (!message.isEmpty() && m_runInfoPage)
				{
					m_runInfoPage->appendInfo(message);
				}
			});
	}
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
	const QString pluginReport = [this]() {
		DocumentPage* page = currentPage();
		if (!page)
		{
			return QStringLiteral("Ready");
		}
		return page->render().pointCloudPluginReport();
	}();
	statusBar()->showMessage(pluginReport, 12000);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("Application started."),
			QStringLiteral("\u5e94\u7528\u7a0b\u5e8f\u5df2\u542f\u52a8\u3002")));
		m_runInfoPage->appendInfo(pluginReport);
	}
	onDocumentTabChanged(m_documentTabs ? m_documentTabs->currentIndex() : -1);

	// Reasonable default geometry so the window does not maximize overly wide on first show.
	resize(1180, 760);
	setMinimumSize(900, 560);
}

void MainWindow::setupMenuBar()
{
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
	connect(m_gizmoLocalFrameAction, &QAction::triggered, this, [this](bool checked) {
		if (!checked)
		{
			return;
		}
		if (IRobotOsgViewHost* view = activeOsgViewHost())
		{
			view->setTransformGizmoFrame(1);
		}
	});
	connect(m_gizmoWorldFrameAction, &QAction::triggered, this, [this](bool checked) {
		if (!checked)
		{
			return;
		}
		if (IRobotOsgViewHost* view = activeOsgViewHost())
		{
			view->setTransformGizmoFrame(0);
		}
	});
	 m_viewMenu->addSeparator();
	 m_simulationStartAction = m_viewMenu->addAction(QStringLiteral("Start Simulation"), this, &MainWindow::onSimulationStartTriggered);

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

	UiIconDecorators::apply(m_newDocumentAction, UiIconId::NewDocument);
	UiIconDecorators::apply(m_openProjectAction, UiIconId::OpenProject);
	UiIconDecorators::apply(m_saveAction, UiIconId::SaveProject);
	UiIconDecorators::apply(m_openModelAction, UiIconId::OpenModel);
	UiIconDecorators::apply(m_openPointCloudAction, UiIconId::OpenPointCloud);
	UiIconDecorators::apply(m_viewModeAction, UiIconId::ViewMode);
	UiIconDecorators::apply(m_objectModeAction, UiIconId::ObjectSelect);
	UiIconDecorators::apply(m_pointPickModeAction, UiIconId::PointPick);
	UiIconDecorators::apply(m_meshLinePickModeAction, UiIconId::LinePick);
	UiIconDecorators::apply(m_meshFacePickModeAction, UiIconId::FacePick);
}

void MainWindow::setupDockWidgets()
{
	m_propertyDock = new QDockWidget(QStringLiteral("Property"), this);
	m_propertyDock->setObjectName(QStringLiteral("PropertyDock"));
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
	if (QTreeWidget* propTree = m_propertyBrowser->findChild<QTreeWidget*>())
	{
		propTree->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked
			| QAbstractItemView::EditKeyPressed | QAbstractItemView::AnyKeyPressed);
	}
	connect(m_variantManager, &QtVariantPropertyManager::valueChanged, this, &MainWindow::onVariantPropertyValueChanged);
	installPropertyPanelEventFilter();
	m_propertyDockTabs = new QTabWidget(m_propertyDock);
	setupDockTabWidget(m_propertyDockTabs);
	m_propertyDockTabs->addTab(m_propertyBrowser, QStringLiteral("Property"));
	m_devicePage = new DevicePageWidget(m_propertyDockTabs);
	m_propertyDockTabs->addTab(m_devicePage, QStringLiteral("Devices"));
	m_propertyDock->setWidget(m_propertyDockTabs);
	hideDockTitleBar(m_propertyDock);
	connect(m_devicePage, &DevicePageWidget::urdfImportRequested, this, &MainWindow::onUrdfImportRequested);
	addDockWidget(Qt::LeftDockWidgetArea, m_propertyDock);
	resizeDocks({ m_propertyDock }, { 340 }, Qt::Horizontal);

	m_unitDock = new QDockWidget(QStringLiteral("Workspace"), this);
	m_unitDock->setObjectName(QStringLiteral("UnitDock"));
	m_unitDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	m_unitDockTabs = new QTabWidget(m_unitDock);
	setupDockTabWidget(m_unitDockTabs);
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
		MainWindowSelectionService::handleBackendTreeItemChanged(*this, item, column);
	});
	m_backendTree->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(m_backendTree, &QTreeWidget::customContextMenuRequested, this, &MainWindow::onBackendTreeContextMenu);
	m_robotSimulation->createSimulationDock(m_unitDockTabs);
	m_unitDockTabs->addTab(m_backendTree, QStringLiteral("Units"));
	m_unitDockTabs->addTab(m_robotSimulation->simulationDock(), QStringLiteral("Simulation"));
	m_robotSimulation->wireSimulationSignals();
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

	m_rightPanelTabs = new QTabWidget(m_unitDock);
	setupDockTabWidget(m_rightPanelTabs);
	m_rightPanelTabs->addTab(m_unitDockTabs, QStringLiteral("Workspace"));
	m_aiAssistantPage = new AiAssistantDockWidget(m_rightPanelTabs);
	m_rightPanelTabs->addTab(m_aiAssistantPage, QStringLiteral("AI"));
	m_unitDock->setWidget(m_rightPanelTabs);
	hideDockTitleBar(m_unitDock);
	addDockWidget(Qt::RightDockWidgetArea, m_unitDock);
	setTabPosition(Qt::RightDockWidgetArea, QTabWidget::North);
	setTabPosition(Qt::LeftDockWidgetArea, QTabWidget::North);
	setTabPosition(Qt::BottomDockWidgetArea, QTabWidget::North);

	m_runDock = new QDockWidget(QStringLiteral("Runtime Output"), this);
	m_runDock->setObjectName(QStringLiteral("RunDock"));
	m_runDock->setAllowedAreas(Qt::BottomDockWidgetArea);
	m_runInfoPage = new RunInfoPage(m_runDock);
	m_runDock->setWidget(m_runInfoPage);
	hideDockTitleBar(m_runDock);
	addDockWidget(Qt::BottomDockWidgetArea, m_runDock);
	resizeDocks({ m_runDock }, { 160 }, Qt::Vertical);

	setupAiAssistantCoordinator();

	m_toggleAiAssistantAction = m_viewMenu->addAction(QStringLiteral("AI Assistant"));
	m_toggleAiAssistantAction->setCheckable(true);
	m_toggleAiAssistantAction->setChecked(true);
	connect(m_toggleAiAssistantAction, &QAction::toggled, this, [this](const bool visible) {
		if (!m_rightPanelTabs || !m_aiAssistantPage)
		{
			return;
		}
		int aiTab = m_rightPanelTabs->indexOf(m_aiAssistantPage);
		if (visible)
		{
			if (aiTab < 0)
			{
				aiTab = m_rightPanelTabs->addTab(
					m_aiAssistantPage,
					i18n(QStringLiteral("AI"), QStringLiteral("AI")));
			}
			m_rightPanelTabs->setCurrentIndex(aiTab);
		}
		else if (aiTab >= 0)
		{
			if (m_rightPanelTabs->currentWidget() == m_aiAssistantPage)
			{
				m_rightPanelTabs->setCurrentIndex(0);
			}
			m_rightPanelTabs->removeTab(aiTab);
		}
	});

	m_runDock->setMinimumHeight(72);

	// Defer plugin load until the dock/tab hierarchy is fully attached (avoids addTab crash at startup).
	QTimer::singleShot(0, this, [this]() { loadPlugins(); });
}

MainWindow::~MainWindow()
{
	if (m_pluginManager)
	{
		m_pluginManager->shutdownAll();
	}
}

void MainWindow::shutdownApplicationLogging()
{
	RunLogger::info("Application shutdown.");
	RunLogger::shutdown();
}
