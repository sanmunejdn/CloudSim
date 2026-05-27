#pragma once

#include <QByteArray>
#include <QMainWindow>
#include <QHash>
#include <QElapsedTimer>
#include <QTimer>
#include <QTabWidget>
#include <QVariant>
#include <QHash>
#include <QVector>
#include <memory>
#include <string>
#include <vector>

#include "widget_global.h"
#include "BackendFollowSolve.h"
#include "MainWindowSelectionState.h"

#include <json.hpp>

class QWidget;
class OsgWidget;
class DocumentPage;
namespace osg { class Matrixd; }
class QTabWidget;
class QTreeWidget;
class QTreeWidgetItem;
class QtProperty;
class QtTreePropertyBrowser;
class QtVariantEditorFactory;
class QtVariantPropertyManager;
class BackendDataBase;
class BackendDataManager;
class BackendHierarchyModel;
class PointCloudBackendData;
class MeshBackendData;
class QAction;
class QPoint;
class RunInfoPage;
class QMenu;
class QDockWidget;
class QActionGroup;
class MainWindowImportCaptureRenderController;
class JobSystem;
class DevicePageWidget;
class RobotSimulationController;
class MainWindowRobotHost;
class MainWindowSelectionService;
class MainWindowObjectRepository;
class AiAssistantDockWidget;
class AiAssistantCoordinator;
class PluginManager;

namespace cloudsim::core {
class EventHub;
}

namespace RobotInstruction {
class Base;
struct Vec3;
struct FeasibleMotionAxisConfigurationOptions;
enum class Type;
}

/// 应用程序主窗口：菜单、停靠栏、文档页、属性面板与 OsgWidget 协调入口
class WIDGET_EXPORT MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(cloudsim::core::EventHub& appEvents, QWidget* parent = nullptr);
	~MainWindow() override;

	/// QApplication::exec 返回后调用一次（同 RunInfoPage 的 RunLogger）
	static void shutdownApplicationLogging();

	DocumentPage* currentPage() const;
	QTabWidget* documentTabs() const { return m_documentTabs; }
	int documentTabCount() const;
	RunInfoPage* runInfoPage() const { return m_runInfoPage; }
	PluginManager* pluginManager() const { return m_pluginManager; }
	void loadPlugins();
	/// 工作区/AI 旁插件页签（返回索引，失败 -1）
	int addPluginSidePanelTab(const QString& title, QWidget* widget);
	void removePluginSidePanelTab(QWidget* widget);
	QTabWidget* rightPanelTabs() const { return m_rightPanelTabs; }
	int currentSimulationRobotInstanceIndex() const;
	RobotSimulationController* robotSimulation() { return m_robotSimulation.get(); }
	class SimulationCommandWidget* simulationCommandPage() const;
	void refreshSimulationJointListFromCurrentDoc();
	void syncRobotFrameSettingsFromDocument(int instanceIndex);
	void refreshRobotCoordinateFrameOverlays(
		const std::shared_ptr<RobotInstruction::Base>& instruction = nullptr);
	void applyRobotPoseForInstructionPreview(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void syncInstructionRenderMatricesFromPose(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void refreshInstructionPoseAxes();
	void stopRobotSimulation();
	/// 层级/工程批量导入时抑制逐对象 refreshBackendTree
	class ScopedBackendTreeRefreshSuppress
	{
	public:
		explicit ScopedBackendTreeRefreshSuppress(MainWindow& mw);
		~ScopedBackendTreeRefreshSuppress();
	private:
		MainWindow& m_mw;
	};

	void focusBackendInTreeAfterImport(const std::shared_ptr<BackendDataBase>& backendObject);

	void afterBackendFollowPropertyEdited(const QString& propertyKey, const QString& valueText);

private:
	friend class MainWindowImportCaptureRenderController;
	friend class MainWindowSelectionService;
	friend class MainWindowObjectRepository;
	friend class PluginHostContext;
	friend class MainWindowRobotHost;
	void setupMenuBar();
	void setupDockWidgets();
	void applyLanguage();
	void notifyPluginsLanguageChanged();
	QString i18n(const QString& en, const QString& zh) const;
	void refreshBackendTree();
	void beginBackendTreeEventRefreshSuppress();
	void endBackendTreeEventRefreshSuppress();
	void refreshOsgSceneTree();
	/// 选中后端树行并刷新属性面板（导入/工程加载）
	void focusBackendInTree(const std::shared_ptr<BackendDataBase>& backendObject);
	/// 优先 Data/CGAL 加载几何；LAS/LAZ 等回退 OSG
	bool registerBackendObject(const QString& filePath, const QString& typeName, bool isPointCloud, bool quietUi = false);
	void updatePropertyPanel(const std::shared_ptr<BackendDataBase>& data);
	void updateInstructionPropertyPanel(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		bool refreshFeasibleAxisOptions = true);
	void invalidateFeasibleAxisConfigurationCache();
	QString instructionEnumTokenFromProperty(QtProperty* property, const QVariant& value) const;
	void applySuggestedAxisPresetFromSeedIfNeeded(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		const QVector<double>& seedJointRad,
		const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible);
	void appendPropertyBrowserRow(
		const QString& propertyKey,
		const QString& displayLabel,
		const QString& value,
		bool editable,
		const std::vector<std::string>* enumOptionTokens = nullptr,
		const QStringList* enumDisplayNames = nullptr,
		const QString& toolTip = QString());
	RobotInstruction::FeasibleMotionAxisConfigurationOptions feasibleMotionAxisConfigurationOptionsForInstruction(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		QVector<double>* outSeedJointRad = nullptr);
	QString propertyDisplayLabelForKey(const QString& key, const QString& labelEnFallback) const;
	bool tryCaptureCurrentRobotTcpPose(
		RobotInstruction::Vec3& outPoseMm,
		RobotInstruction::Vec3& outEulerDeg,
		osg::Matrixd* outTcpLocalMat,
		osg::Matrixd* outTcpRenderWorldMat,
		QString* outTcpLinkName,
		QString* errMsg) const;
	void syncRobotKinematicsAfterPoseEdit(const std::shared_ptr<BackendDataBase>& data);
	void onSaveProject();
	void onOpenProjectFile();
	void onOpenModel();
	void onOpenPointCloud();
	void onBackendTreeSelectionChanged();
	void onSelectedObjectPoseChanged(float x, float y, float z);
	void onSelectedObjectRotationChanged(float rx, float ry, float rz);
	void onSelectedObjectColorChanged(float r, float g, float b, float a);
	void onTransformGizmoCommitted();
	void onPropertyPanelCommitTimer();
	void onActiveAxisChanged(const QString& axisName);
	void onVariantPropertyValueChanged(QtProperty* property, const QVariant& value);
	void onViewModeTriggered();
	void onObjectModeTriggered();
	void onPointPickModeTriggered();
	void onMeshLinePickModeTriggered();
	void onMeshFacePickModeTriggered();
	void onSelectionCanceledByEsc();
	void onAnnotationCreated(const QString& annotationId, const QString& displayText);
	void onAnnotationRemoved(const QString& annotationId);
	void onAnnotationVisibilityChanged(const QString& annotationId, bool visible);
	void onBackendTreeContextMenu(const QPoint& pos);
	void removeBackendObjectFromDocument(const QString& backendId);
	void onLanguageEnglishTriggered();
	void onLanguageChineseTriggered();
	void onThemeActionGroupTriggered(QAction* action);
	void onNewDocument();
	void onDocumentTabChanged(int index);
	void onPointPickFeedback(const QString& text);
	void onMeshPickFeedback(const QString& text);
	void onOsgBackendObjectPicked(const QString& backendId);
	void onUrdfImportRequested(const QString& urdfPath);
	void onSimulationStartTriggered();
	void onSimulationRunRequested();
	void onSimulationStopRequested();
	void onSimulationAddInstructionRequested(RobotInstruction::Type type);
	void onSimulationInstructionSelectionChanged(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void onRobotSimulationTick();
	void onSimulationExportRequested();
	void onRobotCoordinateFramesChanged();
	void onCaptureToolFrameFromTcp();
	void onCaptureUserFrameFromTcp();
	void onResetToolFrame();
	void onSimulationRobotSelectionChanged(int instanceIndex, const QString& sceneBackendId);
	void onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad);
	void onSimulationTcpDragTeachModeChanged(bool enabled);
	void onTcpDragTeachPoseChanged(double pxMm, double pyMm, double pzMm, double exDeg, double eyDeg, double ezDeg);
	void onTcpDragTeachEnded();
	void setupAiAssistantCoordinator();
	void refreshAiAssistantHost();
	void onAiParseFailed(const QString& message, const QString& parserVia);
	void finishAiAssistantReply(const QString& reply, bool isError, const QString& parserVia = QString());

	BackendDataManager& activeBackend();
	BackendHierarchyModel* activeHierarchyModel();
	const BackendHierarchyModel* activeHierarchyModel() const;
	OsgWidget* currentOsgWidget() const;
	JobSystem* jobSystem() const { return m_jobSystem; }
	void wireDocumentPageSignals(DocumentPage* page);
	void installBackendFollowFrameHook(DocumentPage* page);
	void runBackendFollowSolveAndSync(DocumentPage& page, OsgWidget& osg,
		const std::string* manualPoseAuthorityBackendId = nullptr);
	cloudsim::host::FollowSolveContext makeFollowSolveContext(OsgWidget& osg) const;
	/// 属性编辑防抖全量重建（避免每步 spin 都 clear）
	void schedulePropertyPanelCommitRefresh(const std::shared_ptr<BackendDataBase>& data);
	/// gizmo 写后端位姿/色后：跟随求解+属性面板（每次 mouse move，非仅帧定时器）
	void refreshFollowSolveAndPropertyPanelFromOsgWrite(const std::shared_ptr<BackendDataBase>& data);
	/// 从后端父子边同步 FollowAttachment：子世界=父*局部
	void applyHierarchyFollowBinding(DocumentPage* page, const std::string& childId, const std::string& parentId);
	/// 防抖应用 follow.targetName（逐键 emit，避免输入时 clear）
	void flushFollowTargetNamePropertyEdit();
	void syncViewModeActionsFromCurrentOsg();
	void setAllDocumentViewerDarkBackground(bool dark);
	bool viewerUsesDarkBackground() const;

	cloudsim::core::EventHub& m_appEvents;
	int m_backendTreeEventRefreshSuppress = 0;
	QTabWidget* m_documentTabs = nullptr;
	QTreeWidget* m_backendTree = nullptr;
	QTreeWidget* m_osgSceneTree = nullptr;
	QHash<QString, QTreeWidgetItem*> m_backendTreeItemsById;
	QTreeWidgetItem* m_backendRootItem = nullptr;
	QTreeWidgetItem* m_annotationRootItem = nullptr;
	QtTreePropertyBrowser* m_propertyBrowser = nullptr;
	QtVariantPropertyManager* m_variantManager = nullptr;
	QtVariantEditorFactory* m_variantFactory = nullptr;
	RunInfoPage* m_runInfoPage = nullptr;
	QAction* m_resetLayoutAction = nullptr;
	QAction* m_viewModeAction = nullptr;
	QAction* m_objectModeAction = nullptr;
	QAction* m_pointPickModeAction = nullptr;
	QAction* m_meshLinePickModeAction = nullptr;
	QAction* m_meshFacePickModeAction = nullptr;
	QActionGroup* m_interactionModeGroup = nullptr;
	QAction* m_gizmoLocalFrameAction = nullptr;
	QAction* m_gizmoWorldFrameAction = nullptr;
	QActionGroup* m_gizmoFrameGroup = nullptr;
	QAction* m_newDocumentAction = nullptr;
	QAction* m_openModelAction = nullptr;
	QAction* m_openPointCloudAction = nullptr;
	QAction* m_saveAction = nullptr;
	QAction* m_openProjectAction = nullptr;
	QAction* m_exitAction = nullptr;
	QAction* m_languageEnglishAction = nullptr;
	QAction* m_languageChineseAction = nullptr;
	QActionGroup* m_languageActionGroup = nullptr;
	QMenu* m_fileMenu = nullptr;
	QMenu* m_viewMenu = nullptr;
	QMenu* m_settingsMenu = nullptr;
	QMenu* m_languageMenu = nullptr;
	QMenu* m_appearanceMenu = nullptr;
	QAction* m_lightThemeAction = nullptr;
	QAction* m_darkThemeAction = nullptr;
	QActionGroup* m_themeActionGroup = nullptr;
	QDockWidget* m_propertyDock = nullptr;
	QTabWidget* m_propertyDockTabs = nullptr;
	DevicePageWidget* m_devicePage = nullptr;
	QDockWidget* m_unitDock = nullptr;
	QTabWidget* m_unitDockTabs = nullptr;
	std::unique_ptr<RobotSimulationController> m_robotSimulation;
	std::unique_ptr<MainWindowRobotHost> m_robotHost;
	QTimer m_robotSimTimer;
	QTimer m_followTargetNameDebounceTimer;
	QTimer m_propertyPanelCommitTimer;
	QString m_propertyPanelCommitPendingBackendId;
	QString m_followTargetNameDebounceBackendId;
	QString m_followTargetNameDebounceText;
	QDockWidget* m_runDock = nullptr;
	/// 右侧 Dock 顶栏：工作区（单元/仿真/场景）与 AI，替代 tabifyDockWidget 底部页签
	QTabWidget* m_rightPanelTabs = nullptr;
	AiAssistantDockWidget* m_aiAssistantPage = nullptr;
	AiAssistantCoordinator* m_aiCoordinator = nullptr;
	QAction* m_toggleAiAssistantAction = nullptr;
	QAction* m_simulationStartAction = nullptr;
	bool m_useChinese = true;
	bool m_updatingPropertyBrowser = false;
	MainWindowSelectionState m_selectionState;
	JobSystem* m_jobSystem = nullptr;
	PluginManager* m_pluginManager = nullptr;
	bool m_pluginsLoadStarted = false;
	QString m_activeAxisName = QStringLiteral("None");
	std::shared_ptr<RobotInstruction::Base> m_activeInstructionForProperty;
	QHash<QtProperty*, QStringList> m_propertyEnumTokens;
};
