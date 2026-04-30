#pragma once

#include <QMainWindow>
#include <QTimer>
#include <QTabWidget>
#include <QVariant>
#include <QVector>
#include <memory>
#include <string>
#include <vector>

#include "widget_global.h"
#include "RobotAxisControlWidget.h"
#include "RobotInstructionController.h"
#include "RobotInstructionPlaybackEngine.h"
#include "SimulationCommandWidget.h"

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
class PointCloudBackendData;
class MeshBackendData;
class QAction;
class QPoint;
class RunInfoPage;
class QMenu;
class QDockWidget;
class QActionGroup;
class MainWindowImportCaptureRenderController;
class DevicePageWidget;

/// 应用程序主窗口：菜单、停靠栏、文档页、属性面板与 OsgWidget 的协调入口。
class WIDGET_EXPORT MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget* parent = nullptr);
	~MainWindow() override = default;

private:
	friend class MainWindowImportCaptureRenderController;
	void setupMenuBar();
	void setupDockWidgets();
	void applyLanguage();
	QString i18n(const QString& en, const QString& zh) const;
	void refreshBackendTree();
	void refreshOsgSceneTree();
	/** Selects the tree row for this backend id and refreshes the property panel (import / project load). */
	void focusBackendInTree(const std::shared_ptr<BackendDataBase>& backendObject);
	/** Load geometry via Data (CGAL) when possible; LAS/LAZ and exotic mesh formats fall back to OSG. */
	bool registerBackendObject(const QString& filePath, const QString& typeName, bool isPointCloud, bool quietUi = false);
	bool registerExistingBackendObject(std::shared_ptr<BackendDataBase> backendObject, const QString& sourcePath,
		const QString& typeName, const QString& persistedId = QString(), bool selectInTree = true,
		const QString& parentId = QString());
	void updatePropertyPanel(const std::shared_ptr<BackendDataBase>& data);
	void updateInstructionPropertyPanel(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void appendPropertyBrowserRow(const QString& propertyKey, const QString& displayLabel, const QString& value, bool editable);
	QString propertyDisplayLabelForKey(const QString& key, const QString& labelEnFallback) const;
	bool tryCaptureCurrentRobotTcpPose(
		RobotInstruction::Vec3& outPoseMm,
		RobotInstruction::Vec3& outEulerDeg,
		osg::Matrixd* outTcpLocalMat,
		osg::Matrixd* outTcpRenderWorldMat,
		QString* outTcpLinkName,
		QString* errMsg) const;
	void syncOsgViewerFromPointCloudBackend(const std::shared_ptr<PointCloudBackendData>& pc);
	void syncOsgViewerFromMeshBackend(const std::shared_ptr<MeshBackendData>& mesh);
	void onSaveProject();
	void onOpenProjectFile();
	void onOpenModel();
	void onOpenPointCloud();
	void onBackendTreeSelectionChanged();
	void onSelectedObjectPoseChanged(float x, float y, float z);
	void onSelectedObjectRotationChanged(float rx, float ry, float rz);
	void onSelectedObjectColorChanged(float r, float g, float b, float a);
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
	void onUrdfImportRequested(const QString& urdfPath);
	void onSimulationStartTriggered();
	void onSimulationRunRequested();
	void onSimulationStopRequested();
	void onSimulationAddInstructionRequested(RobotInstruction::Type type);
	void onSimulationInstructionSelectionChanged(const std::shared_ptr<RobotInstruction::Base>& instruction);
	void onRobotSimulationTick();
	void stopRobotSimulation();
	void logPlaybackFrameComparison(const QVector<double>& finalJointAnglesRad);
	void refreshInstructionPoseAxes();
	void refreshSimulationJointListFromCurrentDoc();
	void onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad);

	DocumentPage* currentPage() const;
	BackendDataManager& activeBackend();
	OsgWidget* currentOsgWidget() const;
	void wireDocumentPageSignals(DocumentPage* page);
	void syncViewModeActionsFromCurrentOsg();
	void setAllDocumentViewerDarkBackground(bool dark);
	bool viewerUsesDarkBackground() const;

	QTabWidget* m_documentTabs = nullptr;
	QTreeWidget* m_backendTree = nullptr;
	QTreeWidget* m_osgSceneTree = nullptr;
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
	QTabWidget* m_simulationDockTabs = nullptr;
	SimulationCommandWidget* m_simulationCommandPage = nullptr;
	RobotAxisControlWidget* m_robotAxisControlPage = nullptr;
	QTimer m_robotSimTimer;
	RobotInstruction::Controller m_robotInstructionController;
	RobotInstructionPlaybackEngine m_robotInstructionPlayback;
	QDockWidget* m_runDock = nullptr;
	QAction* m_simulationStartAction = nullptr;
	bool m_useChinese = true;
	bool m_updatingPropertyBrowser = false;
	QString m_activeAxisName = QStringLiteral("None");
	std::shared_ptr<RobotInstruction::Base> m_activeInstructionForProperty;
};
