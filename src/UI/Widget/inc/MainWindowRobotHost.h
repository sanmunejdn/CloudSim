#ifndef WIDGET_MAINWINDOWROBOTHOST_H
#define WIDGET_MAINWINDOWROBOTHOST_H

/// @file MainWindowRobotHost.h
/// @brief MainWindow 的 IRobotMainWindowHost + 指令属性 IRobotService 委托实现

#include "../RobotWidget/inc/IRobotMainWindowHost.h"
#include "../RobotWidget/inc/ICustomDeviceAssemblyHost.h"
#include "IRobotInstructionPropertyDelegate.h"

#include <functional>
#include <memory>

class MainWindow;
class DocumentPage;
class WidgetOsgViewHost;
class CustomDeviceBackendData;

struct PickResult;
enum class PickKind;

/// MainWindow 的 IRobotMainWindowHost + 组装宿主 + 指令属性委托
class MainWindowRobotHost : public IRobotMainWindowHost,
							public ICustomDeviceAssemblyHost,
							public cloudsim::host::IRobotInstructionPropertyDelegate
{
public:
	explicit MainWindowRobotHost(MainWindow* mw);
	~MainWindowRobotHost() override;

	IRobotDocumentHost* document() override;
	const IRobotDocumentHost* document() const override;
	IRobotOsgViewHost* osgView() override;
	void endMeshSectionPlaneEditDirect() override;
	void hideMeshSectionPlaneDirect() override;

	bool useChinese() const override;
	QString i18n(const QString& en, const QString& zh) const override;

	RunInfoPage* runInfoPage() override;
	void appendRunInfo(const QString& message) override;
	void appendRunWarning(const QString& message) override;
	QStatusBar* statusBar() override;

	SimulationCommandWidget* simulationCommandPage() override;
	RobotAxisControlWidget* robotAxisControlPage() override;
	RobotFrameSettingsWidget* robotFrameSettingsPage() override;
	RobotExternalAxisSettingsWidget* robotExternalAxisSettingsPage() override;
	DevicePageWidget* devicePage() override;

	QAction* simulationStartAction() override;
	int currentSimulationRobotInstanceIndex() const override;

	void refreshBackendTree() override;
	void runFollowSolveAndSyncForCurrentDocument() override;
	void refreshInstructionPropertyPanel(const std::shared_ptr<RobotInstruction::Base>& instruction,
										 bool refreshFeasibleAxisOptions = true) override;
	void clearInstructionPropertyPanel() override;
	void invalidateInstructionPropertyCache() override;

	void clearBackendObjectSelection(bool clearTreeSelection) override;
	QString selectedBackendId() const override;

	std::shared_ptr<RobotInstruction::Base> activeInstructionForProperty() const override;
	void applySuggestedAxisPresetFromSeedIfNeeded(
		const std::shared_ptr<RobotInstruction::Base>& instruction, const QVector<double>& seedJointRad,
		const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible) override;

	bool registerUrdfRobot(const QString& urdfPath, bool quietUi) override;

	bool planRobotMotionInstruction(RobotInstruction::Base& instruction, const QVector<double>& seedJointRad,
									int instanceIndex, const QString& urdfPath, const QString& defaultTcpLinkName,
									const QString& sceneRootBackendId, RobotInstruction::PlanResult& out,
									std::string* outErr) override;

	void enqueueBackgroundJob(const QString& title, std::function<void()> work,
							  std::function<void(bool threw, const QString& msg)> onFinished) override;

	void setMeshPickCommittedHandler(std::function<void(const PickResult&, PickKind)> handler) override;
	void clearMeshPickCommittedHandler() override;
	void notifyMeshPickCommitted(const PickResult& pick, PickKind kind);

	void setMeshTriangleLabelingPickHandlers(MeshTriangleLabelingPickHandlers handlers) override;
	void clearMeshTriangleLabelingPickHandlers() override;
	void notifyMeshTriangleLabelingClick(const PickResult& pick);
	void notifyMeshTriangleLabelingBrush(const std::vector<int>& triangleIndices);
	void notifyMeshTriangleLabelingPolyline(const QVector<float>& polylineScreenXy, const QVector<double>& mvpMatrix,
											int viewportWidth, int viewportHeight);

	void wireDocumentPageSceneSignals(DocumentPage* page);

	// ICustomDeviceAssemblyHost
	bool registerCustomDevice(const std::shared_ptr<CustomDeviceBackendData>& device, QString* outError) override;
	bool attachChildToCustomDevice(const std::string& deviceId, const std::string& childId,
								   QString* outError) override;
	QStringList importModelsForAssembly(QWidget* parent, const QStringList& paths, QStringList* outErrors) override;
	void beginPickSolidInView(std::function<void(const QString& partId)> onPartPicked) override;
	void endPickSolidInView() override;
	bool exportCustomDeviceUrdfInteractive(const QString& deviceBackendId) override;
	void markFollowAttachmentDirty(const QString& deviceBackendId) override;
	void focusBackendInTree(const QString& backendId) override;
	void runFollowSolveAndSync() override;
	void onCustomDeviceAssemblyCommitted(const QString& deviceBackendId) override;

	QVector<cloudsim::core::PropertyRowDto> instructionPropertyRows(const QString& instructionId) override;
	bool applyInstructionPropertyChange(const QString& instructionId, const QString& key, const QString& value,
										QString* outError = nullptr) override;
	cloudsim::core::FeasibleMotionAxisOptionsDto
	queryFeasibleMotionAxisOptions(const QString& instructionId, QVector<double>* outSeedJointRad = nullptr) override;
	cloudsim::core::FeasibleMotionAxisOptionsDto cachedFeasibleMotionAxisOptions() override;

private:
	MainWindow* m_mw = nullptr;
	class DocumentHost;
	std::unique_ptr<DocumentHost> m_docHost;
	std::unique_ptr<WidgetOsgViewHost> m_osgHost;
	DocumentPage* m_osgHostPage = nullptr;
	std::function<void(const PickResult&, PickKind)> m_meshPickHandler;
	std::function<void(const QString&)> m_solidPickCallback;
	IRobotMainWindowHost::MeshTriangleLabelingPickHandlers m_meshTriangleLabelingHandlers;
};

#endif // WIDGET_MAINWINDOWROBOTHOST_H
