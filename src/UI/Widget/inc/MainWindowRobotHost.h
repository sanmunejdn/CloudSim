#pragma once

#include "../RobotWidget/inc/IRobotMainWindowHost.h"

#include <memory>

class MainWindow;
class DocumentPage;
class OsgWidget;

/// MainWindow 的 IRobotMainWindowHost 实现
class MainWindowRobotHost : public IRobotMainWindowHost
{
public:
	explicit MainWindowRobotHost(MainWindow* mw);

	IRobotDocumentHost* document() override;
	const IRobotDocumentHost* document() const override;
	IRobotOsgViewHost* osgView() override;

	bool useChinese() const override;
	QString i18n(const QString& en, const QString& zh) const override;

	RunInfoPage* runInfoPage() override;
	void appendRunInfo(const QString& message) override;
	void appendRunWarning(const QString& message) override;
	QStatusBar* statusBar() override;

	SimulationCommandWidget* simulationCommandPage() override;
	RobotAxisControlWidget* robotAxisControlPage() override;
	RobotFrameSettingsWidget* robotFrameSettingsPage() override;
	DevicePageWidget* devicePage() override;

	QAction* simulationStartAction() override;
	int currentSimulationRobotInstanceIndex() const override;

	void refreshBackendTree() override;
	void runFollowSolveAndSyncForCurrentDocument() override;
	void refreshInstructionPropertyPanel(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		bool refreshFeasibleAxisOptions = true) override;
	void clearInstructionPropertyPanel() override;
	void invalidateInstructionPropertyCache() override;

	void clearBackendObjectSelection(bool clearTreeSelection) override;

	std::shared_ptr<RobotInstruction::Base> activeInstructionForProperty() const override;
	void applySuggestedAxisPresetFromSeedIfNeeded(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		const QVector<double>& seedJointRad,
		const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible) override;

	bool registerUrdfRobot(const QString& urdfPath, bool quietUi) override;

	bool planRobotMotionInstruction(
		RobotInstruction::Base& instruction,
		const QVector<double>& seedJointRad,
		int instanceIndex,
		const QString& urdfPath,
		const QString& defaultTcpLinkName,
		const QString& sceneRootBackendId,
		RobotInstruction::PlanResult& out,
		std::string* outErr) override;

private:
	MainWindow* m_mw = nullptr;
	class DocumentHost;
	class OsgViewHost;
	std::unique_ptr<DocumentHost> m_docHost;
	std::unique_ptr<OsgViewHost> m_osgHost;
	OsgWidget* m_osgHostWidget = nullptr;
};
