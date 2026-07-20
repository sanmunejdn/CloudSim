#ifndef WIDGET_MAINWINDOWINSTRUCTIONPROPERTYUIHOST_H
#define WIDGET_MAINWINDOWINSTRUCTIONPROPERTYUIHOST_H

/// @file MainWindowInstructionPropertyUiHost.h
/// @brief MainWindow 作为仿真指令属性面板宿主

#include "IRobotInstructionPropertyUiHost.h"

class MainWindow;
class QtProperty;
class QtTreePropertyBrowser;
class QtVariantPropertyManager;

/// MainWindow 作为仿真指令属性面板宿主
class MainWindowInstructionPropertyUiHost final : public IRobotInstructionPropertyUiHost
{
public:
	explicit MainWindowInstructionPropertyUiHost(MainWindow& mw);

	QtTreePropertyBrowser* propertyBrowser() override;
	QtVariantPropertyManager* variantManager() override;
	bool& updatingPropertyBrowserFlag() override;
	QHash<QtProperty*, QStringList>& propertyEnumTokens() override;

	IRobotDocumentHost* currentRobotDocument() override;
	bool applyInstructionPropertyChange(const QString& instructionId, const QString& key, const QString& value,
										QString* outError = nullptr) override;
	SimulationCommandWidget* simulationCommandPage() override;
	int currentSimulationRobotInstanceIndex() const override;
	void appendRunInfoMessage(const QString& en, const QString& zh) override;

	bool useChinese() const override;
	QString i18n(const QString& en, const QString& zh) const override;

	void appendPropertyBrowserRow(const QString& propertyKey, const QString& displayLabel, const QString& value,
								  bool editable, const std::vector<std::string>* enumOptionTokens = nullptr,
								  const QStringList* enumDisplayNames = nullptr,
								  const QString& toolTip = QString()) override;

	QString propertyDisplayLabelForKey(const QString& key, const QString& labelEnFallback) const override;
	QString instructionEnumTokenFromProperty(QtProperty* property, const QVariant& value) const override;

	RobotInstruction::FeasibleMotionAxisConfigurationOptions
	feasibleMotionAxisConfigurationOptionsForInstruction(const std::shared_ptr<RobotInstruction::Base>& instruction,
														 QVector<double>* outSeedJointRad = nullptr) override;

	cloudsim::core::FeasibleMotionAxisOptionsDto cachedFeasibleMotionAxisOptionsDto() const override;

	void applySuggestedAxisPresetFromSeedIfNeeded(
		const std::shared_ptr<RobotInstruction::Base>& instruction, const QVector<double>& seedJointRad,
		const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible) override;

	void setActiveInstructionForProperty(const std::shared_ptr<RobotInstruction::Base>& instruction) override;
	std::shared_ptr<RobotInstruction::Base> activeInstructionForProperty() const override;
	void invalidateFeasibleAxisConfigurationCache() override;

	void refreshInstructionPoseAxes() override;
	void syncInstructionRenderMatricesFromPose(const std::shared_ptr<RobotInstruction::Base>& instruction) override;
	void applyRobotPoseForInstructionPreview(const std::shared_ptr<RobotInstruction::Base>& instruction) override;
	void refreshRobotCoordinateFrameOverlays(const std::shared_ptr<RobotInstruction::Base>& instruction) override;

	void scheduleInstructionPropertyRefresh(const std::shared_ptr<RobotInstruction::Base>& instruction,
											bool refreshFeasibleAxisOptions) override;

	void notifyPropertyPanelNumericEditStarted(const QString& contextId, const QString& propertyKey) override;
	bool deferPropertyPanelVisualFullSync(const QString& contextId) const override;
	void clearPropertyKeyVariantMap() override;

	void scheduleDeferredFeasibleAxisProbe(const std::shared_ptr<RobotInstruction::Base>& instruction) override;

private:
	MainWindow& m_mw;
};

#endif // WIDGET_MAINWINDOWINSTRUCTIONPROPERTYUIHOST_H
