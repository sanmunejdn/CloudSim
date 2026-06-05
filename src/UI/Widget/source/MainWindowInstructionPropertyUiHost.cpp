#include "MainWindowInstructionPropertyUiHost.h"

#include "MainWindow.h"
#include "MainWindowRobotHost.h"

#include "../RobotWidget/inc/IRobotDocumentHost.h"
#include "RunInfoPage.h"

#include "../RobotWidget/inc/InstructionPropertyPanel.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"

#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

MainWindowInstructionPropertyUiHost::MainWindowInstructionPropertyUiHost(MainWindow& mw) : m_mw(mw) {}

QtTreePropertyBrowser* MainWindowInstructionPropertyUiHost::propertyBrowser()
{
	return m_mw.m_propertyBrowser;
}

QtVariantPropertyManager* MainWindowInstructionPropertyUiHost::variantManager()
{
	return m_mw.m_variantManager;
}

bool& MainWindowInstructionPropertyUiHost::updatingPropertyBrowserFlag()
{
	return m_mw.m_updatingPropertyBrowser;
}

QHash<QtProperty*, QStringList>& MainWindowInstructionPropertyUiHost::propertyEnumTokens()
{
	return m_mw.m_propertyEnumTokens;
}

IRobotDocumentHost* MainWindowInstructionPropertyUiHost::currentRobotDocument()
{
	return m_mw.m_robotHost ? m_mw.m_robotHost->document() : nullptr;
}

bool MainWindowInstructionPropertyUiHost::applyInstructionPropertyChange(const QString& instructionId,
	const QString& key, const QString& value, QString* outError)
{
	if (m_mw.m_robotHost)
	{
		return m_mw.m_robotHost->applyInstructionPropertyChange(instructionId, key, value, outError);
	}
	return false;
}

SimulationCommandWidget* MainWindowInstructionPropertyUiHost::simulationCommandPage()
{
	return m_mw.simulationCommandPage();
}

int MainWindowInstructionPropertyUiHost::currentSimulationRobotInstanceIndex() const
{
	return m_mw.currentSimulationRobotInstanceIndex();
}

void MainWindowInstructionPropertyUiHost::appendRunInfoMessage(const QString& en, const QString& zh)
{
	m_mw.appendRunInfo(m_mw.i18n(en, zh));
}

bool MainWindowInstructionPropertyUiHost::useChinese() const
{
	return m_mw.useChinese();
}

QString MainWindowInstructionPropertyUiHost::i18n(const QString& en, const QString& zh) const
{
	return m_mw.i18n(en, zh);
}

void MainWindowInstructionPropertyUiHost::appendPropertyBrowserRow(const QString& propertyKey,
	const QString& displayLabel, const QString& value, const bool editable,
	const std::vector<std::string>* enumOptionTokens, const QStringList* enumDisplayNames, const QString& toolTip)
{
	m_mw.appendPropertyBrowserRow(propertyKey, displayLabel, value, editable, enumOptionTokens, enumDisplayNames,
		toolTip);
}

QString MainWindowInstructionPropertyUiHost::propertyDisplayLabelForKey(const QString& key,
	const QString& labelEnFallback) const
{
	return m_mw.propertyDisplayLabelForKey(key, labelEnFallback);
}

QString MainWindowInstructionPropertyUiHost::instructionEnumTokenFromProperty(QtProperty* property,
	const QVariant& value) const
{
	return m_mw.instructionEnumTokenFromProperty(property, value);
}

RobotInstruction::FeasibleMotionAxisConfigurationOptions
MainWindowInstructionPropertyUiHost::feasibleMotionAxisConfigurationOptionsForInstruction(
	const std::shared_ptr<RobotInstruction::Base>& instruction, QVector<double>* outSeedJointRad)
{
	return m_mw.feasibleMotionAxisConfigurationOptionsForInstruction(instruction, outSeedJointRad);
}

cloudsim::core::FeasibleMotionAxisOptionsDto MainWindowInstructionPropertyUiHost::cachedFeasibleMotionAxisOptionsDto() const
{
	if (!m_mw.m_robotHost)
	{
		return {};
	}
	return m_mw.m_robotHost->cachedFeasibleMotionAxisOptions();
}

void MainWindowInstructionPropertyUiHost::applySuggestedAxisPresetFromSeedIfNeeded(
	const std::shared_ptr<RobotInstruction::Base>& instruction, const QVector<double>& seedJointRad,
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible)
{
	InstructionPropertyPanel::applySuggestedAxisPresetFromSeedIfNeeded(*this, instruction, seedJointRad, feasible);
}

void MainWindowInstructionPropertyUiHost::setActiveInstructionForProperty(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	m_mw.m_activeInstructionForProperty = instruction;
}

std::shared_ptr<RobotInstruction::Base> MainWindowInstructionPropertyUiHost::activeInstructionForProperty() const
{
	return m_mw.m_activeInstructionForProperty;
}

void MainWindowInstructionPropertyUiHost::invalidateFeasibleAxisConfigurationCache()
{
	m_mw.invalidateFeasibleAxisConfigurationCache();
}

void MainWindowInstructionPropertyUiHost::refreshInstructionPoseAxes()
{
	m_mw.refreshInstructionPoseAxes();
}

void MainWindowInstructionPropertyUiHost::syncInstructionRenderMatricesFromPose(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	m_mw.syncInstructionRenderMatricesFromPose(instruction);
}

void MainWindowInstructionPropertyUiHost::applyRobotPoseForInstructionPreview(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	m_mw.applyRobotPoseForInstructionPreview(instruction);
}

void MainWindowInstructionPropertyUiHost::refreshRobotCoordinateFrameOverlays(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	m_mw.refreshRobotCoordinateFrameOverlays(instruction);
}

void MainWindowInstructionPropertyUiHost::scheduleInstructionPropertyRefresh(
	const std::shared_ptr<RobotInstruction::Base>& instruction, const bool refreshFeasibleAxisOptions)
{
	InstructionPropertyPanel::update(*this, instruction, refreshFeasibleAxisOptions);
}

void MainWindowInstructionPropertyUiHost::scheduleDeferredFeasibleAxisProbe(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_mw.m_robotSimulation)
	{
		m_mw.m_robotSimulation->scheduleDeferredFeasibleAxisProbe(instruction);
	}
}
