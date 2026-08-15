/// @file MainWindowInstructionPropertyUiHost.cpp
/// @brief MainWindowInstructionPropertyUiHost 实现

#include "MainWindowInstructionPropertyUiHost.h"

#include "../RobotWidget/inc/IRobotDocumentHost.h"
#include "../RobotWidget/inc/InstructionPropertyPanel.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"
#include "BackendDataManager.h"
#include "BackendTypeIds.h"
#include "MainWindow.h"
#include "MainWindowRobotHost.h"
#include "NamedSignalTable.h"
#include "RunInfoPage.h"
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
																		 const QString& key, const QString& value,
																		 QString* outError)
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

void MainWindowInstructionPropertyUiHost::appendPropertyBrowserRow(
	const QString& propertyKey, const QString& displayLabel, const QString& value, const bool editable,
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

cloudsim::core::FeasibleMotionAxisOptionsDto
MainWindowInstructionPropertyUiHost::cachedFeasibleMotionAxisOptionsDto() const
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
	m_mw.scheduleInstructionPropertyRefreshDebounced(instruction, refreshFeasibleAxisOptions);
}

void MainWindowInstructionPropertyUiHost::notifyPropertyPanelNumericEditStarted(const QString& contextId,
																				const QString& propertyKey)
{
	m_mw.beginPropertyPanelNumericEdit(contextId, propertyKey);
}

bool MainWindowInstructionPropertyUiHost::deferPropertyPanelVisualFullSync(const QString& contextId) const
{
	return m_mw.shouldDeferPropertyPanelRebuild(contextId);
}

void MainWindowInstructionPropertyUiHost::clearPropertyKeyVariantMap()
{
	m_mw.clearPropertyKeyVariantMap();
}

void MainWindowInstructionPropertyUiHost::scheduleDeferredFeasibleAxisProbe(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (m_mw.m_robotSimulation)
	{
		m_mw.m_robotSimulation->scheduleDeferredFeasibleAxisProbe(instruction);
	}
}

QStringList MainWindowInstructionPropertyUiHost::namedIoSignalNames(const QString& kindFilter) const
{
	QStringList out;
	if (!m_mw.m_robotSimulation)
	{
		return out;
	}
	QString ownerId = m_mw.m_robotSimulation->ioUiOwnerId();
	IRobotDocumentHost* doc = m_mw.m_robotHost ? m_mw.m_robotHost->document() : nullptr;
	if (doc && m_mw.simulationCommandPage())
	{
		const int inst = m_mw.simulationCommandPage()->currentRobotInstanceIndex();
		if (inst >= 0)
		{
			ownerId = doc->robotSceneBackendIdForInstance(inst);
		}
	}
	RobotIo::SignalKind kind = RobotIo::SignalKind::DI;
	const bool filter = !kindFilter.isEmpty() &&
						RobotIo::NamedSignalTable::kindFromString(kindFilter.toStdString(), kind);
	if (filter)
	{
		return m_mw.m_robotSimulation->ioSignalNamesForOwner(ownerId, kind);
	}
	out += m_mw.m_robotSimulation->ioSignalNamesForOwner(ownerId, RobotIo::SignalKind::DI);
	out += m_mw.m_robotSimulation->ioSignalNamesForOwner(ownerId, RobotIo::SignalKind::DO);
	out += m_mw.m_robotSimulation->ioSignalNamesForOwner(ownerId, RobotIo::SignalKind::AI);
	out += m_mw.m_robotSimulation->ioSignalNamesForOwner(ownerId, RobotIo::SignalKind::AO);
	return out;
}

int MainWindowInstructionPropertyUiHost::resolveNamedIoSignalPort(const QString& signalName) const
{
	if (!m_mw.m_robotSimulation || signalName.isEmpty())
	{
		return -1;
	}
	QString ownerId = m_mw.m_robotSimulation->ioUiOwnerId();
	IRobotDocumentHost* doc = m_mw.m_robotHost ? m_mw.m_robotHost->document() : nullptr;
	if (doc && m_mw.simulationCommandPage())
	{
		const int inst = m_mw.simulationCommandPage()->currentRobotInstanceIndex();
		if (inst >= 0)
		{
			ownerId = doc->robotSceneBackendIdForInstance(inst);
		}
	}
	if (const RobotIo::NamedSignalTable* table = m_mw.m_robotSimulation->namedSignalTableForOwner(ownerId))
	{
		if (const RobotIo::SignalDef* s = table->findByName(signalName.toStdString()))
		{
			return s->port;
		}
	}
	return -1;
}

QStringList MainWindowInstructionPropertyUiHost::customDeviceBackendIds() const
{
	QStringList out;
	IRobotDocumentHost* doc =
		m_mw.m_robotHost ? m_mw.m_robotHost->document() : nullptr;
	if (!doc)
	{
		return out;
	}
	for (const auto& data : doc->backend().findByClass(backend_type::kClassCustomDevice))
	{
		if (data)
		{
			out << QString::fromStdString(data->id());
		}
	}
	return out;
}
