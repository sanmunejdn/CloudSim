/// @file AxisControlTargetService.cpp
/// @brief 轴控双目标

#include "AxisControlTargetService.h"

#include "BackendTypeIds.h"
#include "CustomDeviceBackendData.h"
#include "CustomDeviceKinematics.h"
#include "CustomDeviceKinematicModel.h"
#include "CompositeKinematicModel.h"
#include "KinematicModelRegistry.h"
#include "RobotExternalAxisSceneApply.h"
#include "RobotKinematicModelRegistration.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"
#include "RobotAxisControlWidget.h"
#include "RobotExternalAxes.h"
#include "RobotSimulationController.h"
#include "SimulationCommandWidget.h"
#include "UrdfRobotKinematicModel.h"

#include <QHash>

namespace
{
QVector<double> enabledValuesFromFullQ(const RobotExternal::RobotExternalAxisConfigSet& set,
									   const std::vector<double>& fullQ)
{
	QVector<double> out;
	const std::vector<int> idxs = RobotExternal::enabledExternalAxisIndices(set);
	out.reserve(static_cast<int>(idxs.size()));
	for (const int idx : idxs)
	{
		double v = 0.0;
		if (idx >= 0 && idx < static_cast<int>(fullQ.size()))
		{
			v = fullQ[static_cast<size_t>(idx)];
		}
		else if (idx >= 0 && idx < static_cast<int>(set.axes.size()))
		{
			v = set.axes[static_cast<size_t>(idx)].home;
		}
		out.push_back(v);
	}
	return out;
}
} // namespace

AxisControlTargetService::AxisControlTargetService(QObject* parent) : QObject(parent) {}

void AxisControlTargetService::setHost(IRobotMainWindowHost* host)
{
	m_host = host;
}

void AxisControlTargetService::setSimulationController(RobotSimulationController* controller)
{
	m_controller = controller;
}

void AxisControlTargetService::refreshTargets()
{
	RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (axis && doc)
	{
		QVector<AxisControlTargetItem> targets;
		const int robotCount = doc->robotKinematicInstanceCount();
		QHash<QString, int> labelUseCount;
		for (int i = 0; i < robotCount; ++i)
		{
			QString label = doc->robotDisplayLabelForInstance(i);
			if (label.isEmpty())
			{
				label = QStringLiteral("Robot %1").arg(i + 1);
			}
			++labelUseCount[label];
		}
		for (int i = 0; i < robotCount; ++i)
		{
			AxisControlTargetItem item;
			item.kind = AxisControlTargetKind::RobotInstance;
			item.robotInstanceIndex = i;
			item.id = doc->robotSceneBackendIdForInstance(i);
			item.displayLabel = doc->robotDisplayLabelForInstance(i);
			if (item.displayLabel.isEmpty())
			{
				item.displayLabel = QStringLiteral("Robot %1").arg(i + 1);
			}
			if (labelUseCount.value(item.displayLabel) > 1)
			{
				item.displayLabel = QStringLiteral("%1 (#%2)").arg(item.displayLabel).arg(i + 1);
			}
			targets.push_back(item);
		}
		for (const auto& id : doc->documentData().findByClassName(QString::fromUtf8(backend_type::kClassCustomDevice)))
		{
			if (id.isEmpty())
			{
				continue;
			}
			AxisControlTargetItem item;
			item.kind = AxisControlTargetKind::CustomDevice;
			item.id = id;
			item.displayLabel = doc->documentData().displayName(id);
			if (item.displayLabel.isEmpty())
			{
				item.displayLabel = item.id;
			}
			targets.push_back(item);
		}
		axis->setControlTargets(targets);
	}
	emit catalogChanged();
}

void AxisControlTargetService::onTargetChanged(const AxisControlTargetKind kind, const QString& id)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr;
	if (!doc || !axis)
	{
		return;
	}
	if (kind == AxisControlTargetKind::CustomDevice)
	{
		if (m_host)
		{
			m_host->prepareCustomDeviceAxisControlTarget(id);
		}
		axis->clearJoints();
		const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(id.toStdString()));
		if (!device)
		{
			axis->clearExternalAxes();
			return;
		}
		device->ensureQSize();
		KinematicModelRegistry::registerModel(KinematicModelRegistry::keyCustomDevice(id.toStdString()),
											  CustomDeviceKinematicModel::create(*device));
		const RobotExternal::RobotExternalAxisConfigSet ext =
			CustomDeviceKinematics::toExternalAxisConfigSet(device->axes());
		axis->setExternalAxes(ext);
		axis->setExternalAxisValuesSilent(enabledValuesFromFullQ(ext, device->qValues()));
		return;
	}
	int instIdx = -1;
	for (int i = 0; i < doc->robotKinematicInstanceCount(); ++i)
	{
		if (doc->robotSceneBackendIdForInstance(i) == id)
		{
			instIdx = i;
			break;
		}
	}
	if (instIdx < 0 && m_host->simulationCommandPage())
	{
		instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	}
	if (instIdx < 0)
	{
		return;
	}
	if (SimulationCommandWidget* cmd = m_host->simulationCommandPage())
	{
		cmd->setCurrentRobotInstanceIndex(instIdx);
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (!urdfPath.isEmpty())
	{
		(void)RobotKinematicModelRegistration::registerRobotInstance(doc, instIdx, id);
	}
	QVector<double> lower;
	QVector<double> upper;
	doc->robotJointLimitsForInstance(instIdx, lower, upper);
	axis->setJoints(doc->robotRevoluteJointNamesForInstance(instIdx), lower, upper);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (m_controller)
	{
		const QVector<double> agg = m_controller->aggregatedJointAnglesRad();
		if (agg.size() >= jointOffset + nj && nj > 0)
		{
			axis->setJointAnglesRadSilent(agg.mid(jointOffset, nj));
		}
		m_controller->syncRobotAxisControlExternalAxes(instIdx);
	}
}
