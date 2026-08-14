/// @file RobotSimulationController_AxisTargets.cpp
/// @brief 轴控目标列表（从主文件拆出的首个安全编译单元）

#include "RobotSimulationController.h"

#include "BackendTypeIds.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"
#include "RobotAxisControlWidget.h"

void RobotSimulationController::refreshAxisControlTargets()
{
	RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!axis || !doc)
	{
		return;
	}
	QVector<AxisControlTargetItem> targets;
	const int robotCount = doc->robotKinematicInstanceCount();
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
