/// @file HeadlessProgramEditBridge.cpp

#include "headless/HeadlessProgramEditBridge.h"

#include "DocumentHost.h"
#include "RobotProgramStore.h"

#include <QJsonArray>

namespace cloudsim::host
{
namespace
{
QJsonObject fail(const QString& err)
{
	return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
}
} // namespace

HeadlessProgramEditBridge::HeadlessProgramEditBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessProgramEditBridge::undo(const QJsonObject& body)
{
	(void)body;
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("canUndo"), false);
	o.insert(QStringLiteral("stub"), true);
	return o;
}

QJsonObject HeadlessProgramEditBridge::redo(const QJsonObject& body)
{
	(void)body;
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("canRedo"), false);
	o.insert(QStringLiteral("stub"), true);
	return o;
}

QJsonObject HeadlessProgramEditBridge::switchProgram(const QJsonObject& body)
{
	const QString programId = body.value(QStringLiteral("programId")).toString();
	if (programId.isEmpty())
		return fail(QStringLiteral("programId required."));

	const QString sceneRoot = body.value(QStringLiteral("sceneRootBackendId")).toString();
	RobotProgramStore& store = m_host.robotProgramStore();
	if (!sceneRoot.isEmpty())
		store.setActiveRobotBackendId(sceneRoot);

	store.setActiveProgramIdUtf8(programId.toStdString());
	const RobotInstruction::RobotProgram* prog =
		store.activeCatalog().findProgram(programId.toStdString());
	if (!prog)
		return fail(QStringLiteral("Program not found."));

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("programId"), programId);
	o.insert(QStringLiteral("programName"), QString::fromStdString(prog->name));
	return o;
}

QJsonObject HeadlessProgramEditBridge::groupCrud(const QJsonObject& body)
{
	const QString action = body.value(QStringLiteral("action")).toString(QStringLiteral("list"));
	RobotInstruction::RobotProgramCatalog& catalog = m_host.robotProgramStore().activeCatalog();
	RobotInstruction::RobotProgram* prog = catalog.findProgram(catalog.activeProgramId());
	if (!prog)
		prog = catalog.mainProgram();
	if (!prog)
		return fail(QStringLiteral("No active program."));

	if (action == QStringLiteral("list"))
	{
		QJsonArray groups;
		for (const RobotInstruction::InstructionGroup& g : prog->groups)
		{
			QJsonObject go;
			go.insert(QStringLiteral("id"), QString::fromStdString(g.id));
			go.insert(QStringLiteral("name"), QString::fromStdString(g.name));
			go.insert(QStringLiteral("memberCount"), static_cast<int>(g.memberInstructionIds.size()));
			groups.append(go);
		}
		QJsonObject o;
		o.insert(QStringLiteral("ok"), true);
		o.insert(QStringLiteral("groups"), groups);
		return o;
	}

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("stub"), true);
	o.insert(QStringLiteral("action"), action);
	return o;
}

} // namespace cloudsim::host
