/// @file HeadlessProgramEditBridge.cpp
/// @brief Web 程序编辑：undo/redo/切换/分组/程序 CRUD

#include "headless/HeadlessProgramEditBridge.h"

#include "DocumentHost.h"
#include "ProgramEditCommand.h"
#include "RobotProgramCatalog.h"
#include "RobotProgramStore.h"

#include <QJsonArray>
#include <QUuid>

namespace cloudsim::host
{
namespace
{
QJsonObject fail(const QString& err)
{
	return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
}

/// body 带 sceneRoot 时登记并激活，避免落到静态空 catalog
bool activateSceneFromBody(RobotProgramStore& store, const QJsonObject& body, QString* err)
{
	const QString sceneRoot = body.value(QStringLiteral("sceneRootBackendId")).toString();
	if (sceneRoot.isEmpty())
		return true;
	if (!store.ensureRobotBackendId(sceneRoot))
	{
		if (err)
			*err = QStringLiteral("invalid sceneRootBackendId");
		return false;
	}
	return true;
}

RobotInstruction::RobotProgram* activeProg(RobotProgramStore& store)
{
	RobotInstruction::RobotProgramCatalog& catalog = store.activeCatalog();
	RobotInstruction::RobotProgram* prog = catalog.findProgram(catalog.activeProgramId());
	if (!prog)
		prog = catalog.mainProgram();
	return prog;
}

std::vector<std::string> readIdArray(const QJsonArray& arr)
{
	std::vector<std::string> out;
	out.reserve(static_cast<size_t>(arr.size()));
	for (const QJsonValue& v : arr)
	{
		const QString s = v.toString();
		if (!s.isEmpty())
			out.push_back(s.toStdString());
	}
	return out;
}
} // namespace

HeadlessProgramEditBridge::HeadlessProgramEditBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessProgramEditBridge::undo(const QJsonObject& body)
{
	QString actErr;
	if (!activateSceneFromBody(m_host.robotProgramStore(), body, &actErr))
		return fail(actErr);
	RobotInstruction::InstructionProgramDocument doc(&m_host.robotProgramStore().activeProgram());
	std::string err;
	if (!m_editStack.undo(doc, &err))
		return fail(err.empty() ? QStringLiteral("无可撤销") : QString::fromStdString(err));
	doc.renumberAndNotify();
	return QJsonObject{{QStringLiteral("ok"), true},
					   {QStringLiteral("canUndo"), m_editStack.canUndo()},
					   {QStringLiteral("canRedo"), m_editStack.canRedo()}};
}

QJsonObject HeadlessProgramEditBridge::redo(const QJsonObject& body)
{
	QString actErr;
	if (!activateSceneFromBody(m_host.robotProgramStore(), body, &actErr))
		return fail(actErr);
	RobotInstruction::InstructionProgramDocument doc(&m_host.robotProgramStore().activeProgram());
	std::string err;
	if (!m_editStack.redo(doc, &err))
		return fail(err.empty() ? QStringLiteral("无可重做") : QString::fromStdString(err));
	doc.renumberAndNotify();
	return QJsonObject{{QStringLiteral("ok"), true},
					   {QStringLiteral("canUndo"), m_editStack.canUndo()},
					   {QStringLiteral("canRedo"), m_editStack.canRedo()}};
}

QJsonObject HeadlessProgramEditBridge::switchProgram(const QJsonObject& body)
{
	const QString programId = body.value(QStringLiteral("programId")).toString();
	if (programId.isEmpty())
		return fail(QStringLiteral("programId required."));

	RobotProgramStore& store = m_host.robotProgramStore();
	QString actErr;
	if (!activateSceneFromBody(store, body, &actErr))
		return fail(actErr);

	const RobotInstruction::RobotProgram* prog =
		store.activeCatalog().findProgram(programId.toStdString());
	if (!prog)
		return fail(QStringLiteral("Program not found."));

	store.setActiveProgramIdUtf8(programId.toStdString());
	m_editStack.clear();
	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("programId"), programId);
	o.insert(QStringLiteral("programName"), QString::fromStdString(prog->name));
	return o;
}

QJsonObject HeadlessProgramEditBridge::groupCrud(const QJsonObject& body)
{
	const QString action = body.value(QStringLiteral("action")).toString(QStringLiteral("list"));
	RobotProgramStore& store = m_host.robotProgramStore();
	QString actErr;
	if (!activateSceneFromBody(store, body, &actErr))
		return fail(actErr);

	RobotInstruction::RobotProgram* prog = activeProg(store);
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
			QJsonArray members;
			for (const std::string& mid : g.memberInstructionIds)
				members.append(QString::fromStdString(mid));
			go.insert(QStringLiteral("memberInstructionIds"), members);
			groups.append(go);
		}
		return QJsonObject{{QStringLiteral("ok"), true}, {QStringLiteral("groups"), groups}};
	}

	RobotInstruction::InstructionProgramDocument doc(&store.activeProgram());
	std::string err;
	if (action == QStringLiteral("create"))
	{
		const QString name = body.value(QStringLiteral("name")).toString(QStringLiteral("Group"));
		const auto members = readIdArray(body.value(QStringLiteral("memberInstructionIds")).toArray());
		auto cmd = std::make_shared<RobotInstruction::CreateInstructionGroupCommand>(prog, name.toStdString(),
																					 members);
		if (!m_editStack.execute(cmd, doc, &err))
			return fail(QString::fromStdString(err));
		doc.renumberAndNotify();
		return QJsonObject{{QStringLiteral("ok"), true}};
	}
	if (action == QStringLiteral("remove") || action == QStringLiteral("dissolve"))
	{
		const QString groupId = body.value(QStringLiteral("groupId")).toString();
		if (groupId.isEmpty())
			return fail(QStringLiteral("groupId required."));
		auto cmd = std::make_shared<RobotInstruction::RemoveInstructionGroupCommand>(prog, groupId.toStdString());
		if (!m_editStack.execute(cmd, doc, &err))
			return fail(QString::fromStdString(err));
		doc.renumberAndNotify();
		return QJsonObject{{QStringLiteral("ok"), true}};
	}
	if (action == QStringLiteral("rename"))
	{
		const QString groupId = body.value(QStringLiteral("groupId")).toString();
		const QString newName = body.value(QStringLiteral("name")).toString();
		if (groupId.isEmpty() || newName.isEmpty())
			return fail(QStringLiteral("groupId and name required."));
		auto cmd = std::make_shared<RobotInstruction::RenameInstructionGroupCommand>(
			prog, groupId.toStdString(), newName.toStdString());
		if (!m_editStack.execute(cmd, doc, &err))
			return fail(QString::fromStdString(err));
		doc.renumberAndNotify();
		return QJsonObject{{QStringLiteral("ok"), true}};
	}
	return fail(QStringLiteral("Unknown group action."));
}

QJsonObject HeadlessProgramEditBridge::programCrud(const QJsonObject& body)
{
	const QString action = body.value(QStringLiteral("action")).toString();
	RobotProgramStore& store = m_host.robotProgramStore();
	QString actErr;
	if (!activateSceneFromBody(store, body, &actErr))
		return fail(actErr);
	if (store.activeRobotBackendId().isEmpty())
		return fail(QStringLiteral("sceneRootBackendId required."));
	RobotInstruction::RobotProgramCatalog& catalog = store.activeCatalog();
	std::string err;

	if (action == QStringLiteral("create"))
	{
		RobotInstruction::RobotProgram prog;
		const QString givenId = body.value(QStringLiteral("programId")).toString();
		prog.id = givenId.isEmpty() ? QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString()
									: givenId.toStdString();
		prog.name = body.value(QStringLiteral("name")).toString(QStringLiteral("Program")).toStdString();
		const std::string newId = prog.id;
		if (!catalog.addProgram(std::move(prog), &err))
			return fail(QString::fromStdString(err));
		catalog.setActiveProgramId(newId);
		m_editStack.clear();
		return QJsonObject{{QStringLiteral("ok"), true},
						   {QStringLiteral("programId"), QString::fromStdString(newId)}};
	}
	if (action == QStringLiteral("rename"))
	{
		const QString programId = body.value(QStringLiteral("programId")).toString();
		const QString newName = body.value(QStringLiteral("name")).toString();
		if (programId.isEmpty() || newName.isEmpty())
			return fail(QStringLiteral("programId and name required."));
		if (!catalog.renameProgram(programId.toStdString(), newName.toStdString(), &err))
			return fail(QString::fromStdString(err));
		return QJsonObject{{QStringLiteral("ok"), true}};
	}
	if (action == QStringLiteral("delete") || action == QStringLiteral("remove"))
	{
		const QString programId = body.value(QStringLiteral("programId")).toString();
		if (programId.isEmpty())
			return fail(QStringLiteral("programId required."));
		if (!catalog.removeProgram(programId.toStdString(), &err))
			return fail(QString::fromStdString(err));
		m_editStack.clear();
		return QJsonObject{{QStringLiteral("ok"), true},
						   {QStringLiteral("activeProgramId"), QString::fromStdString(catalog.activeProgramId())}};
	}
	return fail(QStringLiteral("Unknown program action."));
}

} // namespace cloudsim::host
