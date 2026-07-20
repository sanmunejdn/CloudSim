/// @file ProgramEditService.cpp
/// @brief ProgramEditService 实现

#include "ProgramEditService.h"

#include "RobotProgramStore.h"

ProgramEditService::ProgramEditService(QObject* parent) : QObject(parent) {}

void ProgramEditService::bindStore(RobotProgramStore* store)
{
	m_store = store;
}

bool ProgramEditService::execute(std::shared_ptr<RobotInstruction::ProgramEditCommand> cmd, QString* outError)
{
	if (!m_store)
	{
		if (outError)
		{
			*outError = QStringLiteral("no program store");
		}
		return false;
	}
	RobotInstruction::InstructionProgramDocument doc(&m_store->activeProgram());
	std::string err;
	if (!m_stack.execute(cmd, doc, &err))
	{
		if (outError)
		{
			*outError = QString::fromStdString(err);
		}
		return false;
	}
	doc.renumberAndNotify();
	bumpRevision();
	return true;
}

bool ProgramEditService::execute(std::unique_ptr<RobotInstruction::ProgramEditCommand> cmd, QString* outError)
{
	return execute(std::shared_ptr<RobotInstruction::ProgramEditCommand>(std::move(cmd)), outError);
}

bool ProgramEditService::executeBatch(const std::vector<RobotInstruction::ProgramEditStack::CommandPtr>& cmds,
									  QString* outError)
{
	if (!m_store)
	{
		if (outError)
		{
			*outError = QStringLiteral("no program store");
		}
		return false;
	}
	if (cmds.empty())
	{
		return true;
	}
	RobotInstruction::InstructionProgramDocument doc(&m_store->activeProgram());
	std::string err;
	for (const auto& cmd : cmds)
	{
		if (!m_stack.execute(cmd, doc, &err))
		{
			if (outError)
			{
				*outError = QString::fromStdString(err);
			}
			return false;
		}
	}
	doc.renumberAndNotify();
	bumpRevision();
	return true;
}

bool ProgramEditService::undo(QString* outError)
{
	if (!m_store)
	{
		return false;
	}
	RobotInstruction::InstructionProgramDocument doc(&m_store->activeProgram());
	std::string err;
	if (!m_stack.undo(doc, &err))
	{
		if (outError)
		{
			*outError = QString::fromStdString(err);
		}
		return false;
	}
	doc.renumberAndNotify();
	bumpRevision();
	return true;
}

bool ProgramEditService::redo(QString* outError)
{
	if (!m_store)
	{
		return false;
	}
	RobotInstruction::InstructionProgramDocument doc(&m_store->activeProgram());
	std::string err;
	if (!m_stack.redo(doc, &err))
	{
		if (outError)
		{
			*outError = QString::fromStdString(err);
		}
		return false;
	}
	doc.renumberAndNotify();
	bumpRevision();
	return true;
}

bool ProgramEditService::canUndo() const
{
	return m_stack.canUndo();
}

bool ProgramEditService::canRedo() const
{
	return m_stack.canRedo();
}

RobotInstruction::InstructionProgramDocument ProgramEditService::currentDocument()
{
	if (!m_store)
	{
		static std::vector<std::shared_ptr<RobotInstruction::Base>> s_empty;
		return RobotInstruction::InstructionProgramDocument(&s_empty);
	}
	return RobotInstruction::InstructionProgramDocument(&m_store->activeProgram());
}

void ProgramEditService::bumpRevision()
{
	++m_revision;
	emit revisionChanged(m_revision);
}
