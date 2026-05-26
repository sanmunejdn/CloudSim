#pragma once

#include "InstructionProgramDocument.h"
#include "ProgramEditCommand.h"
#include "robotwidget_global.h"

#include <QObject>

#include <memory>

class RobotProgramStore;

/// 程序编辑撤销栈与文档门面
class ROBOTWIDGET_EXPORT ProgramEditService : public QObject
{
	Q_OBJECT

public:
	explicit ProgramEditService(QObject* parent = nullptr);

	void bindStore(RobotProgramStore* store);

	bool execute(std::shared_ptr<RobotInstruction::ProgramEditCommand> cmd, QString* outError = nullptr);
	bool execute(std::unique_ptr<RobotInstruction::ProgramEditCommand> cmd, QString* outError = nullptr);
	bool undo(QString* outError = nullptr);
	bool redo(QString* outError = nullptr);
	bool canUndo() const;
	bool canRedo() const;

	RobotInstruction::InstructionProgramDocument currentDocument();

signals:
	void revisionChanged(int revision);

private:
	void bumpRevision();

	RobotProgramStore* m_store = nullptr;
	RobotInstruction::ProgramEditStack m_stack;
	int m_revision = 0;
};
