#ifndef ROBOTWIDGET_PROGRAMEDITSERVICE_H
#define ROBOTWIDGET_PROGRAMEDITSERVICE_H

/// @file ProgramEditService.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 程序编辑撤销栈与文档门面

#include "robotwidget_global.h"

#include "InstructionProgramDocument.h"
#include "ProgramEditCommand.h"

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
	bool executeBatch(const std::vector<RobotInstruction::ProgramEditStack::CommandPtr>& cmds,
					  QString* outError = nullptr);
	bool undo(QString* outError = nullptr);
	bool redo(QString* outError = nullptr);
	bool canUndo() const;
	bool canRedo() const;
	int revision() const { return m_revision; }

	RobotInstruction::InstructionProgramDocument currentDocument();

signals:
	void revisionChanged(int revision);

private:
	void bumpRevision();

	RobotProgramStore* m_store = nullptr;
	RobotInstruction::ProgramEditStack m_stack;
	int m_revision = 0;
};

#endif // ROBOTWIDGET_PROGRAMEDITSERVICE_H
