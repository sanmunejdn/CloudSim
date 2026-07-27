/// @file CommandStack.cpp
/// @brief 移植自 OneCAD CommandProcessor（简化深度 200）

#include "CommandStack.h"

CommandStack::CommandStack(QObject* parent) : QObject(parent) {}

bool CommandStack::execute(std::unique_ptr<GeomodelingCommand> command)
{
	if (!command || !command->execute())
		return false;
	const bool prevU = canUndo();
	const bool prevR = canRedo();
	m_undo.push_back(std::move(command));
	m_redo.clear();
	constexpr std::size_t kMax = 200;
	if (m_undo.size() > kMax)
		m_undo.erase(m_undo.begin());
	if (prevU != canUndo())
		emit canUndoChanged(canUndo());
	if (prevR != canRedo())
		emit canRedoChanged(canRedo());
	return true;
}

void CommandStack::undo()
{
	if (m_undo.empty())
		return;
	const bool prevU = canUndo();
	const bool prevR = canRedo();
	auto cmd = std::move(m_undo.back());
	m_undo.pop_back();
	if (cmd->undo())
		m_redo.push_back(std::move(cmd));
	if (prevU != canUndo())
		emit canUndoChanged(canUndo());
	if (prevR != canRedo())
		emit canRedoChanged(canRedo());
}

void CommandStack::redo()
{
	if (m_redo.empty())
		return;
	const bool prevU = canUndo();
	const bool prevR = canRedo();
	auto cmd = std::move(m_redo.back());
	m_redo.pop_back();
	if (cmd->execute())
		m_undo.push_back(std::move(cmd));
	if (prevU != canUndo())
		emit canUndoChanged(canUndo());
	if (prevR != canRedo())
		emit canRedoChanged(canRedo());
}

bool CommandStack::canUndo() const
{
	return !m_undo.empty();
}

bool CommandStack::canRedo() const
{
	return !m_redo.empty();
}

void CommandStack::clear()
{
	m_undo.clear();
	m_redo.clear();
	emit canUndoChanged(false);
	emit canRedoChanged(false);
}
