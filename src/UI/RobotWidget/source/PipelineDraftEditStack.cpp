/// @file PipelineDraftEditStack.cpp
/// @brief 轨迹流水线草稿撤销栈

#include "PipelineDraftEditStack.h"

void PipelineDraftEditStack::push(PipelineDraftSnapshot before, const bool coalesce)
{
	if (coalesce && m_lastWasCoalesce && !m_undo.empty())
	{
		// 连续参数微调合并为一条，保留 coalesce 窗口起点快照
		m_redo.clear();
		return;
	}
	m_undo.push_back(std::move(before));
	m_redo.clear();
	m_lastWasCoalesce = coalesce;
}

bool PipelineDraftEditStack::undo(const PipelineDraftSnapshot& current, PipelineDraftSnapshot& outRestored)
{
	if (m_undo.empty())
	{
		return false;
	}
	m_redo.push_back(current);
	outRestored = std::move(m_undo.back());
	m_undo.pop_back();
	m_lastWasCoalesce = false;
	return true;
}

bool PipelineDraftEditStack::redo(const PipelineDraftSnapshot& current, PipelineDraftSnapshot& outRestored)
{
	if (m_redo.empty())
	{
		return false;
	}
	m_undo.push_back(current);
	outRestored = std::move(m_redo.back());
	m_redo.pop_back();
	m_lastWasCoalesce = false;
	return true;
}

void PipelineDraftEditStack::clear()
{
	m_undo.clear();
	m_redo.clear();
	m_lastWasCoalesce = false;
}
