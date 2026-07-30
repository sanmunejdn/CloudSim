#ifndef ROBOTWIDGET_PIPELINEDRAFTEDITSTACK_H
#define ROBOTWIDGET_PIPELINEDRAFTEDITSTACK_H

/// @file PipelineDraftEditStack.h
/// @brief 轨迹流水线草稿撤销栈（与 ProgramEditStack 分离）

#include "robotwidget_global.h"

#include "TrajectoryPipelineTypes.h"

#include <vector>

struct ROBOTWIDGET_EXPORT PipelineDraftSnapshot
{
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops;
	int selectedIndex = -1;
};

/// 草稿期算子快照栈；参数微调可 coalesce 合并
class ROBOTWIDGET_EXPORT PipelineDraftEditStack
{
public:
	void push(PipelineDraftSnapshot before, bool coalesce = false);
	void endCoalesceWindow() { m_lastWasCoalesce = false; }
	bool canUndo() const { return !m_undo.empty(); }
	bool canRedo() const { return !m_redo.empty(); }
	bool undo(const PipelineDraftSnapshot& current, PipelineDraftSnapshot& outRestored);
	bool redo(const PipelineDraftSnapshot& current, PipelineDraftSnapshot& outRestored);
	void clear();

private:
	std::vector<PipelineDraftSnapshot> m_undo;
	std::vector<PipelineDraftSnapshot> m_redo;
	bool m_lastWasCoalesce = false;
};

#endif // ROBOTWIDGET_PIPELINEDRAFTEDITSTACK_H
